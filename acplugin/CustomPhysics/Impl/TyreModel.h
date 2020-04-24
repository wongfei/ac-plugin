#pragma once

BEGIN_HOOK_OBJ(SCTM)

	#define RVA_SCTM_solve 4504608
	#define RVA_SCTM_getStaticDY 4504496
	#define RVA_SCTM_getStaticDX 4504368
	#define RVA_SCTM_getPureFY 4504176

	TyreModelOutput _solve(TyreModelInput& tmi);
	float _getStaticDX(float load);
	float _getStaticDY(float load);
	float _getPureFY(float D, float cf, float load, float slip);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

TyreModelOutput _SCTM::_solve(TyreModelInput& tmi)
{
	TyreModelOutput tmo;

	if (tmi.load <= 0.0 || tmi.slipAngleRAD == 0.0 && tmi.slipRatio == 0.0 && tmi.camberRAD == 0.0)
	{
		memset(&tmo, 0, sizeof(tmo));
		return tmo;
	}

	float fAsy = this->asy;
	if (tmi.useSimpleModel)
		this->asy = 1.0;

	float fSlipAngle = tmi.slipAngleRAD;
	float fUnk1 = (sinf(tmi.camberRAD) * this->camberGain) + fSlipAngle;
	float fUnk1Tan = tanf(fUnk1);
	float fSlipAngleSin = sinf(fSlipAngle);

	float fBlister1 = tclamp(tmi.blister * 0.009999999f, 0.0f, 1.0f);
	float fBlister2 = (fBlister1 * 0.2f) + 1.0f;

	float fStaticDy = this->getStaticDY(tmi.load);
	float fStaticDx = this->getStaticDX(tmi.load);

	float fUDy = tmi.u * fStaticDy / fBlister2;
	float fUDx = tmi.u * fStaticDx / fBlister2;

	if (tmi.slipRatio < 0.0f)
		fUDx = fUDx * this->brakeDXMod;

	float fCamberRad = tmi.camberRAD;
	float fCamberRadTmp = fabsf(fCamberRad);

	if ((fCamberRad < 0.0 || fUnk1 < 0.0) && (fCamberRad > 0.0 || fUnk1 > 0.0))
		fCamberRadTmp = -fCamberRadTmp;

	fCamberRadTmp = -fCamberRadTmp;

	if (this->dCamberCurve.getCount())
	{
		float fCamberDeg = fCamberRadTmp * 57.29578f;

		if (this->useSmoothDCamberCurve)
			fUDy *= this->dCamberCurve.getCubicSplineValue(fCamberDeg);
		else
			fUDy *= this->dCamberCurve.getValue(fCamberDeg);
	}
	else
	{
		float fCamberUnk = (fCamberRadTmp * this->dcamber0) - ((fCamberRadTmp * fCamberRadTmp) * this->dcamber1);
		if (fCamberUnk <= -1.0f)
			fCamberUnk = -0.8999999f;

		fUDy += (((fUDy / (fCamberUnk + 1.0f)) - fUDy) * this->dCamberBlend);
	}

	float fSlipRatio = tmi.slipRatio;
	float fSlipAngleCos = cosf(tmi.slipAngleRAD);
	float fSlipRatioClamped = (fSlipRatio > -0.99998999f ? fSlipRatio : -0.99999f); // ???

	float fSpeed = tmi.speed;
	float a = fSpeed * fSlipAngleSin;
	float b = (fSpeed * fSlipRatio) * fSlipAngleCos;
	float fUnk2 = sqrtf((a * a) + (b * b));
	float fUnk2Scaled = fUnk2 * this->speedSensitivity;

	float fDy = fUDy / (fUnk2Scaled + 1.0f);
	float fDx = fUDx / (fUnk2Scaled + 1.0f);

	float fLoadSubFz0 = tmi.load - this->Fz0;
	float fPCfGain = this->pressureCfGain;
	float fCF = ((((1.0f / ((((fLoadSubFz0 / this->Fz0) * (this->maxSlip1 - this->maxSlip0)) + this->maxSlip0) * (((tmi.u - 1.0f) * 0.75f) + 1.0f))) * 3.0f) * 78.125f) / ((tmi.grain * 0.0099999998f) + 1.0f)) * ((fPCfGain * tmi.pressureRatio) + 1.0f);

	float fUnk3 = fSlipRatio / (fSlipRatioClamped + 1.0f);
	float fUnk4 = fUnk1Tan / (fSlipRatioClamped + 1.0f);
	float fSlip;

	float fCombFactor = this->combinedFactor;
	if (fCombFactor <= 0.0f || fCombFactor == 2.0f)
	{
		fSlip = sqrtf((fUnk4 * fUnk4) + (fUnk3 * fUnk3));
	}
	else
	{
		float fUnk34Comb = powf(fabsf(fUnk4), fCombFactor) + powf(fabsf(fUnk3), fCombFactor);
		fSlip = powf(fUnk34Comb, 1.0f / fCombFactor);
	}

	float fPureFyDx = this->getPureFY(fDx, fCF * this->cfXmult, tmi.load, fSlip) * fDx;
	float fPureFyDy = this->getPureFY(fDy, fCF, tmi.load, fSlip);

	tmo.Fy = ((fPureFyDy * fDy) * (fUnk4 / fSlip)) * tmi.load;
	tmo.Fx = ((fUnk3 / fSlip) * fPureFyDx) * tmi.load;

	float fNdSlip = fSlip / (1.0f / (((fCF * 2.0f) * 0.0063999998f) / 3.0f));
	tmo.ndSlip = fNdSlip;

	float fUnk5 = 1.0f - (fNdSlip * 0.80000001f);
	fUnk5 = tclamp(fUnk5, 0.0f, 1.0f);

	float fTrail = (((((3.0f - (fUnk5 * 2.0f)) * (fUnk5 * fUnk5)) * 1.1f) - 0.1f) * tmi.cpLength) * 0.12f;
	tmo.trail = fTrail * tclamp(tmi.speed, 0.0f, 1.0f);

	tmo.Mz = -(fTrail * tmo.Fy);

	tmo.Dy = fDy;
	tmo.Dx = fDx;

	this->asy = fAsy;

	return tmo;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _SCTM::_getStaticDX(float load)
{
	if (this->dxLoadCurve.getCount() <= 0)
	{
		if (load != 0.0)
			return (powf(load, this->lsExpX) * this->lsMultX) / load;
	}
	else
	{
		return this->dxLoadCurve.getCubicSplineValue(load);
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _SCTM::_getStaticDY(float load)
{
	if (this->dyLoadCurve.getCount() <= 0)
	{
		if (load != 0.0f)
			return (powf(load, this->lsExpY) * this->lsMultY) / load;
	}
	else
	{
		return this->dyLoadCurve.getCubicSplineValue(load);
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _SCTM::_getPureFY(float D, float cf, float load, float slip)
{
	float v5 = (cf * 2.0f) * 0.0063999998f;
	float v6 = 1.0f / (v5 / 3.0f);
	float fy;

	if (v6 < slip)
		fy = ((1.0f / (((slip - v6) * this->falloffSpeed) + 1.0f)) * (1.0f - this->asy)) + this->asy;
	else
		fy = (((1.0f - (slip / v6)) * (1.0f - (slip / v6))) * (v5 * slip)) + ((3.0f - ((slip / v6) * 2.0f)) * ((slip / v6) * (slip / v6)));

	return fy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
