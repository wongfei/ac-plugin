#pragma once

BEGIN_HOOK_OBJ(SCTM)

	#define RVA_SCTM_vtable 0x1416580
	#define RVA_SCTM_ctor 4503744
	#define RVA_SCTM_solve 4504608
	#define RVA_SCTM_getStaticDY 4504496
	#define RVA_SCTM_getStaticDX 4504368
	#define RVA_SCTM_getPureFY 4504176

	static void _hook()
	{
		HOOK_METHOD_RVA(SCTM, ctor);
		HOOK_METHOD_RVA(SCTM, solve);
		HOOK_METHOD_RVA(SCTM, getStaticDY);
		HOOK_METHOD_RVA(SCTM, getStaticDX);
		HOOK_METHOD_RVA(SCTM, getPureFY);
	}

	SCTM* _ctor();
	TyreModelOutput _solve(const TyreModelInput& tmi);
	float _getStaticDX(float load);
	float _getStaticDY(float load);
	float _getPureFY(float D, float cf, float load, float slip);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

SCTM* _SCTM::_ctor()
{
	AC_CTOR_THIS_VT(SCTM);

	AC_CTOR_UDT(this->dyLoadCurve)();
	AC_CTOR_UDT(this->dxLoadCurve)();
	AC_CTOR_UDT(this->dCamberCurve)();

	this->cfXmult = 1.0f;
	this->pressureCfGain = 0.1f;
	this->brakeDXMod = 1.0f;
	this->dCamberBlend = 1.0f;
	this->combinedFactor = 2.0f;

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TyreModelOutput _SCTM::_solve(const TyreModelInput& tmi) // TODO: cleanup
{
	TyreModelOutput tmo;
	memset(&tmo, 0, sizeof(tmo));

	if (tmi.load <= 0.0f || tmi.slipAngleRAD == 0.0f && tmi.slipRatio == 0.0f && tmi.camberRAD == 0.0f)
	{
		return tmo;
	}

	float fAsy = this->asy;
	if (tmi.useSimpleModel)
		this->asy = 1.0f;

	float fSlipAngle = tmi.slipAngleRAD;
	float fUnk1 = (sinf(tmi.camberRAD) * this->camberGain) + fSlipAngle;
	float fUnk1Tan = tanf(fUnk1);
	float fSlipAngleSin = sinf(fSlipAngle);

	float fBlister1 = tclamp(tmi.blister * 0.01f, 0.0f, 1.0f);
	float fBlister2 = (fBlister1 * 0.2f) + 1.0f;

	float fStaticDy = this->getStaticDY(tmi.load);
	float fStaticDx = this->getStaticDX(tmi.load);

	float fUDy = tmi.u * fStaticDy / fBlister2;
	float fUDx = tmi.u * fStaticDx / fBlister2;

	if (tmi.slipRatio < 0.0f)
		fUDx = fUDx * this->brakeDXMod;

	float fCamberRad = tmi.camberRAD;
	float fCamberRadTmp = fabsf(fCamberRad);

	if ((fCamberRad < 0.0f || fUnk1 < 0.0f) && (fCamberRad > 0.0f || fUnk1 > 0.0f))
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
	float fSlipRatioClamped = (fSlipRatio > -0.9999999f ? fSlipRatio : -0.9999999f); // TODO: ???

	float fSpeed = tmi.speed;
	float a = fSpeed * fSlipAngleSin;
	float b = (fSpeed * fSlipRatio) * fSlipAngleCos;
	float fUnk2 = sqrtf((a * a) + (b * b));
	float fUnk2Scaled = fUnk2 * this->speedSensitivity;

	float fDy = fUDy / (fUnk2Scaled + 1.0f);
	float fDx = fUDx / (fUnk2Scaled + 1.0f);

	float fLoadSubFz0 = tmi.load - this->Fz0;
	float fPCfGain = this->pressureCfGain;

	float fCF = ((((1.0f / ((((fLoadSubFz0 / this->Fz0) * (this->maxSlip1 - this->maxSlip0)) + this->maxSlip0) * (((tmi.u - 1.0f) * 0.75f) + 1.0f))) * 3.0f) * 78.125f) / ((tmi.grain * 0.01f) + 1.0f)) * ((fPCfGain * tmi.pressureRatio) + 1.0f);

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

	float fNdSlip = fSlip / (1.0f / (((fCF * 2.0f) * 0.0064f) / 3.0f));
	tmo.ndSlip = fNdSlip;

	float fUnk5 = tclamp((1.0f - (fNdSlip * 0.8f)), 0.0f, 1.0f);
	float fUnk6 = (((((3.0f - (fUnk5 * 2.0f)) * (fUnk5 * fUnk5)) * 1.1f) - 0.1f) * tmi.cpLength) * 0.12f;

	tmo.Mz = -(fUnk6 * tmo.Fy);
	tmo.trail = fUnk6 * tclamp(tmi.speed, 0.0f, 1.0f);
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
	float v5 = (cf * 2.0f) * 0.0064f;
	float v6 = 1.0f / (v5 / 3.0f);
	float fy;

	if (v6 < slip)
		fy = ((1.0f / (((slip - v6) * this->falloffSpeed) + 1.0f)) * (1.0f - this->asy)) + this->asy;
	else
		fy = (((1.0f - (slip / v6)) * (1.0f - (slip / v6))) * (v5 * slip)) + ((3.0f - ((slip / v6) * 2.0f)) * ((slip / v6) * (slip / v6)));

	return fy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
