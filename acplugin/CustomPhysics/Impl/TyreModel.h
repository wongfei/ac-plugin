#pragma once

#define RVA_SCTM_solve 4504608
#define RVA_SCTM_getStaticDY 4504496
#define RVA_SCTM_getStaticDX 4504368
#define RVA_SCTM_getPureFY 4504176

///////////////////////////////////////////////////////////////////////////////////////////////////

TyreModelOutput SCTM_solve(SCTM* pThis, TyreModelInput& tmi)
{
	TyreModelOutput tmo;

	if (tmi.load <= 0.0 || tmi.slipAngleRAD == 0.0 && tmi.slipRatio == 0.0 && tmi.camberRAD == 0.0)
	{
		memset(&tmo, 0, sizeof(tmo));
		return tmo;
	}

	float fAsy = pThis->asy;
	if (tmi.useSimpleModel)
		pThis->asy = 1.0;

	float fSlipAngle = tmi.slipAngleRAD;
	float fUnk1 = (sinf(tmi.camberRAD) * pThis->camberGain) + fSlipAngle;
	float fUnk1Tan = tanf(fUnk1);
	float fSlipAngleSin = sinf(fSlipAngle);

	float fBlister1 = tclamp(tmi.blister * 0.009999999f, 0.0f, 1.0f);
	float fBlister2 = (fBlister1 * 0.2f) + 1.0f;

	float fStaticDy = pThis->getStaticDY(tmi.load);
	float fStaticDx = pThis->getStaticDX(tmi.load);

	float fUDy = tmi.u * fStaticDy / fBlister2;
	float fUDx = tmi.u * fStaticDx / fBlister2;

	if (tmi.slipRatio < 0.0f)
		fUDx = fUDx * pThis->brakeDXMod;

	float fCamberRad = tmi.camberRAD;
	float fCamberRadTmp = fabsf(fCamberRad);

	if ((fCamberRad < 0.0 || fUnk1 < 0.0) && (fCamberRad > 0.0 || fUnk1 > 0.0))
		fCamberRadTmp = -fCamberRadTmp;

	fCamberRadTmp = -fCamberRadTmp;

	if (pThis->dCamberCurve.getCount())
	{
		float fCamberDeg = fCamberRadTmp * 57.29578f;

		if (pThis->useSmoothDCamberCurve)
			fUDy *= pThis->dCamberCurve.getCubicSplineValue(fCamberDeg);
		else
			fUDy *= pThis->dCamberCurve.getValue(fCamberDeg);
	}
	else
	{
		float fCamberUnk = (fCamberRadTmp * pThis->dcamber0) - ((fCamberRadTmp * fCamberRadTmp) * pThis->dcamber1);
		if (fCamberUnk <= -1.0f)
			fCamberUnk = -0.8999999f;

		fUDy += (((fUDy / (fCamberUnk + 1.0f)) - fUDy) * pThis->dCamberBlend);
	}

	float fSlipRatio = tmi.slipRatio;
	float fSlipAngleCos = cosf(tmi.slipAngleRAD);
	float fSlipRatioClamped = (fSlipRatio > -0.99998999f ? fSlipRatio : -0.99999f); // ???

	float fSpeed = tmi.speed;
	float a = fSpeed * fSlipAngleSin;
	float b = (fSpeed * fSlipRatio) * fSlipAngleCos;
	float fUnk2 = sqrtf((a * a) + (b * b));
	float fUnk2Scaled = fUnk2 * pThis->speedSensitivity;

	float fDy = fUDy / (fUnk2Scaled + 1.0f);
	float fDx = fUDx / (fUnk2Scaled + 1.0f);

	float fLoadSubFz0 = tmi.load - pThis->Fz0;
	float fPCfGain = pThis->pressureCfGain;
	float fCF = ((((1.0f / ((((fLoadSubFz0 / pThis->Fz0) * (pThis->maxSlip1 - pThis->maxSlip0)) + pThis->maxSlip0) * (((tmi.u - 1.0f) * 0.75f) + 1.0f))) * 3.0f) * 78.125f) / ((tmi.grain * 0.0099999998f) + 1.0f)) * ((fPCfGain * tmi.pressureRatio) + 1.0f);

	float fUnk3 = fSlipRatio / (fSlipRatioClamped + 1.0f);
	float fUnk4 = fUnk1Tan / (fSlipRatioClamped + 1.0f);
	float fSlip;

	float fCombFactor = pThis->combinedFactor;
	if (fCombFactor <= 0.0f || fCombFactor == 2.0f)
	{
		fSlip = sqrtf((fUnk4 * fUnk4) + (fUnk3 * fUnk3));
	}
	else
	{
		float fUnk34Comb = powf(fabsf(fUnk4), fCombFactor) + powf(fabsf(fUnk3), fCombFactor);
		fSlip = powf(fUnk34Comb, 1.0f / fCombFactor);
	}

	float fPureFyDx = pThis->getPureFY(fDx, fCF * pThis->cfXmult, tmi.load, fSlip) * fDx;
	float fPureFyDy = pThis->getPureFY(fDy, fCF, tmi.load, fSlip);

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

	pThis->asy = fAsy;

	return tmo;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float SCTM_getStaticDX(SCTM* pThis, float load)
{
	if (pThis->dxLoadCurve.getCount() <= 0)
	{
		if (load != 0.0)
			return (powf(load, pThis->lsExpX) * pThis->lsMultX) / load;
	}
	else
	{
		return pThis->dxLoadCurve.getCubicSplineValue(load);
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float SCTM_getStaticDY(SCTM* pThis, float load)
{
	if (pThis->dyLoadCurve.getCount() <= 0)
	{
		if (load != 0.0f)
			return (powf(load, pThis->lsExpY) * pThis->lsMultY) / load;
	}
	else
	{
		return pThis->dyLoadCurve.getCubicSplineValue(load);
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float SCTM_getPureFY(SCTM* pThis, float D, float cf, float load, float slip)
{
	float v5 = (cf * 2.0f) * 0.0063999998f;
	float v6 = 1.0f / (v5 / 3.0f);
	float fy;

	if (v6 < slip)
		fy = ((1.0f / (((slip - v6) * pThis->falloffSpeed) + 1.0f)) * (1.0f - pThis->asy)) + pThis->asy;
	else
		fy = (((1.0f - (slip / v6)) * (1.0f - (slip / v6))) * (v5 * slip)) + ((3.0f - ((slip / v6) * 2.0f)) * ((slip / v6) * (slip / v6)));

	return fy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
