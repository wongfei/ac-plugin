#pragma once

#define RVA_BrushTyreModel_solve 2929600
#define RVA_BrushTyreModel_solveV5 2929888
#define RVA_BrushTyreModel_getCFFromSlipAngle 2929536
#define RVA_BrushSlipProvider_getSlipForce 2830736

BrushOutput BrushTyreModel_solve(BrushTyreModel* pThis, float slip, float friction, float load, float cf1_mix, float asy)
{
	BrushOutput result;

	float fCF = pThis->data.CF;
	float v8 = fCF * 0.3333333f;
	float v9 = ((((load * 0.0005f) - 1.0f) * pThis->data.CF1) * cf1_mix) + fCF;

	if (v8 <= v9)
		v8 = v9;

	float v10 = ((v8 * 2.0f) * 0.08f) * 0.08f;
	float v11 = v10 / (friction * 3.0f);

	float fNewSlip = slip / (1.0f / v11);
	result.slip = fNewSlip;

	if (slip > (1.0f / v11))
		result.force = ((1.0f - asy) / (((slip - (1.0f / v11)) * 2.0f) + 1.0f)) + asy;
	else
		result.force = ((((1.0f - fNewSlip) * (1.0f - fNewSlip)) * (v10 * slip)) * ((friction * pThis->data.xu) + 1.0f)) + ((3.0f - (fNewSlip * 2.0f)) * (fNewSlip * fNewSlip));

	return result;
}

BrushOutput BrushTyreModel_solveV5(BrushTyreModel* pThis, float slip, float load, float asy)
{
	BrushOutput result;

	float v8 = ((((1.0f / ((((load - pThis->data.Fz0) / pThis->data.Fz0) * (pThis->data.maxSlip1 - pThis->data.maxSlip0)) + pThis->data.maxSlip0)) * 3.0f) * 78.125f) * 2.0f) * 0.0064f;
	float v9 = 1.0f / (v8 * 0.3333333f);

	float fNewSlip = slip / v9;
	result.slip = fNewSlip;

	if (slip > v9)
		result.force = ((1.0f - asy) / (((slip - v9) * pThis->data.falloffSpeed) + 1.0f)) + asy;
	else
		result.force = (((1.0f - fNewSlip) * (1.0f - fNewSlip)) * (v8 * slip)) + ((3.0f - (fNewSlip * 2.0f)) * (fNewSlip * fNewSlip));

	return result;
}

float BrushTyreModel_getCFFromSlipAngle(BrushTyreModel* pThis, float angle)
{
	return ((1.0f / tanf(angle * 0.017453f)) * 3.0f) * 78.125f;
}

TyreSlipOutput BrushSlipProvider_getSlipForce(BrushSlipProvider* pThis, TyreSlipInput &input, bool useasy)
{
	TyreSlipOutput tso;
	BrushOutput bo;

	if (pThis->version < 5)
	{
		DEBUG_BREAK; // TODO: what car uses this code?

		// (float slip, float friction, float load, float cf1_mix, float asy)
		bo = pThis->brushModel.solve(input.slip, 1.0f, input.load, 1.0f, 1.0f); // TODO: not sure
	}
	else
	{
		// (float slip, float load, float asy)
		bo = pThis->brushModel.solveV5(input.slip, input.load, (useasy ? pThis->asy : 1.0f));
	}

	tso.normalizedForce = bo.force;
	tso.slip = bo.slip;
	return tso;
}
