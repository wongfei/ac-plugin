#pragma once

BEGIN_HOOK_OBJ(BrushTyreModel)

	#define RVA_BrushTyreModel_solve 2929600
	#define RVA_BrushTyreModel_solveV5 2929888
	#define RVA_BrushTyreModel_getCFFromSlipAngle 2929536

	static void _hook()
	{
		HOOK_METHOD_RVA(BrushTyreModel, solve);
		HOOK_METHOD_RVA(BrushTyreModel, solveV5);
		HOOK_METHOD_RVA(BrushTyreModel, getCFFromSlipAngle);
	}

	BrushOutput _solve(float slip, float friction, float load, float cf1_mix, float asy);
	BrushOutput _solveV5(float slip, float load, float asy);
	float _getCFFromSlipAngle(float angle);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

BrushOutput _BrushTyreModel::_solve(float slip, float friction, float load, float cf1_mix, float asy)
{
	BrushOutput result;

	float fCF = this->data.CF;
	float v8 = fCF * 0.3333333f;
	float v9 = ((((load * 0.0005f) - 1.0f) * this->data.CF1) * cf1_mix) + fCF;

	if (v8 <= v9)
		v8 = v9;

	float v10 = ((v8 * 2.0f) * 0.08f) * 0.08f;
	float v11 = v10 / (friction * 3.0f);

	float fNewSlip = slip / (1.0f / v11);
	result.slip = fNewSlip;

	if (slip > (1.0f / v11))
		result.force = ((1.0f - asy) / (((slip - (1.0f / v11)) * 2.0f) + 1.0f)) + asy;
	else
		result.force = ((((1.0f - fNewSlip) * (1.0f - fNewSlip)) * (v10 * slip)) * ((friction * this->data.xu) + 1.0f)) + ((3.0f - (fNewSlip * 2.0f)) * (fNewSlip * fNewSlip));

	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

BrushOutput _BrushTyreModel::_solveV5(float slip, float load, float asy)
{
	BrushOutput result;

	float v8 = ((((1.0f / ((((load - this->data.Fz0) / this->data.Fz0) * (this->data.maxSlip1 - this->data.maxSlip0)) + this->data.maxSlip0)) * 3.0f) * 78.125f) * 2.0f) * 0.0064f;
	float v9 = 1.0f / (v8 * 0.3333333f);

	float fNewSlip = slip / v9;
	result.slip = fNewSlip;

	if (slip > v9)
		result.force = ((1.0f - asy) / (((slip - v9) * this->data.falloffSpeed) + 1.0f)) + asy;
	else
		result.force = (((1.0f - fNewSlip) * (1.0f - fNewSlip)) * (v8 * slip)) + ((3.0f - (fNewSlip * 2.0f)) * (fNewSlip * fNewSlip));

	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _BrushTyreModel::_getCFFromSlipAngle(float angle)
{
	return ((1.0f / tanf(angle * 0.017453f)) * 3.0f) * 78.125f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
