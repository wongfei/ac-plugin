#pragma once

BEGIN_HOOK_OBJ(BrushSlipProvider)

	#define RVA_BrushSlipProvider_getSlipForce 2830736

	static void _hook()
	{
		HOOK_METHOD_RVA(BrushSlipProvider, getSlipForce);
	}

	TyreSlipOutput _getSlipForce(TyreSlipInput &input, bool useasy);

END_HOOK_OBJ()

TyreSlipOutput _BrushSlipProvider::_getSlipForce(TyreSlipInput &input, bool useasy)
{
	TODO_WTF_IS_THIS;

	TyreSlipOutput tso;
	BrushOutput bo;

	if (this->version < 5)
	{
		// (float slip, float friction, float load, float cf1_mix, float asy)
		bo = this->brushModel.solve(input.slip, 1.0f, input.load, 1.0f, 1.0f);
	}
	else
	{
		// (float slip, float load, float asy)
		bo = this->brushModel.solveV5(input.slip, input.load, (useasy ? this->asy : 1.0f));
	}

	tso.normalizedForce = bo.force;
	tso.slip = bo.slip;
	return tso;
}
