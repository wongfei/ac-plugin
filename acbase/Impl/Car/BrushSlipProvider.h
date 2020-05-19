#pragma once

BEGIN_HOOK_OBJ(BrushSlipProvider)

	#define RVA_BrushSlipProvider_vtable 0x4F8258
	#define RVA_BrushSlipProvider_ctor0 2830384
	#define RVA_BrushSlipProvider_ctor3 2830208
	#define RVA_BrushSlipProvider_getSlipForce 2830736
	#define RVA_BrushSlipProvider_calcMaximum 2830480
	#define RVA_BrushSlipProvider_recomputeMaximum 2830896

	static void _hook()
	{
		HOOK_OV_METHOD_RVA(BrushSlipProvider, ctor, ctor0);
		HOOK_OV_METHOD_RVA(BrushSlipProvider, ctor, ctor3, float, float, float);
		HOOK_METHOD_RVA(BrushSlipProvider, getSlipForce);
		HOOK_METHOD_RVA(BrushSlipProvider, calcMaximum);
		HOOK_METHOD_RVA(BrushSlipProvider, recomputeMaximum);
	}

	BrushSlipProvider* _ctor();
	BrushSlipProvider* _ctor(float maxAngle, float xu, float flex);
	TyreSlipOutput _getSlipForce(TyreSlipInput &input, bool useasy);
	void _calcMaximum(float load, float* maximum, float* max_slip);
	void _recomputeMaximum();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

BrushSlipProvider* _BrushSlipProvider::_ctor()
{
	AC_CTOR_THIS_VT(BrushSlipProvider);
	AC_CTOR_UDT(this->brushModel)();
	this->asy = 1.0f;
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

BrushSlipProvider* _BrushSlipProvider::_ctor(float maxAngle, float xu, float flex)
{
	AC_CTOR_THIS_VT(BrushSlipProvider);
	AC_CTOR_UDT(this->brushModel)();
	this->brushModel.data.CF = this->brushModel.getCFFromSlipAngle(maxAngle);
	this->brushModel.data.CF1 = flex * -50000.0f;
	this->calcMaximum(2000.0f, &this->maximum, &this->maxSlip);
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TyreSlipOutput _BrushSlipProvider::_getSlipForce(TyreSlipInput &input, bool useasy)
{
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

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrushSlipProvider::_calcMaximum(float load, float* maximum, float* max_slip)
{
	*maximum = 0.0f;
	*max_slip = 0.0f;

	float fLoad = (load >= 0.0f) ? load : 2000.0f;
	float fSlip = 0;

	do
	{
		BrushOutput bo;
		if (this->version < 5)
		{
			// (float slip, float friction, float load, float cf1_mix, float asy)
			bo = this->brushModel.solve(fSlip, 1.0f, fLoad, 1.0f, this->asy);
		}
		else
		{
			// (float slip, float load, float asy)
			bo = this->brushModel.solveV5(fSlip, fLoad, this->asy);
		}

		if (bo.force > *maximum)
		{
			*maximum = bo.force;
			*max_slip = fSlip;
		}

		fSlip += 0.001f;
	}
	while (fSlip < 1.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrushSlipProvider::_recomputeMaximum()
{
	this->calcMaximum(2000.0f, &this->maximum, &this->maxSlip);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
