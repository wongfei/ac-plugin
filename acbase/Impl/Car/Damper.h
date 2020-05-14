#pragma once

BEGIN_HOOK_OBJ(Damper)

	#define RVA_Damper_ctor 2830928
	#define RVA_Damper_getForce 2830976

	static void _hook()
	{
		HOOK_METHOD_RVA(Damper, ctor);
		HOOK_METHOD_RVA(Damper, getForce);
	}

	Damper* _ctor();
	float _getForce(float fSpeed);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Damper* _Damper::_ctor()
{
	AC_CTOR_POD(Damper);

	this->reboundSlow = 5000.0f;
	this->reboundFast = 300.0f;
	this->bumpSlow = 2000.0f;
	this->bumpFast = 300.0f;
	this->fastThresholdBump = 0.2f;
	this->fastThresholdRebound = 0.2f;

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _Damper::_getForce(float fSpeed)
{
	float fForce;
	if (fSpeed <= 0.0f)
	{
		if (fabsf(fSpeed) <= this->fastThresholdRebound)
			fForce = -(fSpeed * this->reboundSlow);
		else
			fForce = (this->fastThresholdRebound * this->reboundSlow) - ((this->fastThresholdRebound + fSpeed) * this->reboundFast);
	}
	else
	{
		if (fSpeed <= this->fastThresholdBump)
			fForce = -(fSpeed * this->bumpSlow);
		else
			fForce = -(((fSpeed - this->fastThresholdBump) * this->bumpFast) + (this->fastThresholdBump * this->bumpSlow));
	}
	return fForce;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
