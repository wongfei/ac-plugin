#pragma once

BEGIN_HOOK_OBJ(Damper)

	#define RVA_Damper_getForce 2830976

	static void _hook()
	{
		HOOK_METHOD_RVA(Damper, getForce);
	}

	float _getForce(float fSpeed);

END_HOOK_OBJ()

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
