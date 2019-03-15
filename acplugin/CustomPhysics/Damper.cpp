#include "precompiled.h"
#include "GameHooks.h"

float Damper_getForce(Damper* pThis, float fSpeed)
{
	float fForce;

	if (fSpeed <= 0.0f)
	{
		float fFastThreshRebound = pThis->fastThresholdRebound;

		if (fabsf(fSpeed) <= fFastThreshRebound)
			fForce = -(fSpeed * pThis->reboundSlow);
		else
			fForce = (pThis->fastThresholdRebound * pThis->reboundSlow) - ((fFastThreshRebound + fSpeed) * pThis->reboundFast);
	}
	else
	{
		float fFastThreshBump = pThis->fastThresholdBump;

		if (fSpeed <= fFastThreshBump)
			fForce = -(fSpeed * pThis->bumpSlow);
		else
			fForce = -(((fSpeed - fFastThreshBump) * pThis->bumpFast) + (fFastThreshBump * pThis->bumpSlow));
	}

	return fForce;
}
