#pragma once

#define RVA_Damper_getForce 2830976

float Damper_getForce(Damper* pThis, float fSpeed)
{
	float fForce;
	if (fSpeed <= 0.0f)
	{
		if (fabsf(fSpeed) <= pThis->fastThresholdRebound)
			fForce = -(fSpeed * pThis->reboundSlow);
		else
			fForce = (pThis->fastThresholdRebound * pThis->reboundSlow) - ((pThis->fastThresholdRebound + fSpeed) * pThis->reboundFast);
	}
	else
	{
		if (fSpeed <= pThis->fastThresholdBump)
			fForce = -(fSpeed * pThis->bumpSlow);
		else
			fForce = -(((fSpeed - pThis->fastThresholdBump) * pThis->bumpFast) + (pThis->fastThresholdBump * pThis->bumpSlow));
	}
	return fForce;
}
