#include "precompiled.h"
#include "GameHooks.h"

void Turbo_step(Turbo* pThis, float gas, float rpms, float dt)
{
	float fGamma = 0.0f;

	if (rpms > 0.0f && gas > 0.0f)
	{
		float fRef = (float)(gas * rpms) / pThis->data.rpmRef;
		fRef = tclamp(fRef, 0.0f, 1.0f);
		fGamma = powf(fRef, pThis->data.gamma);
	}

	float fLag = 0.0f;
	float fRotation = pThis->rotation;

	if (fGamma <= fRotation)
	{
		fLag = dt * pThis->data.lagDN;
		fLag = tclamp(fLag, 0.0f, 1.0f);
	}
	else
	{
		fLag = dt * pThis->data.lagUP;
		fLag = tclamp(fLag, 0.0f, 1.0f);
	}

	float fFinalRotation = (float)((float)(fGamma - fRotation) * fLag) + fRotation;
	pThis->rotation = fFinalRotation;

	float fWastegate = pThis->data.wastegate;

	if (fWastegate != 0.0f)
	{
		float fUserWG = fWastegate * pThis->userSetting;
		if ((float)(pThis->data.maxBoost * pThis->rotation) > fUserWG)
		{
			pThis->rotation = fUserWG / pThis->data.maxBoost;
		}
	}
}
