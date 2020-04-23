#pragma once

#define RVA_Tyre_stepRelaxationLength 2640448

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepRelaxationLength(Tyre* pThis, float svx, float svy, float hubVelocity, float dt)
{
	float fRelaxLen = pThis->modelData.relaxationLength;
	float fNdSlip = tclamp(pThis->status.ndSlip, 0.0f, 1.0f);

	float v8 = ((((pThis->status.load / pThis->modelData.Fz0) * fRelaxLen) - fRelaxLen) * 0.3f) + fRelaxLen;
	float v9 = ((fRelaxLen - (v8 * 2.0f)) * fNdSlip) + (v8 * 2.0f);

	if (v9 == 0.0f)
	{
		pThis->rSlidingVelocityX = 0;
		pThis->rSlidingVelocityY = 0;
	}
	else
	{
		float v10 = (hubVelocity * dt) / v9;
		if (v10 < 1.0f)
		{
			if (v10 < 0.04f)
				v10 = 0.04f;

			pThis->rSlidingVelocityY = (v10 * (svy - pThis->rSlidingVelocityY)) + pThis->rSlidingVelocityY;
			pThis->rSlidingVelocityX = (v10 * (svx - pThis->rSlidingVelocityX)) + pThis->rSlidingVelocityX;
		}
		else
		{
			pThis->rSlidingVelocityX = svx;
			pThis->rSlidingVelocityY = svy;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
