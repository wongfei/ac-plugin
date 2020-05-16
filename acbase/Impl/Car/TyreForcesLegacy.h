#pragma once

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepRelaxationLength(float svx, float svy, float hubVelocity, float dt)
{
	float fRelaxLen = this->modelData.relaxationLength;
	float fNdSlip = tclamp(this->status.ndSlip, 0.0f, 1.0f);

	float v8 = ((((this->status.load / this->modelData.Fz0) * fRelaxLen) - fRelaxLen) * 0.3f) + fRelaxLen;
	float v9 = ((fRelaxLen - (v8 * 2.0f)) * fNdSlip) + (v8 * 2.0f);

	if (v9 == 0.0f)
	{
		this->rSlidingVelocityX = 0;
		this->rSlidingVelocityY = 0;
	}
	else
	{
		float v10 = (hubVelocity * dt) / v9;
		if (v10 < 1.0f)
		{
			if (v10 < 0.04f)
				v10 = 0.04f;

			this->rSlidingVelocityY += (v10 * (svy - this->rSlidingVelocityY));
			this->rSlidingVelocityX += (v10 * (svx - this->rSlidingVelocityX));
		}
		else
		{
			this->rSlidingVelocityX = svx;
			this->rSlidingVelocityY = svy;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
