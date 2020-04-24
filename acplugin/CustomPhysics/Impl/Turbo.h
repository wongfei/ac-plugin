#pragma once

BEGIN_HOOK_OBJ(Turbo)

	#define RVA_Turbo_step 2811840

	void _step(float gas, float rpms, float dt);

END_HOOK_OBJ()

void _Turbo::_step(float gas, float rpms, float dt)
{
	float fGamma = 0.0f;

	if (rpms > 0.0f && gas > 0.0f)
	{
		float fRef = (gas * rpms) / this->data.rpmRef;
		fRef = tclamp(fRef, 0.0f, 1.0f);
		fGamma = powf(fRef, this->data.gamma);
	}

	float fLag = 0.0f;
	float fRotation = this->rotation;

	if (fGamma <= fRotation)
	{
		fLag = dt * this->data.lagDN;
		fLag = tclamp(fLag, 0.0f, 1.0f);
	}
	else
	{
		fLag = dt * this->data.lagUP;
		fLag = tclamp(fLag, 0.0f, 1.0f);
	}

	float fFinalRotation = ((fGamma - fRotation) * fLag) + fRotation;
	this->rotation = fFinalRotation;

	float fWastegate = this->data.wastegate;
	if (fWastegate != 0.0f)
	{
		float fUserWG = fWastegate * this->userSetting;
		if ((this->data.maxBoost * this->rotation) > fUserWG)
		{
			this->rotation = fUserWG / this->data.maxBoost;
		}
	}
}
