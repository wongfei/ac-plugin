#pragma once

BEGIN_HOOK_OBJ(Turbo)

	#define RVA_Turbo_step 2811840

	static void _hook()
	{
		HOOK_METHOD_RVA(Turbo, step);
	}

	void _step(float gas, float rpms, float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Turbo::_step(float gas, float rpms, float dt)
{
	float fNewRotation = 0.0f;
	float fLag = 0.0f;

	if (rpms > 0.0f && gas > 0.0f)
	{
		fNewRotation = powf(tclamp(((gas * rpms) / this->data.rpmRef), 0.0f, 1.0f), this->data.gamma);
	}

	if (fNewRotation <= this->rotation)
	{
		fLag = tclamp((dt * this->data.lagDN), 0.0f, 1.0f);
	}
	else
	{
		fLag = tclamp((dt * this->data.lagUP), 0.0f, 1.0f);
	}

	this->rotation += ((fNewRotation - this->rotation) * fLag);

	if (this->data.wastegate != 0.0f)
	{
		float fUserWG = this->data.wastegate * this->userSetting;
		if ((this->data.maxBoost * this->rotation) > fUserWG)
		{
			this->rotation = fUserWG / this->data.maxBoost;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
