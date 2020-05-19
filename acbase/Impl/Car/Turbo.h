#pragma once

BEGIN_HOOK_OBJ(Turbo)

	#define RVA_Turbo_ctor 2811696
	#define RVA_Turbo_step 2811840
	#define RVA_Turbo_setTurboBoostLevel 2811808

	static void _hook()
	{
		HOOK_METHOD_RVA(Turbo, ctor);
		HOOK_METHOD_RVA(Turbo, step);
		HOOK_METHOD_RVA(Turbo, setTurboBoostLevel);
	}

	Turbo* _ctor(TurboDef* data);
	void _step(float gas, float rpms, float dt);
	void _setTurboBoostLevel(float value);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Turbo* _Turbo::_ctor(TurboDef* data)
{
	AC_CTOR_THIS_POD(Turbo);

	this->userSetting = 1.0f;
	this->data = *data;

	return this;
}

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

void _Turbo::_setTurboBoostLevel(float value)
{
	if (this->data.isAdjustable)
		this->userSetting = value;
	else
		this->userSetting = 1.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
