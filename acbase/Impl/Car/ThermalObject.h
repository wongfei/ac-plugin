#pragma once

BEGIN_HOOK_OBJ(ThermalObject)

	#define RVA_ThermalObject_vtable 0x4FDEE0
	#define RVA_ThermalObject_ctor 2829952
	#define RVA_ThermalObject_step 2830080
	#define RVA_ThermalObject_addHeadSource 2830064

	static void _hook()
	{
		HOOK_METHOD_RVA(ThermalObject, ctor);
		HOOK_METHOD_RVA(ThermalObject, step);
		HOOK_METHOD_RVA(ThermalObject, addHeadSource);
	}

	ThermalObject* _ctor();
	void _step(float dt, float ambientTemp, const Speed &speed);
	void _addHeadSource(float heat);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

ThermalObject* _ThermalObject::_ctor()
{
	AC_CTOR_THIS_VT(ThermalObject);
	this->tmass = 1.0f;
	this->coolFactor = 0.2f;
	this->heatFactor = 1.0f;
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _ThermalObject::_step(float dt, float ambientTemp, const Speed &speed)
{
	float fCool = 1.0f - (this->coolSpeedK * speed.value);
	float fOneDivMass = 1.0f / this->tmass;

	this->t += (((((fCool * ambientTemp) - this->t) * fOneDivMass) * dt) * this->coolFactor);

	float fAccum = this->heatAccumulator;
	this->heatAccumulator = 0.0f;

	if (fAccum != 0.0f)
		this->t += ((((fAccum - this->t) * fOneDivMass) * dt) * this->heatFactor);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _ThermalObject::_addHeadSource(float heat)
{
	this->heatAccumulator += heat;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
