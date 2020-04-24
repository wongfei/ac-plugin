#pragma once

BEGIN_HOOK_OBJ(ThermalObject)

	#define RVA_ThermalObject_step 2830080

	void _step(float dt, float ambientTemp, const Speed &speed);

END_HOOK_OBJ()

void _ThermalObject::_step(float dt, float ambientTemp, const Speed &speed)
{
	float fAccum = this->heatAccumulator;
	float fOneDivMass = 1.0f / this->tmass;
	float fCool = 1.0f - this->coolSpeedK * speed.value;
	this->heatAccumulator = 0.0f;

	float fTemp = (((((fCool * ambientTemp) - this->t) * fOneDivMass) * dt) * this->coolFactor) + this->t;
	this->t = fTemp;

	if (fAccum != 0.0f)
		this->t = ((((fAccum - fTemp) * fOneDivMass) * dt) * this->heatFactor) + fTemp;
}
