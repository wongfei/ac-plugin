#pragma once

#define RVA_ThermalObject_step 2830080

void ThermalObject_step(ThermalObject* pThis, float dt, float ambientTemp, const Speed &speed)
{
	float fAccum = pThis->heatAccumulator;
	float fOneDivMass = 1.0f / pThis->tmass;
	float fCool = 1.0f - pThis->coolSpeedK * speed.value;
	pThis->heatAccumulator = 0.0f;

	float fTemp = (((((fCool * ambientTemp) - pThis->t) * fOneDivMass) * dt) * pThis->coolFactor) + pThis->t;
	pThis->t = fTemp;

	if (fAccum != 0.0f)
		pThis->t = ((((fAccum - fTemp) * fOneDivMass) * dt) * pThis->heatFactor) + fTemp;
}
