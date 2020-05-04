#pragma once

BEGIN_HOOK_OBJ(GearChanger)

	#define RVA_GearChanger_step 2861904

	static void _hook()
	{
		HOOK_METHOD_RVA(GearChanger, step);
	}

	void _step(float dt);

END_HOOK_OBJ()

void _GearChanger::_step(float dt)
{
	this->wasGearUpTriggered = false;

	int gearId = this->car->controls.requestedGearIndex;
	if (gearId == -1)
	{
		if (this->car->controls.gearUp && !this->lastGearUp)
			this->wasGearUpTriggered = this->car->drivetrain.gearUp();

		if (this->car->controls.gearDn && !this->lastGearDn)
			this->wasGearDnTriggered = this->car->drivetrain.gearDown();

		this->lastGearUp = this->car->controls.gearUp;
		this->lastGearDn = this->car->controls.gearDn;
	}
	else
	{
		this->car->drivetrain.setCurrentGear(gearId, false);
	}
}
