#pragma once

BEGIN_HOOK_OBJ(GearChanger)

	#define RVA_GearChanger_init 2861888
	#define RVA_GearChanger_step 2861904

	static void _hook()
	{
		HOOK_METHOD_RVA(GearChanger, init);
		HOOK_METHOD_RVA(GearChanger, step);
	}

	// ctor optimized
	void _init(Car* car);
	void _step(float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _GearChanger::_init(Car* car)
{
	this->car = car;
	this->lastGearUp = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////
