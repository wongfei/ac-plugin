#include "precompiled.h"
#include "AppCustomPhysics.h"
#include "GameHooks.h"

AppCustomPhysics::AppCustomPhysics(ACPlugin* plugin) : PluginApp(plugin, L"custom_physics")
{
	log_printf(L"+AppCustomPhysics %p", this);

	HOOK_FUNC_RVA(Engine_step);
	HOOK_FUNC_RVA(Engine_stepTurbos);
	HOOK_FUNC_RVA(Turbo_step);

	HOOK_FUNC_RVA(Drivetrain_step);
	HOOK_FUNC_RVA(Drivetrain_stepControllers);
	HOOK_FUNC_RVA(Drivetrain_step2WD);

	HOOK_FUNC_RVA(Suspension_step);
	HOOK_FUNC_RVA(Damper_getForce);

	HOOK_FUNC_RVA(Tyre_step);
	HOOK_FUNC_RVA(Tyre_addGroundContact);
}

AppCustomPhysics::~AppCustomPhysics()
{
	log_printf(L"~AppCustomPhysics %p", this);
}
