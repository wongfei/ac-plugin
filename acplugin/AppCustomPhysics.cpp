#include "precompiled.h"
using namespace acsdk;

#include "AppCustomPhysics.h"
#include "CustomHooks.h"

inline void CreateHookX(const wchar_t* name, LPVOID pDetour, unsigned int rva)
{
	LPVOID pTarget = _drva(rva);
	LPVOID pOriginal = nullptr;

	MH_STATUS status = MH_CreateHook(pTarget, pDetour, &pOriginal);
	if (status != MH_OK)
	{
		log_printf(L"MH_CreateHook failed: status=%d name=%s", (int)status, name);
	}
}

AppCustomPhysics::AppCustomPhysics(ACPlugin* plugin) : PluginApp(plugin, L"custom_physics")
{
	log_printf(L"+AppCustomPhysics %p", this);

	MH_STATUS status = MH_Initialize();
	if (status != MH_OK)
	{
		log_printf(L"MH_Initialize failed: status=%d", (int)status);
		return;
	}

	CreateHookX(L"Engine_step", &Engine_step, Engine_step_rva);
	CreateHookX(L"Engine_stepTurbos", &Engine_stepTurbos, Engine_stepTurbos_rva);
	CreateHookX(L"Turbo_step", &Turbo_step, Turbo_step_rva);

	status = MH_EnableHook(MH_ALL_HOOKS);
	if (status != MH_OK)
	{
		log_printf(L"MH_EnableHook failed: status=%d", (int)status);
		return;
	}
}

AppCustomPhysics::~AppCustomPhysics()
{
	log_printf(L"~AppCustomPhysics %p", this);

	MH_Uninitialize();
}
