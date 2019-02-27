#include "precompiled.h"
#include "AppCustomPhysics.h"
#include "CustomHooks.h"

#define X_CAT(A, B) A##B
#define X_WSTRING(A) X_CAT(L, #A)
#define X_HOOK(name) CreateHookX(X_WSTRING(name), &name, RVA_##name)

inline MH_STATUS CreateHookX(const wchar_t* name, LPVOID pDetour, unsigned int rva)
{
	LPVOID pTarget = _drva(rva);
	LPVOID pOriginal = nullptr;

	MH_STATUS status = MH_CreateHook(pTarget, pDetour, &pOriginal);
	if (status != MH_OK)
	{
		log_printf(L"MH_CreateHook failed: status=%d name=%s", (int)status, name);
	}

	return status;
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

	X_HOOK(Engine_step);
	X_HOOK(Engine_stepTurbos);
	X_HOOK(Turbo_step);

	X_HOOK(Drivetrain_step);
	X_HOOK(Drivetrain_stepControllers);
	X_HOOK(Drivetrain_step2WD);

	//X_HOOK(Suspension_step);

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
