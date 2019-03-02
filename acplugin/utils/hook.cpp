#include "precompiled.h"
#include "plugin/plugin.h"
#include "hook.h"

bool hook_init()
{
	MH_STATUS status = MH_Initialize();
	if (status != MH_OK)
	{
		log_printf(L"MH_Initialize failed: status=%d", (int)status);
		return false;
	}
	return true;
}

void hook_shut()
{
	MH_Uninitialize();
}

bool hook_create(void* pTarget, void* pDetour, void** ppOriginal, const wchar_t* name)
{
	MH_STATUS status = MH_CreateHook(pTarget, pDetour, ppOriginal);
	if (status != MH_OK)
	{
		log_printf(L"MH_CreateHook failed: status=%d target=%p detour=%p name=%s", (int)status, pTarget, pDetour, name);
		return false;
	}
	return true;
}

bool hook_enable(void* pTarget)
{
	MH_STATUS status = MH_EnableHook(pTarget);
	if (status != MH_OK)
	{
		log_printf(L"MH_EnableHook failed: status=%d target=%p", (int)status, pTarget);
		return false;
	}
	return true;
}

void hook_disable(void* pTarget)
{
	MH_DisableHook(pTarget);
}
