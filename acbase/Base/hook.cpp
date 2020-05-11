#include "precompiled.h"

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

bool hook_create(const wchar_t* name, void* pTarget, void* pDetour, void** ppOriginal)
{
	log_printf(L"hook_create name=%s pTarget=%p pDetour=%p", name, pTarget, pDetour);

	MH_STATUS status = MH_CreateHook(pTarget, pDetour, ppOriginal);
	if (status != MH_OK)
	{
		log_printf(L"MH_CreateHook failed: status=%d", (int)status);
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
