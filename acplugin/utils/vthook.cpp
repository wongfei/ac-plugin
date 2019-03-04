#include "precompiled.h"
#include "vthook.h"
#include "log.h"

void* vthook_set(const wchar_t* className, size_t vfid, void* pInstance, void* pHook)
{
	log_printf(L"vthook_set className=%s vfid=%u pInstance=%p pHook=%p", className, (unsigned int)vfid, pInstance, pHook);

	intptr_t pVTable = *((intptr_t*)pInstance);
	intptr_t pEntry = pVTable + sizeof(intptr_t) * vfid;

	MEMORY_BASIC_INFORMATION mbi;
	ZeroMemory(&mbi, sizeof(mbi));

	if (!VirtualQuery((LPCVOID)pEntry, &mbi, sizeof(mbi)))
	{
		log_printf(L"VirtualQuery failed: err=0x%X", (unsigned int)GetLastError());
		return nullptr;
	}

	DWORD OrigProtect = 0;
	if (!VirtualProtect(mbi.BaseAddress, mbi.RegionSize, PAGE_READWRITE, &OrigProtect))
	{
		log_printf(L"VirtualProtect failed: err=0x%X", (unsigned int)GetLastError());
		return nullptr;
	}

	intptr_t pOriginal = *((intptr_t*)pEntry);
	*((intptr_t*)pEntry) = (intptr_t)pHook;

	VirtualProtect(mbi.BaseAddress, mbi.RegionSize, OrigProtect, &mbi.Protect);

	return (void*)pOriginal;
}
