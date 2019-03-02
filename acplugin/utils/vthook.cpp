#include "precompiled.h"
#include "plugin/plugin.h"
#include "vthook.h"

void* vtablehook_hook(void* instance, void* hook, size_t offset)
{
	intptr_t vtable = *((intptr_t*)instance);
	intptr_t entry = vtable + sizeof(intptr_t) * offset;

	MEMORY_BASIC_INFORMATION mbi;
	::ZeroMemory(&mbi, sizeof(mbi));

	SIZE_T size = ::VirtualQuery((LPCVOID)entry, &mbi, sizeof(mbi));
	if (!size) {
		log_printf(L"VirtualQuery failed: err=0x%X entry=%p", (unsigned int)GetLastError(), (void*)entry);
		return nullptr;
	}

	DWORD dwOrigProtect = 0;
	if (FALSE == ::VirtualProtect(mbi.BaseAddress, mbi.RegionSize, PAGE_READWRITE, &dwOrigProtect)) {
		log_printf(L"VirtualProtect failed: err=0x%X entry=%p", (unsigned int)GetLastError(), (void*)entry);
		return nullptr;
	}

	intptr_t original = *((intptr_t*)entry);
	*((intptr_t*)entry) = (intptr_t)hook;

	::VirtualProtect(mbi.BaseAddress, mbi.RegionSize, dwOrigProtect, &mbi.Protect);

	return (void*)original;
}
