//
// https://github.com/Thordin/vtable-hook
//

#include "precompiled.h"
#include "vthook.h"

int vtablehook_unprotect(void* region)
{
	MEMORY_BASIC_INFORMATION mbi;
	VirtualQuery((LPCVOID)region, &mbi, sizeof(mbi));
	VirtualProtect(mbi.BaseAddress, mbi.RegionSize, PAGE_READWRITE, &mbi.Protect);
	return mbi.Protect;
}

void vtablehook_protect(void* region, int protection)
{
	MEMORY_BASIC_INFORMATION mbi;
	VirtualQuery((LPCVOID)region, &mbi, sizeof(mbi));
	VirtualProtect(mbi.BaseAddress, mbi.RegionSize, protection, &mbi.Protect);
}

void* vtablehook_hook(void* instance, void* hook, int offset)
{
	intptr_t vtable = *((intptr_t*)instance);
	intptr_t entry = vtable + sizeof(intptr_t) * offset;
	intptr_t original = *((intptr_t*) entry);

	int original_protection = vtablehook_unprotect((void*)entry);
	*((intptr_t*)entry) = (intptr_t)hook;
	vtablehook_protect((void*)entry, original_protection);

	return (void*)original;
}
