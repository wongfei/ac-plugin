#pragma once

bool hook_init();
void hook_shut();
bool hook_create(void* pTarget, void* pDetour, void** ppOriginal = nullptr, const wchar_t* name = nullptr);
bool hook_enable(void* pTarget = nullptr);
void hook_disable(void* pTarget = nullptr);
