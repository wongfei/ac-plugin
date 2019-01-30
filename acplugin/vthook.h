#pragma once

int vtablehook_unprotect(void* region);
void vtablehook_protect(void* region, int protection);
void* vtablehook_hook(void* instance, void* hook, int offset);
