#pragma once

#include <windows.h>
#include <stdio.h>
#include <stdarg.h>
#include <string>

#pragma warning(disable: 4100) // warning C4100: unreferenced formal parameter
#pragma warning(disable: 4189) // warning C4189: local variable is initialized but not referenced

#define AC_API __fastcall
#define AC_EXPORT __declspec(dllexport)

#define AC_PLUGIN_NAME L"hello_plugin"
#define AC_PLUGIN_TITLE L"Hello Plugin"

#define AC_LOG_ENABLED 1
#define AC_LOG_NAME AC_PLUGIN_NAME L".log"

#define AC_WAIT_DEBUGGER MessageBoxA(NULL, "_acplugin", "_acplugin", MB_OK)

#if (AC_LOG_ENABLED)
	void log_reset();
	void log_printf(const wchar_t* format, ...);
#else
	#define log_reset()
	#define log_printf(format, ...)
#endif
