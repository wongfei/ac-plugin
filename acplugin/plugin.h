#pragma once

#define AC_API __fastcall
#define AC_EXPORT __declspec(dllexport)

#define AC_PLUGIN_NAME L"hello_plugin"

#define AC_LOG_ENABLED 1
#define AC_LOG_NAME AC_PLUGIN_NAME L".log"

#if (AC_LOG_ENABLED)
	void log_reset();
	void log_printf(const wchar_t* format, ...);
#else
	#define log_reset()
	#define log_printf(format, ...)
#endif

extern void* _g_module;
