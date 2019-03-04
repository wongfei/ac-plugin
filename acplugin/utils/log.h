#pragma once

#if (AC_LOG_ENABLED)
	void log_init(const wchar_t* filename);
	void log_printf(const wchar_t* format, ...);
	void log_release();
#else
	#define log_init(filename)
	#define log_printf(format, ...)
	#define log_release()
#endif
