#include "precompiled.h"
#include "plugin.h"

#if (AC_LOG_ENABLED)

void log_reset()
{
	FILE* fd = NULL;
	_wfopen_s(&fd, AC_LOG_NAME, L"wt");
	if (fd)
	{
		fclose(fd);
	}
}

void log_printf(const wchar_t* format, ...)
{
	const size_t BufLen = 1024;
	wchar_t buf[BufLen];

	va_list args;
	va_start(args, format);
	const int count = _vsnwprintf_s(buf, BufLen - 1, _TRUNCATE, format, args);
	va_end(args);
	buf[count] = 0;

	FILE* fd = NULL;
	_wfopen_s(&fd, AC_LOG_NAME, L"at");
	if (fd)
	{
		fwprintf(fd, L"%s\n", buf);
		fclose(fd);
	}
}

#endif // AC_LOG_ENABLED
