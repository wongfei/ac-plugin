#include "precompiled.h"
#include "log.h"

#if (AC_LOG_ENABLED)

static wchar_t LogName[MAX_PATH] = {0};

void log_init(const wchar_t* filename)
{
	wcsncpy_s(LogName, filename, MAX_PATH - 1);
	if (LogName[0])
	{
		FILE* fd = NULL;
		_wfopen_s(&fd, LogName, L"wt");
		if (fd)
		{
			fclose(fd);
		}
	}
}

void log_printf(const wchar_t* format, ...)
{
	if (!LogName[0]) return;

	const size_t BufLen = 1024;
	wchar_t buf[BufLen];

	va_list args;
	va_start(args, format);
	const int count = _vsnwprintf_s(buf, BufLen - 1, _TRUNCATE, format, args);
	va_end(args);
	buf[count] = 0;

	FILE* fd = NULL;
	_wfopen_s(&fd, LogName, L"at");
	if (fd)
	{
		fwprintf(fd, L"%s\n", buf);
		fclose(fd);
	}
}

void log_release()
{
	LogName[0] = 0;
}

#endif
