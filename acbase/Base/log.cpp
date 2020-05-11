#include "precompiled.h"

static FILE* _LogFile = nullptr;

void log_init(const wchar_t* filename)
{
	log_close();
	if (_wfopen_s(&_LogFile, filename, L"wt") != 0)
	{
		// epic fail
		_LogFile = nullptr;
	}
}

void log_printf(const wchar_t* format, ...)
{
	if (!_LogFile)
		return;

	const size_t BufLen = 1024;
	wchar_t buf[BufLen];

	va_list args;
	va_start(args, format);
	const int count = _vsnwprintf_s(buf, BufLen - 1, _TRUNCATE, format, args);
	va_end(args);
	buf[count] = 0;

	fwprintf(_LogFile, L"%s\n", buf);
}

void log_close()
{
	if (_LogFile) 
	{
		fclose(_LogFile);
		_LogFile = nullptr;
	}
}
