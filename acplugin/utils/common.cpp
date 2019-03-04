#include "precompiled.h"
#include "common.h"

std::wstring strf(const wchar_t* format, ...)
{
	wchar_t buf[1024] = {0};
	va_list args;
	va_start(args, format);
	_vsnwprintf_s(buf, _countof(buf) - 1, _TRUNCATE, format, args);
	va_end(args);
	return std::wstring(buf);
}
