#include "precompiled.h"

void* _ac_module = nullptr;

void* ac_get_module() { return _ac_module; }
void ac_set_module(void* base) { _ac_module = base; }

std::wstring strf(const wchar_t* format, ...)
{
	wchar_t buf[1024] = {0};
	va_list args;
	va_start(args, format);
	_vsnwprintf_s(buf, _countof(buf) - 1, _TRUNCATE, format, args);
	va_end(args);
	return std::wstring(buf);
}
