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

std::wstring getDocumentsPath()
{
	wchar_t buf[MAX_PATH];
	::SHGetFolderPathW(0, 5, 0, 0, buf);
	return std::wstring(buf);
}

bool fileExists(LPCWSTR Path)
{
	DWORD dwAttrib = ::GetFileAttributesW(Path);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && !(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

bool dirExists(LPCWSTR Path)
{
	DWORD dwAttrib = ::GetFileAttributesW(Path);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

void ensureDirExists(const std::wstring& path)
{
	if (!dirExists(path.c_str())) {
		::CreateDirectoryW(path.c_str(), nullptr);
	}
}
