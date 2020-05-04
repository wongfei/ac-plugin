#include "precompiled.h"
#include "os.h"

BOOL osFileExists(LPCWSTR Path)
{
	DWORD dwAttrib = GetFileAttributesW(Path);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && !(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

BOOL osDirExists(LPCWSTR Path)
{
	DWORD dwAttrib = GetFileAttributesW(Path);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

VOID osEnsureDirExists(const std::wstring& path)
{
	if (!osDirExists(path.c_str()))
	{
		CreateDirectoryW(path.c_str(), nullptr);
	}
}

VOID osCreateDirectoryTree(const std::wstring& path)
{
	std::wstring::size_type pos = 0;
	do
	{
		pos = path.find_first_of(L"\\/", pos + 1);
		osEnsureDirExists(path.substr(0, pos).c_str());
	}
	while (pos != std::wstring::npos);
}

std::wstring osGetDocumentsPath()
{
	wchar_t buf[MAX_PATH] = {0};
	SHGetFolderPathW(0, CSIDL_PERSONAL, 0, 0, buf);
	return std::wstring(buf);
}

BOOL osGetProcessModuleInfo(DWORD ProcessId, MODULEENTRY32* pModule)
{
	BOOL Result = FALSE;

	HANDLE Snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, ProcessId);
	if (Snap)
	{
		MODULEENTRY32 Entry;
		Entry.dwSize = sizeof(Entry);

		if (Module32First(Snap, &Entry))
		{
			do
			{
				if (Entry.th32ProcessID == ProcessId)
				{
					if (pModule) *pModule = Entry;
					Result = TRUE;
					break;
				}
			}
			while (Module32Next(Snap, &Entry));
		}

		CloseHandle(Snap);
	}

	return Result;
}

PVOID osGetProcessBaseAddress(HANDLE Process)
{
	PVOID Result = nullptr;

	MODULEENTRY32 Module;
	if (osGetProcessModuleInfo(GetProcessId(Process), &Module))
	{
		Result = Module.modBaseAddr;
	}

	return Result;
}
