#pragma once

BOOL osFileExists(LPCWSTR Path);
BOOL osDirExists(LPCWSTR Path);
VOID osEnsureDirExists(const std::wstring& path);
std::wstring osGetDocumentsPath();

BOOL osGetProcessModuleInfo(DWORD ProcessId, MODULEENTRY32* pModule);
PVOID osGetProcessBaseAddress(HANDLE Process);
