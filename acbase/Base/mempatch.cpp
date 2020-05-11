#include "precompiled.h"

BOOL WriteProtectedMemory(HANDLE Process, PVOID Addr, SIZE_T Offset, const std::vector<uint8_t>& PatchBytes)
{
	BOOL Result = FALSE;
	log_printf(L"WriteProtectedMemory Process=%p Addr=%p Offset=%u Size=%u", Process, Addr, (unsigned int)Offset, (unsigned int)PatchBytes.size());

	MEMORY_BASIC_INFORMATION mbi;
	ZeroMemory(&mbi, sizeof(mbi));

	if (!VirtualQuery(Addr, &mbi, sizeof(mbi)))
	{
		log_printf(L"VirtualQuery failed: err=0x%X", (unsigned int)GetLastError());
		SHOULD_NOT_REACH_WARN;
		return FALSE;
	}

	DWORD OrigProtect = 0;
	if (!VirtualProtect(mbi.BaseAddress, mbi.RegionSize, PAGE_READWRITE, &OrigProtect))
	{
		log_printf(L"VirtualProtect failed: err=0x%X", (unsigned int)GetLastError());
		SHOULD_NOT_REACH_WARN;
		return FALSE;
	}

	SIZE_T IoSize = 0;
	Result = WriteProcessMemory(Process, (PVOID)((UINT64)Addr + (UINT64)Offset), &PatchBytes[0], PatchBytes.size(), &IoSize);
	if (!Result)
	{
		log_printf(L"WriteProcessMemory failed: err=0x%X", (unsigned int)GetLastError());
		SHOULD_NOT_REACH_WARN;
	}

	VirtualProtect(mbi.BaseAddress, mbi.RegionSize, OrigProtect, &mbi.Protect);

	//log_printf(L"Result=%d", (int)Result);
	return Result;
}

BOOL PatchProcess(HANDLE Process, const std::vector<uint8_t>& OrigBytes, const std::vector<uint8_t>& PatchBytes, PVOID Addr, SIZE_T Offset)
{
	BOOL Result = FALSE;

	if (Addr)
	{
		Result = WriteProtectedMemory(Process, Addr, Offset, PatchBytes);
	}
	else
	{
		LPVOID BaseAddress = osGetProcessBaseAddress(Process);
		if (BaseAddress)
		{
			PVOID Pattern = FindPattern(Process, BaseAddress, OrigBytes);
			if (Pattern)
			{
				Result = WriteProtectedMemory(Process, Pattern, Offset, PatchBytes);
			}
		}
	}

	return Result;
}

static inline bool CompareBytes(const uint8_t &a, const uint8_t &b)
{
	return (a == b || b == 0xCC); // use 0xCC (int3) as wildcard
}

PVOID FindPattern(HANDLE Process, PVOID ImageBase, const std::vector<uint8_t>& Pattern)
{
	PVOID Result = nullptr;
	log_printf(L"FindPattern Process=%p ImageBase=%p Size=%u", Process, ImageBase, (unsigned int)Pattern.size());

	uint8_t* Ptr = (uint8_t*)ImageBase;
	MEMORY_BASIC_INFORMATION MemInfo;
	std::vector<uint8_t> Buffer;

	for (;;)
	{
		SIZE_T QuerySize = VirtualQueryEx(Process, Ptr, &MemInfo, sizeof(MemInfo));
		if (QuerySize != sizeof(MemInfo))
		{
			log_printf(L"VirtualQueryEx failed: err=0x%X", (unsigned int)GetLastError());
			SHOULD_NOT_REACH_WARN;;
			break;
		}

		const BOOL bExecutable = (
			(MemInfo.Protect & PAGE_EXECUTE) ||
			(MemInfo.Protect & PAGE_EXECUTE_READ) ||
			(MemInfo.Protect & PAGE_EXECUTE_READWRITE) ||
			(MemInfo.Protect & PAGE_EXECUTE_WRITECOPY));

		if (bExecutable && ((MemInfo.Protect & PAGE_GUARD) == 0) && ((MemInfo.Protect & PAGE_NOACCESS) == 0))
		{
			if (Buffer.size() < MemInfo.RegionSize)
			{
				Buffer.resize(MemInfo.RegionSize);
			}

			SIZE_T NumBytes = 0;
			BOOL Res = ReadProcessMemory(Process, MemInfo.BaseAddress, &Buffer[0], MemInfo.RegionSize, &NumBytes);
			if (!Res || NumBytes != MemInfo.RegionSize)
			{
				log_printf(L"ReadProcessMemory failed: err=0x%X", (unsigned int)GetLastError());
				SHOULD_NOT_REACH_WARN;;
				break;
			}

			auto BufferEnd = Buffer.begin() + MemInfo.RegionSize;
			auto Iter = Buffer.begin();

			if ((Iter = std::search(Iter, BufferEnd, Pattern.begin(), Pattern.end(), CompareBytes)) != BufferEnd)
			{
				Result = (uint8_t*)MemInfo.BaseAddress + (Iter - Buffer.begin());
				break;
			}
		}

		Ptr += MemInfo.RegionSize;
	}

	//log_printf(L"Result=%p", Result);
	return Result;
}

static inline uint8_t ParseHex(uint8_t Val)
{
	if (Val >= '0' && Val <= '9') return (Val - '0');
	if (Val >= 'a' && Val <= 'f') return (Val - 'a') + 10;
	if (Val >= 'A' && Val <= 'F') return (Val - 'A') + 10;
	return 0;
}

static inline uint8_t ParseByte(const char* Str)
{
	uint8_t hi = ParseHex((uint8_t)Str[0]);
	uint8_t lo = ParseHex((uint8_t)Str[1]);
	return ((hi << 4) | lo);
}

BOOL ParseByteArray(std::vector<uint8_t>& Vec, LPCSTR Str)
{
	Vec.clear();
	if (!Str) return FALSE;

	const size_t Len = strlen(Str);
	if (!Len) return FALSE;

	size_t SpaceCount = 0;
	for (size_t i = 0; i < Len; ++i)
	{
		if (Str[i] == ' ') SpaceCount++;
	}

	const size_t NumBytes = (SpaceCount + 1);
	if (Len != (NumBytes * 2 + SpaceCount))
	{
		return FALSE;
	}

	Vec.resize(NumBytes);
	for (size_t i = 0; i < NumBytes; i++)
	{
		Vec[i] = ParseByte(Str);
		Str += 3;
	}

	return TRUE;
}
