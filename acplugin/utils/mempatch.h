#pragma once

BOOL PatchProcess(HANDLE Process, const std::vector<uint8_t>& OrigBytes, const std::vector<uint8_t>& PatchBytes, PVOID Addr, SIZE_T Offset);
PVOID FindPattern(HANDLE Process, PVOID ImageBase, const std::vector<uint8_t>& Pattern);
BOOL ParseByteArray(std::vector<uint8_t>& Vec, LPCSTR Str);
