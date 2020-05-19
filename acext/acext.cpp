#include "precompiled.h"
#include "Custom/CustomPhysics.h"

static void* _orig_WinMain = nullptr;
static CustomPhysics* _physics = nullptr;

#define RVA_wWinMain 0xC29C0

static int WINAPI hook_wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
	log_printf(L"ENTER wWinMain");

	#if defined(AC_DEBUG)
		MessageBoxA(NULL, "_acext", "_acext", MB_OK);
	#endif

	_physics = new CustomPhysics();
	_physics->installHooks();

	//============================================================================
	// void Sim::Sim(Sim* this, Game* igame)
	// bDevMode = INIReader::getInt(pIni, L"AC_APPS", L"ENABLE_DEV_APPS");

	std::vector<uint8_t> OrigBytes;
	std::vector<uint8_t> PatchBytes;

	#if 1
	ParseByteArray(OrigBytes, "90 48 8D 15 CC CC CC CC 48 8D 4D F0 E8 CC CC CC CC 90 4C 8D 45 B0 48 8D 55 F0 48 8D 8D CC CC CC CC E8 CC CC CC CC 85 C0 40 0F 9F C6");
	ParseByteArray(PatchBytes, "B8 01 00 00 00 40 0F 9F C6 90 90");
	PatchProcess(GetCurrentProcess(), OrigBytes, PatchBytes, nullptr, OrigBytes.size() - 11);
	#endif

	//============================================================================
	// void Telemetry::init(Telemetry *this, Car *car)
	// v3->debugPhysics = v7 != 0;

	#if 0
	ParseByteArray(OrigBytes, "E8 CC CC CC CC 90 4C 8D 45 B0 48 8D 55 D0 48 8D 8D CC CC CC CC E8 CC CC CC CC 85 C0 0F 95 C0 88 47 40 48 83 7D CC CC 72 09 48 8B 4D D0 E8 CC CC CC CC 48 C7 45 CC CC CC CC CC");
	ParseByteArray(PatchBytes, "B8 01 00 00 00 40 0F 9F C6 90 90");
	PatchProcess(GetCurrentProcess(), OrigBytes, PatchBytes, nullptr, OrigBytes.size() - 11);
	#endif

	int rc = ((int(WINAPI*)(HINSTANCE, HINSTANCE, PWSTR, int))_orig_WinMain)(hInstance, hPrevInstance, pCmdLine, nCmdShow);

	log_printf(L"LEAVE wWinMain rc=%d", rc);
	return rc;
}

static void OnProcessAttach()
{
	ac_set_module(GetModuleHandleA(nullptr));
	log_printf(L"_ac_module -> %p", ac_get_module());

	if (hook_init())
	{
		hook_create(L"wWinMain", _drva(RVA_wWinMain), &hook_wWinMain, &_orig_WinMain);
		hook_enable();
	}
}

BOOL WINAPI DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	(void)hModule;
	(void)lpReserved;

	switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		{
			log_init(L"acext.log");
			log_printf(L"DLL_PROCESS_ATTACH");
			OnProcessAttach();
			break;
		}

		case DLL_PROCESS_DETACH:
		{
			log_printf(L"DLL_PROCESS_DETACH");
			log_close();
			break;
		}
	}

	return TRUE;
}

HRESULT WINAPI DirectInput8Create(HINSTANCE hinst, DWORD dwVersion, REFIID riidltf, LPVOID* ppvOut, LPUNKNOWN punkOuter)
{
	log_printf(L"DirectInput8Create");

	char LibPath[MAX_PATH] = {0};
	GetSystemDirectoryA(LibPath, sizeof(LibPath));
	strcat_s(LibPath, MAX_PATH, "\\dinput8.dll");
	HMODULE hLibrary = LoadLibraryA(LibPath);

	if (hLibrary)
	{
		FARPROC originalProc = GetProcAddress(hLibrary, "DirectInput8Create");
		if (originalProc)
		{
			return ((HRESULT(WINAPI*)(HINSTANCE, DWORD, REFIID, LPVOID*, LPUNKNOWN))originalProc)(hinst, dwVersion, riidltf, ppvOut, punkOuter);
		}
	}

	return E_FAIL;
}
