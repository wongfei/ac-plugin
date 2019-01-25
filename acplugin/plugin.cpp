#include "CheaterDetector.h"

void* _g_module = nullptr;
static ACPlugin* s_plugin = nullptr;
static PluginBase* s_app = nullptr;

extern "C" {

AC_EXPORT bool AC_API acpGetName(wchar_t* name)
{
	wcscpy(name, AC_PLUGIN_NAME);
	return true;
}

AC_EXPORT bool AC_API acpInit(ACPlugin* plugin)
{
	_g_module = GetModuleHandleA(nullptr);
	s_plugin = plugin;

	log_reset();
	log_printf(L"acpInit module=%p plugin=%p", _g_module, plugin);

	#if defined(AC_DEBUG)
		MessageBoxA(NULL, "_acplugin", "_acplugin", MB_OK);
	#endif

	s_app = new CheaterDetector(plugin);
	return true;
}

AC_EXPORT bool AC_API acpShutdown()
{
	log_printf(L"acpShutdown");
	safe_delete(s_app);
	return true;
}

AC_EXPORT bool AC_API acpUpdate(ACCarState* car, float deltaT)
{
	return (s_app ? s_app->acpUpdate(car, deltaT) : true);
}

AC_EXPORT bool AC_API acpOnGui(ACPluginContext* context)
{
	return (s_app ? s_app->acpOnGui(context) : true);
}

AC_EXPORT bool AC_API acpGetControls(ICarControlsProvider** controls)
{
	return false;
}

} // extern "C"

BOOL WINAPI DllMain(
  _In_ HINSTANCE hinstDLL,
  _In_ DWORD     fdwReason,
  _In_ LPVOID    lpvReserved)
{
	switch (fdwReason)
	{
		case DLL_PROCESS_ATTACH:
			break;
	}
	return TRUE;
}
