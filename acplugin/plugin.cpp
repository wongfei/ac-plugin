#include "PluginForm.h"

void* _g_module = nullptr;
static ACPlugin* s_plugin = nullptr;
static PluginForm* s_app = nullptr;

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

	#if defined(_DEBUG)
	AC_WAIT_DEBUGGER;
	#endif

	s_app = new PluginForm(plugin);
	return true;
}

AC_EXPORT bool AC_API acpShutdown()
{
	log_printf(L"acpShutdown");
	if (s_app) {
		delete s_app;
		s_app = nullptr;
	}
	return true;
}

AC_EXPORT bool AC_API acpUpdate(ACCarState* car, float deltaT)
{
	if (s_app) {
		s_app->acpUpdate(deltaT);
	}
	return true;
}

AC_EXPORT bool AC_API acpOnGui(ACPluginContext* context)
{
	return true;
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
