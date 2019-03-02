#include "precompiled.h"
#include "CheaterDetector/AppCheaterDetector.h"
#include "CustomPhysics/AppCustomPhysics.h"

void* _ac_module = nullptr;
static ACPlugin* s_plugin = nullptr;

typedef std::shared_ptr<PluginApp> PluginAppPtr;
typedef std::vector<PluginAppPtr> PluginAppVector;
static PluginAppVector s_apps;

extern "C" {

AC_EXPORT bool AC_API acpGetName(wchar_t* name)
{
	wcscpy(name, AC_PLUGIN_NAME);
	return true;
}

AC_EXPORT bool AC_API acpInit(ACPlugin* plugin)
{
	_ac_module = ::GetModuleHandleA(nullptr);
	s_plugin = plugin;

	log_reset();
	log_printf(L"acpInit module=%p plugin=%p", _ac_module, plugin);

	#if defined(AC_DEBUG)
		MessageBoxA(NULL, "_acplugin", "_acplugin", MB_OK);
	#endif

	if (!hook_init()) return false;

	s_apps.push_back(std::make_shared<AppCheaterDetector>(plugin));
	s_apps.push_back(std::make_shared<AppCustomPhysics>(plugin));

	hook_enable();

	log_printf(L"acpInit DONE");
	return true;
}

AC_EXPORT bool AC_API acpShutdown()
{
	log_printf(L"acpShutdown");

	hook_disable();
	s_apps.clear();
	hook_shut();

	log_printf(L"acpShutdown DONE");
	return true;
}

AC_EXPORT bool AC_API acpUpdate(ACCarState* car, float deltaT)
{
	for (auto& app : s_apps) {
		app->acpUpdate(car, deltaT);
	}
	return true;
}

AC_EXPORT bool AC_API acpOnGui(ACPluginContext* context)
{
	for (auto& app : s_apps) {
		app->acpOnGui(context);
	}
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
