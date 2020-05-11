#include "precompiled.h"
#include "AppCustomPhysics.h"
#include "Custom/CustomPhysics.h"

AppCustomPhysics::AppCustomPhysics(ACPlugin* plugin) : PluginApp(plugin, L"custom_physics")
{
	// plugin executes after game initialized and cars loaded

	log_printf(L"+AppCustomPhysics %p", this);

	#if 0
	// better use acext for physics
	_impl.reset(new CustomPhysics());
	_impl->installHooks();
	_impl->printCarInfo(plugin->carAvatar);
	#endif

	writeConsole(strf(L"APP \"%s\" initialized", _appName.c_str()));
}

AppCustomPhysics::~AppCustomPhysics()
{
	log_printf(L"~AppCustomPhysics %p", this);

	_impl.reset();
}
