#pragma once

#include "plugin/PluginApp.h"

class CustomPhysics;

class AppCustomPhysics : public PluginApp
{
public:

	AppCustomPhysics(ACPlugin* plugin);
	virtual ~AppCustomPhysics();

	virtual bool acpUpdate(ACCarState* carState, float deltaT) { return true; }
	virtual bool acpOnGui(ACPluginContext* context) { return true; }

private:

	std::unique_ptr<CustomPhysics> _impl;
};
