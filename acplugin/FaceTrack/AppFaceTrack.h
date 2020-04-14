#pragma once

#include "plugin/PluginApp.h"
#include "FaceTrack/facetrack.h"

class AppFaceTrack : public PluginApp
{
public:

	AppFaceTrack(ACPlugin* plugin);
	virtual ~AppFaceTrack();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

private:

	void loadFtLib();
	void startTracking();
	void stopTracking();
	void toggleTracking();

public:

	ksgui_Label* _lb_status = nullptr;

	facetrack_api _ft_api;
	facetrack_handle _ft_handle;
	vec3f _pos;
	vec3f _rot;
};
