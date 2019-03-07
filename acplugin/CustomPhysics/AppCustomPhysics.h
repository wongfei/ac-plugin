#pragma once

#include "plugin/PluginApp.h"

struct CarControlsRawHeader
{
	uint32_t sampleCount;
	uint32_t sampleSize;
	mat44f bodyMat;
};

struct CarControlsSample
{
	uint64_t sampleId;
	double currentTime;
	double gameTime;
	CarControls controls;
};

class AppCustomPhysics : public PluginApp
{
public:

	AppCustomPhysics(ACPlugin* plugin);
	virtual ~AppCustomPhysics();

	virtual bool acpUpdate(ACCarState* carState, float deltaT) { return true; }
	virtual bool acpOnGui(ACPluginContext* context) { return true; }

	void run(PhysicsDriveThread* pThread);
	void pollControls(Car* pCar, float dt);

	void saveControlSamples(const wchar_t* filename);
	void loadControlSamples(const wchar_t* filename);

protected:

	enum class EControlMode {
		Default = 0,
		Record,
		Replay
	};
	EControlMode _controlMode = EControlMode::Default;

	std::vector<CarControlsSample> _controlSamples;
	mat44f _bodyMat;
	uint64_t _sampleId = 0;
	bool _gameStarted = false;

	void* _orig_PhysicsDriveThread_run = nullptr;
	void* _orig_Car_pollControls = nullptr;
};
