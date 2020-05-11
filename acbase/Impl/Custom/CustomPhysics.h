#pragma once

#define RND_SEED 0

#define BEGIN_HOOK_OBJ(name) struct _##name : public name {
#define END_HOOK_OBJ() };
#define HOOK_OBJ(name) _##name::_hook()

#define HOOK_FUNC_RVA(func) hook_create(UT_WSTRING(func), _drva(RVA_##func), &func)
#define HOOK_FUNC_RVA_ORIG(func) hook_create(UT_WSTRING(func), _drva(RVA_##func), &func, &_orig_##func)

#define HOOK_METHOD_RVA(obj, func) hook_create(UT_WSTRING(obj##::##func), _drva(RVA_##obj##_##func), xcast<void*>(&_##obj##::_##func))
#define HOOK_METHOD_RVA_ORIG(obj, func) hook_create(UT_WSTRING(obj##::##func), _drva(RVA_##obj##_##func), xcast<void*>(&_##obj##::_##func), &_orig_##obj##_##func)

//#define ORIG_METHOD(obj, func) xcast<decltype(&_##obj::_##func)>(_orig_##obj##_##func)
#define ORIG_METHOD(obj, func) xcast<decltype(&obj::func)>(_orig_##obj##_##func)
#define THIS_CALL(func) (this->*func)

struct CarControlsRawHeader
{
	uint32_t sampleCount;
	uint32_t sampleSize;
	mat44f bodyMat;
};

struct CarControlsSample
{
	uint64_t sampleId;
	uint64_t stepCounter;
	double currentTime;
	double gameTime;
	CarControls controls;
};

class CustomPhysics
{
public:

	CustomPhysics();
	virtual ~CustomPhysics();

	void printCarInfo(CarAvatar* car);
	void installHooks();

	void run(PhysicsDriveThread* pThread);
	void stepNormal(PhysicsDriveThread* pThis);
	void stepReplay(PhysicsDriveThread* pThis);
	void stepPhysicsEngine(PhysicsDriveThread* pThis, double fCurTime, double fGt);

	void pollControls(Car* pCar, float dt);
	void processCustomInput();

	void addConsoleCommands();
	void cmdRecord(const std::wstring& name);
	void cmdReplay(const std::wstring& name);

	void resetTelemetry();
	void saveTelemetry(const std::wstring& filename);
	void saveControlSamples(const std::wstring& filename);
	void loadControlSamples(const std::wstring& filename);

	void printMatrix(const mat44f& mat);

	inline void setSim(Sim* pSim) { _sim = pSim; }

protected:

	enum class EControlMode {
		Default = 0,
		Record,
		Replay
	};

	Sim* _sim = nullptr;
	CarAvatar* _car = nullptr;

	EControlMode _controlMode = EControlMode::Default;
	bool _gameStarted = false;
	bool _cmdRecord = false;
	bool _cmdReplay = false;

	std::wstring _recName;
	std::vector<CarControlsSample> _controlSamples;
	mat44f _bodyMat;
	uint64_t _sampleId = 0;
	double _recStartTime = 0;
	double _recStopTime = 0;

	ICarControlsProvider* _playerControls = nullptr;
	AIDriver* _aiDriver = nullptr;
	bool _aiEnabled = false;
};
