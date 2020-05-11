#pragma once

typedef std::function<void (float deltaT)> TimerCallback;

struct TimerNode
{
	TimerCallback callback;
	float rate;
	float accum;
};

class PluginApp
{
public:

	enum FormFlags : unsigned int
	{
		FFCanBeScaled = 1 << 1,
		FFAutoHide = 1 << 2,
	};

	PluginApp(ACPlugin* plugin, const std::wstring& appName);
	virtual ~PluginApp();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

	void initForm(const std::wstring& title, float width, float height, unsigned int flags, void* tag);
	void activateForm();
	void destroyForm();
	void loadFormConfig();
	void writeFormConfig();
	inline ksgui_Form* getForm() { return _form; }

	void addTimer(float rate, TimerCallback callback);
	void updateTimers(float deltaT);

	void writeConsole(const std::wstring& text, bool writeToLog = false);
	std::wstring getAppConfigPath();

protected:

	ACPlugin* _plugin = nullptr;
	Sim* _sim = nullptr;
	Game* _game = nullptr;
	ksgui_Form* _form = nullptr;

	std::wstring _appName;
	std::vector<TimerNode> _timers;
};
