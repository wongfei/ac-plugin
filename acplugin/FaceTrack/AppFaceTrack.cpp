#include "precompiled.h"
#include "AppFaceTrack.h"

static AppFaceTrack* s_facetrack = nullptr;

#define RVA_TrackIR_getOffsets 1893680
static void TrackIR_getOffsets(TrackIR* pthis, vec3f& rot, vec3f& pos)
{
	pos = s_facetrack->_pos;
	rot = s_facetrack->_rot;
}

AppFaceTrack::AppFaceTrack(ACPlugin* plugin) : PluginApp(plugin, L"facetrack")
{
	log_printf(L"+AppFaceTrack %p", this);
	auto pthis = this;
	s_facetrack = this;

	initForm(L"Face Track", 300, 300, PluginApp::FFAutoHide, this);

	auto font(new_udt_shared<Font>(eFontType::eFontMonospaced, 16.0f, false, false));
	std::wstring name, text;

	name.assign(L"lb_status");
	_lb_status = new_udt<ksgui_Label>(&name, _game->gui);
	_lb_status->font = font;
	_lb_status->setPosition(10, 40);
	_form->addControl(_lb_status);

	name.assign(L"btn_track");
	text.assign(L"track");
	auto btn = new_udt<ksgui_ActiveButton>(&name, _game->gui);
	btn->font = font;
	btn->setPosition(10, 60);
	btn->setSize(80, 25);
	btn->setText(text);
	btn->drawBorder = true;
	btn->drawBackground = false;
	btn->borderColor.ctor(1, 0, 0, 1);
	btn->unselectedColor.ctor(0, 0, 0, 0);
	btn->selectedColor.ctor(0, 0, 0, 0);
	btn->rollOnColor.ctor(1, 0, 0, 0.3f);
	btn->inactiveColor.ctor(0, 0, 0, 0);
	auto dumpClick = [pthis](const ksgui_OnControlClicked&) { pthis->toggleTracking(); };
	btn->evClicked.add(btn, dumpClick);
	_form->addControl(btn);

	activateForm();

	// facetrack

	memset(&_ft_api, 0, sizeof(_ft_api));
	_ft_handle = NULL;
	_pos = vec3f(0, 0, 0);
	_rot = vec3f(0, 0, 0);

	loadFtLib();
	//startTracking();
	hook_create(L"TrackIR::getOffsets", _drva(RVA_TrackIR_getOffsets), &TrackIR_getOffsets);

	writeConsole(strf(L"APP \"%s\" initialized", _appName.c_str()));
}

AppFaceTrack::~AppFaceTrack()
{
	log_printf(L"~AppFaceTrack %p", this);
	stopTracking();
}

bool AppFaceTrack::acpUpdate(ACCarState* carState, float deltaT)
{
	uint8_t tracking = false;
	vec3f euler = vec3f(0, 0, 0);

	if (_ft_handle)
	{
		facetrack_pose pose;
		if (_ft_api.get_pose(_ft_handle, &pose) == FACETRACK_OK)
		{
			tracking = pose.is_tracking;
			euler.x = pose.euler_kf[0];
			euler.y = pose.euler_kf[1];
			euler.z = pose.euler_kf[2];
		}
	}

	auto cam = _sim->cameraManager->cameraOnBoard;
	cam->trackIR.isValid = tracking ? true : false;
	//_rot.x = tclamp<float>(euler.y, -10, 10);
	_rot.x = 0;
	_rot.y = -tclamp<float>(euler.x, -25, 25);
	_rot.z = 0;

	std::wstring status;
	status.assign(strf(L"tracking=%d ; YPR %.2f %.2f %.2f", tracking, euler.x, euler.y, euler.z));
	_lb_status->setText(status);

	return PluginApp::acpUpdate(carState, deltaT);
}

bool AppFaceTrack::acpOnGui(ACPluginContext* context)
{
	return PluginApp::acpOnGui(context);
}

void AppFaceTrack::loadFtLib()
{
	auto lib = ::LoadLibraryA("plugins\\facetrack.dll");
	if (lib)
	{
		log_printf(L"Loaded facetrack.dll");

		pft_get_api ft_get_api = (pft_get_api)GetProcAddress(lib, "ft_get_api");
		if (ft_get_api)
		{
			log_printf(L"Found ft_get_api");

			if (ft_get_api(&_ft_api) == FACETRACK_OK)
			{
				auto ver = _ft_api.get_version();
				writeConsole(strf(L"facetrack v%d.%d", ver.major, ver.minor));
			}
		}
	}
}

void AppFaceTrack::startTracking()
{
	if (!_ft_handle && _ft_api.initialize)
	{
		facetrack_config config;
		memset(&config, 0, sizeof(config));

		config.draw_debug_info = false;
		config.resx = 1280;
		config.resy = 720;

		auto err = _ft_api.initialize(&config, &_ft_handle);
		log_printf(L"ft_initialize rc=%d handle=%p", err, _ft_handle);
	}
}

void AppFaceTrack::stopTracking()
{
	if (_ft_handle && _ft_api.shutdown)
	{
		auto err = _ft_api.shutdown(_ft_handle);
		_ft_handle = NULL;
		log_printf(L"ft_shutdown rc=%d", err);
	}
}

void AppFaceTrack::toggleTracking()
{
	if (!_ft_handle) 
		startTracking(); 
	else 
		stopTracking();
}
