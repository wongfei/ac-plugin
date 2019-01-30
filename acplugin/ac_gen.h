// ### AUTO-GENERATED ###

enum class GearChangeRequest {
	eNoGearRequest = 0x0,
	eChangeUp = 0x1,
	eChangeDown = 0x2,
	eChangeToGear = 0x3,
};

enum class JumpStartPenaltyMode {
	eLockOnGridMode = 0x0,
	eTeleportToPitMode = 0x1,
	eDriveThroughMode = 0x2,
};

enum class VoteType {
	eVoteNextSession = 0x0,
	eVoteRestartSession = 0x1,
	eVoteKickUser = 0x2,
	eVoteUnkonw = 0x3,
};

enum class PenaltyDescription {
	eNothing = 0x0,
	eJumpStart = 0x1,
	eCantPitPenalty = 0x2,
	eMandatoryPit = 0x3,
	eCut = 0x4,
};

enum class eReplayStatus {
	eReplayPlay = 0x0,
	eReplayPause = 0x1,
	eReplayStop = 0x2,
	eReplayRewind = 0x3,
	eReplayFastForward = 0x4,
	eReplaySlowMotion = 0x5,
	eReplayModeEnter = 0x6,
	eReplayModeExit = 0x7,
	eReplayChangeCar = 0x8,
	eReplayClearing = 0x9,
	eReplaySliderInteraction = 0xA,
	ePhotoMode = 0xB,
	eSingleFrame = 0xC,
};

enum class eTimeLineCheckResponse {
	eOutOfRange = 0x0,
	eNegativeSide = 0x1,
	ePositiveSide = 0x2,
};

enum class DifferentialType {
	LSD = 0x0,
	Spool = 0x1,
};

enum class MouseButton {
	Unknown = 0xFF,
	Left = 0x0,
	Middle = 0x1,
	Right = 0x2,
};

enum class SessionType {
	Undefined = 0x0,
	Pratice = 0x1,
	Qualify = 0x2,
	Race = 0x3,
	Hotlap = 0x4,
	TimeAttack = 0x5,
	Drift = 0x6,
	Drag = 0x7,
};

enum class DriverActions {
	eLookingLeft = 0x0,
	eLookingRight = 0x1,
	eDeprecated_2 = 0x2,
	eDeprecated_1 = 0x3,
	eHeadlightsSwitch = 0x4,
	eChangingCamera = 0x5,
	eHorn = 0x6,
	eShiftingUp = 0x7,
	eShiftingDown = 0x8,
	eLookingBack = 0x9,
	eHeadlightsFlash = 0xA,
	eTcUp = 0xB,
	eTcDown = 0xC,
	eAbsUp = 0xD,
	eAbsDown = 0xE,
	eTurboUp = 0xF,
	eTurboDown = 0x10,
	eBrakeBiasUp = 0x11,
	eBrakeBiasDown = 0x12,
	eDRS = 0x13,
	eKERS = 0x14,
	eEngineBrakeUp = 0x15,
	eEngineBrakeDown = 0x16,
	eMGUKDeliveryUp = 0x17,
	eMGUKDeliveryDown = 0x18,
	eMGUKRecoveryUp = 0x19,
	eMGUKRecoveryDown = 0x1A,
	eMGUHMode = 0x1B,
};

enum class PenaltyType {
	eNothing = 0x0,
	eSecsOnResult5 = 0x1,
	eSecsOnResult10 = 0x2,
	eSecsOnResult20 = 0x3,
	eSecsOnResult30 = 0x4,
	eDriveThrough = 0x5,
	eStopAndGo = 0x6,
};

enum class DRWWingConnectionMode {
	UseEffect = 0x0,
	UseAngle = 0x1,
};

enum class KGLTexture_ImageFileFormat {
	eDDS = 0x0,
	ePNG = 0x1,
	eJPG = 0x2,
	eBMP = 0x3,
	eUnknown = 0x4,
};

enum class TelemetryUnits {
	eUnitMeters = 0x0,
	eUnitC = 0x1,
	eUnitG = 0x2,
	eUnitRadSec = 0x3,
	eUnitGeneric = 0x4,
	eUnitMeterSec = 0x5,
	eUnitBar = 0x6,
	eUnitMS = 0x7,
	eUnitNumber = 0x8,
	eUnitPercentage = 0x9,
	eUnitPowerWatt = 0xA,
	eUnitVolumeMQ = 0xB,
	eUnitVoltageV = 0xC,
	eUnitTorqueNM = 0xD,
	eUnitRad = 0xE,
	eUnitForceN = 0xF,
	eUnitMillimiters = 0x10,
	eUnitDeg = 0x11,
	eUnitForceKG = 0x12,
	eUnitRPM = 0x13,
};

enum class eRenderTargetFormat {
	eOriginalTarget = 0x0,
	eR8G8B8A8 = 0x1,
	eR16G16B16A16 = 0x2,
	eR32F = 0x3,
	eR32Typeless = 0x4,
	eR32Typeless_MS = 0x5,
	eR16F = 0x6,
	eR16G16B16A16_MS = 0x7,
	eR8G8B8A8_MS = 0x8,
};

enum class FlagEventType {
	BlackFlag = 0x0,
	BlackFlagClear = 0x1,
};

enum class eVariableType {
	eFloat = 0x0,
	eFloat2 = 0x1,
	eFloat3 = 0x2,
	eFloat4 = 0x3,
	eMatrix = 0x4,
	eTypeUndefined = 0x5,
};

enum class ILType {
	Default = 0x0,
	Skinned = 0x1,
	Particle = 0x2,
	GUI2D = 0x3,
};

enum class eACEventType {
	acEvent_OnCollision = 0x0,
};

enum class PixelFormat {
	eUnknownFormat = 0x0,
	eRGBA8 = 0x1,
	eRGBA32 = 0x2,
	eBC3Unorm = 0x3,
};

enum class OpenTrackTimeState {
	NotStarted = 0x0,
	Started = 0x1,
};

enum class CarSetupState {
	UnKnown = 0x0,
	Legal = 0x1,
	Illegal = 0x2,
};

enum class DynamicControllerInput {
	UndefinedInput = 0x0,
	Brake = 0x1,
	Gas = 0x2,
	LatG = 0x3,
	LonG = 0x4,
	Steer = 0x5,
	Speed = 0x6,
	Gear = 0x7,
	SlipRatioMAX = 0x8,
	SlipRatioAVG = 0x9,
	SlipAngleFrontAVG = 0xA,
	SlipAngleRearAVG = 0xB,
	SlipAngleFrontMAX = 0xC,
	SlipAngleRearMAX = 0xD,
	OversteerFactor = 0xE,
	RearSpeedRatio = 0xF,
	SteerDEG = 0x10,
	Const = 0x11,
	RPMS = 0x12,
	WheelSteerDEG = 0x13,
	LoadSpreadLF = 0x14,
	LoadSpreadRF = 0x15,
	AvgTravelRear = 0x16,
	SusTravelLR = 0x17,
	SusTravelRR = 0x18,
};

enum class DynamicControllerCombinatorMode {
	eUndefinedMode = 0x0,
	eAdd = 0x1,
	eMult = 0x2,
};

enum class eFontType {
	eFontProportional = 0x0,
	eFontMonospaced = 0x1,
	eFontCustom = 0x2,
};

enum class eFontAlign {
	eAlignLeft = 0x0,
	eAlignRight = 0x1,
	eAlignCenter = 0x2,
};

enum class RenderPassID {
	Opaque = 0x0,
	Transparent = 0x1,
	Shadowgen = 0x2,
};

enum class CullMode {
	eCullFront = 0x0,
	eCullBack = 0x1,
	eCullNone = 0x2,
	eCullBiased = 0x3,
	eCullWireFrame = 0x4,
	eCullFrontNoMS = 0x5,
};

enum class BlendMode {
	eOpaque = 0x0,
	eAlphaBlend = 0x1,
	eAlphaToCoverage = 0x2,
};

enum class DepthMode {
	eDepthNormal = 0x0,
	eDepthNoWrite = 0x1,
	eDepthOff = 0x2,
	eDepthLessEqual = 0x3,
};

enum class DynamicWingController_eCombinatorMode {
	eUndefinedMode = 0x0,
	eAdd = 0x1,
	eMult = 0x2,
};

enum class DynamicWingController_eInputVar {
	eUndefined = 0x0,
	eBrake = 0x1,
	eGas = 0x2,
	eLatG = 0x3,
	eLonG = 0x4,
	eSteer = 0x5,
	eSpeed = 0x6,
	SusTravelLR = 0x7,
	SusTravelRR = 0x8,
};

enum class ksgui_VariableConnection {
	ConnectFloat = 0x0,
	ConnectInt = 0x1,
};

enum class KersAttachment {
	Engine = 0x0,
	Wheels = 0x1,
};

enum class CustomSpinnerMode {
	eStandardSpinner = 0x0,
	eCircularSpinner = 0x1,
	eVerticalSpinBar = 0x2,
};

enum class ksgui_eActiveButtonStates {
	eABUnselected = 0x0,
	eABSelected = 0x1,
	eABRollOver = 0x2,
};

enum class eTaskBarStatus {
	eSelected = 0x0,
	eUnselected = 0x1,
};

enum class FindTyreCompoundLogic {
	Random = 0x0,
	Fastest = 0x1,
	Preferred = 0x2,
};

enum class ksgui_eArrowsDirection {
	eLeft = 0x0,
	eRight = 0x1,
	eUp = 0x2,
	eDown = 0x3,
};

enum class eGLPrimitiveType {
	eLines = 0x0,
	eLinesStrip = 0x1,
	eTriangles = 0x2,
	eQuads = 0x3,
};

enum class ksgui_eListBoxSelectionMode {
	eLineSelected = 0x0,
	eTextHighlighted = 0x1,
};

enum class PenaltyMode {
	CutGas = 0x0,
	InvalidateLap = 0x1,
	RecoverTime = 0x2,
	Nothing = 0x3,
	CutDetection = 0x4,
};

enum class EBBMode {
	Disabled = 0x0,
	Internal = 0x1,
	DynamicController = 0x2,
};

enum class SamplerState {
	eDefaultSampler = 0x0,
	eLinearClamped = 0x1,
};

enum class TractionType {
	RWD = 0x0,
	FWD = 0x1,
	AWD = 0x2,
	AWD_NEW = 0x3,
};

enum class TorqueModeEX {
	original = 0x0,
	reactionTorques = 0x1,
	driveTorques = 0x2,
};

enum class SuspensionType {
	DoubleWishbone = 0x0,
	Strut = 0x1,
	Axle = 0x2,
	Multilink = 0x3,
};

class RenderWindow;
class CarAvatar;
class Car;
struct TyreThermalPatch;
class ksgui_Spinner;
class ksgui_Control;
class ksgui_CheckBox;
class ksgui_ScrollBar;
class ksgui_Slider;
struct SVar;
class ESCMenu;
struct HWND__;
struct HINSTANCE__;
class Wing;
class ksgui_ListBox;
struct ksgui_ListBoxRowData;
class ICarControlsProvider;
class IMaterialOptionChangeListener;
class GraphicsManager;
class Game;
class GameObject;
class Shader;
class CBuffer;
class RaceManager;
class ShaderVariable;
class ShaderResource;
class NetCarStateProvider;
class IRigidBody;
class ICollisionObject;
class Task;
class Suspension;
class AISpline;
class Track;
class SetupItem;
struct KGLShaderTexture;
struct KGLShaderVar;
class Sim;
struct ACCarState;
struct ACPluginContext;
class Joypad;
struct TimeTransponder;
class LapDB;
class Font;
class ConsoleCommand;
class IVarCallback;
class PhysicsEngine;
class ISphereCollisionCallback;
class Renderable;
class IRayTrackCollisionProvider;
class ACClient;
class PhysicsAvatar;
class Material;
struct CarPhysicsState;
class MaterialVar;
class IJoint;
class ICollisionCallback;
class IRayCaster;
class Node;
struct CompileContext;
struct RenderContext;
class MaterialResource;
class ksgui_GUI;
class GLRenderer;
class Turbo;
class ISuspension;
class IKeyEventListener;
class ksgui_TextBox;
class ksgui_Label;
class ksgui_ActiveButton;
struct Tyre;
class ksgui_MovingBar;
class ksgui_Form;
class ksgui_TaskBarIcon;
class ksgui_PopOver;
class IndexBuffer;
struct MeshVertex;
class ksgui_ListBoxRow;
class ITyreModel;
struct SurfaceDef;
struct RayCastResult;
class RaceTimingServices;
class ksgui_Taskbar;
class AudioEngine;
class RenderTarget;
class Camera;
class KeyboardManager;
class ICarPhysicsStateProvider;
struct SlipStream;
class IPhysicsCore;
class IDebugVisualizer;
class TrackAvatar;
class SkyBox;
class ReplayManager;
class DebugVisualizer;
class ksgui_GameScreen;
class PauseMenu;
class ksgui_FormRenderStats;
class CarNodeSorter;
class NodeEvent;
class PitStop;
class DrivingAssistManager;
class MirrorTextureRenderer;
class ACCameraManager;
class NodeDirtCamera;
class SystemMessage;
class SystemNotification;
class ScreenCapturer;
class Texture;
class Console;
class TimeLimitedTest;
class QuickMenu;
class MicroSectors;
class SimScreen;
class ACErrorHandler;
class CameraForward;
class EndSessionDisplayer;
class SessionLeaderboard;
class VirtualMirrorRenderer;
struct ksgui_ValueSerie;
class ICoastGenerator;
class ITorqueGenerator;
class PvsProcessor;
class GPUProfiler;
class CubeMap;
class IVertexBuffer;
class ACClientVotingManager;
class WrongWayIndicator;
class DriverModel;
struct SetupManager;
class NodeBoundingSphere;
class IEventTrigger;
class CarAudioFMOD;
class SkidMarkBuffer;
class ISuspensionAvatar;
class CarLodManager;
class ConstrainedObjectsManager;
struct BackfireParams;
class CarAnimations;
struct CarColliderManager;
struct TyreThermalState;
struct AIState;
class Mesh;

class KGLRenderTarget {
public:
	ID3D11Texture2D * rtTexture;
	ID3D11RenderTargetView * renderTargetView;
	ID3D11ShaderResourceView * shaderResourceView;
	ID3D11DepthStencilView * renderTargetViewDepth;
	DXGI_FORMAT format;
	unsigned int width;
	unsigned int height;
	int samples;
	inline KGLRenderTarget()  { }
	inline void ctor(ID3D11Device * device, DXGI_FORMAT fmt, unsigned int iwidth, unsigned int iheight, bool isDepth, int aSamples, int mips) { typedef void (*_fpt)(KGLRenderTarget *pthis, ID3D11Device *, DXGI_FORMAT, unsigned int, unsigned int, bool, int, int); _fpt _f=(_fpt)_drva(144080); _f(this, device, fmt, iwidth, iheight, isDepth, aSamples, mips); }
};

struct ACClient_ClientEndSession {
public:
	bool isConnectionFinished;
	float sendToPitsTimer;
	float shutdownTimer;
	inline ACClient_ClientEndSession()  { }
};

struct OnSectorSplitEvent {
public:
	unsigned int carIndex;
	unsigned int sectorIndex;
	unsigned int sectorTime;
	unsigned int cuts;
	inline OnSectorSplitEvent()  { }
};

struct BrushTyreModelData {
public:
	float CF;
	float xu;
	float CF1;
	float Fz0;
	float maxSlip0;
	float maxSlip1;
	float falloffSpeed;
	inline BrushTyreModelData()  { }
};

struct TyreData {
public:
	float width;
	float radius;
	float k;
	float d;
	float angularInertia;
	float thermalFrictionK;
	float thermalRollingK;
	float thermalRollingSurfaceK;
	float grainThreshold;
	float blisterThreshold;
	float grainGamma;
	float blisterGamma;
	float grainGain;
	float blisterGain;
	float rimRadius;
	float optimumTemp;
	float softnessIndex;
	float radiusRaiseK;
	inline TyreData()  { }
};

struct TyrePatchData {
public:
	float surfaceTransfer;
	float patchTransfer;
	float patchCoreTransfer;
	float internalCoreTransfer;
	float coolFactorGain;
	inline TyrePatchData()  { }
};

struct ksgui_GUI_FormData {
public:
	float x;
	float y;
	bool visible;
	bool blocked;
	float scale;
	inline ksgui_GUI_FormData()  { }
};

struct TyreStatus {
public:
	float depth;
	float load;
	float camberRAD;
	float slipAngleRAD;
	float slipRatio;
	float angularVelocity;
	float Fy;
	float Fx;
	float Mz;
	bool isLocked;
	float slipFactor;
	float ndSlip;
	float distToGround;
	float Dy;
	float Dx;
	float D;
	float dirtyLevel;
	float rollingResistence;
	float thermalInput;
	float feedbackTorque;
	float loadedRadius;
	float effectiveRadius;
	float liveRadius;
	float pressureStatic;
	float pressureDynamic;
	double virtualKM;
	float lastTempIMO[0x3];
	float peakSA;
	double grain;
	double blister;
	float inflation;
	double flatSpot;
	float lastGrain;
	float lastBlister;
	float normalizedSlideX;
	float normalizedSlideY;
	float finalDY;
	float wearMult;
	inline TyreStatus()  { }
};

struct ACCarState {
public:
	float wheelLF_localPos[0x3];
	float wheelRF_localPos[0x3];
	float wheelLR_localPos[0x3];
	float wheelRR_localPos[0x3];
	float localVelocity[0x3];
	float worldVelocity[0x3];
	float accG[0x3];
	float engineRPMS;
	float worldPosition[0x3];
	float bodyMatrix[0x10];
	int gear;
	bool isEngineLimiterOn;
	float wheelAngularSpeed[0x4];
	float steer;
	float gas;
	float brake;
	float clutch;
	float localAngularVelocity[0x3];
	float ndSlip[0x4];
	float load[0x4];
	float Mz[0x4];
	float tyreDirtyLevel[0x4];
	float lastFF;
	float drivetrainSpeed;
	float turboBoost;
	float performanceMeter;
	bool isGearGrinding;
	float damageZoneLevel[0x5];
	int limiterRPM;
	float speedMS;
	inline ACCarState()  { }
};

struct TyreInputs {
public:
	float brakeTorque;
	float handBrakeTorque;
	float electricTorque;
	inline TyreInputs()  { }
};

struct TyreThermalState {
public:
	float temps[0x3][0xC];
	float coreTemp;
	float thermalInput;
	float dynamicPressure;
	float staticPressure;
	float lastSetIMO[0x3];
	float cpTemperature;
	float lastGrain;
	float lastBlister;
	float mult;
	bool isHot;
	inline TyreThermalState()  { }
};

struct ClientQOSData {
public:
	bool usingMegapackets;
	int counter;
	double startTime;
	int lastQOS;
	inline ClientQOSData()  { }
};

struct ACPluginContext {
public:
	inline ACPluginContext()  { }
};

struct SplineLocationData {
public:
	int currentIndex;
	inline SplineLocationData()  { }
};

struct AWD2Data {
public:
	double ramp;
	double maxTorque;
	float currentLockTorque;
	inline AWD2Data()  { }
};

struct PushToPass {
public:
	bool enabled;
	bool active;
	float overboost;
	float timeS;
	float coolDownS;
	float timeAccum;
	int activations;
	float baseWastegate;
	int baseActivations;
	int basePositionCoeff;
	int maxActivations;
	inline PushToPass()  { }
};

struct VideoSettings {
public:
	int aaSamples;
	int width;
	int height;
	void * hWnd;
	bool isFullscreen;
	bool vSync;
	int anisotropic;
	int aaQuality;
	int shadowMapSize;
	double fpsCapMS;
	int dxgiModeIndex;
	int worldDetail;
	bool ppHDREnabled;
	bool ppHeatShimmer;
	bool ppFXAA;
	bool ppRaysOfGod;
	int ppQuality;
	int ppGlare;
	int ppDof;
	bool tripleBuffer;
	float refresh;
	inline VideoSettings()  { }
};

struct CoastSettings {
public:
	float coast1;
	float coast2;
	inline CoastSettings()  { }
};

struct OnRaceInitEvent {
public:
	int laps;
	inline OnRaceInitEvent()  { }
};

struct GameStats {
public:
	double cpuTime;
	double updateTime;
	double renderHUDTime;
	double renderTime;
	double renderAudioTime;
	inline GameStats()  { }
};

struct SACEngineInput {
public:
	float gasInput;
	float carSpeed;
	float altitude;
	float rpm;
	inline SACEngineInput()  { }
};

struct TurboDef {
public:
	float maxBoost;
	float lagUP;
	float lagDN;
	float rpmRef;
	float gamma;
	float wastegate;
	bool isAdjustable;
	inline TurboDef()  { }
};

struct DownshiftProtection {
public:
	bool isActive;
	bool isDebug;
	int overrev;
	bool lockN;
	inline DownshiftProtection()  { }
};

struct WreckerProtection {
public:
	float maxContactsPerKM;
	int warningCount;
	int contacts;
	bool blackListRequested;
	inline WreckerProtection()  { }
};

struct OnKeyCharEvent {
public:
	unsigned int key;
	inline OnKeyCharEvent()  { }
};

struct OnKeyEvent {
public:
	unsigned int keyCode;
	inline OnKeyEvent()  { }
};

struct DamageReportDef {
public:
	double lastSendTime;
	float damageZoneLevel[0x5];
	inline DamageReportDef()  { }
};

struct GearElement {
public:
	double velocity;
	double inertia;
	double oldVelocity;
	inline GearElement()  { }
};

class CarControls {
public:
	bool gearUp;
	bool gearDn;
	bool drs;
	bool kers;
	bool brakeBalanceUp;
	bool brakeBalanceDn;
	int requestedGearIndex;
	bool isShifterSupported;
	float handBrake;
	bool absUp;
	bool absDn;
	bool tcUp;
	bool tcDn;
	bool turboUp;
	bool turboDn;
	bool engineBrakeUp;
	bool engineBrakeDn;
	bool MGUKDeliveryUp;
	bool MGUKDeliveryDn;
	bool MGUKRecoveryUp;
	bool MGUKRecoveryDn;
	bool MGUHMode;
	float gas;
	float brake;
	float steer;
	float clutch;
	inline CarControls()  { }
};

struct SplineIndexBound {
public:
	unsigned int minIndex;
	unsigned int maxIndex;
	inline SplineIndexBound()  { }
};

struct NetCarQoS {
public:
	int goodPackets;
	int badPackets;
	inline NetCarQoS()  { }
};

struct PerformancePair {
public:
	unsigned int t;
	float speedMS;
	inline PerformancePair()  { }
};

struct SCarStateAero {
public:
	float CD;
	float CL_Front;
	float CL_Rear;
	inline SCarStateAero()  { }
};

struct SplineLocatorData {
public:
	float npos;
	unsigned int currentIndex;
	float lateralOffset;
	float splineLength;
	float sides[0x2];
	float sidesFromIL[0x2];
	float sideVelocity;
	bool isOutsideTrackLimits;
	inline SplineLocatorData()  { }
};

struct KPI {
public:
	float angleRAD;
	float scrubRadius;
	inline KPI()  { }
};

struct KGLShaderCBufferDesc {
public:
	wchar_t * name;
	unsigned int size;
	unsigned int slot;
	inline KGLShaderCBufferDesc()  { }
};

class Speed {
public:
	float value;
	inline Speed()  { }
	inline void ctor(float v) { typedef void (*_fpt)(Speed *pthis, float); _fpt _f=(_fpt)_drva(2333072); _f(this, v); }
	inline void ctor() { typedef void (*_fpt)(Speed *pthis); _fpt _f=(_fpt)_drva(2333024); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Speed *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline float kmh() { typedef float (*_fpt)(Speed *pthis); _fpt _f=(_fpt)_drva(364368); return _f(this); }
	inline Speed operator*(float v2) { typedef Speed (*_fpt)(Speed *pthis, float); _fpt _f=(_fpt)_drva(2502224); return _f(this, v2); }
};

struct WheelValues {
public:
	float lf;
	float rf;
	float lr;
	float rr;
	inline WheelValues()  { }
};

struct AccelerationProfile {
public:
	float zero;
	float maxTyres;
	inline AccelerationProfile()  { }
};

struct NetCarStateProvider_LagDebug {
public:
	double rcvTime;
	double physicsTime;
	bool wasLagging;
	inline NetCarStateProvider_LagDebug()  { }
};

struct SamplerStates {
public:
	void * samplerAniso;
	void * samplerLinearShadow;
	void * samplerShadow;
	void * samplerPoint;
	void * samplerPointClamp;
	void * samplerLinearSimple;
	void * samplerLinearClamp;
	inline SamplerStates()  { }
};

class ICoastGenerator {
public:
	inline ICoastGenerator()  { }
	virtual ~ICoastGenerator();
	inline void dtor() { typedef void (*_fpt)(ICoastGenerator *pthis); _fpt _f=(_fpt)_drva(2693184); _f(this); }
	virtual float getCoastTorque_vf1() = 0;
	inline float getCoastTorque() { return getCoastTorque_vf1(); }
};

struct ERSCockpitControls {
public:
	bool recovery;
	bool mguHMode;
	bool deliveryProfile;
	inline ERSCockpitControls()  { }
};

struct DRSWingSetting {
public:
	int index;
	float angle;
	inline DRSWingSetting()  { }
};

struct ERSStatus {
public:
	float kineticRecovery;
	float heatRecovery;
	inline ERSStatus()  { }
};

struct OnWindowResize {
public:
	int width;
	int height;
	inline OnWindowResize()  { }
};

struct TyreSlipOutput {
public:
	float normalizedForce;
	float slip;
	inline TyreSlipOutput()  { }
};

class GameTime {
public:
	double now;
	float deltaT;
	float smoothDeltaT;
	float fps;
	bool useStabilizer;
	double cappedFPS;
	double startTime;
	bool m_isUsingHRT;
	inline GameTime()  { }
	inline void ctor() { typedef void (*_fpt)(GameTime *pthis); _fpt _f=(_fpt)_drva(4506064); _f(this); }
	virtual ~GameTime();
	inline void dtor() { typedef void (*_fpt)(GameTime *pthis); _fpt _f=(_fpt)_drva(4506128); _f(this); }
	inline void update() { typedef void (*_fpt)(GameTime *pthis); _fpt _f=(_fpt)_drva(4506192); return _f(this); }
};

struct CoreCPUTimes {
public:
	double solverTime;
	double collisionTime;
	int contactPoints;
	int narrowPhaseTests;
	inline CoreCPUTimes()  { }
};

struct SteerMzLowSpeedReduction {
public:
	float speedKMH;
	float minValue;
	inline SteerMzLowSpeedReduction()  { }
};

class ITorqueGenerator {
public:
	inline ITorqueGenerator()  { }
	virtual ~ITorqueGenerator();
	inline void dtor() { typedef void (*_fpt)(ITorqueGenerator *pthis); _fpt _f=(_fpt)_drva(2549840); _f(this); }
	virtual float getOutputTorque_vf1() = 0;
	inline float getOutputTorque() { return getOutputTorque_vf1(); }
};

struct WindSettings {
public:
	float baseSpeed;
	float baseDirection;
	inline WindSettings()  { }
};

struct EngineStatus {
public:
	double outTorque;
	double externalCoastTorque;
	float turboBoost;
	bool isLimiterOn;
	inline EngineStatus()  { }
};

struct RendererFlags {
public:
	int maxFrameLatency;
	float mipLodBias;
	inline RendererFlags()  { }
};

struct RenderStats {
public:
	int dipCalls;
	int sceneDipCalls;
	int triangles;
	int sceneTriangles;
	bool isInMainRenderPass;
	inline RenderStats()  { }
};

struct KGLShaderVarDesc {
public:
	wchar_t * name;
	wchar_t * cBufferName;
	unsigned int cBufferSlot;
	unsigned int size;
	unsigned int offset;
	inline KGLShaderVarDesc()  { }
};

struct PitStopTimings {
public:
	float tyreChangeTimeSec;
	float fuelChangeTimeSec;
	float bodyRepairTimeSec;
	float engineRepairTimeSec;
	float suspRepairTimeSec;
	inline PitStopTimings()  { }
};

struct ServerDrivingAssists {
public:
	int tc;
	int abs;
	bool stability;
	bool autoClutch;
	inline ServerDrivingAssists()  { }
};

class KGLVertexBuffer {
public:
	ID3D11Buffer * buffer;
	unsigned int stride;
	inline KGLVertexBuffer()  { }
	inline void ctor(ID3D11Device * device, unsigned int size, unsigned int stride, void * data, bool isDynamic) { typedef void (*_fpt)(KGLVertexBuffer *pthis, ID3D11Device *, unsigned int, unsigned int, void *, bool); _fpt _f=(_fpt)_drva(146752); _f(this, device, size, stride, data, isDynamic); }
	inline void map(void * data, unsigned int size, ID3D11DeviceContext * context) { typedef void (*_fpt)(KGLVertexBuffer *pthis, void *, unsigned int, ID3D11DeviceContext *); _fpt _f=(_fpt)_drva(146992); return _f(this, data, size, context); }
	inline void mapNoOverWrite(void * data, unsigned int offset, unsigned int size, ID3D11DeviceContext * context) { typedef void (*_fpt)(KGLVertexBuffer *pthis, void *, unsigned int, unsigned int, ID3D11DeviceContext *); _fpt _f=(_fpt)_drva(147136); return _f(this, data, offset, size, context); }
};

struct NetCarPushToPass {
public:
	bool enabled;
	bool active;
	float coolDownS;
	float timeS;
	float timeAccum;
	int activations;
	inline NetCarPushToPass()  { }
};

struct KGLShaderTextureDesc {
public:
	wchar_t * name;
	unsigned int slot;
	inline KGLShaderTextureDesc()  { }
};

class KGLIndexBuffer {
public:
	ID3D11Buffer * buffer;
	inline KGLIndexBuffer()  { }
	inline void ctor(ID3D11Device * device, unsigned int size, unsigned short * data) { typedef void (*_fpt)(KGLIndexBuffer *pthis, ID3D11Device *, unsigned int, unsigned short *); _fpt _f=(_fpt)_drva(147296); _f(this, device, size, data); }
};

struct ksgui_ksRect {
public:
	float left;
	float right;
	float top;
	float bottom;
	inline ksgui_ksRect()  { }
	inline float getWidth() { typedef float (*_fpt)(ksgui_ksRect *pthis); _fpt _f=(_fpt)_drva(694320); return _f(this); }
	inline float getHeight() { typedef float (*_fpt)(ksgui_ksRect *pthis); _fpt _f=(_fpt)_drva(220096); return _f(this); }
};

struct PitStopTime {
public:
	float total;
	float tyres;
	float repair;
	float fuel;
	inline PitStopTime()  { }
};

class Damper {
public:
	float reboundSlow;
	float reboundFast;
	float bumpSlow;
	float bumpFast;
	float fastThresholdBump;
	float fastThresholdRebound;
	inline Damper()  { }
	inline void ctor() { typedef void (*_fpt)(Damper *pthis); _fpt _f=(_fpt)_drva(2830928); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Damper *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline float getForce(float v) { typedef float (*_fpt)(Damper *pthis, float); _fpt _f=(_fpt)_drva(2830976); return _f(this, v); }
};

struct TyreSlipInput {
public:
	float slip;
	float friction;
	float load;
	float normalizedSlipX;
	float normalizedSlipY;
	float D;
	inline TyreSlipInput()  { }
};

struct PerformanceSplit {
public:
	double t;
	float speedMS;
	inline PerformanceSplit()  { }
};

struct CommandItem {
public:
	int key;
	inline CommandItem()  { }
	inline void ctor(int akey) { typedef void (*_fpt)(CommandItem *pthis, int); _fpt _f=(_fpt)_drva(953184); _f(this, akey); }
};

class vec2f {
public:
	float x;
	float y;
	inline vec2f()  { }
	inline void ctor(float ix, float iy) { typedef void (*_fpt)(vec2f *pthis, float, float); _fpt _f=(_fpt)_drva(216944); _f(this, ix, iy); }
};

struct HeaveSpringStatus {
public:
	float travel;
	inline HeaveSpringStatus()  { }
};

struct ClientRules {
public:
	float maxMetersWrongWay;
	inline ClientRules()  { }
};

struct HDRLevels {
public:
	float minExposure;
	float maxExposure;
	inline HDRLevels()  { }
};

class CBuffer {
public:
	int size;
	int slot;
	bool isSystem;
	bool isPS;
	bool isVS;
	void * kid;
	unsigned char * data;
	bool touched;
	inline CBuffer()  { }
	inline void ctor() { typedef void (*_fpt)(CBuffer *pthis); _fpt _f=(_fpt)_drva(2201856); _f(this); }
	inline void ctor(int islot, int isize) { typedef void (*_fpt)(CBuffer *pthis, int, int); _fpt _f=(_fpt)_drva(2201744); _f(this, islot, isize); }
	inline void dtor() { typedef void (*_fpt)(CBuffer *pthis); _fpt _f=(_fpt)_drva(2202128); _f(this); }
	inline void set(int * value, int offset, int size) { typedef void (*_fpt)(CBuffer *pthis, int *, int, int); _fpt _f=(_fpt)_drva(2202192); return _f(this, value, offset, size); }
	inline void set(float * value, int offset, int size) { typedef void (*_fpt)(CBuffer *pthis, float *, int, int); _fpt _f=(_fpt)_drva(2202192); return _f(this, value, offset, size); }
	inline void get(float * value, int offset, int size) { typedef void (*_fpt)(CBuffer *pthis, float *, int, int); _fpt _f=(_fpt)_drva(2201968); return _f(this, value, offset, size); }
	inline void commit() { typedef void (*_fpt)(CBuffer *pthis); _fpt _f=(_fpt)_drva(2201888); return _f(this); }
	inline void touch() { typedef void (*_fpt)(CBuffer *pthis); _fpt _f=(_fpt)_drva(2202240); return _f(this); }
	inline void map(void * bdata, int bsize) { typedef void (*_fpt)(CBuffer *pthis, void *, int); _fpt _f=(_fpt)_drva(2202096); return _f(this, bdata, bsize); }
	inline void init(int islot, int isize) { typedef void (*_fpt)(CBuffer *pthis, int, int); _fpt _f=(_fpt)_drva(2202000); return _f(this, islot, isize); }
	inline void release() { typedef void (*_fpt)(CBuffer *pthis); _fpt _f=(_fpt)_drva(2202128); return _f(this); }
};

struct TyreModelInput {
public:
	float load;
	float slipAngleRAD;
	float slipRatio;
	float camberRAD;
	float speed;
	float u;
	int tyreIndex;
	float cpLength;
	float grain;
	float blister;
	float pressureRatio;
	bool useSimpleModel;
	inline TyreModelInput()  { }
};

class Trigger {
public:
	bool state;
	bool lastState;
	float accumulator;
	float accumulatorLimit;
	inline Trigger()  { }
	inline void ctor() { typedef void (*_fpt)(Trigger *pthis); _fpt _f=(_fpt)_drva(2339728); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Trigger *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline bool ignoreSubsequentTrue(bool value) { typedef bool (*_fpt)(Trigger *pthis, bool); _fpt _f=(_fpt)_drva(2339744); return _f(this, value); }
	inline bool keepSteady(float dt, bool value) { typedef bool (*_fpt)(Trigger *pthis, float, bool); _fpt _f=(_fpt)_drva(2339776); return _f(this, dt, value); }
};

struct WingOverrideDef {
public:
	float overrideAngle;
	bool isActive;
	inline WingOverrideDef()  { }
};

struct SurfaceDef {
public:
	wchar_t wavString[0x40];
	float wavPitchSpeed;
	void * userPointer;
	float gripMod;
	int sectorID;
	float dirtAdditiveK;
	unsigned int collisionCategory;
	bool isValidTrack;
	float blackFlagTime;
	float sinHeight;
	float sinLength;
	bool isPitlane;
	float damping;
	float granularity;
	float vibrationGain;
	float vibrationLength;
	inline SurfaceDef()  { }
};

struct TyreModelOutput {
public:
	float Fy;
	float Fx;
	float Mz;
	float trail;
	float ndSlip;
	float Dy;
	float Dx;
	inline TyreModelOutput()  { }
};

class SignalGenerator {
public:
	float freqScale;
	int value;
	inline SignalGenerator()  { }
	inline void ctor() { typedef void (*_fpt)(SignalGenerator *pthis); _fpt _f=(_fpt)_drva(2282048); _f(this); }
	virtual ~SignalGenerator();
	inline void dtor() { typedef void (*_fpt)(SignalGenerator *pthis); _fpt _f=(_fpt)_drva(2282080); _f(this); }
	virtual void step_vf1(float dt);
	inline void step_impl(float dt) { typedef void (*_fpt)(SignalGenerator *pthis, float); _fpt _f=(_fpt)_drva(2282240); return _f(this, dt); }
	inline void step(float dt) { return step_vf1(dt); }
	virtual float getValue_vf2() = 0;
	inline float getValue() { return getValue_vf2(); }
};

struct TyreExternalInputs {
public:
	bool isActive;
	float load;
	float slipAngle;
	float slipRatio;
	inline TyreExternalInputs()  { }
};

struct BrushOutput {
public:
	float force;
	float slip;
	inline BrushOutput()  { }
};

struct OnGearRequestEvent {
public:
	GearChangeRequest request;
	int nextGear;
	inline OnGearRequestEvent()  { }
};

struct OnWindowClosedEvent {
public:
	RenderWindow * renderWindow;
	inline OnWindowClosedEvent()  { }
};

struct SGearRatio {
public:
	double ratio;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	inline SGearRatio()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, double iratio) { typedef void (*_fpt)(SGearRatio *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, double); _fpt _f=(_fpt)_drva(2514800); _f(this, iname, iratio); }
	inline void dtor() { typedef void (*_fpt)(SGearRatio *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
};

struct ClientRemoteCarDef {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > model;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > config;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > skin;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > driverName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > team;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > nationCode;
	unsigned char sessionID;
	float damageZoneLevel[0x5];
	inline ClientRemoteCarDef()  { }
	inline void ctor(ClientRemoteCarDef & __that) { typedef void (*_fpt)(ClientRemoteCarDef *pthis, ClientRemoteCarDef &); _fpt _f=(_fpt)_drva(241776); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(ClientRemoteCarDef *pthis); _fpt _f=(_fpt)_drva(246128); _f(this); }
};

struct OnTyreCompoundChanged {
public:
	int tyreIndex;
	int compoundIndex;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > compoundName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > shortName;
	inline OnTyreCompoundChanged()  { }
	inline void dtor() { typedef void (*_fpt)(OnTyreCompoundChanged *pthis); _fpt _f=(_fpt)_drva(847584); _f(this); }
};

struct ClientHandshakeResult {
public:
	bool success;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > model;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > skin;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > track;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > track_config;
	float sunAngle;
	unsigned char sessionID;
	inline ClientHandshakeResult()  { }
	inline void ctor(ClientHandshakeResult & __that) { typedef void (*_fpt)(ClientHandshakeResult *pthis, ClientHandshakeResult &); _fpt _f=(_fpt)_drva(241536); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(ClientHandshakeResult *pthis); _fpt _f=(_fpt)_drva(245968); _f(this); }
};

struct LeaderboardEntry {
public:
	CarAvatar * car;
	double totalTime;
	double bestLap;
	int laps;
	bool isRaceMode;
	bool isBlackFlagged;
	bool hasCompletedLastLap;
	inline LeaderboardEntry()  { }
	inline bool operator<(LeaderboardEntry & l) { typedef bool (*_fpt)(LeaderboardEntry *pthis, LeaderboardEntry &); _fpt _f=(_fpt)_drva(1327408); return _f(this, l); }
};

struct AutoShifter {
public:
	bool isActive;
	int changeUpRpm;
	int changeDnRpm;
	float slipThreshold;
	Car * car;
	bool butGearUp;
	bool butGearDn;
	float gasCutoff;
	float gasCutoffTime;
	inline AutoShifter()  { }
	inline void dtor() { typedef void (*_fpt)(AutoShifter *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(AutoShifter *pthis, Car *); _fpt _f=(_fpt)_drva(2859040); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(AutoShifter *pthis, float); _fpt _f=(_fpt)_drva(2861040); return _f(this, dt); }
	inline void loadINI() { typedef void (*_fpt)(AutoShifter *pthis); _fpt _f=(_fpt)_drva(2859088); return _f(this); }
};

struct TyreThermalPatch {
public:
	std::vector<TyreThermalPatch *,std::allocator<TyreThermalPatch *> > connections;
	float T;
	float inputT;
	int elementIndex;
	int stripeIndex;
	inline TyreThermalPatch()  { }
};

class vec3f {
public:
	float x;
	float y;
	float z;
	inline vec3f()  { }
	inline void ctor(float ix, float iy, float iz) { typedef void (*_fpt)(vec3f *pthis, float, float, float); _fpt _f=(_fpt)_drva(147424); _f(this, ix, iy, iz); }
	inline void normalize() { typedef void (*_fpt)(vec3f *pthis); _fpt _f=(_fpt)_drva(147456); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > toString() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(vec3f *pthis); _fpt _f=(_fpt)_drva(340976); return _f(this); }
	inline bool isFinite() { typedef bool (*_fpt)(vec3f *pthis); _fpt _f=(_fpt)_drva(418800); return _f(this); }
	inline void operator/=(float m) { typedef void (*_fpt)(vec3f *pthis, float); _fpt _f=(_fpt)_drva(908800); return _f(this, m); }
	inline void print(char * name) { typedef void (*_fpt)(vec3f *pthis, char *); _fpt _f=(_fpt)_drva(918176); return _f(this, name); }
};

struct sockaddr_in {
public:
	unsigned short sin_family;
	unsigned short sin_port;
	in_addr sin_addr;
	char sin_zero[0x8];
	inline sockaddr_in()  { }
};

class IndexBuffer {
public:
	void * kid;
	inline IndexBuffer()  { }
	inline void ctor(std::vector<unsigned short,std::allocator<unsigned short> > & indices) { typedef void (*_fpt)(IndexBuffer *pthis, std::vector<unsigned short,std::allocator<unsigned short> > &); _fpt _f=(_fpt)_drva(2258768); _f(this, indices); }
	virtual ~IndexBuffer();
};

struct DisconnectCountdown {
public:
	bool isSignaled;
	float timeToDisconnection;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > message;
	inline DisconnectCountdown()  { }
	inline void dtor() { typedef void (*_fpt)(DisconnectCountdown *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
};

class Task {
public:
	bool isDone;
	std::function<void __cdecl(void)> function;
	inline Task()  { }
	inline void dtor() { typedef void (*_fpt)(Task *pthis); _fpt _f=(_fpt)_drva(175552); _f(this); }
};

struct EDL {
public:
	bool isPresent;
	bool isActive;
	float wheelSpeedGainPower;
	float wheelSpeedGainCoast;
	float deadZonePower;
	float deadZoneCoast;
	float brakeTorquePower;
	float brakeTorqueCoast;
	float outLevel;
	float outBrakeTorque;
	float speedDiff;
	Car * car;
	int leftTyreIndex;
	int rightTyreIndex;
	inline EDL()  { }
	inline void dtor() { typedef void (*_fpt)(EDL *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void step(float td) { typedef void (*_fpt)(EDL *pthis, float); _fpt _f=(_fpt)_drva(2864224); return _f(this, td); }
	inline void init(Car * a_car) { typedef void (*_fpt)(EDL *pthis, Car *); _fpt _f=(_fpt)_drva(2862064); return _f(this, a_car); }
};

struct OnStepCompleteEvent {
public:
	Car * car;
	double physicsTime;
	inline OnStepCompleteEvent()  { }
};

class TCPQueue {
public:
	unsigned char buffer[0x10000];
	unsigned int cursor;
	inline TCPQueue()  { }
	inline void push(unsigned char * data, unsigned int size) { typedef void (*_fpt)(TCPQueue *pthis, unsigned char *, unsigned int); _fpt _f=(_fpt)_drva(2482992); return _f(this, data, size); }
	inline std::vector<unsigned char,std::allocator<unsigned char> > getPacket() { typedef std::vector<unsigned char,std::allocator<unsigned char> > (*_fpt)(TCPQueue *pthis); _fpt _f=(_fpt)_drva(2482752); return _f(this); }
};

struct PenaltyRules {
public:
	JumpStartPenaltyMode jumpStartPenaltyMode;
	short basePitPenaltyLaps;
	inline PenaltyRules()  { }
};

struct RealTimeCarDesc {
public:
	CarAvatar * car;
	bool crossedForTheFirstTime;
	float approxPos;
	inline RealTimeCarDesc()  { }
};

struct MultiplayerStatus {
public:
	std::vector<bool,std::allocator<bool> > completedFlags;
	inline MultiplayerStatus()  { }
	inline void dtor() { typedef void (*_fpt)(MultiplayerStatus *pthis); _fpt _f=(_fpt)_drva(1258160); _f(this); }
};

class ShaderResource {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	int slot;
	inline ShaderResource()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, int islot) { typedef void (*_fpt)(ShaderResource *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, int); _fpt _f=(_fpt)_drva(2282272); _f(this, iname, islot); }
	virtual ~ShaderResource();
};

struct KGLShaderTexture {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	int slot;
	inline KGLShaderTexture()  { }
	inline void dtor() { typedef void (*_fpt)(KGLShaderTexture *pthis); _fpt _f=(_fpt)_drva(113184); _f(this); }
};

struct GearRequestStatus {
public:
	GearChangeRequest request;
	double timeAccumulator;
	double timeout;
	int requestedGear;
	inline GearRequestStatus()  { }
};

struct ksgui_OnSpinnerValueChanged {
public:
	ksgui_Spinner * spinner;
	int value;
	inline ksgui_OnSpinnerValueChanged()  { }
};

struct ksgui_OnControlClicked {
public:
	ksgui_Control * control;
	int localx;
	int localy;
	inline ksgui_OnControlClicked()  { }
};

struct ksgui_OnCheckBoxChanged {
public:
	ksgui_CheckBox * checkBox;
	bool value;
	inline ksgui_OnCheckBoxChanged()  { }
};

struct OnWindowResizeEvent {
public:
	int width;
	int height;
	RenderWindow * renderWindow;
	inline OnWindowResizeEvent()  { }
};

class FileChangeObserver {
public:
	_FILETIME lastFileTime;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > filename;
	unsigned long lastChanged;
	inline FileChangeObserver()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & a_filename) { typedef void (*_fpt)(FileChangeObserver *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2339840); _f(this, a_filename); }
	inline void dtor() { typedef void (*_fpt)(FileChangeObserver *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
	inline bool hasChanged() { typedef bool (*_fpt)(FileChangeObserver *pthis); _fpt _f=(_fpt)_drva(2339904); return _f(this); }
	inline void reset() { typedef void (*_fpt)(FileChangeObserver *pthis); _fpt _f=(_fpt)_drva(2340144); return _f(this); }
	inline void observe(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & a_filename) { typedef void (*_fpt)(FileChangeObserver *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2339952); return _f(this, a_filename); }
};

struct ServerInfo {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > ip;
	unsigned short httpPort;
	unsigned short udpPort;
	unsigned short tcpPort;
	inline ServerInfo()  { }
	inline void ctor() { typedef void (*_fpt)(ServerInfo *pthis); _fpt _f=(_fpt)_drva(243344); _f(this); }
	inline void dtor() { typedef void (*_fpt)(ServerInfo *pthis); _fpt _f=(_fpt)_drva(776352); _f(this); }
};

struct DriverInfo {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > team;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > nationality;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > nationCode;
	inline DriverInfo()  { }
	inline void ctor(DriverInfo & __that) { typedef void (*_fpt)(DriverInfo *pthis, DriverInfo &); _fpt _f=(_fpt)_drva(242096); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(DriverInfo *pthis); _fpt _f=(_fpt)_drva(242288); _f(this); }
	inline void dtor() { typedef void (*_fpt)(DriverInfo *pthis); _fpt _f=(_fpt)_drva(246464); _f(this); }
	inline DriverInfo & operator=(DriverInfo & __that) { typedef DriverInfo & (*_fpt)(DriverInfo *pthis, DriverInfo &); _fpt _f=(_fpt)_drva(848192); return _f(this, __that); }
};

struct ksgui_OnScrollBarValueChanged {
public:
	ksgui_ScrollBar * scrollBar;
	int value;
	inline ksgui_OnScrollBarValueChanged()  { }
};

struct ksgui_OnCutExtremesChanged {
public:
	ksgui_Slider * slider;
	float cutIn;
	float cutOut;
	inline ksgui_OnCutExtremesChanged()  { }
};

struct RaceStatusCarDesc {
public:
	CarAvatar * car;
	float resetTimer;
	bool retired;
	float invalidStateTimer;
	float maxInvalidStateTimer;
	inline RaceStatusCarDesc()  { }
};

struct ReceivedVoteDef {
public:
	VoteType voteType;
	int quorum;
	int votes;
	int timeLeftMS;
	int lastVoterSessionID;
	int lastVoterVote;
	unsigned char targetSessionID;
	bool isMe;
	inline ReceivedVoteDef()  { }
	inline void ctor() { typedef void (*_fpt)(ReceivedVoteDef *pthis); _fpt _f=(_fpt)_drva(242528); _f(this); }
};

struct GearChanger {
public:
	bool wasGearUpTriggered;
	bool wasGearDnTriggered;
	Car * car;
	bool lastGearUp;
	bool lastGearDn;
	inline GearChanger()  { }
	inline void dtor() { typedef void (*_fpt)(GearChanger *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(GearChanger *pthis, Car *); _fpt _f=(_fpt)_drva(2861888); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(GearChanger *pthis, float); _fpt _f=(_fpt)_drva(2861904); return _f(this, dt); }
};

struct StabilityControl {
public:
	float gain;
	bool useBeta;
	Car * car;
	float maxGain;
	inline StabilityControl()  { }
	inline void dtor() { typedef void (*_fpt)(StabilityControl *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(StabilityControl *pthis, Car *); _fpt _f=(_fpt)_drva(2882096); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(StabilityControl *pthis, float); _fpt _f=(_fpt)_drva(2882128); return _f(this, dt); }
};

struct OnLapCompletedEvent {
public:
	unsigned int carIndex;
	unsigned int lapTime;
	unsigned int lapCount;
	std::vector<unsigned int,std::allocator<unsigned int> > splits;
	double eventTime;
	bool isValid;
	int cuts;
	inline OnLapCompletedEvent()  { }
	inline void dtor() { typedef void (*_fpt)(OnLapCompletedEvent *pthis); _fpt _f=(_fpt)_drva(246720); _f(this); }
};

struct OnChatMessageEvent {
public:
	int sessionID;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > message;
	inline OnChatMessageEvent()  { }
	inline void dtor() { typedef void (*_fpt)(OnChatMessageEvent *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
};

struct PenaltyRecord {
public:
	unsigned int lap;
	unsigned int seconds;
	PenaltyDescription descr;
	inline PenaltyRecord()  { }
};

struct DriftModeComponent {
public:
	Car * car;
	float driftStraightTimer;
	double points;
	double instantDrift;
	double currentSpeedMultiplier;
	double currentDriftAngle;
	int comboCounter;
	int lastDriftDirection;
	bool extremeDrifting;
	bool drifting;
	bool invalid;
	float oldDamageZones[0x5];
	inline DriftModeComponent()  { }
	inline void dtor() { typedef void (*_fpt)(DriftModeComponent *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(DriftModeComponent *pthis, Car *); _fpt _f=(_fpt)_drva(2882976); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(DriftModeComponent *pthis, float); _fpt _f=(_fpt)_drva(2883360); return _f(this, dt); }
	inline bool checkExtremeDrifting() { typedef bool (*_fpt)(DriftModeComponent *pthis); _fpt _f=(_fpt)_drva(2882640); return _f(this); }
	inline void validateDrift() { typedef void (*_fpt)(DriftModeComponent *pthis); _fpt _f=(_fpt)_drva(2884208); return _f(this); }
};

struct OnReplayStatusChanged {
public:
	eReplayStatus status;
	float timeMult;
	float slowMotionLevel;
	inline OnReplayStatusChanged()  { }
};

struct LapInvalidator {
public:
	double collisionSafeTime;
	Car * car;
	int currentTyresOut;
	bool isInPenaltyZone;
	float lastBlackFlagTime;
	inline LapInvalidator()  { }
	inline void dtor() { typedef void (*_fpt)(LapInvalidator *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(LapInvalidator *pthis, Car *); _fpt _f=(_fpt)_drva(2865408); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(LapInvalidator *pthis, float); _fpt _f=(_fpt)_drva(2884992); return _f(this, dt); }
	inline void onEnterPenaltyZone(int tyre_count, float black_flag_time) { typedef void (*_fpt)(LapInvalidator *pthis, int, float); _fpt _f=(_fpt)_drva(2884560); return _f(this, tyre_count, black_flag_time); }
};

struct FuelLapEvaluator {
public:
	Car * car;
	int lapCount;
	double lapStartFuel;
	double fuelPerLap;
	double totalM;
	double totalLiters;
	double startFuel;
	double oldFuelPerLap;
	bool ignoreLap;
	inline FuelLapEvaluator()  { }
	inline void dtor() { typedef void (*_fpt)(FuelLapEvaluator *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline float getFuelLaps() { typedef float (*_fpt)(FuelLapEvaluator *pthis); _fpt _f=(_fpt)_drva(2675296); return _f(this); }
	inline void setIgnoreLap(bool value) { typedef void (*_fpt)(FuelLapEvaluator *pthis, bool); _fpt _f=(_fpt)_drva(2675536); return _f(this, value); }
	inline void init(Car * icar) { typedef void (*_fpt)(FuelLapEvaluator *pthis, Car *); _fpt _f=(_fpt)_drva(2675376); return _f(this, icar); }
	inline float getFuelPerLap() { typedef float (*_fpt)(FuelLapEvaluator *pthis); _fpt _f=(_fpt)_drva(2675344); return _f(this); }
	inline void step(float dt) { typedef void (*_fpt)(FuelLapEvaluator *pthis, float); _fpt _f=(_fpt)_drva(2675552); return _f(this, dt); }
};

struct TimeLineStatus {
public:
	bool isValid;
	eTimeLineCheckResponse lastResponse;
	unsigned int lastTime;
	inline TimeLineStatus()  { }
};

struct SpeedLimiter {
public:
	bool shoudLimit;
	bool isLimiting;
	Car * car;
	inline SpeedLimiter()  { }
	inline void dtor() { typedef void (*_fpt)(SpeedLimiter *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(SpeedLimiter *pthis, Car *); _fpt _f=(_fpt)_drva(2865408); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(SpeedLimiter *pthis, float); _fpt _f=(_fpt)_drva(2865424); return _f(this, dt); }
};

struct DifferentialSetting {
public:
	float power;
	float coast;
	float preload;
	DifferentialType type;
	inline DifferentialSetting()  { }
};

struct KGLShaderVar {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > cBufferName;
	unsigned int cBufferSlot;
	unsigned int size;
	unsigned int offset;
	inline KGLShaderVar()  { }
	inline void dtor() { typedef void (*_fpt)(KGLShaderVar *pthis); _fpt _f=(_fpt)_drva(130576); _f(this); }
};

struct MouseEvent {
public:
	int x;
	int y;
	MouseButton button;
	inline MouseEvent()  { }
};

struct KGLShaderCBuffer {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > cBufferName;
	unsigned int size;
	unsigned int slot;
	inline KGLShaderCBuffer()  { }
	inline void dtor() { typedef void (*_fpt)(KGLShaderCBuffer *pthis); _fpt _f=(_fpt)_drva(113184); _f(this); }
};

struct SessionInfo {
public:
	SessionType type;
	double startTimeMS;
	double timeSecs;
	int laps;
	int index;
	inline SessionInfo()  { }
};

struct DriverActionsState {
public:
	int state;
	inline DriverActionsState()  { }
};

struct TrackData {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > configuration;
	int gridPlaces;
	inline TrackData()  { }
	inline void dtor() { typedef void (*_fpt)(TrackData *pthis); _fpt _f=(_fpt)_drva(776352); _f(this); }
};

struct OnNewCarLoadedEvent {
public:
	CarAvatar * car;
	inline OnNewCarLoadedEvent()  { }
};

class IVarCallback {
public:
	inline IVarCallback()  { }
	virtual void onSetVar_vf0(SVar *  _arg0, float  _arg1) = 0;
	inline void onSetVar(SVar *  _arg0, float  _arg1) { return onSetVar_vf0( _arg0,  _arg1); }
};

struct ksgui_OnSliderInteraction {
public:
	ksgui_Slider * slider;
	float value;
	inline ksgui_OnSliderInteraction()  { }
};

struct OnESCMenuTriggered {
public:
	ESCMenu * menu;
	bool visible;
	bool startReplay;
	inline OnESCMenuTriggered()  { }
};

struct OnPenaltyEvent {
public:
	Car * car;
	PenaltyType ptype;
	inline OnPenaltyEvent()  { }
};

class IACPPluginHost {
public:
	inline IACPPluginHost()  { }
	virtual HWND__ * getHwnd_vf0() = 0;
	inline HWND__ * getHwnd() { return getHwnd_vf0(); }
	virtual HINSTANCE__ * getHInstance_vf1() = 0;
	inline HINSTANCE__ * getHInstance() { return getHInstance_vf1(); }
	virtual void setABS_vf2(float  _arg0) = 0;
	inline void setABS(float  _arg0) { return setABS_vf2( _arg0); }
	virtual void setTC_vf3(float  _arg0) = 0;
	inline void setTC(float  _arg0) { return setTC_vf3( _arg0); }
	virtual void setStabilityControl_vf4(float  _arg0) = 0;
	inline void setStabilityControl(float  _arg0) { return setStabilityControl_vf4( _arg0); }
	virtual void setIdealLine_vf5(bool  _arg0, bool  _arg1) = 0;
	inline void setIdealLine(bool  _arg0, bool  _arg1) { return setIdealLine_vf5( _arg0,  _arg1); }
	virtual void setAutoShift_vf6(bool  _arg0) = 0;
	inline void setAutoShift(bool  _arg0) { return setAutoShift_vf6( _arg0); }
	virtual void setBrakeBias_vf7(float  _arg0) = 0;
	inline void setBrakeBias(float  _arg0) { return setBrakeBias_vf7( _arg0); }
	virtual void setSystemMessage_vf8(wchar_t *  _arg0, wchar_t *  _arg1, bool  _arg2) = 0;
	inline void setSystemMessage(wchar_t *  _arg0, wchar_t *  _arg1, bool  _arg2) { return setSystemMessage_vf8( _arg0,  _arg1,  _arg2); }
};

struct Lap {
public:
	unsigned int time;
	std::vector<unsigned int,std::allocator<unsigned int> > splits;
	unsigned int cuts;
	bool isValid;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > compound;
	inline Lap()  { }
	inline void ctor(Lap & __that) { typedef void (*_fpt)(Lap *pthis, Lap &); _fpt _f=(_fpt)_drva(242400); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(Lap *pthis); _fpt _f=(_fpt)_drva(246624); _f(this); }
};

struct INISection {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > > keys;
	inline INISection()  { }
	inline bool hasKey(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & k) { typedef bool (*_fpt)(INISection *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2322496); return _f(this, k); }
	inline void dtor() { typedef void (*_fpt)(INISection *pthis); _fpt _f=(_fpt)_drva(4512704); _f(this); }
};

struct DRSWingConnection {
public:
	Wing * wing;
	float effect;
	float angle;
	DRWWingConnectionMode mode;
	inline DRSWingConnection()  { }
};

struct ksgui_OnListBoxItemClickedEvent {
public:
	ksgui_ListBox * listBox;
	ksgui_ListBoxRowData * row;
	unsigned int itemIndex;
	inline ksgui_OnListBoxItemClickedEvent()  { }
};

struct RemoteSessionResult {
public:
	std::vector<int,std::allocator<int> > positions;
	std::vector<int,std::allocator<int> > times;
	std::vector<int,std::allocator<int> > lapCounter;
	std::vector<bool,std::allocator<bool> > hasFinished;
	unsigned int leaderLapCount;
	inline RemoteSessionResult()  { }
	inline void ctor(RemoteSessionResult & __that) { typedef void (*_fpt)(RemoteSessionResult *pthis, RemoteSessionResult &); _fpt _f=(_fpt)_drva(242752); _f(this, __that); }
	inline void ctor(int carsCount) { typedef void (*_fpt)(RemoteSessionResult *pthis, int); _fpt _f=(_fpt)_drva(242880); _f(this, carsCount); }
	inline void ctor() { typedef void (*_fpt)(RemoteSessionResult *pthis); _fpt _f=(_fpt)_drva(243104); _f(this); }
	inline void dtor() { typedef void (*_fpt)(RemoteSessionResult *pthis); _fpt _f=(_fpt)_drva(246784); _f(this); }
};

struct OnControlsProviderChanged {
public:
	Car * car;
	ICarControlsProvider * newControlsProvider;
	inline OnControlsProviderChanged()  { }
};

struct RemoteSession {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	SessionType type;
	int index;
	int time;
	int laps;
	double startTime;
	bool isSpectator;
	bool isTimedRace;
	bool hasExtraLap;
	inline RemoteSession()  { }
	inline void ctor(RemoteSession & __that) { typedef void (*_fpt)(RemoteSession *pthis, RemoteSession &); _fpt _f=(_fpt)_drva(242560); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(RemoteSession *pthis); _fpt _f=(_fpt)_drva(242672); _f(this); }
	inline void dtor() { typedef void (*_fpt)(RemoteSession *pthis); _fpt _f=(_fpt)_drva(2334528); _f(this); }
};

class Session {
public:
	SessionType sessionType;
	bool isTimedRace;
	bool hasAdditionalLap;
	int laps;
	float durationMinutes;
	float overtime_ms;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > spawSet;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	double startTime;
	int forcedPosition;
	bool isOver;
	unsigned int leaderCompletedLaps;
	inline Session()  { }
	inline void ctor(Session & __that) { typedef void (*_fpt)(Session *pthis, Session &); _fpt _f=(_fpt)_drva(704528); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(Session *pthis); _fpt _f=(_fpt)_drva(610160); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Session *pthis); _fpt _f=(_fpt)_drva(584992); _f(this); }
	inline Session & operator=(Session & __that) { typedef Session & (*_fpt)(Session *pthis, Session &); _fpt _f=(_fpt)_drva(612480); return _f(this, __that); }
};

class SetupItem {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	float & connectedFloat;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > units;
	float multiplier;
	float newValue;
	bool attached;
	std::function<void __cdecl(SetupItem *)> onValueChanged;
	float labelMultiplier;
	inline SetupItem()  : connectedFloat(*((float*)NULL)) { }
	inline void ctor(SetupItem & __that) { typedef void (*_fpt)(SetupItem *pthis, SetupItem &); _fpt _f=(_fpt)_drva(2657424); _f(this, __that); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aname, float & aconnectedFloat, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * units, bool isAttached, float multiplier, float labelMult) { typedef void (*_fpt)(SetupItem *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, float &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, bool, float, float); _fpt _f=(_fpt)_drva(2929008); _f(this, aname, aconnectedFloat, units, isAttached, multiplier, labelMult); }
	virtual ~SetupItem();
	inline void dtor() { typedef void (*_fpt)(SetupItem *pthis); _fpt _f=(_fpt)_drva(2929312); _f(this); }
};

struct MaterialOption {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	IMaterialOptionChangeListener * material;
	bool value;
	inline MaterialOption()  { }
};

class KGLTexture {
public:
	ID3D11ShaderResourceView * shaderResourceView;
	KGLTexture_ImageFileFormat fileFormat;
	DXGI_FORMAT textureFormat;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > fileName;
	unsigned int width;
	unsigned int height;
	bool ownsShaderResourceView;
	inline KGLTexture()  { }
	inline void ctor(void * buffer, int size, ID3D11Device * device) { typedef void (*_fpt)(KGLTexture *pthis, void *, int, ID3D11Device *); _fpt _f=(_fpt)_drva(145776); _f(this, buffer, size, device); }
	inline void ctor(void * buffer, DXGI_FORMAT format, unsigned int width, unsigned int height, ID3D11Device * device) { typedef void (*_fpt)(KGLTexture *pthis, void *, DXGI_FORMAT, unsigned int, unsigned int, ID3D11Device *); _fpt _f=(_fpt)_drva(146080); _f(this, buffer, format, width, height, device); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename, ID3D11Device * device) { typedef void (*_fpt)(KGLTexture *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, ID3D11Device *); _fpt _f=(_fpt)_drva(145440); _f(this, filename, device); }
	inline void initSize() { typedef void (*_fpt)(KGLTexture *pthis); _fpt _f=(_fpt)_drva(146448); return _f(this); }
};

struct TelemetryChannelData {
public:
	std::vector<float,std::allocator<float> > values;
	TelemetryUnits units;
	int frequency;
	inline TelemetryChannelData()  { }
	inline void dtor() { typedef void (*_fpt)(TelemetryChannelData *pthis); _fpt _f=(_fpt)_drva(811392); _f(this); }
};

class RenderTarget {
public:
	eRenderTargetFormat format;
	int width;
	int height;
	bool hasNullDepth;
	void * kidColor;
	void * kidDepth;
	inline RenderTarget()  { }
	inline void ctor(GraphicsManager * graphics, eRenderTargetFormat fmt, unsigned int iwidth, unsigned int iheight, bool hasColor, bool hasDepth, int mips) { typedef void (*_fpt)(RenderTarget *pthis, GraphicsManager *, eRenderTargetFormat, unsigned int, unsigned int, bool, bool, int); _fpt _f=(_fpt)_drva(2211888); _f(this, graphics, fmt, iwidth, iheight, hasColor, hasDepth, mips); }
	virtual ~RenderTarget();
	inline void dtor() { typedef void (*_fpt)(RenderTarget *pthis); _fpt _f=(_fpt)_drva(2212176); _f(this); }
	inline void clear() { typedef void (*_fpt)(RenderTarget *pthis); _fpt _f=(_fpt)_drva(2212336); return _f(this); }
};

struct OnFlagEvent {
public:
	Car * car;
	FlagEventType type;
	PenaltyDescription description;
	inline OnFlagEvent()  { }
};

class GameObject {
public:
	Game * game;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	bool isActive;
	GameObject * parent;
	std::vector<GameObject *,std::allocator<GameObject *> > gameObjects;
	inline GameObject()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, Game * igame) { typedef void (*_fpt)(GameObject *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, Game *); _fpt _f=(_fpt)_drva(2358320); _f(this, iname, igame); }
	virtual ~GameObject();
	inline void dtor() { typedef void (*_fpt)(GameObject *pthis); _fpt _f=(_fpt)_drva(2358496); _f(this); }
	inline GameObject * getParent() { typedef GameObject * (*_fpt)(GameObject *pthis); _fpt _f=(_fpt)_drva(2358672); return _f(this); }
	virtual void update_vf1(float deltaT);
	inline void update_impl(float deltaT) { typedef void (*_fpt)(GameObject *pthis, float); _fpt _f=(_fpt)_drva(96368); return _f(this, deltaT); }
	inline void update(float deltaT) { return update_vf1(deltaT); }
	virtual void render_vf2(float deltaT);
	inline void render_impl(float deltaT) { typedef void (*_fpt)(GameObject *pthis, float); _fpt _f=(_fpt)_drva(96368); return _f(this, deltaT); }
	inline void render(float deltaT) { return render_vf2(deltaT); }
	virtual void renderHUD_vf3(float deltaT);
	inline void renderHUD_impl(float deltaT) { typedef void (*_fpt)(GameObject *pthis, float); _fpt _f=(_fpt)_drva(96368); return _f(this, deltaT); }
	inline void renderHUD(float deltaT) { return renderHUD_vf3(deltaT); }
	virtual void renderAudio_vf4(float deltaT);
	inline void renderAudio_impl(float deltaT) { typedef void (*_fpt)(GameObject *pthis, float); _fpt _f=(_fpt)_drva(96368); return _f(this, deltaT); }
	inline void renderAudio(float deltaT) { return renderAudio_vf4(deltaT); }
	virtual void shutdown_vf5();
	inline void shutdown_impl() { typedef void (*_fpt)(GameObject *pthis); _fpt _f=(_fpt)_drva(96368); return _f(this); }
	inline void shutdown() { return shutdown_vf5(); }
	inline GameObject * getGameObject(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name) { typedef GameObject * (*_fpt)(GameObject *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(1177456); return _f(this, name); }
};

class ShaderManager {
public:
	GraphicsManager * graphics;
	std::vector<Shader *,std::allocator<Shader *> > shaders;
	inline ShaderManager()  { }
	inline void ctor(GraphicsManager * graphics) { typedef void (*_fpt)(ShaderManager *pthis, GraphicsManager *); _fpt _f=(_fpt)_drva(2259120); _f(this, graphics); }
	virtual ~ShaderManager();
	inline void dtor() { typedef void (*_fpt)(ShaderManager *pthis); _fpt _f=(_fpt)_drva(2259152); _f(this); }
	inline Shader * getShader(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef Shader * (*_fpt)(ShaderManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2259776); return _f(this, name); }
	inline void cleanup() { typedef void (*_fpt)(ShaderManager *pthis); _fpt _f=(_fpt)_drva(2259680); return _f(this); }
};

class ShaderVariable {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	eVariableType type;
	int size;
	int offset;
	CBuffer * buffer;
	inline ShaderVariable()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, CBuffer * cbuffer, int ioffset, int isize) { typedef void (*_fpt)(ShaderVariable *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, CBuffer *, int, int); _fpt _f=(_fpt)_drva(2135136); _f(this, iname, cbuffer, ioffset, isize); }
	virtual ~ShaderVariable();
	inline void set(int * value) { typedef void (*_fpt)(ShaderVariable *pthis, int *); _fpt _f=(_fpt)_drva(2135552); return _f(this, value); }
	inline void set(float * value) { typedef void (*_fpt)(ShaderVariable *pthis, float *); _fpt _f=(_fpt)_drva(2135552); return _f(this, value); }
};

class OptionsManager {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,float,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,float> > > options;
	inline OptionsManager()  { }
	inline void ctor() { typedef void (*_fpt)(OptionsManager *pthis); _fpt _f=(_fpt)_drva(1627248); _f(this); }
	virtual ~OptionsManager();
	inline void dtor() { typedef void (*_fpt)(OptionsManager *pthis); _fpt _f=(_fpt)_drva(1628464); _f(this); }
	inline void loadOptions() { typedef void (*_fpt)(OptionsManager *pthis); _fpt _f=(_fpt)_drva(1628704); return _f(this); }
};

class CarRaceInfo {
public:
	CarAvatar * car;
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,int,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,int> > > spawnPositionIndex;
	RaceManager * raceManager;
	inline CarRaceInfo()  { }
	inline void ctor() { typedef void (*_fpt)(CarRaceInfo *pthis); _fpt _f=(_fpt)_drva(948448); _f(this); }
	virtual ~CarRaceInfo();
	inline void dtor() { typedef void (*_fpt)(CarRaceInfo *pthis); _fpt _f=(_fpt)_drva(948528); _f(this); }
	inline void setSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName, int gp) { typedef void (*_fpt)(CarRaceInfo *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, int); _fpt _f=(_fpt)_drva(951456); return _f(this, setName, gp); }
	inline int getSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName) { typedef int (*_fpt)(CarRaceInfo *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(950352); return _f(this, setName); }
	inline void init(CarAvatar * acar) { typedef void (*_fpt)(CarRaceInfo *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(950384); return _f(this, acar); }
};

class Shader {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::vector<ShaderVariable *,std::allocator<ShaderVariable *> > vars;
	std::vector<ShaderResource *,std::allocator<ShaderResource *> > resources;
	std::vector<CBuffer *,std::allocator<CBuffer *> > cBuffers;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > options;
	bool isAlphaTested;
	ILType ilType;
	int guid;
	bool useNullPS;
	GraphicsManager * graphics;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > filename;
	void * kid;
	inline Shader()  { }
	inline void ctor(GraphicsManager * graphics, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename) { typedef void (*_fpt)(Shader *pthis, GraphicsManager *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2203104); _f(this, graphics, ifilename); }
	virtual ~Shader();
	inline void dtor() { typedef void (*_fpt)(Shader *pthis); _fpt _f=(_fpt)_drva(2203376); _f(this); }
	inline void apply() { typedef void (*_fpt)(Shader *pthis); _fpt _f=(_fpt)_drva(2204592); return _f(this); }
	inline ShaderVariable * getVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef ShaderVariable * (*_fpt)(Shader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2204608); return _f(this, name); }
	inline bool initShaderBinary() { typedef bool (*_fpt)(Shader *pthis); _fpt _f=(_fpt)_drva(2204816); return _f(this); }
	inline void reflectVars() { typedef void (*_fpt)(Shader *pthis); _fpt _f=(_fpt)_drva(2205008); return _f(this); }
};

struct PhysicsCPUTimes {
public:
	double carStep;
	CoreCPUTimes coreCPUTimes;
	int currentCPU;
	inline PhysicsCPUTimes()  { }
};

struct ModelBoundariesCoordinates {
public:
	float front;
	float rear;
	float left;
	float right;
	float top;
	float bottom;
	inline ModelBoundariesCoordinates()  { }
};

class ThermalObject {
public:
	float tmass;
	float coolSpeedK;
	float coolFactor;
	float heatFactor;
	float t;
	float heatAccumulator;
	inline ThermalObject()  { }
	inline void ctor() { typedef void (*_fpt)(ThermalObject *pthis); _fpt _f=(_fpt)_drva(2829952); _f(this); }
	virtual ~ThermalObject();
	inline void dtor() { typedef void (*_fpt)(ThermalObject *pthis); _fpt _f=(_fpt)_drva(2830000); _f(this); }
	inline void step(float dt, float ambientTemp, Speed & speed) { typedef void (*_fpt)(ThermalObject *pthis, float, float, Speed &); _fpt _f=(_fpt)_drva(2830080); return _f(this, dt, ambientTemp, speed); }
	inline void addHeadSource(float heat) { typedef void (*_fpt)(ThermalObject *pthis, float); _fpt _f=(_fpt)_drva(2830064); return _f(this, heat); }
};

class vec4f {
public:
	float x;
	float y;
	float z;
	float w;
	inline vec4f()  { }
	inline void ctor(vec3f & v3, float vw) { typedef void (*_fpt)(vec4f *pthis, vec3f &, float); _fpt _f=(_fpt)_drva(962960); _f(this, v3, vw); }
	inline void ctor(float ix, float iy, float iz, float iw) { typedef void (*_fpt)(vec4f *pthis, float, float, float, float); _fpt _f=(_fpt)_drva(3178496); _f(this, ix, iy, iz, iw); }
	inline void ctor() { typedef void (*_fpt)(vec4f *pthis); _fpt _f=(_fpt)_drva(508768); _f(this); }
};

struct LightingSettings {
public:
	vec3f lightDirection;
	vec3f lightColor;
	vec3f horizonLow;
	vec3f horizonHigh;
	vec3f skyLow;
	vec3f skyHigh;
	vec3f sunLow;
	vec3f sunHigh;
	float angle;
	float headingAngle;
	float pitchAngle;
	vec3f ambientLow;
	vec3f ambientHigh;
	vec3f fogColor;
	float fogLinear;
	float fogBlend;
	float cloudCover;
	float cloudCutoff;
	float cloudColor;
	float cloudOffset;
	float saturation;
	float gameTime;
	float sunAngleGamma;
	inline LightingSettings()  { }
	inline void ctor() { typedef void (*_fpt)(LightingSettings *pthis); _fpt _f=(_fpt)_drva(2104736); _f(this); }
};

struct UDPMessage {
public:
	void * data;
	int size;
	sockaddr_in srcAddress;
	inline UDPMessage()  { }
};

struct AIState {
public:
	float currentSteerSignal;
	vec3f steerTarget;
	float gasBrakeTarget;
	bool isActive;
	float targetSpeed;
	float targetLateralOffset;
	float brakeTargetSpeed;
	float brakeTargetDist;
	float currentNormalizedSplinePosition;
	float outsideOffset;
	float projDNRPM;
	float understeerFactor;
	float currentPush;
	inline AIState()  { }
};

class ICollisionCallback {
public:
	inline ICollisionCallback()  { }
	virtual ~ICollisionCallback();
	inline void dtor() { typedef void (*_fpt)(ICollisionCallback *pthis); _fpt _f=(_fpt)_drva(2501856); _f(this); }
	virtual void onCollisionCallBack_vf1(void *  _arg0, void *  _arg1, void *  _arg2, void *  _arg3, vec3f  _arg4, vec3f  _arg5, float  _arg6) = 0;
	inline void onCollisionCallBack(void *  _arg0, void *  _arg1, void *  _arg2, void *  _arg3, vec3f  _arg4, vec3f  _arg5, float  _arg6) { return onCollisionCallBack_vf1( _arg0,  _arg1,  _arg2,  _arg3,  _arg4,  _arg5,  _arg6); }
};

struct OnNewSessionEvent {
public:
	Session newSession;
	short index;
	inline OnNewSessionEvent()  { }
	inline void dtor() { typedef void (*_fpt)(OnNewSessionEvent *pthis); _fpt _f=(_fpt)_drva(584992); _f(this); }
};

struct SplinePoint {
public:
	vec3f point;
	float pointLength;
	int tag;
	inline SplinePoint()  { }
};

class Turbo {
public:
	float userSetting;
	float rotation;
	TurboDef data;
	inline Turbo()  { }
	inline void ctor(TurboDef & data) { typedef void (*_fpt)(Turbo *pthis, TurboDef &); _fpt _f=(_fpt)_drva(2811696); _f(this, data); }
	inline void dtor() { typedef void (*_fpt)(Turbo *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void step(float gas, float rpms, float dt) { typedef void (*_fpt)(Turbo *pthis, float, float, float); _fpt _f=(_fpt)_drva(2811840); return _f(this, gas, rpms, dt); }
	inline float getBoost() { typedef float (*_fpt)(Turbo *pthis); _fpt _f=(_fpt)_drva(2811776); return _f(this); }
	inline void reset() { typedef void (*_fpt)(Turbo *pthis); _fpt _f=(_fpt)_drva(2811792); return _f(this); }
	inline void setTurboBoostLevel(float value) { typedef void (*_fpt)(Turbo *pthis, float); _fpt _f=(_fpt)_drva(2811808); return _f(this, value); }
	inline float getWastegate() { typedef float (*_fpt)(Turbo *pthis); _fpt _f=(_fpt)_drva(2645504); return _f(this); }
};

struct CarCollisionBox {
public:
	vec3f centre;
	vec3f size;
	unsigned __int64 id;
	inline CarCollisionBox()  { }
};

struct ksgui_GraphReferenceAxis {
public:
	vec3f color;
	float refValue;
	bool isVertical;
	inline ksgui_GraphReferenceAxis()  { }
	inline void ctor(vec3f & acolor, float arefValue, bool bisVertical) { typedef void (*_fpt)(ksgui_GraphReferenceAxis *pthis, vec3f &, float, bool); _fpt _f=(_fpt)_drva(2987152); _f(this, acolor, arefValue, bisVertical); }
};

struct WingState {
public:
	float aoa;
	float cd;
	float cl;
	float angle;
	float inputAngle;
	float groundHeight;
	float frontShare;
	float dragKG;
	float liftKG;
	float angleMult;
	float groundEffectLift;
	float groundEffectDrag;
	float yawAngle;
	bool isVertical;
	vec3f liftVector;
	inline WingState()  { }
};

struct NetCarState {
public:
	vec3f pos;
	vec3f rotation;
	vec3f velocity;
	vec3f acceleration;
	double timeStamp;
	double rcvTime;
	unsigned char pakSequenceId;
	float targetHeight;
	float tyreAngularSpeed[0x4];
	unsigned short engineRPM;
	float steerAngle;
	float wheelAngle;
	int gearIndex;
	unsigned int statusBytes;
	float aoa;
	vec3f aoaAxis;
	int ping;
	float performanceDelta;
	float gas;
	inline NetCarState()  { }
};

struct CollisionEvent {
public:
	int carIndex;
	float normalForce;
	vec3f pos;
	vec3f normal;
	float impactAngle;
	float relativeSpeed;
	unsigned long colliderCategory;
	inline CollisionEvent()  { }
};

class Joypad {
public:
	inline Joypad()  { }
	virtual void acquire_vf0() = 0;
	inline void acquire() { return acquire_vf0(); }
	virtual vec2f getLeftStick_vf1() = 0;
	inline vec2f getLeftStick() { return getLeftStick_vf1(); }
	virtual vec2f getRightStick_vf2() = 0;
	inline vec2f getRightStick() { return getRightStick_vf2(); }
	virtual float getLeftTrigger_vf3() = 0;
	inline float getLeftTrigger() { return getLeftTrigger_vf3(); }
	virtual float getRightTrigger_vf4() = 0;
	inline float getRightTrigger() { return getRightTrigger_vf4(); }
	virtual unsigned int getButtonMask_vf5() = 0;
	inline unsigned int getButtonMask() { return getButtonMask_vf5(); }
	virtual void setVibrations_vf6(float  _arg0, float  _arg1) = 0;
	inline void setVibrations(float  _arg0, float  _arg1) { return setVibrations_vf6( _arg0,  _arg1); }
};

struct SystemCBuffers {
public:
	CBuffer cbCamera;
	CBuffer cbPerObject;
	CBuffer cbLighting;
	CBuffer cbShadowMap;
	inline SystemCBuffers()  { }
	inline void dtor() { typedef void (*_fpt)(SystemCBuffers *pthis); _fpt _f=(_fpt)_drva(2106176); _f(this); }
};

struct OnMouseMoveEvent : public MouseEvent {
public:
	inline OnMouseMoveEvent()  { }
};

struct OnMouseWheelMovedEvent : public MouseEvent {
public:
	float delta;
	inline OnMouseWheelMovedEvent()  { }
};

struct CarCollisionBounds {
public:
	vec3f min;
	vec3f max;
	float length;
	float width;
	float lengthFront;
	float lengthRear;
	inline CarCollisionBounds()  { }
};

class plane4f {
public:
	vec3f normal;
	float d;
	inline plane4f()  { }
	inline void ctor(vec3f & point1, vec3f & point2, vec3f & point3) { typedef void (*_fpt)(plane4f *pthis, vec3f &, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(1146576); _f(this, point1, point2, point3); }
};

struct PhysicsValueCache {
public:
	Speed speed;
	inline PhysicsValueCache()  { }
	inline void dtor() { typedef void (*_fpt)(PhysicsValueCache *pthis); _fpt _f=(_fpt)_drva(1183184); _f(this); }
};

class SinSignalGenerator : public SignalGenerator {
public:
	inline SinSignalGenerator()  { }
	inline void ctor() { typedef void (*_fpt)(SinSignalGenerator *pthis); _fpt _f=(_fpt)_drva(2197680); _f(this); }
	virtual ~SinSignalGenerator();
	inline void dtor() { typedef void (*_fpt)(SinSignalGenerator *pthis); _fpt _f=(_fpt)_drva(2197728); _f(this); }
	virtual float getValue_vf2();
	inline float getValue_impl() { typedef float (*_fpt)(SinSignalGenerator *pthis); _fpt _f=(_fpt)_drva(2197808); return _f(this); }
	inline float getValue() { return getValue_vf2(); }
};

class IKeyEventListener {
public:
	inline IKeyEventListener()  { }
	virtual void onKeyDown_vf0(OnKeyEvent &  _arg0) = 0;
	inline void onKeyDown(OnKeyEvent &  _arg0) { return onKeyDown_vf0( _arg0); }
	virtual void onKeyChar_vf1(unsigned int  _arg0) = 0;
	inline void onKeyChar(unsigned int  _arg0) { return onKeyChar_vf1( _arg0); }
};

class IPAddress {
public:
	sockaddr_in sokaddr;
	inline IPAddress()  { }
	inline void ctor() { typedef void (*_fpt)(IPAddress *pthis); _fpt _f=(_fpt)_drva(508768); _f(this); }
	inline void ctor(sockaddr_in & addr) { typedef void (*_fpt)(IPAddress *pthis, sockaddr_in &); _fpt _f=(_fpt)_drva(2479584); _f(this, addr); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & addr, unsigned short port) { typedef void (*_fpt)(IPAddress *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, unsigned short); _fpt _f=(_fpt)_drva(2479600); _f(this, addr, port); }
	inline void dtor() { typedef void (*_fpt)(IPAddress *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
};

struct RemoteSessionResume {
public:
	RemoteSession session;
	RemoteSessionResult results;
	inline RemoteSessionResume()  { }
	inline void ctor() { typedef void (*_fpt)(RemoteSessionResume *pthis); _fpt _f=(_fpt)_drva(243232); _f(this); }
};

struct ClientCollisionEvent {
public:
	NetCarStateProvider * netCar;
	float speed;
	vec3f worldPos;
	vec3f relPos;
	inline ClientCollisionEvent()  { }
};

struct DRS {
public:
	bool isPresent;
	bool isActive;
	bool isAvailable;
	bool ignoreZones;
	std::vector<DRSWingConnection,std::allocator<DRSWingConnection> > wings;
	Car * car;
	bool lastState;
	float limitG;
	inline DRS()  { }
	inline void dtor() { typedef void (*_fpt)(DRS *pthis); _fpt _f=(_fpt)_drva(650384); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(DRS *pthis, Car *); _fpt _f=(_fpt)_drva(2835248); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(DRS *pthis, float); _fpt _f=(_fpt)_drva(2838112); return _f(this, dt); }
};

struct OnCollisionEvent {
public:
	IRigidBody * body;
	float relativeSpeed;
	vec3f worldPos;
	vec3f relPos;
	unsigned long colliderGroup;
	inline OnCollisionEvent()  { }
};

class ITyreModel {
public:
	inline ITyreModel()  { }
	virtual ~ITyreModel();
	inline void dtor() { typedef void (*_fpt)(ITyreModel *pthis); _fpt _f=(_fpt)_drva(4503904); _f(this); }
	virtual TyreModelOutput solve_vf1(TyreModelInput &  _arg0) = 0;
	inline TyreModelOutput solve(TyreModelInput &  _arg0) { return solve_vf1( _arg0); }
};

class BrushTyreModel {
public:
	BrushTyreModelData data;
	inline BrushTyreModel()  { }
	inline void ctor() { typedef void (*_fpt)(BrushTyreModel *pthis); _fpt _f=(_fpt)_drva(2929488); _f(this); }
	inline void dtor() { typedef void (*_fpt)(BrushTyreModel *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline BrushOutput solve(float slip, float friction, float load, float cf1_mix, float asy) { typedef BrushOutput (*_fpt)(BrushTyreModel *pthis, float, float, float, float, float); _fpt _f=(_fpt)_drva(2929600); return _f(this, slip, friction, load, cf1_mix, asy); }
	inline BrushOutput solveV5(float slip, float load, float asy) { typedef BrushOutput (*_fpt)(BrushTyreModel *pthis, float, float, float); _fpt _f=(_fpt)_drva(2929888); return _f(this, slip, load, asy); }
	inline float getCFFromSlipAngle(float angle) { typedef float (*_fpt)(BrushTyreModel *pthis, float); _fpt _f=(_fpt)_drva(2929536); return _f(this, angle); }
};

struct OnMouseUpEvent : public MouseEvent {
public:
	inline OnMouseUpEvent()  { }
};

struct MeshVertex {
public:
	vec3f pos;
	vec3f normal;
	vec2f texCoord;
	vec3f tangent;
	inline MeshVertex()  { }
};

struct Wind {
public:
	vec3f vector;
	Speed speed;
	float directionDeg;
	inline Wind()  { }
	inline void dtor() { typedef void (*_fpt)(Wind *pthis); _fpt _f=(_fpt)_drva(2502208); _f(this); }
};

struct ACPhysicsEvent {
public:
	eACEventType type;
	float param1;
	float param2;
	float param3;
	float param4;
	vec3f vParam1;
	vec3f vParam2;
	void * voidParam0;
	void * voidParam1;
	unsigned long ulParam0;
	inline ACPhysicsEvent()  { }
};

struct OnMouseDownEvent : public MouseEvent {
public:
	inline OnMouseDownEvent()  { }
};

struct RayCastHit {
public:
	vec3f pos;
	vec3f normal;
	ICollisionObject * collisionObject;
	bool hasContact;
	inline RayCastHit()  { }
};

class Curve {
public:
	std::vector<float,std::allocator<float> > references;
	std::vector<float,std::allocator<float> > values;
	float fastStep;
	bool cubicSplineReady;
	CubicSpline<float,float> cSpline;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > filename;
	inline Curve()  { }
	inline void ctor(Curve & __that) { typedef void (*_fpt)(Curve *pthis, Curve &); _fpt _f=(_fpt)_drva(844960); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(Curve *pthis); _fpt _f=(_fpt)_drva(2119856); _f(this); }
	virtual ~Curve();
	inline void dtor() { typedef void (*_fpt)(Curve *pthis); _fpt _f=(_fpt)_drva(2119952); _f(this); }
	inline void load(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename) { typedef void (*_fpt)(Curve *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2124368); return _f(this, ifilename); }
	inline void loadEncrypted(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataFile) { typedef void (*_fpt)(Curve *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2125648); return _f(this, ifilename, dataFile); }
	inline void scale(float v) { typedef void (*_fpt)(Curve *pthis, float); _fpt _f=(_fpt)_drva(2126816); return _f(this, v); }
	inline float getValue(float ref) { typedef float (*_fpt)(Curve *pthis, float); _fpt _f=(_fpt)_drva(2124176); return _f(this, ref); }
	inline float getCubicSplineValue(float ref) { typedef float (*_fpt)(Curve *pthis, float); _fpt _f=(_fpt)_drva(2124016); return _f(this, ref); }
	inline void addValue(float ref, float val) { typedef void (*_fpt)(Curve *pthis, float, float); _fpt _f=(_fpt)_drva(2120416); return _f(this, ref, val); }
	inline float getMaxReference() { typedef float (*_fpt)(Curve *pthis); _fpt _f=(_fpt)_drva(2124080); return _f(this); }
	inline int getCount() { typedef int (*_fpt)(Curve *pthis); _fpt _f=(_fpt)_drva(2124000); return _f(this); }
	inline std::pair<float,float> getPairAtIndex(int index) { typedef std::pair<float,float> (*_fpt)(Curve *pthis, int); _fpt _f=(_fpt)_drva(2124096); return _f(this, index); }
	inline void print() { typedef void (*_fpt)(Curve *pthis); _fpt _f=(_fpt)_drva(2126672); return _f(this); }
	inline Curve & operator=(Curve & __that) { typedef Curve & (*_fpt)(Curve *pthis, Curve &); _fpt _f=(_fpt)_drva(355408); return _f(this, __that); }
};

class ThreadPool {
public:
	int numThreads;
	std::vector<std::thread,std::allocator<std::thread> > workers;
	bool stop;
	std::deque<Task *,std::allocator<Task *> > tasks;
	std::mutex queue_mutex;
	std::condition_variable condition;
	inline ThreadPool()  { }
	inline void ctor(int inumThreads, std::function<void __cdecl(int)> * initFun) { typedef void (*_fpt)(ThreadPool *pthis, int, std::function<void __cdecl(int)> *); _fpt _f=(_fpt)_drva(2950304); _f(this, inumThreads, initFun); }
	virtual ~ThreadPool();
	inline void dtor() { typedef void (*_fpt)(ThreadPool *pthis); _fpt _f=(_fpt)_drva(2951328); _f(this); }
	inline void addTask(Task & task) { typedef void (*_fpt)(ThreadPool *pthis, Task &); _fpt _f=(_fpt)_drva(2953008); return _f(this, task); }
};

class TelemetryChannel {
public:
	std::basic_string<char,std::char_traits<char>,std::allocator<char> > name;
	TelemetryChannelData data;
	float * dataSource;
	double lastTickTime;
	float scale;
	inline TelemetryChannel()  { }
	inline void ctor(TelemetryChannel & __that) { typedef void (*_fpt)(TelemetryChannel *pthis, TelemetryChannel &); _fpt _f=(_fpt)_drva(2866288); _f(this, __that); }
	inline void ctor(std::basic_string<char,std::char_traits<char>,std::allocator<char> > & name, float * adataSource, TelemetryUnits units, int frequency, float scale) { typedef void (*_fpt)(TelemetryChannel *pthis, std::basic_string<char,std::char_traits<char>,std::allocator<char> > &, float *, TelemetryUnits, int, float); _fpt _f=(_fpt)_drva(2866448); _f(this, name, adataSource, units, frequency, scale); }
	inline void dtor() { typedef void (*_fpt)(TelemetryChannel *pthis); _fpt _f=(_fpt)_drva(2549872); _f(this); }
};

struct PerformanceMeter {
public:
	bool isEnabled;
	Car * car;
	std::vector<PerformancePair,std::allocator<PerformancePair> > currentLap;
	std::vector<PerformancePair,std::allocator<PerformancePair> > bestLap;
	double bestLapTime;
	int lastLapIndex;
	double currentDistance;
	double lastRecordedDistance;
	double currentDiff;
	float currentSpeedDiffMS;
	inline PerformanceMeter()  { }
	inline void dtor() { typedef void (*_fpt)(PerformanceMeter *pthis); _fpt _f=(_fpt)_drva(2536400); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(PerformanceMeter *pthis, Car *); _fpt _f=(_fpt)_drva(2537408); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(PerformanceMeter *pthis, float); _fpt _f=(_fpt)_drva(2537696); return _f(this, dt); }
	inline PerformanceSplit getCurrentSplit() { typedef PerformanceSplit (*_fpt)(PerformanceMeter *pthis); _fpt _f=(_fpt)_drva(2537216); return _f(this); }
	inline bool hasData() { typedef bool (*_fpt)(PerformanceMeter *pthis); _fpt _f=(_fpt)_drva(2537376); return _f(this); }
	inline void reset() { typedef void (*_fpt)(PerformanceMeter *pthis); _fpt _f=(_fpt)_drva(2537600); return _f(this); }
};

class LapDB {
public:
	double totalTime;
	std::vector<Lap,std::allocator<Lap> > laps;
	Lap bestLap;
	CarAvatar * car;
	unsigned int lastLapStartTime;
	std::vector<unsigned int,std::allocator<unsigned int> > currentSplits;
	std::vector<unsigned int,std::allocator<unsigned int> > personalBestSplits;
	int bestLapSplit;
	bool hasCompletedLastLap;
	inline LapDB()  { }
	inline void ctor(CarAvatar * icar) { typedef void (*_fpt)(LapDB *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1326000); _f(this, icar); }
	inline void reset() { typedef void (*_fpt)(LapDB *pthis); _fpt _f=(_fpt)_drva(1334192); return _f(this); }
	inline void dtor() { typedef void (*_fpt)(LapDB *pthis); _fpt _f=(_fpt)_drva(1326656); _f(this); }
};

struct SessionResult {
public:
	std::vector<std::vector<Lap,std::allocator<Lap> >,std::allocator<std::vector<Lap,std::allocator<Lap> > > > laps;
	std::vector<Lap,std::allocator<Lap> > bestLaps;
	std::vector<int,std::allocator<int> > positions;
	std::vector<int,std::allocator<int> > lapCount;
	std::vector<float,std::allocator<float> > total;
	inline SessionResult()  { }
	inline void dtor() { typedef void (*_fpt)(SessionResult *pthis); _fpt _f=(_fpt)_drva(611008); _f(this); }
};

class Texture {
public:
	void * kid;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > fileName;
	inline Texture()  { }
	inline void ctor(RenderTarget & rt) { typedef void (*_fpt)(Texture *pthis, RenderTarget &); _fpt _f=(_fpt)_drva(2088512); _f(this, rt); }
	inline void ctor(unsigned char * buffer, unsigned int size) { typedef void (*_fpt)(Texture *pthis, unsigned char *, unsigned int); _fpt _f=(_fpt)_drva(2088576); _f(this, buffer, size); }
	inline void ctor(unsigned char * buffer, unsigned int width, unsigned int height, PixelFormat aFormat) { typedef void (*_fpt)(Texture *pthis, unsigned char *, unsigned int, unsigned int, PixelFormat); _fpt _f=(_fpt)_drva(2088640); _f(this, buffer, width, height, aFormat); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef void (*_fpt)(Texture *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2088384); _f(this, filename); }
	inline void ctor() { typedef void (*_fpt)(Texture *pthis); _fpt _f=(_fpt)_drva(205552); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Texture *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
	inline void release() { typedef void (*_fpt)(Texture *pthis); _fpt _f=(_fpt)_drva(2088720); return _f(this); }
	inline Texture & operator=(Texture & __that) { typedef Texture & (*_fpt)(Texture *pthis, Texture &); _fpt _f=(_fpt)_drva(206640); return _f(this, __that); }
};

class CommandManager {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,CommandItem,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,CommandItem> > > commands;
	inline CommandManager()  { }
	inline void ctor() { typedef void (*_fpt)(CommandManager *pthis); _fpt _f=(_fpt)_drva(953200); _f(this); }
	inline void dtor() { typedef void (*_fpt)(CommandManager *pthis); _fpt _f=(_fpt)_drva(2105504); _f(this); }
	inline int getCommand(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & commandName) { typedef int (*_fpt)(CommandManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(962064); return _f(this, commandName); }
};

struct HeaveSpring {
public:
	bool isPresent;
	float rodLength;
	HeaveSpringStatus status;
	float k;
	float progressiveK;
	float packerRange;
	float bumpStopRate;
	float bumpStopUp;
	float bumpStopDn;
	Damper damper;
	bool isFront;
	Suspension * suspensions[0x2];
	Car * car;
	inline HeaveSpring()  { }
	inline void dtor() { typedef void (*_fpt)(HeaveSpring *pthis); _fpt _f=(_fpt)_drva(2831104); _f(this); }
	inline void init(Car * car, Suspension * s0, Suspension * s1, bool isFront) { typedef void (*_fpt)(HeaveSpring *pthis, Car *, Suspension *, Suspension *, bool); _fpt _f=(_fpt)_drva(2831120); return _f(this, car, s0, s1, isFront); }
	inline void step(float dt) { typedef void (*_fpt)(HeaveSpring *pthis, float); _fpt _f=(_fpt)_drva(2832736); return _f(this, dt); }
	inline void initData() { typedef void (*_fpt)(HeaveSpring *pthis); _fpt _f=(_fpt)_drva(2831168); return _f(this); }
	inline void ctor() { typedef void (*_fpt)(HeaveSpring *pthis); _fpt _f=(_fpt)_drva(2546416); _f(this); }
};

struct TimeTransponder {
public:
	unsigned int t;
	unsigned int lastLap;
	unsigned int bestLap;
	unsigned int lapCount;
	bool finishLinePassed;
	bool wasLastLapValid;
	std::vector<TimeLineStatus,std::allocator<TimeLineStatus> > status;
	std::vector<unsigned int,std::allocator<unsigned int> > lastLapSplits;
	std::vector<unsigned int,std::allocator<unsigned int> > bestLapSplits;
	std::vector<unsigned int,std::allocator<unsigned int> > currentSplits;
	Car * car;
	bool isFirstLapArmed;
	int cuts;
	bool extInvalid;
	OpenTrackTimeState openTrackState;
	bool isOpenTrack;
	inline TimeTransponder()  { }
	inline void dtor() { typedef void (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2688112); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(TimeTransponder *pthis, Car *); _fpt _f=(_fpt)_drva(2688960); return _f(this, car); }
	inline bool isValid() { typedef bool (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2689408); return _f(this); }
	inline void onTimeLinePassed(int index, bool isFinishLine) { typedef void (*_fpt)(TimeTransponder *pthis, int, bool); _fpt _f=(_fpt)_drva(2690080); return _f(this, index, isFinishLine); }
	inline TimeLineStatus & getStatus(int index) { typedef TimeLineStatus & (*_fpt)(TimeTransponder *pthis, int); _fpt _f=(_fpt)_drva(2688736); return _f(this, index); }
	inline std::vector<unsigned int,std::allocator<unsigned int> > getLastLapSplits() { typedef std::vector<unsigned int,std::allocator<unsigned int> > (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2688688); return _f(this); }
	inline void invalidate() { typedef void (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2689264); return _f(this); }
	inline void reset() { typedef void (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2690656); return _f(this); }
	inline void step(float dt) { typedef void (*_fpt)(TimeTransponder *pthis, float); _fpt _f=(_fpt)_drva(2691568); return _f(this, dt); }
	inline void armFirstLap() { typedef void (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2688656); return _f(this); }
	inline void addCut() { typedef void (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2688640); return _f(this); }
	inline int getCuts() { typedef int (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2688672); return _f(this); }
	inline bool isInOpenTrackTransition() { typedef bool (*_fpt)(TimeTransponder *pthis); _fpt _f=(_fpt)_drva(2689376); return _f(this); }
	inline void lap(bool valid) { typedef void (*_fpt)(TimeTransponder *pthis, bool); _fpt _f=(_fpt)_drva(2689616); return _f(this, valid); }
	inline void split(int sectorIndex) { typedef void (*_fpt)(TimeTransponder *pthis, int); _fpt _f=(_fpt)_drva(2691408); return _f(this, sectorIndex); }
};

struct SplineLocator {
public:
	AISpline * currentSpline;
	Car * car;
	int currentIndex;
	Track * track;
	float normalizedPos;
	float offset;
	bool isOutsideLimits;
	inline SplineLocator()  { }
	inline void dtor() { typedef void (*_fpt)(SplineLocator *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(SplineLocator *pthis, Car *); _fpt _f=(_fpt)_drva(2798240); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(SplineLocator *pthis, float); _fpt _f=(_fpt)_drva(2799040); return _f(this, dt); }
	inline void getSides(float * sides, float nsplinepos) { typedef void (*_fpt)(SplineLocator *pthis, float *, float); _fpt _f=(_fpt)_drva(2797872); return _f(this, sides, nsplinepos); }
	inline void reset() { typedef void (*_fpt)(SplineLocator *pthis); _fpt _f=(_fpt)_drva(2798880); return _f(this); }
	inline void resetToClosestPoint() { typedef void (*_fpt)(SplineLocator *pthis); _fpt _f=(_fpt)_drva(2798896); return _f(this); }
};

struct SetupManager {
public:
	std::vector<SetupItem,std::allocator<SetupItem> > items;
	bool checkRules;
	Car * car;
	std::vector<float,std::allocator<float> > gearSettings;
	float minimumHeight_m;
	float maxWaitTime;
	float waitTime;
	CarSetupState setupState;
	inline SetupManager()  { }
	inline void dtor() { typedef void (*_fpt)(SetupManager *pthis); _fpt _f=(_fpt)_drva(2657648); _f(this); }
	inline void init(Car * acar) { typedef void (*_fpt)(SetupManager *pthis, Car *); _fpt _f=(_fpt)_drva(2658960); return _f(this, acar); }
	inline void step(float dt) { typedef void (*_fpt)(SetupManager *pthis, float); _fpt _f=(_fpt)_drva(2674832); return _f(this, dt); }
	inline SetupItem * getSetupItem(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef SetupItem * (*_fpt)(SetupManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2658736); return _f(this, name); }
	inline void load(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef void (*_fpt)(SetupManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2673808); return _f(this, filename); }
	inline void initItems(bool attached) { typedef void (*_fpt)(SetupManager *pthis, bool); _fpt _f=(_fpt)_drva(2659696); return _f(this, attached); }
	inline bool isSetupRespectingRules() { typedef bool (*_fpt)(SetupManager *pthis); _fpt _f=(_fpt)_drva(2673680); return _f(this); }
};

class KGLShader {
public:
	ID3D11VertexShader * vs;
	ID3D11PixelShader * ps;
	ID3D11InputLayout * inputLayout;
	bool isAlphaTested;
	int ilType;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > fileName;
	std::vector<KGLShaderVar,std::allocator<KGLShaderVar> > vars;
	std::vector<KGLShaderTexture,std::allocator<KGLShaderTexture> > textures;
	std::vector<KGLShaderCBuffer,std::allocator<KGLShaderCBuffer> > cBuffers;
	ID3D11Device * device;
	inline KGLShader()  { }
	inline void dtor() { typedef void (*_fpt)(KGLShader *pthis); _fpt _f=(_fpt)_drva(126672); _f(this); }
	inline KGLShaderVar * getVarByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef KGLShaderVar * (*_fpt)(KGLShader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(131024); return _f(this, name); }
	inline void loadShaderBinary(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef void (*_fpt)(KGLShader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(127168); return _f(this, filename); }
	inline void createVertexShader(ID3D10Blob * blob) { typedef void (*_fpt)(KGLShader *pthis, ID3D10Blob *); _fpt _f=(_fpt)_drva(128144); return _f(this, blob); }
	inline void createPixelShader(ID3D10Blob * blob) { typedef void (*_fpt)(KGLShader *pthis, ID3D10Blob *); _fpt _f=(_fpt)_drva(128336); return _f(this, blob); }
	inline void reflectVars(ID3D10Blob * blob, bool isPS) { typedef void (*_fpt)(KGLShader *pthis, ID3D10Blob *, bool); _fpt _f=(_fpt)_drva(128496); return _f(this, blob, isPS); }
	inline void addCBuffer(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, unsigned int size, unsigned int slot) { typedef void (*_fpt)(KGLShader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, unsigned int, unsigned int); _fpt _f=(_fpt)_drva(130672); return _f(this, name, size, slot); }
};

class ACPlugin : public IACPPluginHost {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	HINSTANCE__ * hModule;
	void * getName;
	void * init;
	void * shutdown;
	void * p_update;
	void * p_onGui;
	void * getControls;
	CarAvatar * carAvatar;
	Car * car;
	Sim * sim;
	inline ACPlugin()  { }
	inline void ctor(Sim * aSim, HINSTANCE__ * module) { typedef void (*_fpt)(ACPlugin *pthis, Sim *, HINSTANCE__ *); _fpt _f=(_fpt)_drva(784032); _f(this, aSim, module); }
	virtual ~ACPlugin();
	inline void dtor() { typedef void (*_fpt)(ACPlugin *pthis); _fpt _f=(_fpt)_drva(784720); _f(this); }
	inline void update(ACCarState * state, float dt) { typedef void (*_fpt)(ACPlugin *pthis, ACCarState *, float); _fpt _f=(_fpt)_drva(785920); return _f(this, state, dt); }
	inline void onGui(ACPluginContext * cc) { typedef void (*_fpt)(ACPlugin *pthis, ACPluginContext *); _fpt _f=(_fpt)_drva(785184); return _f(this, cc); }
	virtual HWND__ * getHwnd_vf0();
	inline HWND__ * getHwnd_impl() { typedef HWND__ * (*_fpt)(ACPlugin *pthis); _fpt _f=(_fpt)_drva(785168); return _f(this); }
	inline HWND__ * getHwnd() { return getHwnd_vf0(); }
	virtual HINSTANCE__ * getHInstance_vf1();
	inline HINSTANCE__ * getHInstance_impl() { typedef HINSTANCE__ * (*_fpt)(ACPlugin *pthis); _fpt _f=(_fpt)_drva(785152); return _f(this); }
	inline HINSTANCE__ * getHInstance() { return getHInstance_vf1(); }
	virtual void setABS_vf2(float value);
	inline void setABS_impl(float value) { typedef void (*_fpt)(ACPlugin *pthis, float); _fpt _f=(_fpt)_drva(785200); return _f(this, value); }
	inline void setABS(float value) { return setABS_vf2(value); }
	virtual void setTC_vf3(float value);
	inline void setTC_impl(float value) { typedef void (*_fpt)(ACPlugin *pthis, float); _fpt _f=(_fpt)_drva(785856); return _f(this, value); }
	inline void setTC(float value) { return setTC_vf3(value); }
	virtual void setStabilityControl_vf4(float value);
	inline void setStabilityControl_impl(float value) { typedef void (*_fpt)(ACPlugin *pthis, float); _fpt _f=(_fpt)_drva(785440); return _f(this, value); }
	inline void setStabilityControl(float value) { return setStabilityControl_vf4(value); }
	virtual void setIdealLine_vf5(bool value, bool isFromPhysicsThread);
	inline void setIdealLine_impl(bool value, bool isFromPhysicsThread) { typedef void (*_fpt)(ACPlugin *pthis, bool, bool); _fpt _f=(_fpt)_drva(785296); return _f(this, value, isFromPhysicsThread); }
	inline void setIdealLine(bool value, bool isFromPhysicsThread) { return setIdealLine_vf5(value, isFromPhysicsThread); }
	virtual void setAutoShift_vf6(bool value);
	inline void setAutoShift_impl(bool value) { typedef void (*_fpt)(ACPlugin *pthis, bool); _fpt _f=(_fpt)_drva(785264); return _f(this, value); }
	inline void setAutoShift(bool value) { return setAutoShift_vf6(value); }
	virtual void setBrakeBias_vf7(float value);
	inline void setBrakeBias_impl(float value) { typedef void (*_fpt)(ACPlugin *pthis, float); _fpt _f=(_fpt)_drva(785280); return _f(this, value); }
	inline void setBrakeBias(float value) { return setBrakeBias_vf7(value); }
	virtual void setSystemMessage_vf8(wchar_t * message, wchar_t * description, bool isFromPhysicsThread);
	inline void setSystemMessage_impl(wchar_t * message, wchar_t * description, bool isFromPhysicsThread) { typedef void (*_fpt)(ACPlugin *pthis, wchar_t *, wchar_t *, bool); _fpt _f=(_fpt)_drva(785456); return _f(this, message, description, isFromPhysicsThread); }
	inline void setSystemMessage(wchar_t * message, wchar_t * description, bool isFromPhysicsThread) { return setSystemMessage_vf8(message, description, isFromPhysicsThread); }
};

struct TyreModelData {
public:
	int version;
	float Dy0;
	float Dy1;
	float Dx0;
	float Dx1;
	float Fz0;
	float flexK;
	float speedSensitivity;
	float relaxationLength;
	float rr0;
	float rr1;
	float rr_sa;
	float rr_sr;
	float rr_slip;
	float camberGain;
	float pressureSpringGain;
	float pressureFlexGain;
	float pressureRRGain;
	float pressureGainD;
	float idealPressure;
	float pressureRef;
	Curve wearCurve;
	float dcamber0;
	float dcamber1;
	Curve dyLoadCurve;
	Curve dxLoadCurve;
	float lsMultY;
	float lsExpY;
	float lsMultX;
	float lsExpX;
	float maxWearKM;
	float maxWearMult;
	float asy;
	float cfXmult;
	float brakeDXMod;
	Curve dCamberCurve;
	bool useSmoothDCamberCurve;
	float combinedFactor;
	inline TyreModelData()  { }
	inline void ctor(TyreModelData & __that) { typedef void (*_fpt)(TyreModelData *pthis, TyreModelData &); _fpt _f=(_fpt)_drva(2609552); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(TyreModelData *pthis); _fpt _f=(_fpt)_drva(2547280); _f(this); }
	inline void dtor() { typedef void (*_fpt)(TyreModelData *pthis); _fpt _f=(_fpt)_drva(2550176); _f(this); }
	inline TyreModelData & operator=(TyreModelData & __that) { typedef TyreModelData & (*_fpt)(TyreModelData *pthis, TyreModelData &); _fpt _f=(_fpt)_drva(2610288); return _f(this, __that); }
};

struct DynamicTempData {
public:
	Curve temperatureCurve;
	double temperatureStartTime;
	float baseRoad;
	float baseAir;
	inline DynamicTempData()  { }
	inline void ctor() { typedef void (*_fpt)(DynamicTempData *pthis); _fpt _f=(_fpt)_drva(1256656); _f(this); }
	inline void dtor() { typedef void (*_fpt)(DynamicTempData *pthis); _fpt _f=(_fpt)_drva(1258144); _f(this); }
	inline DynamicTempData & operator=(DynamicTempData & __that) { typedef DynamicTempData & (*_fpt)(DynamicTempData *pthis, DynamicTempData &); _fpt _f=(_fpt)_drva(1259120); return _f(this, __that); }
};

struct acEngineData {
public:
	Curve powerCurve;
	Curve coastCurve;
	float coast2;
	float coast1;
	float coast0;
	bool useCoastCurve;
	int minimum;
	int limiter;
	int limiterCycles;
	float overlapFreq;
	float overlapGain;
	float overlapIdealRPM;
	inline acEngineData()  { }
	inline void ctor() { typedef void (*_fpt)(acEngineData *pthis); _fpt _f=(_fpt)_drva(2643104); _f(this); }
	inline void dtor() { typedef void (*_fpt)(acEngineData *pthis); _fpt _f=(_fpt)_drva(2643504); _f(this); }
};

struct ClutchSequence {
public:
	Curve clutchCurve;
	float currentTime;
	bool isDone;
	inline ClutchSequence()  { }
	inline void ctor(Curve & c) { typedef void (*_fpt)(ClutchSequence *pthis, Curve &); _fpt _f=(_fpt)_drva(2851616); _f(this, c); }
	inline void dtor() { typedef void (*_fpt)(ClutchSequence *pthis); _fpt _f=(_fpt)_drva(1258144); _f(this); }
	inline ClutchSequence & operator=(ClutchSequence & __that) { typedef ClutchSequence & (*_fpt)(ClutchSequence *pthis, ClutchSequence &); _fpt _f=(_fpt)_drva(2851840); return _f(this, __that); }
};

struct BrakeDisc {
public:
	float t;
	float coolTransfer;
	float torqueK;
	float coolSpeedFactor;
	Curve perfCurve;
	inline BrakeDisc()  { }
	inline void ctor() { typedef void (*_fpt)(BrakeDisc *pthis); _fpt _f=(_fpt)_drva(2538976); _f(this); }
	inline void dtor() { typedef void (*_fpt)(BrakeDisc *pthis); _fpt _f=(_fpt)_drva(2547808); _f(this); }
};

struct ksgui_ListBoxRowData {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	unsigned int category;
	vec4f backColor;
	std::vector<vec4f,std::allocator<vec4f> > forecolors;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > columns;
	inline ksgui_ListBoxRowData()  { }
	inline void ctor(ksgui_ListBoxRowData & __that) { typedef void (*_fpt)(ksgui_ListBoxRowData *pthis, ksgui_ListBoxRowData &); _fpt _f=(_fpt)_drva(2395072); _f(this, __that); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * text) { typedef void (*_fpt)(ksgui_ListBoxRowData *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(454352); _f(this, text); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getColumn(unsigned int columnIndex) { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(ksgui_ListBoxRowData *pthis, unsigned int); _fpt _f=(_fpt)_drva(2398208); return _f(this, columnIndex); }
	inline void dtor() { typedef void (*_fpt)(ksgui_ListBoxRowData *pthis); _fpt _f=(_fpt)_drva(454992); _f(this); }
};

class mat44f {
public:
	float M11;
	float M12;
	float M13;
	float M14;
	float M21;
	float M22;
	float M23;
	float M24;
	float M31;
	float M32;
	float M33;
	float M34;
	float M41;
	float M42;
	float M43;
	float M44;
	inline mat44f()  { }
	inline bool isFinite() { typedef bool (*_fpt)(mat44f *pthis); _fpt _f=(_fpt)_drva(2072368); return _f(this); }
	inline vec3f getScale() { typedef vec3f (*_fpt)(mat44f *pthis); _fpt _f=(_fpt)_drva(2071424); return _f(this); }
	inline void setScale(vec3f & s) { typedef void (*_fpt)(mat44f *pthis, vec3f &); _fpt _f=(_fpt)_drva(2134544); return _f(this, s); }
	inline void setTranslation(vec3f & value) { typedef void (*_fpt)(mat44f *pthis, vec3f &); _fpt _f=(_fpt)_drva(394240); return _f(this, value); }
	inline vec3f getTranslation() { typedef vec3f (*_fpt)(mat44f *pthis); _fpt _f=(_fpt)_drva(296464); return _f(this); }
	inline bool isIdentity() { typedef bool (*_fpt)(mat44f *pthis); _fpt _f=(_fpt)_drva(1817520); return _f(this); }
	inline void setFromHeadingUp(vec3f & h, vec3f & u) { typedef void (*_fpt)(mat44f *pthis, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(393968); return _f(this, h, u); }
	inline vec3f toEuler() { typedef vec3f (*_fpt)(mat44f *pthis); _fpt _f=(_fpt)_drva(340800); return _f(this); }
	inline void print() { typedef void (*_fpt)(mat44f *pthis); _fpt _f=(_fpt)_drva(938816); return _f(this); }
};

class JoypadManager {
public:
	std::unique_ptr<Joypad,std::default_delete<Joypad> > joypad;
	inline JoypadManager()  { }
	inline void ctor() { typedef void (*_fpt)(JoypadManager *pthis); _fpt _f=(_fpt)_drva(2375584); _f(this); }
	inline Joypad * getJoypad() { typedef Joypad * (*_fpt)(JoypadManager *pthis); _fpt _f=(_fpt)_drva(100192); return _f(this); }
	inline void dtor() { typedef void (*_fpt)(JoypadManager *pthis); _fpt _f=(_fpt)_drva(2367136); _f(this); }
};

struct OnSessionEndEvent {
public:
	Session currentSession;
	SessionResult result;
	inline OnSessionEndEvent()  { }
	inline void ctor(OnSessionEndEvent & __that) { typedef void (*_fpt)(OnSessionEndEvent *pthis, OnSessionEndEvent &); _fpt _f=(_fpt)_drva(1256720); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(OnSessionEndEvent *pthis); _fpt _f=(_fpt)_drva(610032); _f(this); }
	inline void dtor() { typedef void (*_fpt)(OnSessionEndEvent *pthis); _fpt _f=(_fpt)_drva(610896); _f(this); }
};

struct DebugLine {
public:
	vec3f p0;
	vec3f p1;
	vec4f color;
	float seconds;
	inline DebugLine()  { }
};

class Triangle {
public:
	vec3f points[0x3];
	plane4f plane;
	inline Triangle()  { }
	inline void ctor(vec3f & p1, vec3f & p2, vec3f & p3) { typedef void (*_fpt)(Triangle *pthis, vec3f &, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(2144864); _f(this, p1, p2, p3); }
	virtual ~Triangle();
	inline void dtor() { typedef void (*_fpt)(Triangle *pthis); _fpt _f=(_fpt)_drva(2145440); _f(this); }
	inline float computeArea() { typedef float (*_fpt)(Triangle *pthis); _fpt _f=(_fpt)_drva(2145456); return _f(this); }
};

struct ABS {
public:
	bool isPresent;
	bool isActive;
	float slipRatioLimit;
	float frequency;
	int channels;
	Car * car;
	float timeAccumulator;
	Curve valueCurve;
	unsigned int currentMode;
	float currentValue;
	inline ABS()  { }
	inline void dtor() { typedef void (*_fpt)(ABS *pthis); _fpt _f=(_fpt)_drva(2681552); _f(this); }
	inline void init(Car * acar) { typedef void (*_fpt)(ABS *pthis, Car *); _fpt _f=(_fpt)_drva(2681936); return _f(this, acar); }
	inline void step(float td) { typedef void (*_fpt)(ABS *pthis, float); _fpt _f=(_fpt)_drva(2684432); return _f(this, td); }
	inline void cycleMode(int value) { typedef void (*_fpt)(ABS *pthis, int); _fpt _f=(_fpt)_drva(2681568); return _f(this, value); }
	inline std::pair<unsigned int,unsigned int> getCurrentMode() { typedef std::pair<unsigned int,unsigned int> (*_fpt)(ABS *pthis); _fpt _f=(_fpt)_drva(2681792); return _f(this); }
	inline bool isInAction() { typedef bool (*_fpt)(ABS *pthis); _fpt _f=(_fpt)_drva(2684368); return _f(this); }
};

class BrushSlipProvider {
public:
	BrushTyreModel brushModel;
	float asy;
	int version;
	float maximum;
	float maxSlip;
	inline BrushSlipProvider()  { }
	inline void ctor(float maxAngle, float xu, float flex) { typedef void (*_fpt)(BrushSlipProvider *pthis, float, float, float); _fpt _f=(_fpt)_drva(2830208); _f(this, maxAngle, xu, flex); }
	inline void ctor() { typedef void (*_fpt)(BrushSlipProvider *pthis); _fpt _f=(_fpt)_drva(2830384); _f(this); }
	inline void dtor() { typedef void (*_fpt)(BrushSlipProvider *pthis); _fpt _f=(_fpt)_drva(2830448); _f(this); }
	virtual TyreSlipOutput getSlipForce_vf0(TyreSlipInput & input, bool useasy);
	inline TyreSlipOutput getSlipForce_impl(TyreSlipInput & input, bool useasy) { typedef TyreSlipOutput (*_fpt)(BrushSlipProvider *pthis, TyreSlipInput &, bool); _fpt _f=(_fpt)_drva(2830736); return _f(this, input, useasy); }
	inline TyreSlipOutput getSlipForce(TyreSlipInput & input, bool useasy) { return getSlipForce_vf0(input, useasy); }
	inline void calcMaximum(float load, float * maximum, float * max_slip) { typedef void (*_fpt)(BrushSlipProvider *pthis, float, float *, float *); _fpt _f=(_fpt)_drva(2830480); return _f(this, load, maximum, max_slip); }
	inline void recomputeMaximum() { typedef void (*_fpt)(BrushSlipProvider *pthis); _fpt _f=(_fpt)_drva(2830896); return _f(this); }
	inline BrushSlipProvider & operator=(BrushSlipProvider & __that) { typedef BrushSlipProvider & (*_fpt)(BrushSlipProvider *pthis, BrushSlipProvider &); _fpt _f=(_fpt)_drva(2610224); return _f(this, __that); }
};

class UDPSocket {
public:
	unsigned __int64 soc;
	bool isBlocking;
	std::vector<std::function<void __cdecl(UDPMessage const &)>,std::allocator<std::function<void __cdecl(UDPMessage const &)> > > listeners;
	unsigned short ping;
	bool shutdownFlag;
	double lastPingTime;
	inline UDPSocket()  { }
	inline void ctor() { typedef void (*_fpt)(UDPSocket *pthis); _fpt _f=(_fpt)_drva(2477984); _f(this); }
	inline void dtor() { typedef void (*_fpt)(UDPSocket *pthis); _fpt _f=(_fpt)_drva(2478064); _f(this); }
	inline void setBlockingMode(bool imode) { typedef void (*_fpt)(UDPSocket *pthis, bool); _fpt _f=(_fpt)_drva(2479568); return _f(this, imode); }
	inline void addListener(std::function<void __cdecl(UDPMessage const &)> * listener) { typedef void (*_fpt)(UDPSocket *pthis, std::function<void __cdecl(UDPMessage const &)> *); _fpt _f=(_fpt)_drva(2478736); return _f(this, listener); }
	inline int receive(int maxPackets) { typedef int (*_fpt)(UDPSocket *pthis, int); _fpt _f=(_fpt)_drva(2479104); return _f(this, maxPackets); }
	inline void send(void * data, int length, sockaddr_in target) { typedef void (*_fpt)(UDPSocket *pthis, void *, int, sockaddr_in); _fpt _f=(_fpt)_drva(2479488); return _f(this, data, length, target); }
};

struct DebugString {
public:
	vec3f p;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > text;
	vec4f color;
	float seconds;
	float scale;
	int stringId;
	inline DebugString()  { }
	inline void ctor(DebugString & __that) { typedef void (*_fpt)(DebugString *pthis, DebugString &); _fpt _f=(_fpt)_drva(508160); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(DebugString *pthis); _fpt _f=(_fpt)_drva(2353488); _f(this); }
};

struct TractionControl {
public:
	bool isPresent;
	bool isActive;
	float slipRatioLimit;
	bool isInAction;
	float frequency;
	Car * car;
	float minSpeedMS;
	float timeAccumulator;
	unsigned int currentMode;
	bool lastValue;
	Curve valueCurve;
	inline TractionControl()  { }
	inline void dtor() { typedef void (*_fpt)(TractionControl *pthis); _fpt _f=(_fpt)_drva(2685136); _f(this); }
	inline void init(Car * acar) { typedef void (*_fpt)(TractionControl *pthis, Car *); _fpt _f=(_fpt)_drva(2685504); return _f(this, acar); }
	inline void step(float dt) { typedef void (*_fpt)(TractionControl *pthis, float); _fpt _f=(_fpt)_drva(2687488); return _f(this, dt); }
	inline void cycleMode(int value) { typedef void (*_fpt)(TractionControl *pthis, int); _fpt _f=(_fpt)_drva(2685152); return _f(this, value); }
	inline std::pair<unsigned int,unsigned int> getCurrentMode() { typedef std::pair<unsigned int,unsigned int> (*_fpt)(TractionControl *pthis); _fpt _f=(_fpt)_drva(2685360); return _f(this); }
};

struct DynamicControllerStage {
public:
	DynamicControllerInput inputVar;
	DynamicControllerCombinatorMode combinatorMode;
	Curve lut;
	float filter;
	float upLimit;
	float downLimit;
	float currentValue;
	float constValue;
	inline DynamicControllerStage()  { }
	inline void ctor(DynamicControllerStage & __that) { typedef void (*_fpt)(DynamicControllerStage *pthis, DynamicControllerStage &); _fpt _f=(_fpt)_drva(2546288); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(DynamicControllerStage *pthis); _fpt _f=(_fpt)_drva(2819968); _f(this); }
	inline void dtor() { typedef void (*_fpt)(DynamicControllerStage *pthis); _fpt _f=(_fpt)_drva(2549824); _f(this); }
};

struct AutoBlip {
public:
	bool isActive;
	Car * car;
	Curve blipProfile;
	double blipStartTime;
	bool isElectronic;
	double blipPerformTime;
	inline AutoBlip()  { }
	inline void dtor() { typedef void (*_fpt)(AutoBlip *pthis); _fpt _f=(_fpt)_drva(2547808); _f(this); }
	inline void init(Car * acar) { typedef void (*_fpt)(AutoBlip *pthis, Car *); _fpt _f=(_fpt)_drva(2857232); return _f(this, acar); }
	inline void step(float dt) { typedef void (*_fpt)(AutoBlip *pthis, float); _fpt _f=(_fpt)_drva(2858736); return _f(this, dt); }
	inline void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * carModel) { typedef void (*_fpt)(AutoBlip *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2857392); return _f(this, carModel); }
};

class MaterialResource {
public:
	int slot;
	Texture texture;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	inline MaterialResource()  { }
	inline void ctor(MaterialResource & __that) { typedef void (*_fpt)(MaterialResource *pthis, MaterialResource &); _fpt _f=(_fpt)_drva(2137040); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(MaterialResource *pthis); _fpt _f=(_fpt)_drva(2137904); _f(this); }
};

struct WingData {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	float chord;
	float span;
	vec3f position;
	Curve lutAOA_CL;
	Curve lutAOA_CD;
	Curve lutGH_CL;
	Curve lutGH_CD;
	float clGain;
	float cdGain;
	bool hasController;
	float yawGain;
	float area;
	bool isVertical;
	inline WingData()  { }
	inline void ctor(WingData & __that) { typedef void (*_fpt)(WingData *pthis, WingData &); _fpt _f=(_fpt)_drva(845872); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(WingData *pthis); _fpt _f=(_fpt)_drva(2826944); _f(this); }
	inline void dtor() { typedef void (*_fpt)(WingData *pthis); _fpt _f=(_fpt)_drva(847680); _f(this); }
};

class ResourceStore {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,Texture,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,Texture> > > store;
	GraphicsManager * graphics;
	inline ResourceStore()  { }
	inline void ctor(GraphicsManager * rm) { typedef void (*_fpt)(ResourceStore *pthis, GraphicsManager *); _fpt _f=(_fpt)_drva(2097040); _f(this, rm); }
	virtual ~ResourceStore();
	inline void dtor() { typedef void (*_fpt)(ResourceStore *pthis); _fpt _f=(_fpt)_drva(2097120); _f(this); }
	inline Texture getTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename, bool onlyExisting) { typedef Texture (*_fpt)(ResourceStore *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, bool); _fpt _f=(_fpt)_drva(2097360); return _f(this, filename, onlyExisting); }
	inline Texture getTextureFromBuffer(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, unsigned char * buffer, int size) { typedef Texture (*_fpt)(ResourceStore *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, unsigned char *, int); _fpt _f=(_fpt)_drva(2097744); return _f(this, name, buffer, size); }
	inline bool hasTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef bool (*_fpt)(ResourceStore *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2098112); return _f(this, name); }
};

class Font {
public:
	float scale;
	bool shadowed;
	float shadowPixelDistance;
	void * kid;
	unsigned int color;
	float currentAlpha;
	inline Font()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & fontFamily, eFontType fontType, float size, bool italic, bool bold) { typedef void (*_fpt)(Font *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, eFontType, float, bool, bool); _fpt _f=(_fpt)_drva(2099808); _f(this, fontFamily, fontType, size, italic, bold); }
	inline void ctor(eFontType fontType, float size, bool italic, bool bold) { typedef void (*_fpt)(Font *pthis, eFontType, float, bool, bool); _fpt _f=(_fpt)_drva(2100144); _f(this, fontType, size, italic, bold); }
	inline void dtor() { typedef void (*_fpt)(Font *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void blitString(float x, float y, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & s, float ascale, eFontAlign align) { typedef void (*_fpt)(Font *pthis, float, float, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, float, eFontAlign); _fpt _f=(_fpt)_drva(2100768); return _f(this, x, y, s, ascale, align); }
	inline void setColor(vec4f & cc) { typedef void (*_fpt)(Font *pthis, vec4f &); _fpt _f=(_fpt)_drva(2102576); return _f(this, cc); }
	inline void setColor(float r, float g, float b, float a) { typedef void (*_fpt)(Font *pthis, float, float, float, float); _fpt _f=(_fpt)_drva(2102704); return _f(this, r, g, b, a); }
};

class SCTM : public ITyreModel {
public:
	float lsMultY;
	float lsExpY;
	float lsMultX;
	float lsExpX;
	float Fz0;
	float maxSlip0;
	float maxSlip1;
	float asy;
	float falloffSpeed;
	float speedSensitivity;
	float camberGain;
	float dcamber0;
	float dcamber1;
	float cfXmult;
	Curve dyLoadCurve;
	Curve dxLoadCurve;
	float pressureCfGain;
	float brakeDXMod;
	Curve dCamberCurve;
	bool useSmoothDCamberCurve;
	float dCamberBlend;
	float combinedFactor;
	float dy0;
	float dx0;
	float pacE;
	float pacCf;
	float pacFlex;
	inline SCTM()  { }
	inline void ctor() { typedef void (*_fpt)(SCTM *pthis); _fpt _f=(_fpt)_drva(4503744); _f(this); }
	virtual ~SCTM();
	inline void dtor() { typedef void (*_fpt)(SCTM *pthis); _fpt _f=(_fpt)_drva(4503920); _f(this); }
	virtual TyreModelOutput solve_vf1(TyreModelInput & in);
	inline TyreModelOutput solve_impl(TyreModelInput & in) { typedef TyreModelOutput (*_fpt)(SCTM *pthis, TyreModelInput &); _fpt _f=(_fpt)_drva(4504608); return _f(this, in); }
	inline TyreModelOutput solve(TyreModelInput & in) { return solve_vf1(in); }
	inline float getStaticDY(float load) { typedef float (*_fpt)(SCTM *pthis, float); _fpt _f=(_fpt)_drva(4504496); return _f(this, load); }
	inline float getStaticDX(float load) { typedef float (*_fpt)(SCTM *pthis, float); _fpt _f=(_fpt)_drva(4504368); return _f(this, load); }
	inline float getPureFY(float D, float cf, float load, float slip) { typedef float (*_fpt)(SCTM *pthis, float, float, float, float); _fpt _f=(_fpt)_drva(4504176); return _f(this, D, cf, load, slip); }
};

struct Telemetry {
public:
	std::vector<TelemetryChannel,std::allocator<TelemetryChannel> > channels;
	bool isEnabled;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > driverName;
	bool debugPhysics;
	bool debugAI;
	Car * car;
	float totLift;
	float totDrag;
	float lapBeacon;
	float carSpeedMS;
	float roty;
	float steerAngle;
	float gas;
	float brake;
	float gear;
	float clutch;
	float aiTargetSpeed;
	float aiBGTargetSpeed;
	float aiOutsideOffset;
	float rpms;
	float susTravel[0x4];
	float wheelSpeed[0x4];
	TimeTransponder * timeTransponder;
	bool exportEntireSession;
	float awdFrontShare;
	float awdCenterLock;
	float oversteerFactor;
	float rearSpeedRatio;
	float ebbInstant;
	float clutchOpenState;
	float engineVel;
	float driveVel;
	float rootVel;
	float clutchSlip;
	float avgSurfaceTemps[0x4];
	float practicalTemps[0x4];
	float splinePosition;
	inline Telemetry()  { }
	inline void dtor() { typedef void (*_fpt)(Telemetry *pthis); _fpt _f=(_fpt)_drva(2866608); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(Telemetry *pthis, Car *); _fpt _f=(_fpt)_drva(2867216); return _f(this, car); }
	inline void step(float td) { typedef void (*_fpt)(Telemetry *pthis, float); _fpt _f=(_fpt)_drva(2880944); return _f(this, td); }
	inline void save(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename) { typedef void (*_fpt)(Telemetry *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2879600); return _f(this, filename); }
	inline void ctor() { typedef void (*_fpt)(Telemetry *pthis); _fpt _f=(_fpt)_drva(2546480); _f(this); }
};

struct TyreThermalModel {
public:
	int elements;
	int stripes;
	std::vector<TyreThermalPatch,std::allocator<TyreThermalPatch> > patches;
	double phase;
	TyrePatchData patchData;
	float coreTemp;
	Curve performanceCurve;
	bool isActive;
	float thermalMultD;
	float practicalTemp;
	float camberSpreadK;
	Car * car;
	float coreTInput;
	inline TyreThermalModel()  { }
	inline void init(int a_elements, int a_stripes, Car * car) { typedef void (*_fpt)(TyreThermalModel *pthis, int, int, Car *); _fpt _f=(_fpt)_drva(2810224); return _f(this, a_elements, a_stripes, car); }
	inline void step(float dt, float angularSpeed, float camberRAD) { typedef void (*_fpt)(TyreThermalModel *pthis, float, float, float); _fpt _f=(_fpt)_drva(2810688); return _f(this, dt, angularSpeed, camberRAD); }
	inline void addThermalInput(float xpos, float pressureRel, float temp) { typedef void (*_fpt)(TyreThermalModel *pthis, float, float, float); _fpt _f=(_fpt)_drva(2805904); return _f(this, xpos, pressureRel, temp); }
	inline TyreThermalPatch & getPatchAt(int x, int y) { typedef TyreThermalPatch & (*_fpt)(TyreThermalModel *pthis, int, int); _fpt _f=(_fpt)_drva(2809776); return _f(this, x, y); }
	inline float getCurrentCPTemp(float camber) { typedef float (*_fpt)(TyreThermalModel *pthis, float); _fpt _f=(_fpt)_drva(2808800); return _f(this, camber); }
	inline float getCorrectedD(float d, float camberRAD) { typedef float (*_fpt)(TyreThermalModel *pthis, float, float); _fpt _f=(_fpt)_drva(2808768); return _f(this, d, camberRAD); }
	inline float getAvgSurfaceTemp() { typedef float (*_fpt)(TyreThermalModel *pthis); _fpt _f=(_fpt)_drva(2808720); return _f(this); }
	inline void reset() { typedef void (*_fpt)(TyreThermalModel *pthis); _fpt _f=(_fpt)_drva(2810304); return _f(this); }
	inline void getIMO(float * out) { typedef void (*_fpt)(TyreThermalModel *pthis, float *); _fpt _f=(_fpt)_drva(2809264); return _f(this, out); }
	inline void setTemperature(float optimumTemp) { typedef void (*_fpt)(TyreThermalModel *pthis, float); _fpt _f=(_fpt)_drva(2810640); return _f(this, optimumTemp); }
	inline float getPracticalTemp(float camberRAD) { typedef float (*_fpt)(TyreThermalModel *pthis, float); _fpt _f=(_fpt)_drva(2810176); return _f(this, camberRAD); }
	inline void addThermalCoreInput(float temp) { typedef void (*_fpt)(TyreThermalModel *pthis, float); _fpt _f=(_fpt)_drva(2805872); return _f(this, temp); }
	inline void buildTyre() { typedef void (*_fpt)(TyreThermalModel *pthis); _fpt _f=(_fpt)_drva(2806528); return _f(this); }
	inline void ctor() { typedef void (*_fpt)(TyreThermalModel *pthis); _fpt _f=(_fpt)_drva(2547584); _f(this); }
	inline void dtor() { typedef void (*_fpt)(TyreThermalModel *pthis); _fpt _f=(_fpt)_drva(2550256); _f(this); }
};

struct CarColliderManager {
public:
	bool isLive;
	std::vector<CarCollisionBox,std::allocator<CarCollisionBox> > boxes;
	FileChangeObserver observer;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > carModel;
	IRigidBody * carBody;
	Car * car;
	inline CarColliderManager()  { }
	inline void ctor() { typedef void (*_fpt)(CarColliderManager *pthis); _fpt _f=(_fpt)_drva(2545584); _f(this); }
	inline void dtor() { typedef void (*_fpt)(CarColliderManager *pthis); _fpt _f=(_fpt)_drva(2766048); _f(this); }
	inline void init(Car * acar) { typedef void (*_fpt)(CarColliderManager *pthis, Car *); _fpt _f=(_fpt)_drva(2766672); return _f(this, acar); }
	inline CarCollisionBox getBox(int index) { typedef CarCollisionBox (*_fpt)(CarColliderManager *pthis, int); _fpt _f=(_fpt)_drva(2766528); return _f(this, index); }
	inline void step(float dt) { typedef void (*_fpt)(CarColliderManager *pthis, float); _fpt _f=(_fpt)_drva(2768208); return _f(this, dt); }
	inline void loadINI() { typedef void (*_fpt)(CarColliderManager *pthis); _fpt _f=(_fpt)_drva(2766752); return _f(this); }
};

struct PenaltyManager {
public:
	Event<OnPenaltyEvent> evOnPenalty;
	Car * car;
	PenaltyType pendingPenaltyType;
	bool isServingPitPenalty;
	bool rearmPitPenalty;
	short pitPenaltyLaps;
	std::vector<PenaltyRecord,std::allocator<PenaltyRecord> > penaltyRecords;
	unsigned int penaltySecs;
	float inPitMeterCounter;
	inline PenaltyManager()  { }
	inline void dtor() { typedef void (*_fpt)(PenaltyManager *pthis); _fpt _f=(_fpt)_drva(2513088); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(PenaltyManager *pthis, Car *); _fpt _f=(_fpt)_drva(2514000); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(PenaltyManager *pthis, float); _fpt _f=(_fpt)_drva(2514448); return _f(this, dt); }
	inline void reset() { typedef void (*_fpt)(PenaltyManager *pthis); _fpt _f=(_fpt)_drva(2514240); return _f(this); }
	inline void resetPenalty() { typedef void (*_fpt)(PenaltyManager *pthis); _fpt _f=(_fpt)_drva(2514320); return _f(this); }
	inline bool checkBlackFlag() { typedef bool (*_fpt)(PenaltyManager *pthis); _fpt _f=(_fpt)_drva(2513936); return _f(this); }
	inline void addJumpStartPenalty() { typedef void (*_fpt)(PenaltyManager *pthis); _fpt _f=(_fpt)_drva(2513552); return _f(this); }
	inline void decreasePitPenaltyLaps(bool isValidLap) { typedef void (*_fpt)(PenaltyManager *pthis, bool); _fpt _f=(_fpt)_drva(2513968); return _f(this, isValidLap); }
};

class INIReader {
public:
	bool ready;
	bool suppressErrorReporting;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > filename;
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,INISection,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,INISection> > > sections;
	bool m_isEncrypted;
	bool verbose;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > code;
	inline INIReader()  { }
	inline void ctor(INIReader & __that) { typedef void (*_fpt)(INIReader *pthis, INIReader &); _fpt _f=(_fpt)_drva(776000); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(INIReader *pthis); _fpt _f=(_fpt)_drva(2310160); _f(this); }
	inline void ctor(std::basic_stringstream<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & stream) { typedef void (*_fpt)(INIReader *pthis, std::basic_stringstream<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2310928); _f(this, stream); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2310304); _f(this, ifilename); }
	virtual ~INIReader();
	inline void dtor() { typedef void (*_fpt)(INIReader *pthis); _fpt _f=(_fpt)_drva(2311168); _f(this); }
	inline bool hasSection(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section) { typedef bool (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2322608); return _f(this, section); }
	inline bool hasKey(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef bool (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2322176); return _f(this, section, key); }
	inline int getHex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef int (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2316912); return _f(this, section, key); }
	inline int getInt(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef int (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2317424); return _f(this, section, key); }
	inline float getFloat(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef float (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2316480); return _f(this, section, key); }
	inline void getSections(std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > & outSections) { typedef void (*_fpt)(INIReader *pthis, std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > &); _fpt _f=(_fpt)_drva(2318272); return _f(this, outSections); }
	inline void getKeyes(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * section, std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > & keyes) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > &); _fpt _f=(_fpt)_drva(2317840); return _f(this, section, keyes); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getString(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2318576); return _f(this, section, key); }
	inline vec2f getFloat2(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef vec2f (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2316128); return _f(this, section, key); }
	inline vec3f getFloat3(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef vec3f (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2316192); return _f(this, section, key); }
	inline vec4f getFloat4(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef vec4f (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2316384); return _f(this, section, key); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getFileName() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(INIReader *pthis); _fpt _f=(_fpt)_drva(2316064); return _f(this); }
	inline void setVerboseMode(bool mode) { typedef void (*_fpt)(INIReader *pthis, bool); _fpt _f=(_fpt)_drva(2327152); return _f(this, mode); }
	inline Curve getCurve(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key) { typedef Curve (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2314304); return _f(this, section, key); }
	inline void printCode() { typedef void (*_fpt)(INIReader *pthis); _fpt _f=(_fpt)_drva(2327120); return _f(this); }
	inline void load(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2322752); return _f(this, filename); }
	inline void loadEncrypt(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataFile) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2323824); return _f(this, filename, dataFile); }
	inline void errorReport(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * message) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2313840); return _f(this, message); }
	inline void parse() { typedef void (*_fpt)(INIReader *pthis); _fpt _f=(_fpt)_drva(2324096); return _f(this); }
	inline void getVector2(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key, float & x, float & y) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, float &, float &); _fpt _f=(_fpt)_drva(2319104); return _f(this, section, key, x, y); }
	inline void getVector3(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key, float & x, float & y, float & z) { typedef void (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, float &, float &, float &); _fpt _f=(_fpt)_drva(2319680); return _f(this, section, key, x, y, z); }
	inline bool getVector4(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key, float & x, float & y, float & z, float & w) { typedef bool (*_fpt)(INIReader *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, float &, float &, float &, float &); _fpt _f=(_fpt)_drva(2321104); return _f(this, section, key, x, y, z, w); }
	inline INIReader & operator=(INIReader & __that) { typedef INIReader & (*_fpt)(INIReader *pthis, INIReader &); _fpt _f=(_fpt)_drva(776720); return _f(this, __that); }
};

class TCPSocket {
public:
	BufferedChannel<unsigned __int64> chNewConnections;
	std::vector<std::function<void __cdecl(UDPMessage const &)>,std::allocator<std::function<void __cdecl(UDPMessage const &)> > > listeners;
	std::thread listenerThread;
	bool isExiting;
	unsigned char * recvBuffer;
	unsigned __int64 soc;
	unsigned __int64 listenSock;
	bool isBlocking;
	bool isServer;
	TCPQueue buffer;
	inline TCPSocket()  { }
	inline void ctor() { typedef void (*_fpt)(TCPSocket *pthis); _fpt _f=(_fpt)_drva(2481840); _f(this); }
	inline void dtor() { typedef void (*_fpt)(TCPSocket *pthis); _fpt _f=(_fpt)_drva(2482080); _f(this); }
	inline bool connect(IPAddress & target) { typedef bool (*_fpt)(TCPSocket *pthis, IPAddress &); _fpt _f=(_fpt)_drva(2482592); return _f(this, target); }
	inline void setBlockingMode(bool imode) { typedef void (*_fpt)(TCPSocket *pthis, bool); _fpt _f=(_fpt)_drva(2484144); return _f(this, imode); }
	inline void addListener(std::function<void __cdecl(UDPMessage const &)> * listener) { typedef void (*_fpt)(TCPSocket *pthis, std::function<void __cdecl(UDPMessage const &)> *); _fpt _f=(_fpt)_drva(2482464); return _f(this, listener); }
	inline int receive(int maxPackets) { typedef int (*_fpt)(TCPSocket *pthis, int); _fpt _f=(_fpt)_drva(2483216); return _f(this, maxPackets); }
	inline void send(void * data, int length, sockaddr_in target) { typedef void (*_fpt)(TCPSocket *pthis, void *, int, sockaddr_in); _fpt _f=(_fpt)_drva(2484128); return _f(this, data, length, target); }
	inline std::vector<unsigned char,std::allocator<unsigned char> > receivePacket() { typedef std::vector<unsigned char,std::allocator<unsigned char> > (*_fpt)(TCPSocket *pthis); _fpt _f=(_fpt)_drva(2483504); return _f(this); }
};

class RaceTimingServices : public GameObject {
public:
	std::vector<LapDB *,std::allocator<LapDB *> > lapDBs;
	std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > leaderboard;
	Sim * sim;
	std::map<CarAvatar *,Lap,std::less<CarAvatar *>,std::allocator<std::pair<CarAvatar * const,Lap> > > instanceBestTimes;
	std::vector<unsigned int,std::allocator<unsigned int> > bestSplits;
	unsigned int globalBestlap;
	inline RaceTimingServices()  { }
	inline void ctor(Sim * asim) { typedef void (*_fpt)(RaceTimingServices *pthis, Sim *); _fpt _f=(_fpt)_drva(1326176); _f(this, asim); }
	virtual ~RaceTimingServices();
	inline void dtor() { typedef void (*_fpt)(RaceTimingServices *pthis); _fpt _f=(_fpt)_drva(1326912); _f(this); }
	inline void onLapCompleted(OnLapCompletedEvent & event) { typedef void (*_fpt)(RaceTimingServices *pthis, OnLapCompletedEvent &); _fpt _f=(_fpt)_drva(1331968); return _f(this, event); }
	inline void onSectorSplit(OnSectorSplitEvent & event) { typedef void (*_fpt)(RaceTimingServices *pthis, OnSectorSplitEvent &); _fpt _f=(_fpt)_drva(1333568); return _f(this, event); }
	inline Lap getLastLap(CarAvatar * car) { typedef Lap (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1331024); return _f(this, car); }
	inline Lap getBestLap(CarAvatar * car) { typedef Lap (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330128); return _f(this, car); }
	inline int getLapCount(CarAvatar * car) { typedef int (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330560); return _f(this, car); }
	inline unsigned int getSplit(CarAvatar * car, int & sector) { typedef unsigned int (*_fpt)(RaceTimingServices *pthis, CarAvatar *, int &); _fpt _f=(_fpt)_drva(1331728); return _f(this, car, sector); }
	inline int getSplit(CarAvatar * car) { typedef int (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1331792); return _f(this, car); }
	inline void getCurrentLapSplits(CarAvatar * car, std::vector<unsigned int,std::allocator<unsigned int> > & currentSplits) { typedef void (*_fpt)(RaceTimingServices *pthis, CarAvatar *, std::vector<unsigned int,std::allocator<unsigned int> > &); _fpt _f=(_fpt)_drva(1330448); return _f(this, car, currentSplits); }
	inline std::vector<Lap,std::allocator<Lap> > getLaps(CarAvatar * car) { typedef std::vector<Lap,std::allocator<Lap> > (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330976); return _f(this, car); }
	inline unsigned int getLastSplit(CarAvatar * car, int & sector) { typedef unsigned int (*_fpt)(RaceTimingServices *pthis, CarAvatar *, int &); _fpt _f=(_fpt)_drva(1331152); return _f(this, car, sector); }
	inline unsigned int getBestSplit(int & sector, bool & isGlobal, CarAvatar * car) { typedef unsigned int (*_fpt)(RaceTimingServices *pthis, int &, bool &, CarAvatar *); _fpt _f=(_fpt)_drva(1330176); return _f(this, sector, isGlobal, car); }
	inline bool isBestSplit(int & sector, int & t, bool & isGlobal, CarAvatar * car) { typedef bool (*_fpt)(RaceTimingServices *pthis, int &, int &, bool &, CarAvatar *); _fpt _f=(_fpt)_drva(1331856); return _f(this, sector, t, isGlobal, car); }
	inline void resetCurrentLaps() { typedef void (*_fpt)(RaceTimingServices *pthis); _fpt _f=(_fpt)_drva(1334576); return _f(this); }
	inline double getTotalTime(CarAvatar * car) { typedef double (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1331824); return _f(this, car); }
	virtual void shutdown_vf5();
	inline void shutdown_impl() { typedef void (*_fpt)(RaceTimingServices *pthis); _fpt _f=(_fpt)_drva(1335120); return _f(this); }
	inline void shutdown() { return shutdown_vf5(); }
	inline std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > getLeaderboard() { typedef std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > (*_fpt)(RaceTimingServices *pthis); _fpt _f=(_fpt)_drva(1331680); return _f(this); }
	inline int getCarLeaderboardPosition(CarAvatar * car) { typedef int (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330400); return _f(this, car); }
	inline int getInstanceBestTime(CarAvatar * car) { typedef int (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330512); return _f(this, car); }
	inline void setHasCompletedFlag(CarAvatar * car, bool flag) { typedef void (*_fpt)(RaceTimingServices *pthis, CarAvatar *, bool); _fpt _f=(_fpt)_drva(1335088); return _f(this, car, flag); }
	inline bool getHasCompletedFlag(CarAvatar * car) { typedef bool (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330480); return _f(this, car); }
	virtual void update_vf1(float dt);
	inline void update_impl(float dt) { typedef void (*_fpt)(RaceTimingServices *pthis, float); _fpt _f=(_fpt)_drva(1336912); return _f(this, dt); }
	inline void update(float dt) { return update_vf1(dt); }
	inline LapDB * getLapDB(CarAvatar * car) { typedef LapDB * (*_fpt)(RaceTimingServices *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1330624); return _f(this, car); }
	inline void updateLeaderboard() { typedef void (*_fpt)(RaceTimingServices *pthis); _fpt _f=(_fpt)_drva(1336928); return _f(this); }
	inline void onNewSession(OnNewSessionEvent & message) { typedef void (*_fpt)(RaceTimingServices *pthis, OnNewSessionEvent &); _fpt _f=(_fpt)_drva(1333360); return _f(this, message); }
};

class Console : public GameObject, public IVarCallback, public IKeyEventListener {
public:
	Sim * sim;
	std::vector<SVar *,std::allocator<SVar *> > vars;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > lines;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > history;
	unsigned int historyPos;
	Font * font;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > commandLine;
	std::vector<ConsoleCommand *,std::allocator<ConsoleCommand *> > commands;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > accumulator;
	bool disableAutoShow;
	inline Console()  { }
	inline void ctor(Sim * isim) { typedef void (*_fpt)(Console *pthis, Sim *); _fpt _f=(_fpt)_drva(1607872); _f(this, isim); }
	virtual ~Console();
	inline void dtor() { typedef void (*_fpt)(Console *pthis); _fpt _f=(_fpt)_drva(1608752); _f(this); }
	virtual void renderHUD_vf3(float dt);
	inline void renderHUD_impl(float dt) { typedef void (*_fpt)(Console *pthis, float); _fpt _f=(_fpt)_drva(1623376); return _f(this, dt); }
	inline void renderHUD(float dt) { return renderHUD_vf3(dt); }
	virtual void onSetVar_vf0(SVar * var, float value);
	inline void onSetVar_impl(SVar * var, float value) { typedef void (*_fpt)(Console *pthis, SVar *, float); _fpt _f=(_fpt)_drva(1619680); return _f(this, var, value); }
	inline void onSetVar(SVar * var, float value) { return onSetVar_vf0(var, value); }
	inline void addVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, float * var, IVarCallback * callback, bool readOnly, float multiplier) { typedef void (*_fpt)(Console *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, float *, IVarCallback *, bool, float); _fpt _f=(_fpt)_drva(1615552); return _f(this, name, var, callback, readOnly, multiplier); }
	inline void addVarLambda(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, std::function<void __cdecl(SVar *,float)> * lambda, bool readyonly, float multiplier) { typedef void (*_fpt)(Console *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, std::function<void __cdecl(SVar *,float)> *, bool, float); _fpt _f=(_fpt)_drva(1615920); return _f(this, name, lambda, readyonly, multiplier); }
	inline void show(bool mode) { typedef void (*_fpt)(Console *pthis, bool); _fpt _f=(_fpt)_drva(1624048); return _f(this, mode); }
	inline bool isVisible() { typedef bool (*_fpt)(Console *pthis); _fpt _f=(_fpt)_drva(1680480); return _f(this); }
	inline void addCommand(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, std::function<bool __cdecl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >)> * exeFun, std::function<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > __cdecl(void)> * helpFun) { typedef void (*_fpt)(Console *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::function<bool __cdecl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >)> *, std::function<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > __cdecl(void)> *); _fpt _f=(_fpt)_drva(1615152); return _f(this, name, exeFun, helpFun); }
	virtual void onKeyChar_vf1(unsigned int key);
	inline void onKeyChar_impl(unsigned int key) { typedef void (*_fpt)(Console *pthis, unsigned int); _fpt _f=(_fpt)_drva(1619072); return _f(this, key); }
	inline void onKeyChar(unsigned int key) { return onKeyChar_vf1(key); }
	virtual void onKeyDown_vf0(OnKeyEvent & message);
	inline void onKeyDown_impl(OnKeyEvent & message) { typedef void (*_fpt)(Console *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(1619488); return _f(this, message); }
	inline void onKeyDown(OnKeyEvent & message) { return onKeyDown_vf0(message); }
	inline Console & operator<<(float & v) { typedef Console & (*_fpt)(Console *pthis, float &); _fpt _f=(_fpt)_drva(1609952); return _f(this, v); }
	inline Console & operator<<(int & v) { typedef Console & (*_fpt)(Console *pthis, int &); _fpt _f=(_fpt)_drva(1609536); return _f(this, v); }
	inline Console & operator<<(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * s) { typedef Console & (*_fpt)(Console *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(1610928); return _f(this, s); }
	inline Console & operator<<(std::basic_string<char,std::char_traits<char>,std::allocator<char> > * s) { typedef Console & (*_fpt)(Console *pthis, std::basic_string<char,std::char_traits<char>,std::allocator<char> > *); _fpt _f=(_fpt)_drva(1610368); return _f(this, s); }
	inline void parse(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * c) { typedef void (*_fpt)(Console *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(1621840); return _f(this, c); }
	inline void initCommands() { typedef void (*_fpt)(Console *pthis); _fpt _f=(_fpt)_drva(1616512); return _f(this); }
};

struct CameraCarDefinition {
public:
	mat44f matrix;
	float fov;
	float exposure;
	bool externalSound;
	inline CameraCarDefinition()  { }
	inline void ctor() { typedef void (*_fpt)(CameraCarDefinition *pthis); _fpt _f=(_fpt)_drva(839936); _f(this); }
};

class INIReaderDocuments : public INIReader {
public:
	inline INIReaderDocuments()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iniName, bool createFile) { typedef void (*_fpt)(INIReaderDocuments *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, bool); _fpt _f=(_fpt)_drva(2327376); _f(this, iniName, createFile); }
	virtual ~INIReaderDocuments();
	inline void dtor() { typedef void (*_fpt)(INIReaderDocuments *pthis); _fpt _f=(_fpt)_drva(2329024); _f(this); }
};

struct SlipStream {
public:
	Triangle triangle;
	float speedFactorMult;
	float effectGainMult;
	vec3f dir;
	PhysicsEngine * physicsEngine;
	float length;
	float speedFactor;
	inline SlipStream()  { }
	inline void dtor() { typedef void (*_fpt)(SlipStream *pthis); _fpt _f=(_fpt)_drva(2796272); _f(this); }
	inline void init(PhysicsEngine * pe) { typedef void (*_fpt)(SlipStream *pthis, PhysicsEngine *); _fpt _f=(_fpt)_drva(2796992); return _f(this, pe); }
	inline float getSlipEffect(vec3f & p) { typedef float (*_fpt)(SlipStream *pthis, vec3f &); _fpt _f=(_fpt)_drva(2796640); return _f(this, p); }
	inline void setPosition(vec3f & pos, vec3f & vel) { typedef void (*_fpt)(SlipStream *pthis, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(2797184); return _f(this, pos, vel); }
};

class IRigidBody {
public:
	inline IRigidBody()  { }
	virtual void setMassExplicitInertia_vf0(float  _arg0, float  _arg1, float  _arg2, float  _arg3) = 0;
	inline void setMassExplicitInertia(float  _arg0, float  _arg1, float  _arg2, float  _arg3) { return setMassExplicitInertia_vf0( _arg0,  _arg1,  _arg2,  _arg3); }
	virtual bool isEnabled_vf1() = 0;
	inline bool isEnabled() { return isEnabled_vf1(); }
	virtual void setEnabled_vf2(bool  _arg0) = 0;
	inline void setEnabled(bool  _arg0) { return setEnabled_vf2( _arg0); }
	virtual void setAutoDisable_vf3(bool  _arg0) = 0;
	inline void setAutoDisable(bool  _arg0) { return setAutoDisable_vf3( _arg0); }
	virtual void removeCollisionObjects_vf4() = 0;
	inline void removeCollisionObjects() { return removeCollisionObjects_vf4(); }
	virtual float getMass_vf5() = 0;
	inline float getMass() { return getMass_vf5(); }
	virtual void release_vf6() = 0;
	inline void release() { return release_vf6(); }
	virtual void setMassBox_vf7(float  _arg0, float  _arg1, float  _arg2, float  _arg3) = 0;
	inline void setMassBox(float  _arg0, float  _arg1, float  _arg2, float  _arg3) { return setMassBox_vf7( _arg0,  _arg1,  _arg2,  _arg3); }
	virtual vec3f getLocalInertia_vf8() = 0;
	inline vec3f getLocalInertia() { return getLocalInertia_vf8(); }
	virtual mat44f getWorldMatrix_vf9(float  _arg0) = 0;
	inline mat44f getWorldMatrix(float  _arg0) { return getWorldMatrix_vf9( _arg0); }
	virtual vec3f localToWorld_vf10(vec3f &  _arg0) = 0;
	inline vec3f localToWorld(vec3f &  _arg0) { return localToWorld_vf10( _arg0); }
	virtual vec3f worldToLocal_vf11(vec3f &  _arg0) = 0;
	inline vec3f worldToLocal(vec3f &  _arg0) { return worldToLocal_vf11( _arg0); }
	virtual vec3f localToWorldNormal_vf12(vec3f &  _arg0) = 0;
	inline vec3f localToWorldNormal(vec3f &  _arg0) { return localToWorldNormal_vf12( _arg0); }
	virtual vec3f worldToLocalNormal_vf13(vec3f &  _arg0) = 0;
	inline vec3f worldToLocalNormal(vec3f &  _arg0) { return worldToLocalNormal_vf13( _arg0); }
	virtual void stop_vf14(float  _arg0) = 0;
	inline void stop(float  _arg0) { return stop_vf14( _arg0); }
	virtual vec3f getVelocity_vf15() = 0;
	inline vec3f getVelocity() { return getVelocity_vf15(); }
	virtual void setVelocity_vf16(vec3f &  _arg0) = 0;
	inline void setVelocity(vec3f &  _arg0) { return setVelocity_vf16( _arg0); }
	virtual void setAngularVelocity_vf17(vec3f &  _arg0) = 0;
	inline void setAngularVelocity(vec3f &  _arg0) { return setAngularVelocity_vf17( _arg0); }
	virtual void setPosition_vf18(vec3f &  _arg0) = 0;
	inline void setPosition(vec3f &  _arg0) { return setPosition_vf18( _arg0); }
	virtual void setRotation_vf19(mat44f &  _arg0) = 0;
	inline void setRotation(mat44f &  _arg0) { return setRotation_vf19( _arg0); }
	virtual vec3f getPosition_vf20(float  _arg0) = 0;
	inline vec3f getPosition(float  _arg0) { return getPosition_vf20( _arg0); }
	virtual vec3f getAngularVelocity_vf21() = 0;
	inline vec3f getAngularVelocity() { return getAngularVelocity_vf21(); }
	virtual vec3f getLocalAngularVelocity_vf22() = 0;
	inline vec3f getLocalAngularVelocity() { return getLocalAngularVelocity_vf22(); }
	virtual vec3f getLocalVelocity_vf23() = 0;
	inline vec3f getLocalVelocity() { return getLocalVelocity_vf23(); }
	virtual void setBoxColliderMask_vf24(unsigned __int64  _arg0, unsigned long  _arg1) = 0;
	inline void setBoxColliderMask(unsigned __int64  _arg0, unsigned long  _arg1) { return setBoxColliderMask_vf24( _arg0,  _arg1); }
	virtual unsigned __int64 addBoxCollider_vf25(vec3f &  _arg0, vec3f &  _arg1, unsigned int  _arg2, unsigned long  _arg3, unsigned int  _arg4) = 0;
	inline unsigned __int64 addBoxCollider(vec3f &  _arg0, vec3f &  _arg1, unsigned int  _arg2, unsigned long  _arg3, unsigned int  _arg4) { return addBoxCollider_vf25( _arg0,  _arg1,  _arg2,  _arg3,  _arg4); }
	virtual void addSphereCollider_vf26(vec3f &  _arg0, float  _arg1, unsigned int  _arg2, ISphereCollisionCallback *  _arg3) = 0;
	inline void addSphereCollider(vec3f &  _arg0, float  _arg1, unsigned int  _arg2, ISphereCollisionCallback *  _arg3) { return addSphereCollider_vf26( _arg0,  _arg1,  _arg2,  _arg3); }
	virtual void addLocalForce_vf27(vec3f &  _arg0) = 0;
	inline void addLocalForce(vec3f &  _arg0) { return addLocalForce_vf27( _arg0); }
	virtual void addLocalTorque_vf28(vec3f &  _arg0) = 0;
	inline void addLocalTorque(vec3f &  _arg0) { return addLocalTorque_vf28( _arg0); }
	virtual void addLocalForceAtPos_vf29(vec3f &  _arg0, vec3f &  _arg1) = 0;
	inline void addLocalForceAtPos(vec3f &  _arg0, vec3f &  _arg1) { return addLocalForceAtPos_vf29( _arg0,  _arg1); }
	virtual void addLocalForceAtLocalPos_vf30(vec3f &  _arg0, vec3f &  _arg1) = 0;
	inline void addLocalForceAtLocalPos(vec3f &  _arg0, vec3f &  _arg1) { return addLocalForceAtLocalPos_vf30( _arg0,  _arg1); }
	virtual void addForceAtLocalPos_vf31(vec3f &  _arg0, vec3f &  _arg1) = 0;
	inline void addForceAtLocalPos(vec3f &  _arg0, vec3f &  _arg1) { return addForceAtLocalPos_vf31( _arg0,  _arg1); }
	virtual vec3f getLocalPointVelocity_vf32(vec3f &  _arg0) = 0;
	inline vec3f getLocalPointVelocity(vec3f &  _arg0) { return getLocalPointVelocity_vf32( _arg0); }
	virtual vec3f getPointVelocity_vf33(vec3f &  _arg0) = 0;
	inline vec3f getPointVelocity(vec3f &  _arg0) { return getPointVelocity_vf33( _arg0); }
	virtual void addForceAtPos_vf34(vec3f &  _arg0, vec3f &  _arg1) = 0;
	inline void addForceAtPos(vec3f &  _arg0, vec3f &  _arg1) { return addForceAtPos_vf34( _arg0,  _arg1); }
	virtual void addTorque_vf35(vec3f &  _arg0) = 0;
	inline void addTorque(vec3f &  _arg0) { return addTorque_vf35( _arg0); }
	virtual void addMeshCollider_vf36(float *  _arg0, unsigned int  _arg1, unsigned short *  _arg2, unsigned int  _arg3, mat44f  _arg4, unsigned long  _arg5, unsigned long  _arg6, unsigned int  _arg7) = 0;
	inline void addMeshCollider(float *  _arg0, unsigned int  _arg1, unsigned short *  _arg2, unsigned int  _arg3, mat44f  _arg4, unsigned long  _arg5, unsigned long  _arg6, unsigned int  _arg7) { return addMeshCollider_vf36( _arg0,  _arg1,  _arg2,  _arg3,  _arg4,  _arg5,  _arg6,  _arg7); }
	virtual void setMeshCollideCategory_vf37(unsigned int  _arg0, unsigned long  _arg1) = 0;
	inline void setMeshCollideCategory(unsigned int  _arg0, unsigned long  _arg1) { return setMeshCollideCategory_vf37( _arg0,  _arg1); }
	virtual void setMeshCollideMask_vf38(unsigned int  _arg0, unsigned long  _arg1) = 0;
	inline void setMeshCollideMask(unsigned int  _arg0, unsigned long  _arg1) { return setMeshCollideMask_vf38( _arg0,  _arg1); }
	virtual unsigned long getMeshCollideCategory_vf39(unsigned int  _arg0) = 0;
	inline unsigned long getMeshCollideCategory(unsigned int  _arg0) { return getMeshCollideCategory_vf39( _arg0); }
	virtual unsigned long getMeshCollideMask_vf40(unsigned int  _arg0) = 0;
	inline unsigned long getMeshCollideMask(unsigned int  _arg0) { return getMeshCollideMask_vf40( _arg0); }
	virtual ~IRigidBody();
};

class IMeshRenderFilter {
public:
	RenderPassID passID;
	int maxLayer;
	inline IMeshRenderFilter()  { }
	virtual ~IMeshRenderFilter();
	virtual bool isVisible_vf1(Renderable *  _arg0, mat44f &  _arg1) = 0;
	inline bool isVisible(Renderable *  _arg0, mat44f &  _arg1) { return isVisible_vf1( _arg0,  _arg1); }
};

struct CarPhysicsInfo {
public:
	float steerLock;
	int maxGear;
	float caster;
	float tyreRadius[0x4];
	float tyreWidth[0x4];
	float totalMass;
	float bodyMass;
	float unsprungWeights[0x4];
	std::vector<WingData,std::allocator<WingData> > wingData;
	float maxTorqueNM;
	float maxPowerW;
	float bumpStopsUp[0x4];
	float bumpStopsDn[0x4];
	double maxFuel;
	vec3f ridePickupPoint[0x2];
	vec3f susBasePos[0x4];
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > tyreCompounds;
	float minHeightM;
	float wheelAngularInertia[0x4];
	KPI kpi;
	float maxTurboBoost;
	int turboCount;
	float kersMaxJ;
	float ersMaxJ;
	float powerClassIndex;
	bool hasKERS;
	bool hasERS;
	bool hasCockpitERSRecovery;
	bool hasCockpitERSDeliveryProfile;
	bool hasCockpitEngineBrake;
	bool hasCockpitERSMguHMode;
	bool hasDRS;
	int engineBrakeSettingsCount;
	int ersPowerControllerCount;
	int engineDamageRPM;
	bool hasAdjustableTurbo;
	int maxRpm;
	inline CarPhysicsInfo()  { }
	inline void ctor() { typedef void (*_fpt)(CarPhysicsInfo *pthis); _fpt _f=(_fpt)_drva(844768); _f(this); }
	inline void dtor() { typedef void (*_fpt)(CarPhysicsInfo *pthis); _fpt _f=(_fpt)_drva(847488); _f(this); }
	inline CarPhysicsInfo & operator=(CarPhysicsInfo & __that) { typedef CarPhysicsInfo & (*_fpt)(CarPhysicsInfo *pthis, CarPhysicsInfo &); _fpt _f=(_fpt)_drva(1596080); return _f(this, __that); }
};

class DynamicController {
public:
	Car * car;
	std::vector<DynamicControllerStage,std::allocator<DynamicControllerStage> > stages;
	bool ready;
	inline DynamicController()  { }
	inline void ctor() { typedef void (*_fpt)(DynamicController *pthis); _fpt _f=(_fpt)_drva(2819936); _f(this); }
	inline void ctor(Car * car, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef void (*_fpt)(DynamicController *pthis, Car *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2814768); _f(this, car, filename); }
	inline void dtor() { typedef void (*_fpt)(DynamicController *pthis); _fpt _f=(_fpt)_drva(2820032); _f(this); }
	inline float eval() { typedef float (*_fpt)(DynamicController *pthis); _fpt _f=(_fpt)_drva(2821120); return _f(this); }
	inline float getInput(DynamicControllerInput input) { typedef float (*_fpt)(DynamicController *pthis, DynamicControllerInput); _fpt _f=(_fpt)_drva(2821488); return _f(this, input); }
	inline bool isReady() { typedef bool (*_fpt)(DynamicController *pthis); _fpt _f=(_fpt)_drva(2864688); return _f(this); }
	inline DynamicController & operator=(DynamicController & __that) { typedef DynamicController & (*_fpt)(DynamicController *pthis, DynamicController &); _fpt _f=(_fpt)_drva(2550784); return _f(this, __that); }
};

struct NetCarStateProviderDef {
public:
	ACClient * client;
	unsigned char sessionID;
	unsigned char guid;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > driverName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > team;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > nationCode;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > model;
	IRayTrackCollisionProvider * rayCastProvider;
	PhysicsAvatar * physicsAvatar;
	mat44f pitPosition;
	inline NetCarStateProviderDef()  { }
	inline void dtor() { typedef void (*_fpt)(NetCarStateProviderDef *pthis); _fpt _f=(_fpt)_drva(1660448); _f(this); }
};

struct Autoclutch {
public:
	float rpmMin;
	float rpmMax;
	float clutchSpeed;
	bool useAutoOnStart;
	bool useAutoOnChange;
	bool isForced;
	Car * car;
	ClutchSequence clutchSequence;
	Curve upshiftProfile;
	Curve downshiftProfile;
	float clutchValueSignal;
	inline Autoclutch()  { }
	inline void dtor() { typedef void (*_fpt)(Autoclutch *pthis); _fpt _f=(_fpt)_drva(2851776); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(Autoclutch *pthis, Car *); _fpt _f=(_fpt)_drva(2852816); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(Autoclutch *pthis, float); _fpt _f=(_fpt)_drva(2856336); return _f(this, dt); }
	inline void onGearRequest(OnGearRequestEvent & ev) { typedef void (*_fpt)(Autoclutch *pthis, OnGearRequestEvent &); _fpt _f=(_fpt)_drva(2855760); return _f(this, ev); }
	inline void setDownshiftProfile(Curve & dp) { typedef void (*_fpt)(Autoclutch *pthis, Curve &); _fpt _f=(_fpt)_drva(2750528); return _f(this, dp); }
	inline float getDownshiftSequenceDuration() { typedef float (*_fpt)(Autoclutch *pthis); _fpt _f=(_fpt)_drva(2741152); return _f(this); }
	inline void stepSequence(float dt) { typedef void (*_fpt)(Autoclutch *pthis, float); _fpt _f=(_fpt)_drva(2856864); return _f(this, dt); }
	inline void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * carModel) { typedef void (*_fpt)(Autoclutch *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2853008); return _f(this, carModel); }
	inline void ctor() { typedef void (*_fpt)(Autoclutch *pthis); _fpt _f=(_fpt)_drva(2538848); _f(this); }
};

struct TyreCompoundDef {
public:
	unsigned int index;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > shortName;
	TyreModelData modelData;
	TyreData data;
	BrushSlipProvider slipProvider;
	float pressureStatic;
	TyrePatchData thermalPatchData;
	Curve thermalPerformanceCurve;
	inline TyreCompoundDef()  { }
	inline void ctor(TyreCompoundDef & __that) { typedef void (*_fpt)(TyreCompoundDef *pthis, TyreCompoundDef &); _fpt _f=(_fpt)_drva(2608864); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(TyreCompoundDef *pthis); _fpt _f=(_fpt)_drva(2609248); _f(this); }
	inline void dtor() { typedef void (*_fpt)(TyreCompoundDef *pthis); _fpt _f=(_fpt)_drva(2549984); _f(this); }
};

struct RenderState {
public:
	void * textures[0x20];
	CullMode cullMode;
	BlendMode blendMode;
	DepthMode depthState;
	Material * material;
	mat44f projectionMatrix;
	mat44f viewMatrix;
	mat44f worldMatrix;
	Shader * shader;
	void * currentRenderTarget;
	void * currentDepth;
	bool overrideNoMS;
	inline RenderState()  { }
	inline void ctor() { typedef void (*_fpt)(RenderState *pthis); _fpt _f=(_fpt)_drva(2105216); _f(this); }
};

class Spline {
public:
	std::vector<SplinePoint,std::allocator<SplinePoint> > points;
	float m_length;
	bool m_closed;
	inline Spline()  { }
	inline void ctor() { typedef void (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2019712); _f(this); }
	virtual ~Spline();
	inline void dtor() { typedef void (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2019760); _f(this); }
	inline vec3f pointAt(unsigned int index) { typedef vec3f (*_fpt)(Spline *pthis, unsigned int); _fpt _f=(_fpt)_drva(2025504); return _f(this, index); }
	inline void setPointAt(unsigned int index, vec3f point) { typedef void (*_fpt)(Spline *pthis, unsigned int, vec3f); _fpt _f=(_fpt)_drva(2025904); return _f(this, index, point); }
	inline SplinePoint splinePointAt(unsigned int index) { typedef SplinePoint (*_fpt)(Spline *pthis, unsigned int); _fpt _f=(_fpt)_drva(2025984); return _f(this, index); }
	inline void addPoint(vec3f & p, int tag) { typedef void (*_fpt)(Spline *pthis, vec3f &, int); _fpt _f=(_fpt)_drva(2020960); return _f(this, p, tag); }
	inline void addSplinePoint(SplinePoint & p) { typedef void (*_fpt)(Spline *pthis, SplinePoint &); _fpt _f=(_fpt)_drva(2021152); return _f(this, p); }
	virtual void clear_vf1();
	inline void clear_impl() { typedef void (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2021616); return _f(this); }
	inline void clear() { return clear_vf1(); }
	inline float length() { typedef float (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2024352); return _f(this); }
	inline unsigned int pointsCount() { typedef unsigned int (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2025600); return _f(this); }
	inline unsigned int closestPointIndex(vec3f & point, float * distance) { typedef unsigned int (*_fpt)(Spline *pthis, vec3f &, float *); _fpt _f=(_fpt)_drva(2021632); return _f(this, point, distance); }
	inline unsigned int closestPointIndexWithBounds(vec3f & point, std::vector<SplineIndexBound,std::allocator<SplineIndexBound> > & bounds, float * distance) { typedef unsigned int (*_fpt)(Spline *pthis, vec3f &, std::vector<SplineIndexBound,std::allocator<SplineIndexBound> > &, float *); _fpt _f=(_fpt)_drva(2021904); return _f(this, point, bounds, distance); }
	inline std::vector<unsigned int,std::allocator<unsigned int> > closestPointIndicesFlat(vec3f & point, unsigned int numberOfNeigbours, float * distance) { typedef std::vector<unsigned int,std::allocator<unsigned int> > (*_fpt)(Spline *pthis, vec3f &, unsigned int, float *); _fpt _f=(_fpt)_drva(2022144); return _f(this, point, numberOfNeigbours, distance); }
	inline vec3f getSplineDirection(vec3f & point) { typedef vec3f (*_fpt)(Spline *pthis, vec3f &); _fpt _f=(_fpt)_drva(2023888); return _f(this, point); }
	inline unsigned int wrapIndex(int index) { typedef unsigned int (*_fpt)(Spline *pthis, int); _fpt _f=(_fpt)_drva(2026192); return _f(this, index); }
	inline void loadFromCSV(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename) { typedef void (*_fpt)(Spline *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2024384); return _f(this, filename); }
	inline vec3f getPointSmooth(float v) { typedef vec3f (*_fpt)(Spline *pthis, float); _fpt _f=(_fpt)_drva(2023504); return _f(this, v); }
	inline void applyMatrix(mat44f & m) { typedef void (*_fpt)(Spline *pthis, mat44f &); _fpt _f=(_fpt)_drva(2021168); return _f(this, m); }
	inline int tagAt(unsigned int index) { typedef int (*_fpt)(Spline *pthis, unsigned int); _fpt _f=(_fpt)_drva(2026128); return _f(this, index); }
	inline bool isClosed() { typedef bool (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2024208); return _f(this); }
	inline void reverse() { typedef void (*_fpt)(Spline *pthis); _fpt _f=(_fpt)_drva(2025840); return _f(this); }
	inline float boundInsideSpline(float value) { typedef float (*_fpt)(Spline *pthis, float); _fpt _f=(_fpt)_drva(2021472); return _f(this, value); }
};

struct DynamicWingController {
public:
	float outputAngle;
	float upLimit;
	float downLimit;
	DynamicWingController_eCombinatorMode combinatorMode;
	DynamicWingController_eInputVar inputVar;
	Curve lut;
	float filter;
	CarPhysicsState * state;
	Car * car;
	inline DynamicWingController()  { }
	inline void ctor(DynamicWingController & __that) { typedef void (*_fpt)(DynamicWingController *pthis, DynamicWingController &); _fpt _f=(_fpt)_drva(2839024); _f(this, __that); }
	inline void ctor(CarPhysicsState * state, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & carUnixName, INIReader & ini, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section) { typedef void (*_fpt)(DynamicWingController *pthis, CarPhysicsState *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, INIReader &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2793072); _f(this, state, carUnixName, ini, section); }
	inline void ctor(Car * car, INIReader & ini, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section) { typedef void (*_fpt)(DynamicWingController *pthis, Car *, INIReader &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2792944); _f(this, car, ini, section); }
	inline void dtor() { typedef void (*_fpt)(DynamicWingController *pthis); _fpt _f=(_fpt)_drva(1147056); _f(this); }
	inline void step() { typedef void (*_fpt)(DynamicWingController *pthis); _fpt _f=(_fpt)_drva(2796144); return _f(this); }
	inline float getInput() { typedef float (*_fpt)(DynamicWingController *pthis); _fpt _f=(_fpt)_drva(2793200); return _f(this); }
	inline void initCommon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & carUnixName, INIReader & ini, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section) { typedef void (*_fpt)(DynamicWingController *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, INIReader &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2793680); return _f(this, carUnixName, ini, section); }
};

class UDPPacket {
public:
	IPAddress targetIP;
	unsigned char * data;
	unsigned int currentDataPos;
	unsigned int size;
	inline UDPPacket()  { }
	inline void ctor(UDPPacket & r) { typedef void (*_fpt)(UDPPacket *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(2479904); _f(this, r); }
	inline void ctor(UDPMessage & msg) { typedef void (*_fpt)(UDPPacket *pthis, UDPMessage &); _fpt _f=(_fpt)_drva(2479744); _f(this, msg); }
	inline void ctor(std::vector<unsigned char,std::allocator<unsigned char> > & idata) { typedef void (*_fpt)(UDPPacket *pthis, std::vector<unsigned char,std::allocator<unsigned char> > &); _fpt _f=(_fpt)_drva(2480016); _f(this, idata); }
	inline void ctor() { typedef void (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480128); _f(this); }
	inline void dtor() { typedef void (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480192); _f(this); }
	inline UDPPacket & operator=(UDPPacket & r) { typedef UDPPacket & (*_fpt)(UDPPacket *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(2480240); return _f(this, r); }
	inline void writeString(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & st) { typedef void (*_fpt)(UDPPacket *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2481520); return _f(this, st); }
	inline void writeString(std::basic_string<char,std::char_traits<char>,std::allocator<char> > & st) { typedef void (*_fpt)(UDPPacket *pthis, std::basic_string<char,std::char_traits<char>,std::allocator<char> > &); _fpt _f=(_fpt)_drva(2481312); return _f(this, st); }
	inline void writeStringANSI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & st) { typedef void (*_fpt)(UDPPacket *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2481744); return _f(this, st); }
	inline std::basic_string<char,std::char_traits<char>,std::allocator<char> > readString() { typedef std::basic_string<char,std::char_traits<char>,std::allocator<char> > (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480592); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > readStringW() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480784); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > readBigStringW() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480368); return _f(this); }
	inline unsigned int getSize() { typedef unsigned int (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480320); return _f(this); }
	inline void send(TCPSocket & sok) { typedef void (*_fpt)(UDPPacket *pthis, TCPSocket &); _fpt _f=(_fpt)_drva(2481008); return _f(this, sok); }
	inline void send(UDPSocket & sok) { typedef void (*_fpt)(UDPPacket *pthis, UDPSocket &); _fpt _f=(_fpt)_drva(2481264); return _f(this, sok); }
	inline bool isEOF() { typedef bool (*_fpt)(UDPPacket *pthis); _fpt _f=(_fpt)_drva(2480336); return _f(this); }
};

class MaterialVar {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	float fValue;
	vec2f fValue2;
	vec3f fValue3;
	vec4f fValue4;
	mat44f mValue;
	ShaderVariable * var;
	inline MaterialVar()  { }
	inline void ctor(ShaderVariable * ivar) { typedef void (*_fpt)(MaterialVar *pthis, ShaderVariable *); _fpt _f=(_fpt)_drva(2137184); _f(this, ivar); }
	inline void copyValues(MaterialVar * mv) { typedef void (*_fpt)(MaterialVar *pthis, MaterialVar *); _fpt _f=(_fpt)_drva(2140240); return _f(this, mv); }
	inline void setFloat(float v) { typedef void (*_fpt)(MaterialVar *pthis, float); _fpt _f=(_fpt)_drva(383184); return _f(this, v); }
	inline void setFloat3(vec3f & v) { typedef void (*_fpt)(MaterialVar *pthis, vec3f &); _fpt _f=(_fpt)_drva(918464); return _f(this, v); }
	inline vec3f getFloat3() { typedef vec3f (*_fpt)(MaterialVar *pthis); _fpt _f=(_fpt)_drva(909296); return _f(this); }
	inline void set() { typedef void (*_fpt)(MaterialVar *pthis); _fpt _f=(_fpt)_drva(383104); return _f(this); }
};

struct CarPhysicsState {
public:
	unsigned char physicsGUID;
	mat44f worldMatrix;
	mat44f suspensionMatrix[0x4];
	mat44f tyreMatrix[0x4];
	float engineRPM;
	bool isEngineLimiterOn;
	float wheelAngularSpeed[0x4];
	float steer;
	float gas;
	float brake;
	float clutch;
	int gear;
	Speed speed;
	vec3f velocity;
	vec3f localVelocity;
	vec3f localAngularVelocity;
	vec3f angularVelocity;
	float slipAngle[0x4];
	float slipRatio[0x4];
	float tyreSlip[0x4];
	float ndSlip[0x4];
	float load[0x4];
	float Dy[0x4];
	float Mz[0x4];
	float tyreDirtyLevel[0x4];
	SurfaceDef tyreSurfaceDef[0x4];
	float cgHeight;
	vec3f accG;
	unsigned int lapTime;
	unsigned int lastLap;
	unsigned int bestLap;
	unsigned int lapCount;
	float lastFF_Pure;
	float lastFF_Final;
	SCarStateAero aero;
	vec3f tyreContactPoint[0x4];
	vec3f tyreContactNormal[0x4];
	float camberRAD[0x4];
	float tyreRadius[0x4];
	float tyreLoadedRadius[0x4];
	float suspensionTravel[0x4];
	float normalizedSplinePosition;
	float driftPoints;
	float instantDrift;
	bool isDriftValid;
	int driftComboCounter;
	bool driftBonusOn;
	float drivetrainSpeed;
	float turboBoost;
	float performanceMeter;
	float performanceMeterSpeedDiffMS;
	bool isGearGrinding;
	float bodyWorkVolume;
	float tyreVirtualKM[0x4];
	float damageZoneLevel[0x5];
	int limiterRPM;
	plane4f groundPlane;
	double timeStamp;
	float airDensity;
	float fuel;
	float fuelLaps;
	float rideHeight[0x2];
	bool isRetired;
	float engineLifeLeft;
	float turboBov;
	float turboBoostLevel;
	float tyreGrain[0x4];
	float tyreBlister[0x4];
	DriverActionsState actionsState;
	CarSetupState setupState;
	float tyreInflation[0x4];
	float kersCharge;
	float kersInput;
	float gearRpmWindow;
	float susDamage[0x4];
	float tyreFlatSpot[0x4];
	float water;
	TyreThermalState tyreThermalStates[0x4];
	float discTemps[0x4];
	float wear[0x4];
	float wearMult[0x4];
	float lockControlsTime;
	float kersCurrentKJ;
	bool kersIsCharging;
	unsigned int statusBytes;
	unsigned char p2pStatus;
	unsigned char p2pActivations;
	float antiSquat;
	float caster[0x2];
	inline CarPhysicsState()  { }
	inline void ctor(CarPhysicsState & __that) { typedef void (*_fpt)(CarPhysicsState *pthis, CarPhysicsState &); _fpt _f=(_fpt)_drva(1179376); _f(this, __that); }
	inline void ctor() { typedef void (*_fpt)(CarPhysicsState *pthis); _fpt _f=(_fpt)_drva(781920); _f(this); }
	inline void dtor() { typedef void (*_fpt)(CarPhysicsState *pthis); _fpt _f=(_fpt)_drva(783504); _f(this); }
};

class IPhysicsCore {
public:
	inline IPhysicsCore()  { }
	virtual void resetCollisions_vf0() = 0;
	inline void resetCollisions() { return resetCollisions_vf0(); }
	virtual void initMultithreading_vf1() = 0;
	inline void initMultithreading() { return initMultithreading_vf1(); }
	virtual void step_vf2(float  _arg0) = 0;
	inline void step(float  _arg0) { return step_vf2( _arg0); }
	virtual IRigidBody * createRigidBody_vf3() = 0;
	inline IRigidBody * createRigidBody() { return createRigidBody_vf3(); }
	virtual void release_vf4() = 0;
	inline void release() { return release_vf4(); }
	virtual IJoint * createDistanceJoint_vf5(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2, vec3f &  _arg3) = 0;
	inline IJoint * createDistanceJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2, vec3f &  _arg3) { return createDistanceJoint_vf5( _arg0,  _arg1,  _arg2,  _arg3); }
	virtual IJoint * createBumpJoint_vf6(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2, float  _arg3, float  _arg4) = 0;
	inline IJoint * createBumpJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2, float  _arg3, float  _arg4) { return createBumpJoint_vf6( _arg0,  _arg1,  _arg2,  _arg3,  _arg4); }
	virtual ICollisionObject * createCollisionMesh_vf7(float *  _arg0, unsigned int  _arg1, unsigned short *  _arg2, int  _arg3, mat44f &  _arg4, IRigidBody *  _arg5, unsigned long  _arg6, unsigned long  _arg7, unsigned int  _arg8) = 0;
	inline ICollisionObject * createCollisionMesh(float *  _arg0, unsigned int  _arg1, unsigned short *  _arg2, int  _arg3, mat44f &  _arg4, IRigidBody *  _arg5, unsigned long  _arg6, unsigned long  _arg7, unsigned int  _arg8) { return createCollisionMesh_vf7( _arg0,  _arg1,  _arg2,  _arg3,  _arg4,  _arg5,  _arg6,  _arg7,  _arg8); }
	virtual RayCastHit rayCast_vf8(vec3f &  _arg0, vec3f &  _arg1, float  _arg2) = 0;
	inline RayCastHit rayCast(vec3f &  _arg0, vec3f &  _arg1, float  _arg2) { return rayCast_vf8( _arg0,  _arg1,  _arg2); }
	virtual void setCollisionCallback_vf9(ICollisionCallback *  _arg0) = 0;
	inline void setCollisionCallback(ICollisionCallback *  _arg0) { return setCollisionCallback_vf9( _arg0); }
	virtual void reseatDistanceJointLocal_vf10(IJoint *  _arg0, vec3f &  _arg1, vec3f &  _arg2) = 0;
	inline void reseatDistanceJointLocal(IJoint *  _arg0, vec3f &  _arg1, vec3f &  _arg2) { return reseatDistanceJointLocal_vf10( _arg0,  _arg1,  _arg2); }
	virtual void reseatDistanceJointLength_vf11(IJoint *  _arg0, float  _arg1) = 0;
	inline void reseatDistanceJointLength(IJoint *  _arg0, float  _arg1) { return reseatDistanceJointLength_vf11( _arg0,  _arg1); }
	virtual void setRigidBodyIterations_vf12(int  _arg0) = 0;
	inline void setRigidBodyIterations(int  _arg0) { return setRigidBodyIterations_vf12( _arg0); }
	virtual IJoint * createSliderJoint_vf13(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2) = 0;
	inline IJoint * createSliderJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2) { return createSliderJoint_vf13( _arg0,  _arg1,  _arg2); }
	virtual void setSliderAxis_vf14(IJoint *  _arg0, vec3f &  _arg1, vec3f &  _arg2) = 0;
	inline void setSliderAxis(IJoint *  _arg0, vec3f &  _arg1, vec3f &  _arg2) { return setSliderAxis_vf14( _arg0,  _arg1,  _arg2); }
	virtual IJoint * createBallJoint_vf15(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2) = 0;
	inline IJoint * createBallJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2) { return createBallJoint_vf15( _arg0,  _arg1,  _arg2); }
	virtual IJoint * createFixedJoint_vf16(IRigidBody *  _arg0, IRigidBody *  _arg1) = 0;
	inline IJoint * createFixedJoint(IRigidBody *  _arg0, IRigidBody *  _arg1) { return createFixedJoint_vf16( _arg0,  _arg1); }
	virtual IRayCaster * createRayCaster_vf17(float  _arg0) = 0;
	inline IRayCaster * createRayCaster(float  _arg0) { return createRayCaster_vf17( _arg0); }
	virtual CoreCPUTimes getCoreCPUTimes_vf18() = 0;
	inline CoreCPUTimes getCoreCPUTimes() { return getCoreCPUTimes_vf18(); }
	virtual void setNoCollisionSteps_vf19(int  _arg0) = 0;
	inline void setNoCollisionSteps(int  _arg0) { return setNoCollisionSteps_vf19( _arg0); }
	virtual ~IPhysicsCore();
	inline void dtor() { typedef void (*_fpt)(IPhysicsCore *pthis); _fpt _f=(_fpt)_drva(2931728); _f(this); }
};

class Node {
public:
	mat44f matrix;
	mat44f matrixWS;
	bool isWSIdentity;
	std::vector<Node *,std::allocator<Node *> > nodes;
	Node * parent;
	int priority;
	bool needsMatrixWS;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	bool isActive;
	inline Node()  { }
	inline void ctor(Node & node) { typedef void (*_fpt)(Node *pthis, Node &); _fpt _f=(_fpt)_drva(2152896); _f(this, node); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & n) { typedef void (*_fpt)(Node *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2153232); _f(this, n); }
	virtual ~Node();
	inline void dtor() { typedef void (*_fpt)(Node *pthis); _fpt _f=(_fpt)_drva(2153568); _f(this); }
	inline vec3f worldToLocal(vec3f & pos) { typedef vec3f (*_fpt)(Node *pthis, vec3f &); _fpt _f=(_fpt)_drva(2156336); return _f(this, pos); }
	inline vec3f worldToLocalNormal(vec3f & pos) { typedef vec3f (*_fpt)(Node *pthis, vec3f &); _fpt _f=(_fpt)_drva(2156752); return _f(this, pos); }
	inline vec3f localToWorld(vec3f & pos) { typedef vec3f (*_fpt)(Node *pthis, vec3f &); _fpt _f=(_fpt)_drva(2155056); return _f(this, pos); }
	inline Node * getParent() { typedef Node * (*_fpt)(Node *pthis); _fpt _f=(_fpt)_drva(2154880); return _f(this); }
	virtual void addChild_vf1(Node * n);
	inline void addChild_impl(Node * n) { typedef void (*_fpt)(Node *pthis, Node *); _fpt _f=(_fpt)_drva(2153856); return _f(this, n); }
	inline void addChild(Node * n) { return addChild_vf1(n); }
	virtual void compile_vf2(CompileContext * cc);
	inline void compile_impl(CompileContext * cc) { typedef void (*_fpt)(Node *pthis, CompileContext *); _fpt _f=(_fpt)_drva(2153936); return _f(this, cc); }
	inline void compile(CompileContext * cc) { return compile_vf2(cc); }
	virtual void render_vf3(RenderContext * rc);
	inline void render_impl(RenderContext * rc) { typedef void (*_fpt)(Node *pthis, RenderContext *); _fpt _f=(_fpt)_drva(2155792); return _f(this, rc); }
	inline void render(RenderContext * rc) { return render_vf3(rc); }
	virtual void removeChild_vf4(Node * n);
	inline void removeChild_impl(Node * n) { typedef void (*_fpt)(Node *pthis, Node *); _fpt _f=(_fpt)_drva(2155680); return _f(this, n); }
	inline void removeChild(Node * n) { return removeChild_vf4(n); }
	virtual Node * findChildByName_vf5(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & sname, bool recursive);
	inline Node * findChildByName_impl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & sname, bool recursive) { typedef Node * (*_fpt)(Node *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, bool); _fpt _f=(_fpt)_drva(2154048); return _f(this, sname, recursive); }
	inline Node * findChildByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & sname, bool recursive) { return findChildByName_vf5(sname, recursive); }
	virtual void findChildrenByName_vf6(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & nname, std::vector<Node *,std::allocator<Node *> > & result);
	inline void findChildrenByName_impl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & nname, std::vector<Node *,std::allocator<Node *> > & result) { typedef void (*_fpt)(Node *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::vector<Node *,std::allocator<Node *> > &); _fpt _f=(_fpt)_drva(2154320); return _f(this, nname, result); }
	inline void findChildrenByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & nname, std::vector<Node *,std::allocator<Node *> > & result) { return findChildrenByName_vf6(nname, result); }
	virtual void findChildrenByPrefix_vf7(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & prefix, std::vector<Node *,std::allocator<Node *> > & result);
	inline void findChildrenByPrefix_impl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & prefix, std::vector<Node *,std::allocator<Node *> > & result) { typedef void (*_fpt)(Node *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::vector<Node *,std::allocator<Node *> > &); _fpt _f=(_fpt)_drva(2154576); return _f(this, prefix, result); }
	inline void findChildrenByPrefix(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & prefix, std::vector<Node *,std::allocator<Node *> > & result) { return findChildrenByPrefix_vf7(prefix, result); }
	virtual mat44f getWorldMatrix_vf8();
	inline mat44f getWorldMatrix_impl() { typedef mat44f (*_fpt)(Node *pthis); _fpt _f=(_fpt)_drva(2154896); return _f(this); }
	inline mat44f getWorldMatrix() { return getWorldMatrix_vf8(); }
	virtual void printTree_vf9(int ident);
	inline void printTree_impl(int ident) { typedef void (*_fpt)(Node *pthis, int); _fpt _f=(_fpt)_drva(2155120); return _f(this, ident); }
	inline void printTree(int ident) { return printTree_vf9(ident); }
	inline void visit(std::function<bool __cdecl(Node *)> * onVisit, std::function<void __cdecl(Node *)> * onPostVisit) { typedef void (*_fpt)(Node *pthis, std::function<bool __cdecl(Node *)> *, std::function<void __cdecl(Node *)> *); _fpt _f=(_fpt)_drva(2156048); return _f(this, onVisit, onPostVisit); }
	virtual void renderAudio_vf10();
	inline void renderAudio_impl() { typedef void (*_fpt)(Node *pthis); _fpt _f=(_fpt)_drva(2155936); return _f(this); }
	inline void renderAudio() { return renderAudio_vf10(); }
	inline void setActive(bool a) { typedef void (*_fpt)(Node *pthis, bool); _fpt _f=(_fpt)_drva(673008); return _f(this, a); }
	inline bool getActive() { typedef bool (*_fpt)(Node *pthis); _fpt _f=(_fpt)_drva(418752); return _f(this); }
};

class Material {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	Shader * shader;
	bool doubleFace;
	bool wireFrame;
	GraphicsManager * graphics;
	std::vector<MaterialVar *,std::allocator<MaterialVar *> > vars;
	std::vector<MaterialResource,std::allocator<MaterialResource> > resources;
	std::vector<CBuffer,std::allocator<CBuffer> > cBuffers;
	std::vector<ShaderVariable *,std::allocator<ShaderVariable *> > shaderVars;
	int guid;
	DepthMode depthMode;
	BlendMode blendMode;
	CullMode cullMode;
	bool doubleFaceShadow;
	inline Material()  { }
	inline void ctor(Material * mat) { typedef void (*_fpt)(Material *pthis, Material *); _fpt _f=(_fpt)_drva(2136640); _f(this, mat); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, GraphicsManager * graphics) { typedef void (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, GraphicsManager *); _fpt _f=(_fpt)_drva(2136432); _f(this, iname, graphics); }
	inline void dtor() { typedef void (*_fpt)(Material *pthis); _fpt _f=(_fpt)_drva(2137440); _f(this); }
	inline void apply(RenderContext * rc) { typedef void (*_fpt)(Material *pthis, RenderContext *); _fpt _f=(_fpt)_drva(2139872); return _f(this, rc); }
	inline void setShader(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef void (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2144000); return _f(this, name); }
	inline void setVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, vec4f & value) { typedef void (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, vec4f &); _fpt _f=(_fpt)_drva(2144528); return _f(this, name, value); }
	inline void setVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, vec3f & value) { typedef void (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, vec3f &); _fpt _f=(_fpt)_drva(2144368); return _f(this, name, value); }
	inline void setVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, float value) { typedef void (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, float); _fpt _f=(_fpt)_drva(2144688); return _f(this, name, value); }
	inline MaterialVar * getVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & vname) { typedef MaterialVar * (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2140976); return _f(this, vname); }
	inline void setTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & rname, Texture & tex) { typedef void (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, Texture &); _fpt _f=(_fpt)_drva(2144064); return _f(this, rname, tex); }
	inline int getResourceIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef int (*_fpt)(Material *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2140736); return _f(this, name); }
	inline void resetVars() { typedef void (*_fpt)(Material *pthis); _fpt _f=(_fpt)_drva(2143552); return _f(this); }
	inline void initShaderVars(bool updateOptions) { typedef void (*_fpt)(Material *pthis, bool); _fpt _f=(_fpt)_drva(2141232); return _f(this, updateOptions); }
	inline void createCBuffers() { typedef void (*_fpt)(Material *pthis); _fpt _f=(_fpt)_drva(2140480); return _f(this); }
};

class ksgui_Control {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	ksgui_ksRect rect;
	vec4f backColor;
	vec4f borderColor;
	vec4f foreColor;
	vec4f highlightTextColor;
	bool highlight;
	vec4f backTextureColor;
	std::shared_ptr<Font> font;
	std::vector<ksgui_Control *,std::allocator<ksgui_Control *> > controls;
	ksgui_Control * parent;
	eFontAlign fontAlign;
	Event<ksgui_OnControlClicked> evClicked;
	void * tag;
	Texture backTexture;
	bool drawBorder;
	bool drawBackground;
	float repeatInterval;
	float fontScale;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > text;
	bool isHighlight;
	ksgui_GUI * gui;
	bool visible;
	bool isMouseDown;
	float intervalCounter;
	bool isRepeating;
	float scaleMult;
	GLRenderer * controlGLR;
	ksgui_ksRect rectBase;
	inline ksgui_Control()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_Control *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, ksgui_GUI *); _fpt _f=(_fpt)_drva(2376016); _f(this, iname, igui); }
	virtual ~ksgui_Control();
	inline void dtor() { typedef void (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(2376960); _f(this); }
	inline void setVisible(bool value) { typedef void (*_fpt)(ksgui_Control *pthis, bool); _fpt _f=(_fpt)_drva(2380192); return _f(this, value); }
	inline bool isVisible() { typedef bool (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(349312); return _f(this); }
	virtual bool hitTest_vf1(int x, int y);
	inline bool hitTest_impl(int x, int y) { typedef bool (*_fpt)(ksgui_Control *pthis, int, int); _fpt _f=(_fpt)_drva(2377520); return _f(this, x, y); }
	inline bool hitTest(int x, int y) { return hitTest_vf1(x, y); }
	virtual void addControl_vf2(ksgui_Control * c);
	inline void addControl_impl(ksgui_Control * c) { typedef void (*_fpt)(ksgui_Control *pthis, ksgui_Control *); _fpt _f=(_fpt)_drva(2377360); return _f(this, c); }
	inline void addControl(ksgui_Control * c) { return addControl_vf2(c); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Control *pthis, float); _fpt _f=(_fpt)_drva(2378624); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual vec2f localToWorld_vf4(vec2f & pos);
	inline vec2f localToWorld_impl(vec2f & pos) { typedef vec2f (*_fpt)(ksgui_Control *pthis, vec2f &); _fpt _f=(_fpt)_drva(2377584); return _f(this, pos); }
	inline vec2f localToWorld(vec2f & pos) { return localToWorld_vf4(pos); }
	virtual vec2f worldToLocal_vf5(vec2f & pos);
	inline vec2f worldToLocal_impl(vec2f & pos) { typedef vec2f (*_fpt)(ksgui_Control *pthis, vec2f &); _fpt _f=(_fpt)_drva(2380464); return _f(this, pos); }
	inline vec2f worldToLocal(vec2f & pos) { return worldToLocal_vf5(pos); }
	virtual void getWorldRect_vf6(ksgui_ksRect & irect);
	inline void getWorldRect_impl(ksgui_ksRect & irect) { typedef void (*_fpt)(ksgui_Control *pthis, ksgui_ksRect &); _fpt _f=(_fpt)_drva(2377408); return _f(this, irect); }
	inline void getWorldRect(ksgui_ksRect & irect) { return getWorldRect_vf6(irect); }
	inline float getWidth() { typedef float (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(220128); return _f(this); }
	inline float getHeight() { typedef float (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(220080); return _f(this); }
	inline void setPosition(float x, float y) { typedef void (*_fpt)(ksgui_Control *pthis, float, float); _fpt _f=(_fpt)_drva(2379936); return _f(this, x, y); }
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_Control *pthis, float, float); _fpt _f=(_fpt)_drva(2380064); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	virtual void onVisibleChanged_vf8(bool newValue);
	inline void onVisibleChanged_impl(bool newValue) { typedef void (*_fpt)(ksgui_Control *pthis, bool); _fpt _f=(_fpt)_drva(221168); return _f(this, newValue); }
	inline void onVisibleChanged(bool newValue) { return onVisibleChanged_vf8(newValue); }
	virtual bool isMousePressing_vf9();
	inline bool isMousePressing_impl() { typedef bool (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(220144); return _f(this); }
	inline bool isMousePressing() { return isMousePressing_vf9(); }
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_Control *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2377872); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void onMouseUp_vf11(OnMouseUpEvent & message);
	inline void onMouseUp_impl(OnMouseUpEvent & message) { typedef void (*_fpt)(ksgui_Control *pthis, OnMouseUpEvent &); _fpt _f=(_fpt)_drva(2378384); return _f(this, message); }
	inline void onMouseUp(OnMouseUpEvent & message) { return onMouseUp_vf11(message); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_Control *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2378160); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void onMouseWheelMovedEvent_vf13(OnMouseWheelMovedEvent & message);
	inline void onMouseWheelMovedEvent_impl(OnMouseWheelMovedEvent & message) { typedef void (*_fpt)(ksgui_Control *pthis, OnMouseWheelMovedEvent &); _fpt _f=(_fpt)_drva(2378512); return _f(this, message); }
	inline void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message) { return onMouseWheelMovedEvent_vf13(message); }
	virtual void onKeyChar_vf14(unsigned int key);
	inline void onKeyChar_impl(unsigned int key) { typedef void (*_fpt)(ksgui_Control *pthis, unsigned int); _fpt _f=(_fpt)_drva(2377680); return _f(this, key); }
	inline void onKeyChar(unsigned int key) { return onKeyChar_vf14(key); }
	virtual void onKeyDown_vf15(OnKeyEvent & message);
	inline void onKeyDown_impl(OnKeyEvent & message) { typedef void (*_fpt)(ksgui_Control *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(2377776); return _f(this, message); }
	inline void onKeyDown(OnKeyEvent & message) { return onKeyDown_vf15(message); }
	virtual void setRepeatInterval_vf16(float i);
	inline void setRepeatInterval_impl(float i) { typedef void (*_fpt)(ksgui_Control *pthis, float); _fpt _f=(_fpt)_drva(2380048); return _f(this, i); }
	inline void setRepeatInterval(float i) { return setRepeatInterval_vf16(i); }
	virtual void setText_vf17(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText);
	inline void setText_impl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText) { typedef void (*_fpt)(ksgui_Control *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2011536); return _f(this, aText); }
	inline void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText) { return setText_vf17(aText); }
	virtual std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & getText_vf18();
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & getText_impl() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(220112); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & getText() { return getText_vf18(); }
	virtual void scaleByMult_vf19(float value);
	inline void scaleByMult_impl(float value) { typedef void (*_fpt)(ksgui_Control *pthis, float); _fpt _f=(_fpt)_drva(2379600); return _f(this, value); }
	inline void scaleByMult(float value) { return scaleByMult_vf19(value); }
	virtual void scaleByMult_vf20();
	inline void scaleByMult_impl() { typedef void (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(2386448); return _f(this); }
	inline void scaleByMult() { return scaleByMult_vf20(); }
	inline void resetBaseRect() { typedef void (*_fpt)(ksgui_Control *pthis); _fpt _f=(_fpt)_drva(2379472); return _f(this); }
	inline void stepRepeatInterval(float dt) { typedef void (*_fpt)(ksgui_Control *pthis, float); _fpt _f=(_fpt)_drva(2380224); return _f(this, dt); }
};

class RenderWindow {
public:
	void * hWnd;
	Event<OnMouseDownEvent> evOnMouseDown;
	Event<OnMouseWheelMovedEvent> evOnMouseWheelMoved;
	Event<OnMouseUpEvent> evOnMouseUp;
	Event<OnMouseMoveEvent> evOnMouseMove;
	Event<OnWindowResizeEvent> evOnWindowResize;
	Event<OnKeyCharEvent> evOnKeyChar;
	Event<OnKeyEvent> evOnKeyDown;
	Event<OnWindowClosedEvent> evOnWindowClosed;
	Event<OnKeyEvent> evOnKeyUp;
	bool disableEventsWithMouseHidden;
	inline RenderWindow()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, VideoSettings & videoSettings) { typedef void (*_fpt)(RenderWindow *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, VideoSettings &); _fpt _f=(_fpt)_drva(2086464); _f(this, name, videoSettings); }
	virtual ~RenderWindow();
	inline void dtor() { typedef void (*_fpt)(RenderWindow *pthis); _fpt _f=(_fpt)_drva(2086880); _f(this); }
	inline bool step() { typedef bool (*_fpt)(RenderWindow *pthis); _fpt _f=(_fpt)_drva(2088352); return _f(this); }
	inline bool hasFocus() { typedef bool (*_fpt)(RenderWindow *pthis); _fpt _f=(_fpt)_drva(2087216); return _f(this); }
	inline void setFocus() { typedef void (*_fpt)(RenderWindow *pthis); _fpt _f=(_fpt)_drva(2088336); return _f(this); }
};

struct DrivetrainControllers {
public:
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > awdFrontShare;
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > awdCenterLock;
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > singleDiffLock;
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > awd2;
	inline DrivetrainControllers()  { }
	inline void dtor() { typedef void (*_fpt)(DrivetrainControllers *pthis); _fpt _f=(_fpt)_drva(2515184); _f(this); }
};

struct SteerBrake {
public:
	bool isActive;
	DynamicController controller;
	inline SteerBrake()  { }
	inline void dtor() { typedef void (*_fpt)(SteerBrake *pthis); _fpt _f=(_fpt)_drva(2549856); _f(this); }
};

struct ERSPowerController {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	DynamicController ctrl;
	inline ERSPowerController()  { }
	inline void ctor(ERSPowerController & __that) { typedef void (*_fpt)(ERSPowerController *pthis, ERSPowerController &); _fpt _f=(_fpt)_drva(2692640); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(ERSPowerController *pthis); _fpt _f=(_fpt)_drva(2693104); _f(this); }
};

struct ACClient_ClientSessionTransition {
public:
	bool isTransitioning;
	UDPPacket sessionPacket;
	RemoteSessionResume sessionResume;
	inline ACClient_ClientSessionTransition()  { }
	inline void dtor() { typedef void (*_fpt)(ACClient_ClientSessionTransition *pthis); _fpt _f=(_fpt)_drva(246384); _f(this); }
};

class ICarPhysicsStateProvider {
public:
	inline ICarPhysicsStateProvider()  { }
	virtual ~ICarPhysicsStateProvider();
	inline void dtor() { typedef void (*_fpt)(ICarPhysicsStateProvider *pthis); _fpt _f=(_fpt)_drva(783520); _f(this); }
	virtual void getPhysicsState_vf1(CarPhysicsState &  _arg0) = 0;
	inline void getPhysicsState(CarPhysicsState &  _arg0) { return getPhysicsState_vf1( _arg0); }
	virtual void getWingState_vf2(std::vector<WingState,std::allocator<WingState> > &  _arg0) = 0;
	inline void getWingState(std::vector<WingState,std::allocator<WingState> > &  _arg0) { return getWingState_vf2( _arg0); }
};

struct SteeringSystem {
public:
	float linearRatio;
	Car * car;
	bool has4ws;
	DynamicController ctrl4ws;
	inline SteeringSystem()  { }
	inline void dtor() { typedef void (*_fpt)(SteeringSystem *pthis); _fpt _f=(_fpt)_drva(2864640); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(SteeringSystem *pthis, Car *); _fpt _f=(_fpt)_drva(2851024); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(SteeringSystem *pthis, float); _fpt _f=(_fpt)_drva(2851248); return _f(this, dt); }
};

struct TurboDynamicController {
public:
	Turbo * turbo;
	DynamicController controller;
	bool isWastegate;
	inline TurboDynamicController()  { }
	inline void ctor() { typedef void (*_fpt)(TurboDynamicController *pthis); _fpt _f=(_fpt)_drva(2643056); _f(this); }
	inline void dtor() { typedef void (*_fpt)(TurboDynamicController *pthis); _fpt _f=(_fpt)_drva(2549856); _f(this); }
};

struct AntirollBar {
public:
	IRigidBody * carBody;
	ISuspension * hubs[0x2];
	DynamicController ctrl;
	float k;
	inline AntirollBar()  { }
	inline void dtor() { typedef void (*_fpt)(AntirollBar *pthis); _fpt _f=(_fpt)_drva(2864640); _f(this); }
	inline void init(IRigidBody * cb, ISuspension * * sus) { typedef void (*_fpt)(AntirollBar *pthis, IRigidBody *, ISuspension * *); _fpt _f=(_fpt)_drva(2864656); return _f(this, cb, sus); }
	inline void step(float dt) { typedef void (*_fpt)(AntirollBar *pthis, float); _fpt _f=(_fpt)_drva(2864704); return _f(this, dt); }
	inline void ctor() { typedef void (*_fpt)(AntirollBar *pthis); _fpt _f=(_fpt)_drva(2538800); _f(this); }
};

class MaterialFilter {
public:
	Material * lastMaterial;
	inline MaterialFilter()  { }
	inline void ctor() { typedef void (*_fpt)(MaterialFilter *pthis); _fpt _f=(_fpt)_drva(2202928); _f(this); }
	virtual ~MaterialFilter();
	inline void dtor() { typedef void (*_fpt)(MaterialFilter *pthis); _fpt _f=(_fpt)_drva(2202960); _f(this); }
	virtual void apply_vf1(std::shared_ptr<Material> & material, RenderContext * rc);
	inline void apply_impl(std::shared_ptr<Material> & material, RenderContext * rc) { typedef void (*_fpt)(MaterialFilter *pthis, std::shared_ptr<Material> &, RenderContext *); _fpt _f=(_fpt)_drva(2203024); return _f(this, material, rc); }
	inline void apply(std::shared_ptr<Material> & material, RenderContext * rc) { return apply_vf1(material, rc); }
	inline void resetMaterialCache() { typedef void (*_fpt)(MaterialFilter *pthis); _fpt _f=(_fpt)_drva(2203088); return _f(this); }
};

class ksgui_Label : public ksgui_Control {
public:
	unsigned int maxNumberOfCharDisplayed;
	inline ksgui_Label()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_Label *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2403696); _f(this, iname, igui); }
	virtual ~ksgui_Label();
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Label *pthis, float); _fpt _f=(_fpt)_drva(2404288); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void scaleByMult_vf19(float value);
	inline void scaleByMult_impl(float value) { typedef void (*_fpt)(ksgui_Label *pthis, float); _fpt _f=(_fpt)_drva(2404688); return _f(this, value); }
	inline void scaleByMult(float value) { return scaleByMult_vf19(value); }
};

class ksgui_ProgressBar : public ksgui_Control {
public:
	float maxValue;
	float minValue;
	float value;
	bool isVertical;
	bool isInverted;
	inline ksgui_ProgressBar()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_ProgressBar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2416368); _f(this, iname, igui); }
	virtual ~ksgui_ProgressBar();
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_ProgressBar *pthis, float); _fpt _f=(_fpt)_drva(2416624); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void renderVertical(float dt) { typedef void (*_fpt)(ksgui_ProgressBar *pthis, float); _fpt _f=(_fpt)_drva(2417120); return _f(this, dt); }
	inline void renderHorizontal(float dt) { typedef void (*_fpt)(ksgui_ProgressBar *pthis, float); _fpt _f=(_fpt)_drva(2416688); return _f(this, dt); }
};

class ksgui_ConnectedLabel : public ksgui_Control {
public:
	ksgui_VariableConnection variableConnection;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > label;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > suffix;
	int precision;
	float * fValue;
	int * iValue;
	inline ksgui_ConnectedLabel()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui, int * var) { typedef void (*_fpt)(ksgui_ConnectedLabel *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, ksgui_GUI *, int *); _fpt _f=(_fpt)_drva(4515840); _f(this, iname, igui, var); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui, float * var) { typedef void (*_fpt)(ksgui_ConnectedLabel *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, ksgui_GUI *, float *); _fpt _f=(_fpt)_drva(4516000); _f(this, iname, igui, var); }
	virtual ~ksgui_ConnectedLabel();
	inline void dtor() { typedef void (*_fpt)(ksgui_ConnectedLabel *pthis); _fpt _f=(_fpt)_drva(4516160); _f(this); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_ConnectedLabel *pthis, float); _fpt _f=(_fpt)_drva(4516784); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void init() { typedef void (*_fpt)(ksgui_ConnectedLabel *pthis); _fpt _f=(_fpt)_drva(4516352); return _f(this); }
};

class KeyboardManager {
public:
	RenderWindow & renderWindow;
	std::vector<IKeyEventListener *,std::allocator<IKeyEventListener *> > listeners;
	IKeyEventListener * focusListener;
	inline KeyboardManager()  : renderWindow(*((RenderWindow*)NULL)) { }
	inline void ctor(RenderWindow & arenderWindow) { typedef void (*_fpt)(KeyboardManager *pthis, RenderWindow &); _fpt _f=(_fpt)_drva(2372096); _f(this, arenderWindow); }
	virtual ~KeyboardManager();
	inline void dtor() { typedef void (*_fpt)(KeyboardManager *pthis); _fpt _f=(_fpt)_drva(2372176); _f(this); }
	inline void registerEventHandlers() { typedef void (*_fpt)(KeyboardManager *pthis); _fpt _f=(_fpt)_drva(2375440); return _f(this); }
	inline void onKeyDownEvent(OnKeyEvent & message) { typedef void (*_fpt)(KeyboardManager *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(2374432); return _f(this, message); }
	inline void onKeyPressEvent(OnKeyCharEvent & message) { typedef void (*_fpt)(KeyboardManager *pthis, OnKeyCharEvent &); _fpt _f=(_fpt)_drva(2374560); return _f(this, message); }
	inline void addGodListener(IKeyEventListener * l) { typedef void (*_fpt)(KeyboardManager *pthis, IKeyEventListener *); _fpt _f=(_fpt)_drva(2373984); return _f(this, l); }
	inline void getFocus(IKeyEventListener * l) { typedef void (*_fpt)(KeyboardManager *pthis, IKeyEventListener *); _fpt _f=(_fpt)_drva(2944432); return _f(this, l); }
	inline void releaseFocus(IKeyEventListener * l) { typedef void (*_fpt)(KeyboardManager *pthis, IKeyEventListener *); _fpt _f=(_fpt)_drva(2375568); return _f(this, l); }
};

class Kers : public ITorqueGenerator {
public:
	KersAttachment attachment;
	Car * car;
	bool present;
	float input;
	float brakeForMaxCharge;
	float charge;
	float chargeK;
	float dischargeK;
	float angularVelocity;
	float negativeInputChargeK;
	bool hasButtonOverride;
	float currentJ;
	float maxJ;
	Curve torqueLUT;
	DynamicController controller;
	bool hasController;
	inline Kers()  { }
	virtual ~Kers();
	inline void dtor() { typedef void (*_fpt)(Kers *pthis); _fpt _f=(_fpt)_drva(2847200); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(Kers *pthis, Car *); _fpt _f=(_fpt)_drva(2847584); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(Kers *pthis, float); _fpt _f=(_fpt)_drva(2850320); return _f(this, dt); }
	virtual float getOutputTorque_vf1();
	inline float getOutputTorque_impl() { typedef float (*_fpt)(Kers *pthis); _fpt _f=(_fpt)_drva(2847408); return _f(this); }
	inline float getOutputTorque() { return getOutputTorque_vf1(); }
	inline bool isPresent() { typedef bool (*_fpt)(Kers *pthis); _fpt _f=(_fpt)_drva(888160); return _f(this); }
	inline void reset() { typedef void (*_fpt)(Kers *pthis); _fpt _f=(_fpt)_drva(2850304); return _f(this); }
	inline float getDischargeTimeS() { typedef float (*_fpt)(Kers *pthis); _fpt _f=(_fpt)_drva(2847376); return _f(this); }
};

class ksgui_TextBox : public ksgui_Control {
public:
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > textLines;
	inline ksgui_TextBox()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui, eFontType fontType) { typedef void (*_fpt)(ksgui_TextBox *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *, eFontType); _fpt _f=(_fpt)_drva(2417456); _f(this, name, gui, fontType); }
	virtual ~ksgui_TextBox();
	inline void setFormattedText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aText) { typedef void (*_fpt)(ksgui_TextBox *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2418400); return _f(this, aText); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_TextBox *pthis, float); _fpt _f=(_fpt)_drva(2418080); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_TextBox *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2418064); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
};

class ksgui_PopOver : public ksgui_Control {
public:
	ksgui_TextBox * textBox;
	ksgui_Label * title;
	inline ksgui_PopOver()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * aGui) { typedef void (*_fpt)(ksgui_PopOver *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2447104); _f(this, name, aGui); }
	virtual ~ksgui_PopOver();
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_PopOver *pthis, float); _fpt _f=(_fpt)_drva(2447904); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void renderPopOver(float dt) { typedef void (*_fpt)(ksgui_PopOver *pthis, float); _fpt _f=(_fpt)_drva(2447984); return _f(this, dt); }
	inline void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aText) { typedef void (*_fpt)(ksgui_PopOver *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2448672); return _f(this, aText); }
	inline void setLabelTitle(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aTitle) { typedef void (*_fpt)(ksgui_PopOver *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2448560); return _f(this, aTitle); }
};

class ksgui_CheckBox : public ksgui_Control {
public:
	Event<ksgui_OnCheckBoxChanged> evOnCheckBoxChanged;
	bool boxValue;
	ksgui_Control * box;
	ksgui_Control * label;
	inline ksgui_CheckBox()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui) { typedef void (*_fpt)(ksgui_CheckBox *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2402416); _f(this, name, gui); }
	virtual ~ksgui_CheckBox();
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_CheckBox *pthis, float); _fpt _f=(_fpt)_drva(2403312); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void setCheck(bool aCheck) { typedef void (*_fpt)(ksgui_CheckBox *pthis, bool); _fpt _f=(_fpt)_drva(2403648); return _f(this, aCheck); }
};

class ksgui_TextInput : public ksgui_Control {
public:
	vec4f backColorFocus;
	vec4f backColorUnfocus;
	Event<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > evValidate;
	Event<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > evKeyDown;
	bool hasFocus;
	unsigned int maxNumberOfChar;
	inline ksgui_TextInput()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui) { typedef void (*_fpt)(ksgui_TextInput *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2380560); _f(this, name, gui); }
	virtual ~ksgui_TextInput();
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_TextInput *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2381536); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_TextInput *pthis, float); _fpt _f=(_fpt)_drva(2381568); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void onKeyChar_vf14(unsigned int key);
	inline void onKeyChar_impl(unsigned int key) { typedef void (*_fpt)(ksgui_TextInput *pthis, unsigned int); _fpt _f=(_fpt)_drva(2381264); return _f(this, key); }
	inline void onKeyChar(unsigned int key) { return onKeyChar_vf14(key); }
	virtual void onKeyDown_vf15(OnKeyEvent & message);
	inline void onKeyDown_impl(OnKeyEvent & message) { typedef void (*_fpt)(ksgui_TextInput *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(96368); return _f(this, message); }
	inline void onKeyDown(OnKeyEvent & message) { return onKeyDown_vf15(message); }
	inline void validateText() { typedef void (*_fpt)(ksgui_TextInput *pthis); _fpt _f=(_fpt)_drva(2381856); return _f(this); }
};

class ksgui_CustomSpinner : public ksgui_Control {
public:
	bool pressToIncrement;
	ksgui_Control * leftButton;
	ksgui_Control * rightButton;
	ksgui_Control * upButton;
	ksgui_Control * downButton;
	ksgui_Label * title;
	Texture bar;
	float barPositionNormalized;
	CustomSpinnerMode mode;
	inline ksgui_CustomSpinner()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * aGui, CustomSpinnerMode aMode) { typedef void (*_fpt)(ksgui_CustomSpinner *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *, CustomSpinnerMode); _fpt _f=(_fpt)_drva(2419408); _f(this, name, aGui, aMode); }
	virtual ~ksgui_CustomSpinner();
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_CustomSpinner *pthis, float, float); _fpt _f=(_fpt)_drva(2422512); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_CustomSpinner *pthis, float); _fpt _f=(_fpt)_drva(2420928); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void scaleByMult_vf19(float value);
	inline void scaleByMult_impl(float value) { typedef void (*_fpt)(ksgui_CustomSpinner *pthis, float); _fpt _f=(_fpt)_drva(2422464); return _f(this, value); }
	inline void scaleByMult(float value) { return scaleByMult_vf19(value); }
	inline void resize(float value) { typedef void (*_fpt)(ksgui_CustomSpinner *pthis, float); _fpt _f=(_fpt)_drva(2421696); return _f(this, value); }
};

class ksgui_ListBoxRow : public ksgui_Control {
public:
	vec4f rowBackground;
	int id;
	bool drawRowBackground;
	std::vector<ksgui_Label *,std::allocator<ksgui_Label *> > columns;
	inline ksgui_ListBoxRow()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, unsigned int numberOfColumns, ksgui_GUI * aGui) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, unsigned int, ksgui_GUI *); _fpt _f=(_fpt)_drva(2448848); _f(this, name, numberOfColumns, aGui); }
	virtual ~ksgui_ListBoxRow();
	virtual void setSize_vf7(float width, float height);
	inline void setSize_impl(float width, float height) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, float, float); _fpt _f=(_fpt)_drva(2451392); return _f(this, width, height); }
	inline void setSize(float width, float height) { return setSize_vf7(width, height); }
	inline void setPosition(float x, float y) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, float, float); _fpt _f=(_fpt)_drva(2450864); return _f(this, x, y); }
	inline void setRow(ksgui_ListBoxRowData * data) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, ksgui_ListBoxRowData *); _fpt _f=(_fpt)_drva(2451072); return _f(this, data); }
	inline void setFontSize(float size) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, float); _fpt _f=(_fpt)_drva(2450784); return _f(this, size); }
	inline void clear() { typedef void (*_fpt)(ksgui_ListBoxRow *pthis); _fpt _f=(_fpt)_drva(2449984); return _f(this); }
	inline void setFontAlignment(eFontAlign alignment) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, eFontAlign); _fpt _f=(_fpt)_drva(2450704); return _f(this, alignment); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_ListBoxRow *pthis, float); _fpt _f=(_fpt)_drva(2450368); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
};

class ksgui_MovingBar : public ksgui_Control {
public:
	ksgui_ScrollBar * sb;
	Texture upperTexture;
	Texture lowerTexture;
	Texture middleTexture;
	vec2f pressingPoint;
	inline ksgui_MovingBar()  { }
	inline void ctor(ksgui_ScrollBar * scrollBar, ksgui_GUI * aGui) { typedef void (*_fpt)(ksgui_MovingBar *pthis, ksgui_ScrollBar *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2468704); _f(this, scrollBar, aGui); }
	virtual ~ksgui_MovingBar();
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_MovingBar *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2471936); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_MovingBar *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2472112); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void onMouseUp_vf11(OnMouseUpEvent & message);
	inline void onMouseUp_impl(OnMouseUpEvent & message) { typedef void (*_fpt)(ksgui_MovingBar *pthis, OnMouseUpEvent &); _fpt _f=(_fpt)_drva(2472640); return _f(this, message); }
	inline void onMouseUp(OnMouseUpEvent & message) { return onMouseUp_vf11(message); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_MovingBar *pthis, float); _fpt _f=(_fpt)_drva(2472912); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
};

class ksgui_ActiveButton : public ksgui_Control {
public:
	vec2f textCoord;
	vec4f unselectedColor;
	vec4f selectedColor;
	vec4f rollOnColor;
	vec4f inactiveColor;
	ksgui_eActiveButtonStates status;
	bool blanked;
	bool highlightOnCursorOver;
	bool isActive;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > activeButtonText;
	bool selected;
	bool usingMultipleTextures;
	ksgui_eActiveButtonStates oldStatus;
	Texture textureOn;
	Texture textureOff;
	inline ksgui_ActiveButton()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2404704); _f(this, name, gui); }
	virtual ~ksgui_ActiveButton();
	inline void dtor() { typedef void (*_fpt)(ksgui_ActiveButton *pthis); _fpt _f=(_fpt)_drva(2405312); _f(this); }
	inline void setMultipleTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2406144); return _f(this, filename); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, float); _fpt _f=(_fpt)_drva(2405648); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2405488); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	inline void setSelected(bool value) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, bool); _fpt _f=(_fpt)_drva(2407040); return _f(this, value); }
	virtual void onVisibleChanged_vf8(bool newValue);
	inline void onVisibleChanged_impl(bool newValue) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, bool); _fpt _f=(_fpt)_drva(2405632); return _f(this, newValue); }
	inline void onVisibleChanged(bool newValue) { return onVisibleChanged_vf8(newValue); }
	virtual void setText_vf17(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText);
	inline void setText_impl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText) { typedef void (*_fpt)(ksgui_ActiveButton *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2407072); return _f(this, aText); }
	inline void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText) { return setText_vf17(aText); }
};

class ksgui_Form : public ksgui_Control {
public:
	ksgui_Control * formTitle;
	bool devApp;
	bool autohide;
	bool blocked;
	float scaleToBeSaved;
	bool canBeScaled;
	ksgui_Control * icon;
	ksgui_ActiveButton * pinIcon;
	ksgui_Control * zoomInIcon;
	ksgui_Control * zoomOutIcon;
	float HEADER_HEIGHT;
	float scaleStep;
	inline ksgui_Form()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui, bool icanBeScaled) { typedef void (*_fpt)(ksgui_Form *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *, bool); _fpt _f=(_fpt)_drva(2381968); _f(this, iname, igui, icanBeScaled); }
	virtual ~ksgui_Form();
	inline void dtor() { typedef void (*_fpt)(ksgui_Form *pthis); _fpt _f=(_fpt)_drva(2384832); _f(this); }
	inline void setAutoHideMode(bool mode) { typedef void (*_fpt)(ksgui_Form *pthis, bool); _fpt _f=(_fpt)_drva(2386480); return _f(this, mode); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Form *pthis, float); _fpt _f=(_fpt)_drva(2386144); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_Form *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2385696); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void onTitleClicked_vf21(ksgui_OnControlClicked & message);
	inline void onTitleClicked_impl(ksgui_OnControlClicked & message) { typedef void (*_fpt)(ksgui_Form *pthis, ksgui_OnControlClicked &); _fpt _f=(_fpt)_drva(2385968); return _f(this, message); }
	inline void onTitleClicked(ksgui_OnControlClicked & message) { return onTitleClicked_vf21(message); }
	virtual void onKeyChar_vf14(unsigned int key);
	inline void onKeyChar_impl(unsigned int key) { typedef void (*_fpt)(ksgui_Form *pthis, unsigned int); _fpt _f=(_fpt)_drva(2385664); return _f(this, key); }
	inline void onKeyChar(unsigned int key) { return onKeyChar_vf14(key); }
	virtual void onKeyDown_vf15(OnKeyEvent & message);
	inline void onKeyDown_impl(OnKeyEvent & message) { typedef void (*_fpt)(ksgui_Form *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(2385680); return _f(this, message); }
	inline void onKeyDown(OnKeyEvent & message) { return onKeyDown_vf15(message); }
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_Form *pthis, float, float); _fpt _f=(_fpt)_drva(2387296); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	inline void setIcon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * fileName) { typedef void (*_fpt)(ksgui_Form *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2386704); return _f(this, fileName); }
	inline void setPosition(float left, float top) { typedef void (*_fpt)(ksgui_Form *pthis, float, float); _fpt _f=(_fpt)_drva(2387280); return _f(this, left, top); }
	inline void setIconVisible(bool visible) { typedef void (*_fpt)(ksgui_Form *pthis, bool); _fpt _f=(_fpt)_drva(2387136); return _f(this, visible); }
	inline void setIconPosition(float x, float y) { typedef void (*_fpt)(ksgui_Form *pthis, float, float); _fpt _f=(_fpt)_drva(2387104); return _f(this, x, y); }
	inline void setIconSize(float width, float height) { typedef void (*_fpt)(ksgui_Form *pthis, float, float); _fpt _f=(_fpt)_drva(2387120); return _f(this, width, height); }
	virtual void onVisibleChanged_vf8(bool newValue);
	inline void onVisibleChanged_impl(bool newValue) { typedef void (*_fpt)(ksgui_Form *pthis, bool); _fpt _f=(_fpt)_drva(2386016); return _f(this, newValue); }
	inline void onVisibleChanged(bool newValue) { return onVisibleChanged_vf8(newValue); }
	virtual void shutdown_vf22();
	inline void shutdown_impl() { typedef void (*_fpt)(ksgui_Form *pthis); _fpt _f=(_fpt)_drva(96368); return _f(this); }
	inline void shutdown() { return shutdown_vf22(); }
	inline void setBlocked(bool value) { typedef void (*_fpt)(ksgui_Form *pthis, bool); _fpt _f=(_fpt)_drva(2386656); return _f(this, value); }
	virtual bool hitTest_vf1(int x, int y);
	inline bool hitTest_impl(int x, int y) { typedef bool (*_fpt)(ksgui_Form *pthis, int, int); _fpt _f=(_fpt)_drva(2385568); return _f(this, x, y); }
	inline bool hitTest(int x, int y) { return hitTest_vf1(x, y); }
	inline void updateScaleByMult() { typedef void (*_fpt)(ksgui_Form *pthis); _fpt _f=(_fpt)_drva(2387440); return _f(this); }
	inline float getHeaderHeight() { typedef float (*_fpt)(ksgui_Form *pthis); _fpt _f=(_fpt)_drva(3194240); return _f(this); }
	virtual void scaleByMult_vf19(float value);
	inline void scaleByMult_impl(float value) { typedef void (*_fpt)(ksgui_Form *pthis, float); _fpt _f=(_fpt)_drva(2386160); return _f(this, value); }
	inline void scaleByMult(float value) { return scaleByMult_vf19(value); }
	virtual void scaleByMult_vf20();
	inline void scaleByMult_impl() { typedef void (*_fpt)(ksgui_Form *pthis); _fpt _f=(_fpt)_drva(2386448); return _f(this); }
	inline void scaleByMult() { return scaleByMult_vf20(); }
};

class ksgui_TaskBarIcon : public ksgui_Control {
public:
	eTaskBarStatus status;
	bool isOver;
	Texture texture;
	Texture textureOn;
	vec4f unselectedColor;
	vec4f selectedColor;
	vec4f rollOverColor;
	ksgui_Control * descriptionRect;
	ksgui_Label * descriptionLabel;
	float targetX;
	inline ksgui_TaskBarIcon()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, std::shared_ptr<Font> * appFont, ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_TaskBarIcon *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, std::shared_ptr<Font> *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2474464); _f(this, name, appFont, igui); }
	virtual ~ksgui_TaskBarIcon();
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_TaskBarIcon *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2475968); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_TaskBarIcon *pthis, float); _fpt _f=(_fpt)_drva(2476240); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void setDescriptionText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * text) { typedef void (*_fpt)(ksgui_TaskBarIcon *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2476608); return _f(this, text); }
	inline void hide(bool value) { typedef void (*_fpt)(ksgui_TaskBarIcon *pthis, bool); _fpt _f=(_fpt)_drva(2475808); return _f(this, value); }
	inline void setIcon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename) { typedef void (*_fpt)(ksgui_TaskBarIcon *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2476896); return _f(this, filename); }
};

class ksgui_GameScreen : public ksgui_Control {
public:
	bool appInteractionEnabled;
	Event<bool> evDesktopShown;
	std::vector<int,std::allocator<int> > oldAppsOpened;
	inline ksgui_GameScreen()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_GameScreen *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2427856); _f(this, iname, igui); }
	virtual ~ksgui_GameScreen();
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_GameScreen *pthis, float); _fpt _f=(_fpt)_drva(2428384); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void addControl_vf21(ksgui_Control * c, bool addToTaskBar);
	inline void addControl_impl(ksgui_Control * c, bool addToTaskBar) { typedef void (*_fpt)(ksgui_GameScreen *pthis, ksgui_Control *, bool); _fpt _f=(_fpt)_drva(2428176); return _f(this, c, addToTaskBar); }
	inline void addControl(ksgui_Control * c, bool addToTaskBar) { return addControl_vf21(c, addToTaskBar); }
	inline void showDesktop() { typedef void (*_fpt)(ksgui_GameScreen *pthis); _fpt _f=(_fpt)_drva(2428528); return _f(this); }
	inline int getOldAppsCount() { typedef int (*_fpt)(ksgui_GameScreen *pthis); _fpt _f=(_fpt)_drva(2428288); return _f(this); }
	inline void shutdown() { typedef void (*_fpt)(ksgui_GameScreen *pthis); _fpt _f=(_fpt)_drva(2429120); return _f(this); }
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_GameScreen *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2428320); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void onMouseUp_vf11(OnMouseUpEvent & message);
	inline void onMouseUp_impl(OnMouseUpEvent & message) { typedef void (*_fpt)(ksgui_GameScreen *pthis, OnMouseUpEvent &); _fpt _f=(_fpt)_drva(2428352); return _f(this, message); }
	inline void onMouseUp(OnMouseUpEvent & message) { return onMouseUp_vf11(message); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_GameScreen *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2428336); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void onMouseWheelMovedEvent_vf13(OnMouseWheelMovedEvent & message);
	inline void onMouseWheelMovedEvent_impl(OnMouseWheelMovedEvent & message) { typedef void (*_fpt)(ksgui_GameScreen *pthis, OnMouseWheelMovedEvent &); _fpt _f=(_fpt)_drva(2428368); return _f(this, message); }
	inline void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message) { return onMouseWheelMovedEvent_vf13(message); }
};

class ksgui_Slider : public ksgui_Control {
public:
	Event<ksgui_OnSliderInteraction> evOnSliderInteraction;
	Event<ksgui_OnCutExtremesChanged> evOnCutExtremesChanged;
	float value;
	ksgui_Control * mPos;
	ksgui_Control * markerLeft;
	ksgui_Control * markerRight;
	inline ksgui_Slider()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * myGui, bool dressed) { typedef void (*_fpt)(ksgui_Slider *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *, bool); _fpt _f=(_fpt)_drva(2423280); _f(this, name, myGui, dressed); }
	virtual ~ksgui_Slider();
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Slider *pthis, float); _fpt _f=(_fpt)_drva(2426240); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_Slider *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2425280); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_Slider *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2425616); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void onMouseUp_vf11(OnMouseUpEvent & message);
	inline void onMouseUp_impl(OnMouseUpEvent & message) { typedef void (*_fpt)(ksgui_Slider *pthis, OnMouseUpEvent &); _fpt _f=(_fpt)_drva(2426016); return _f(this, message); }
	inline void onMouseUp(OnMouseUpEvent & message) { return onMouseUp_vf11(message); }
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_Slider *pthis, float, float); _fpt _f=(_fpt)_drva(2427504); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	inline void updateMarkers() { typedef void (*_fpt)(ksgui_Slider *pthis); _fpt _f=(_fpt)_drva(2427536); return _f(this); }
	inline void resetMarkers() { typedef void (*_fpt)(ksgui_Slider *pthis); _fpt _f=(_fpt)_drva(2426896); return _f(this); }
	inline void setMarkerLeftPosition(float perc) { typedef void (*_fpt)(ksgui_Slider *pthis, float); _fpt _f=(_fpt)_drva(2427200); return _f(this, perc); }
	inline void setMarkerRightPosition(float perc) { typedef void (*_fpt)(ksgui_Slider *pthis, float); _fpt _f=(_fpt)_drva(2427360); return _f(this, perc); }
};

class RaceEngineer {
public:
	Car * car;
	float fuelPerLapEvaluated;
	inline RaceEngineer()  { }
	inline void ctor(Car * icar) { typedef void (*_fpt)(RaceEngineer *pthis, Car *); _fpt _f=(_fpt)_drva(2595824); _f(this, icar); }
	virtual ~RaceEngineer();
	inline void dtor() { typedef void (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2595856); _f(this); }
	inline float projectCarForwardForceAtGas(float gas) { typedef float (*_fpt)(RaceEngineer *pthis, float); _fpt _f=(_fpt)_drva(2608160); return _f(this, gas); }
	inline AccelerationProfile getAccelerationProfile() { typedef AccelerationProfile (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2602320); return _f(this); }
	inline Car * getCar() { typedef Car * (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(100256); return _f(this); }
	inline float getBetaRAD() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2603664); return _f(this); }
	inline float getFrontCasterRAD() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2604352); return _f(this); }
	inline float getCasterRAD(ISuspension * isus) { typedef float (*_fpt)(RaceEngineer *pthis, ISuspension *); _fpt _f=(_fpt)_drva(2603808); return _f(this, isus); }
	inline float getFrontNaturalFrequencyHZ() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2604832); return _f(this); }
	inline float getRearNaturalFrequencyHZ() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2607344); return _f(this); }
	inline float getWeightDistribution() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2608016); return _f(this); }
	inline std::tuple<float,float> getFrontDampingRatio() { typedef std::tuple<float,float> (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2604384); return _f(this); }
	inline std::tuple<float,float> getRearDampingRatio() { typedef std::tuple<float,float> (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2606800); return _f(this); }
	inline float evalLateralGrip(float speed, float mass, float aeroModifier, float * in_loads, float current_lift_kg) { typedef float (*_fpt)(RaceEngineer *pthis, float, float, float, float *, float); _fpt _f=(_fpt)_drva(2598160); return _f(this, speed, mass, aeroModifier, in_loads, current_lift_kg); }
	inline float getPointGroundHeight(vec3f & p) { typedef float (*_fpt)(RaceEngineer *pthis, vec3f &); _fpt _f=(_fpt)_drva(2606416); return _f(this, p); }
	inline float getPointFrontShare(vec3f & p) { typedef float (*_fpt)(RaceEngineer *pthis, vec3f &); _fpt _f=(_fpt)_drva(2606288); return _f(this, p); }
	inline float projectWingsLift(float speed) { typedef float (*_fpt)(RaceEngineer *pthis, float); _fpt _f=(_fpt)_drva(2608576); return _f(this, speed); }
	inline float projectWingsDrag(float speed) { typedef float (*_fpt)(RaceEngineer *pthis, float); _fpt _f=(_fpt)_drva(2608368); return _f(this, speed); }
	inline float getTorqueToGripRatio() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2607712); return _f(this); }
	inline Tyre * getLeftDrivenTyre() { typedef Tyre * (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2605504); return _f(this); }
	inline Tyre * getRightDrivenTyre() { typedef Tyre * (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2607664); return _f(this); }
	inline float getFrontTrack() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2605072); return _f(this); }
	inline float getRearTrack() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2607584); return _f(this); }
	inline float getDynamicIndex() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2604128); return _f(this); }
	inline float getOptimalBrake() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2605856); return _f(this); }
	inline float getDrivingTyresSlip() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2604064); return _f(this); }
	inline float getMaxBrakingForce() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2605552); return _f(this); }
	inline float getBaseCarHeight() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2603520); return _f(this); }
	inline Speed getMaxSpeedFromGear(unsigned int relativeCarIndex) { typedef Speed (*_fpt)(RaceEngineer *pthis, unsigned int); _fpt _f=(_fpt)_drva(2605632); return _f(this, relativeCarIndex); }
	inline float evalAvailableBrake(float speed, float latg, float weight) { typedef float (*_fpt)(RaceEngineer *pthis, float, float, float); _fpt _f=(_fpt)_drva(2596688); return _f(this, speed, latg, weight); }
	inline float evalFrontRideHeight() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2597104); return _f(this); }
	inline float evalRearRideHeight() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2598992); return _f(this); }
	inline float evalRideHeight(int index) { typedef float (*_fpt)(RaceEngineer *pthis, int); _fpt _f=(_fpt)_drva(2599376); return _f(this, index); }
	inline float evaluateFuelPerLapFromTrackSpline() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2600032); return _f(this); }
	inline int findTyreCompound(FindTyreCompoundLogic logic) { typedef int (*_fpt)(RaceEngineer *pthis, FindTyreCompoundLogic); _fpt _f=(_fpt)_drva(2601888); return _f(this, logic); }
	inline WheelValues evalTyreLoad(float speedMS, float latg, float longg) { typedef WheelValues (*_fpt)(RaceEngineer *pthis, float, float, float); _fpt _f=(_fpt)_drva(2599392); return _f(this, speedMS, latg, longg); }
	inline KPI getKPI_RAD(int index) { typedef KPI (*_fpt)(RaceEngineer *pthis, int); _fpt _f=(_fpt)_drva(2605152); return _f(this, index); }
	inline float getAntiSquat() { typedef float (*_fpt)(RaceEngineer *pthis); _fpt _f=(_fpt)_drva(2602768); return _f(this); }
	inline float evalLateralGFromLoads(float speed, float aeroRatio, float mass, float * loads) { typedef float (*_fpt)(RaceEngineer *pthis, float, float, float, float *); _fpt _f=(_fpt)_drva(2597472); return _f(this, speed, aeroRatio, mass, loads); }
};

class ksgui_ScrollBar : public ksgui_Control {
public:
	Event<ksgui_OnScrollBarValueChanged> evOnValueChanged;
	ksgui_Control * butPlus;
	ksgui_Control * butMinus;
	bool dressed;
	unsigned int minValue;
	unsigned int maxValue;
	unsigned int step;
	unsigned int value;
	ksgui_MovingBar * bar;
	inline ksgui_ScrollBar()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui, bool isDressed) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *, bool); _fpt _f=(_fpt)_drva(2469472); _f(this, iname, igui, isDressed); }
	virtual ~ksgui_ScrollBar();
	inline void setItemsPerPage(unsigned int v) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, unsigned int); _fpt _f=(_fpt)_drva(2474160); return _f(this, v); }
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, float, float); _fpt _f=(_fpt)_drva(2474224); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	inline unsigned int getItemsPerPage() { typedef unsigned int (*_fpt)(ksgui_ScrollBar *pthis); _fpt _f=(_fpt)_drva(2471920); return _f(this); }
	inline void setItemsNumber(unsigned int v) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, unsigned int); _fpt _f=(_fpt)_drva(2474144); return _f(this, v); }
	inline void setValue(unsigned int v) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, unsigned int); _fpt _f=(_fpt)_drva(2474320); return _f(this, v); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, float); _fpt _f=(_fpt)_drva(2473312); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void setRepeatInterval_vf16(float i);
	inline void setRepeatInterval_impl(float i) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, float); _fpt _f=(_fpt)_drva(2474192); return _f(this, i); }
	inline void setRepeatInterval(float i) { return setRepeatInterval_vf16(i); }
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_ScrollBar *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2472096); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void onMouseUp_vf11(OnMouseUpEvent & message);
	inline void onMouseUp_impl(OnMouseUpEvent & message) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, OnMouseUpEvent &); _fpt _f=(_fpt)_drva(2472656); return _f(this, message); }
	inline void onMouseUp(OnMouseUpEvent & message) { return onMouseUp_vf11(message); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2472496); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	virtual void onMouseWheelMovedEvent_vf13(OnMouseWheelMovedEvent & message);
	inline void onMouseWheelMovedEvent_impl(OnMouseWheelMovedEvent & message) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, OnMouseWheelMovedEvent &); _fpt _f=(_fpt)_drva(2472672); return _f(this, message); }
	inline void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message) { return onMouseWheelMovedEvent_vf13(message); }
	inline void drawArrow(ksgui_Control * button, ksgui_eArrowsDirection direction) { typedef void (*_fpt)(ksgui_ScrollBar *pthis, ksgui_Control *, ksgui_eArrowsDirection); _fpt _f=(_fpt)_drva(2471376); return _f(this, button, direction); }
};

class ksgui_Taskbar : public ksgui_Control {
public:
	int targetX;
	bool taskBarFullShowed;
	Event<ksgui_OnControlClicked> evAppOpened;
	Event<ksgui_OnControlClicked> evAppDismiss;
	std::vector<ksgui_Form *,std::allocator<ksgui_Form *> > forms;
	Event<int> evOnDesktopChange;
	vec4f selectedColor;
	vec4f unselectedColor;
	ksgui_Control * topScroller;
	ksgui_Control * bottomScroller;
	std::vector<ksgui_Control *,std::allocator<ksgui_Control *> > desktops;
	float targetOutOfSight;
	float targetFullShow;
	int topAppIndex;
	std::shared_ptr<Font> appFont;
	std::vector<ksgui_TaskBarIcon *,std::allocator<ksgui_TaskBarIcon *> > icons;
	short virtualDesktop;
	inline ksgui_Taskbar()  { }
	inline void ctor(ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_Taskbar *pthis, ksgui_GUI *); _fpt _f=(_fpt)_drva(2460192); _f(this, igui); }
	virtual ~ksgui_Taskbar();
	inline void dtor() { typedef void (*_fpt)(ksgui_Taskbar *pthis); _fpt _f=(_fpt)_drva(2462304); _f(this); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Taskbar *pthis, float); _fpt _f=(_fpt)_drva(2465872); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void addForm(ksgui_Form * form) { typedef void (*_fpt)(ksgui_Taskbar *pthis, ksgui_Form *); _fpt _f=(_fpt)_drva(2464784); return _f(this, form); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_Taskbar *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2465648); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	inline void mouseWheelMoved(OnMouseWheelMovedEvent & message) { typedef void (*_fpt)(ksgui_Taskbar *pthis, OnMouseWheelMovedEvent &); _fpt _f=(_fpt)_drva(2465456); return _f(this, message); }
	inline void selectVirtualDesktop(int id) { typedef void (*_fpt)(ksgui_Taskbar *pthis, int); _fpt _f=(_fpt)_drva(2466480); return _f(this, id); }
	inline short cycleDekstop() { typedef short (*_fpt)(ksgui_Taskbar *pthis); _fpt _f=(_fpt)_drva(2465264); return _f(this); }
	inline void updateAppIconsPosition() { typedef void (*_fpt)(ksgui_Taskbar *pthis); _fpt _f=(_fpt)_drva(2466656); return _f(this); }
	inline ksgui_Control * addDesktop(int id, int x, int y) { typedef ksgui_Control * (*_fpt)(ksgui_Taskbar *pthis, int, int, int); _fpt _f=(_fpt)_drva(2464240); return _f(this, id, x, y); }
};

class ksgui_Spinner : public ksgui_Control {
public:
	bool homogeneousPressingDelay;
	float displayValueMult;
	int minValue;
	int maxValue;
	int step;
	Event<ksgui_OnSpinnerValueChanged> evOnValueChanged;
	ksgui_PopOver * popOver;
	bool displayPopover;
	bool drawUnderline;
	bool displayCentralValue;
	bool displayLabel;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > units;
	float textSize;
	bool isReadOnly;
	float counter;
	float pressionCounter;
	ksgui_Control * butPlus;
	ksgui_Control * butMinus;
	ksgui_Control * selectionSlider;
	ksgui_Label * label;
	Texture valueBack;
	bool dressed;
	ksgui_OnControlClicked lastOnControlClicked;
	int value;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > items;
	bool mirrored;
	float triangleWidth;
	inline ksgui_Spinner()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui, bool isDressed, bool mirrored) { typedef void (*_fpt)(ksgui_Spinner *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, ksgui_GUI *, bool, bool); _fpt _f=(_fpt)_drva(2407104); _f(this, iname, igui, isDressed, mirrored); }
	virtual ~ksgui_Spinner();
	inline void dtor() { typedef void (*_fpt)(ksgui_Spinner *pthis); _fpt _f=(_fpt)_drva(2410368); _f(this); }
	inline void setValue(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & itemName) { typedef void (*_fpt)(ksgui_Spinner *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2415920); return _f(this, itemName); }
	inline void setValue(int v) { typedef void (*_fpt)(ksgui_Spinner *pthis, int); _fpt _f=(_fpt)_drva(2416224); return _f(this, v); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Spinner *pthis, float); _fpt _f=(_fpt)_drva(2411984); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline int getValue() { typedef int (*_fpt)(ksgui_Spinner *pthis); _fpt _f=(_fpt)_drva(2411712); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getItemAt(unsigned int index) { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(ksgui_Spinner *pthis, unsigned int); _fpt _f=(_fpt)_drva(3316640); return _f(this, index); }
	virtual void setRepeatInterval_vf16(float i);
	inline void setRepeatInterval_impl(float i) { typedef void (*_fpt)(ksgui_Spinner *pthis, float); _fpt _f=(_fpt)_drva(2415552); return _f(this, i); }
	inline void setRepeatInterval(float i) { return setRepeatInterval_vf16(i); }
	inline void addItem(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * item) { typedef void (*_fpt)(ksgui_Spinner *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2411392); return _f(this, item); }
	inline void setLabelText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * text) { typedef void (*_fpt)(ksgui_Spinner *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2415296); return _f(this, text); }
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_Spinner *pthis, float, float); _fpt _f=(_fpt)_drva(2415584); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	inline void setPosition(float x, float y) { typedef void (*_fpt)(ksgui_Spinner *pthis, float, float); _fpt _f=(_fpt)_drva(2415408); return _f(this, x, y); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getCurrentItem() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(ksgui_Spinner *pthis); _fpt _f=(_fpt)_drva(2411552); return _f(this); }
	virtual bool onMouseDown_vf10(OnMouseDownEvent & message);
	inline bool onMouseDown_impl(OnMouseDownEvent & message) { typedef bool (*_fpt)(ksgui_Spinner *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2411728); return _f(this, message); }
	inline bool onMouseDown(OnMouseDownEvent & message) { return onMouseDown_vf10(message); }
	virtual void onMouseMove_vf12(OnMouseMoveEvent & message);
	inline void onMouseMove_impl(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_Spinner *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2411792); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { return onMouseMove_vf12(message); }
	inline void setDrawArrows(bool draw) { typedef void (*_fpt)(ksgui_Spinner *pthis, bool); _fpt _f=(_fpt)_drva(2414640); return _f(this, draw); }
	virtual void scaleByMult_vf19(float value);
	inline void scaleByMult_impl(float value) { typedef void (*_fpt)(ksgui_Spinner *pthis, float); _fpt _f=(_fpt)_drva(2414592); return _f(this, value); }
	inline void scaleByMult(float value) { return scaleByMult_vf19(value); }
	inline void resize(float value) { typedef void (*_fpt)(ksgui_Spinner *pthis, float); _fpt _f=(_fpt)_drva(2414256); return _f(this, value); }
};

class GLRenderer {
public:
	std::vector<std::pair<int,void *>,std::allocator<std::pair<int,void *> > > buffers;
	int currentIndex;
	GraphicsManager * graphics;
	eGLPrimitiveType primitive;
	vec4f color;
	bool useTexture;
	vec2f texCoord;
	Shader * shader;
	IndexBuffer * fullQuadIB;
	VertexBuffer<MeshVertex> * fullQuadVB;
	Shader * glShader;
	Shader * glShaderTex;
	unsigned int tempCounter;
	MeshVertex tempVertices[0x3];
	MeshVertex * tempBuffer;
	unsigned int maxVertices;
	inline GLRenderer()  { }
	virtual ~GLRenderer();
	inline void dtor() { typedef void (*_fpt)(GLRenderer *pthis); _fpt _f=(_fpt)_drva(2089920); _f(this); }
	inline void begin(eGLPrimitiveType type, Shader * ishader) { typedef void (*_fpt)(GLRenderer *pthis, eGLPrimitiveType, Shader *); _fpt _f=(_fpt)_drva(2091008); return _f(this, type, ishader); }
	inline void vertex3f(vec3f & v) { typedef void (*_fpt)(GLRenderer *pthis, vec3f &); _fpt _f=(_fpt)_drva(2096128); return _f(this, v); }
	inline void vertex3f(float x, float y, float z) { typedef void (*_fpt)(GLRenderer *pthis, float, float, float); _fpt _f=(_fpt)_drva(2096160); return _f(this, x, y, z); }
	inline void vertex3fv(float * v) { typedef void (*_fpt)(GLRenderer *pthis, float *); _fpt _f=(_fpt)_drva(2096128); return _f(this, v); }
	inline void end() { typedef void (*_fpt)(GLRenderer *pthis); _fpt _f=(_fpt)_drva(2091120); return _f(this); }
	inline void color3f(float r, float g, float b) { typedef void (*_fpt)(GLRenderer *pthis, float, float, float); _fpt _f=(_fpt)_drva(2091040); return _f(this, r, g, b); }
	inline void color4f(vec4f & v) { typedef void (*_fpt)(GLRenderer *pthis, vec4f &); _fpt _f=(_fpt)_drva(2091072); return _f(this, v); }
	inline void color4f(float r, float g, float b, float a) { typedef void (*_fpt)(GLRenderer *pthis, float, float, float, float); _fpt _f=(_fpt)_drva(2091088); return _f(this, r, g, b, a); }
	inline void texCoord2f(float u, float v) { typedef void (*_fpt)(GLRenderer *pthis, float, float); _fpt _f=(_fpt)_drva(2096112); return _f(this, u, v); }
	inline void fullScreenQuad(Shader * ishader) { typedef void (*_fpt)(GLRenderer *pthis, Shader *); _fpt _f=(_fpt)_drva(2091680); return _f(this, ishader); }
	inline void quad(float x, float y, float width, float height, bool textured, Shader * shader) { typedef void (*_fpt)(GLRenderer *pthis, float, float, float, float, bool, Shader *); _fpt _f=(_fpt)_drva(2094112); return _f(this, x, y, width, height, textured, shader); }
	inline void quadCentred(float x, float y, float width, float height, bool textured, Shader * shader) { typedef void (*_fpt)(GLRenderer *pthis, float, float, float, float, bool, Shader *); _fpt _f=(_fpt)_drva(2094400); return _f(this, x, y, width, height, textured, shader); }
	inline void quadCentredRotated(float x, float y, float width, float height, float rotationRAD, bool textured, Shader * shader) { typedef void (*_fpt)(GLRenderer *pthis, float, float, float, float, float, bool, Shader *); _fpt _f=(_fpt)_drva(2094768); return _f(this, x, y, width, height, rotationRAD, textured, shader); }
	inline void spline(Spline & spline) { typedef void (*_fpt)(GLRenderer *pthis, Spline &); _fpt _f=(_fpt)_drva(2095424); return _f(this, spline); }
	inline void ctor(GraphicsManager * r, unsigned int maxVertices, bool withFullScreenQuad) { typedef void (*_fpt)(GLRenderer *pthis, GraphicsManager *, unsigned int, bool); _fpt _f=(_fpt)_drva(2088928); _f(this, r, maxVertices, withFullScreenQuad); }
	inline void initFullScreenQuad() { typedef void (*_fpt)(GLRenderer *pthis); _fpt _f=(_fpt)_drva(2091824); return _f(this); }
};

class ksgui_ListBox : public ksgui_Control {
public:
	bool allowDeselection;
	bool allowSelectionOnClick;
	bool drawUnderline;
	bool drawScrollbarPlaceholder;
	ksgui_eListBoxSelectionMode selectionMode;
	Event<ksgui_OnListBoxItemClickedEvent> evOnListBoxItemClicked;
	Event<ksgui_OnListBoxItemClickedEvent> evOnListBoxItemDeselected;
	ksgui_ListBoxRow * titleRow;
	ksgui_ListBoxRow * bottomRow;
	std::vector<ksgui_ListBoxRowData,std::allocator<ksgui_ListBoxRowData> > items;
	std::vector<ksgui_ListBoxRow *,std::allocator<ksgui_ListBoxRow *> > rows;
	unsigned int columnNumber;
	std::map<unsigned int,vec4f,std::less<unsigned int>,std::allocator<std::pair<unsigned int const ,vec4f> > > category;
	int selectedItemIndex;
	int pivot;
	ksgui_ScrollBar * scrollBar;
	inline ksgui_ListBox()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, unsigned int rowNumber, unsigned int colNumber, ksgui_GUI * aGui, bool isDressed) { typedef void (*_fpt)(ksgui_ListBox *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, unsigned int, unsigned int, ksgui_GUI *, bool); _fpt _f=(_fpt)_drva(2392896); _f(this, name, rowNumber, colNumber, aGui, isDressed); }
	virtual ~ksgui_ListBox();
	inline void dtor() { typedef void (*_fpt)(ksgui_ListBox *pthis); _fpt _f=(_fpt)_drva(2395296); _f(this); }
	inline void addItem(ksgui_ListBoxRowData & item) { typedef void (*_fpt)(ksgui_ListBox *pthis, ksgui_ListBoxRowData &); _fpt _f=(_fpt)_drva(2397680); return _f(this, item); }
	inline void clear() { typedef void (*_fpt)(ksgui_ListBox *pthis); _fpt _f=(_fpt)_drva(2397936); return _f(this); }
	inline ksgui_ListBoxRowData * getSelectedItem() { typedef ksgui_ListBoxRowData * (*_fpt)(ksgui_ListBox *pthis); _fpt _f=(_fpt)_drva(2398592); return _f(this); }
	inline void scrollToTop() { typedef void (*_fpt)(ksgui_ListBox *pthis); _fpt _f=(_fpt)_drva(2400848); return _f(this); }
	inline void scrollToCenterOnSelected() { typedef void (*_fpt)(ksgui_ListBox *pthis); _fpt _f=(_fpt)_drva(2400704); return _f(this); }
	inline void setItemsFontSize(float size) { typedef void (*_fpt)(ksgui_ListBox *pthis, float); _fpt _f=(_fpt)_drva(2401696); return _f(this, size); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_ListBox *pthis, float); _fpt _f=(_fpt)_drva(2399280); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	virtual void setSize_vf7(float w, float h);
	inline void setSize_impl(float w, float h) { typedef void (*_fpt)(ksgui_ListBox *pthis, float, float); _fpt _f=(_fpt)_drva(2401808); return _f(this, w, h); }
	inline void setSize(float w, float h) { return setSize_vf7(w, h); }
	virtual void onMouseWheelMovedEvent_vf13(OnMouseWheelMovedEvent & message);
	inline void onMouseWheelMovedEvent_impl(OnMouseWheelMovedEvent & message) { typedef void (*_fpt)(ksgui_ListBox *pthis, OnMouseWheelMovedEvent &); _fpt _f=(_fpt)_drva(2398640); return _f(this, message); }
	inline void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message) { return onMouseWheelMovedEvent_vf13(message); }
	inline void selectItem(int itemIndex) { typedef void (*_fpt)(ksgui_ListBox *pthis, int); _fpt _f=(_fpt)_drva(2400912); return _f(this, itemIndex); }
	inline void selectItemByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & itemText) { typedef void (*_fpt)(ksgui_ListBox *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2401216); return _f(this, itemText); }
	inline int getItemIdFromName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aName) { typedef int (*_fpt)(ksgui_ListBox *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2398288); return _f(this, aName); }
	inline void drawCellsBackgrounds(bool value) { typedef void (*_fpt)(ksgui_ListBox *pthis, bool); _fpt _f=(_fpt)_drva(2398128); return _f(this, value); }
	inline void scrollListBoxToItem(int itemId) { typedef void (*_fpt)(ksgui_ListBox *pthis, int); _fpt _f=(_fpt)_drva(2400304); return _f(this, itemId); }
};

struct Tyre {
public:
	TyreInputs inputs;
	TyreData data;
	TyreModelData modelData;
	TyreStatus status;
	ISuspension * hub;
	mat44f worldRotation;
	vec3f unmodifiedContactPoint;
	vec3f contactPoint;
	vec3f contactNormal;
	SurfaceDef * surfaceDef;
	bool debugOutput;
	float absOverride;
	TyreThermalModel thermalModel;
	std::vector<TyreCompoundDef,std::allocator<TyreCompoundDef> > compoundDefs;
	std::function<void __cdecl(void)> onStepCompleted;
	float aiMult;
	BrushSlipProvider slipProvider;
	TyreExternalInputs externalInputs;
	vec3f roadRight;
	vec3f roadHeading;
	bool useLoadForVKM;
	bool driven;
	IRayTrackCollisionProvider * rayCollisionProvider;
	float oldAngularVelocity;
	float totalSlideVelocity;
	mat44f localWheelRotation;
	vec3f worldPosition;
	float slidingVelocityY;
	float slidingVelocityX;
	float roadVelocityX;
	float roadVelocityY;
	float totalHubVelocity;
	float rSlidingVelocityX;
	float rSlidingVelocityY;
	IRayCaster * rayCaster;
	int index;
	SinSignalGenerator shakeGenerator;
	Car * car;
	int currentCompoundIndex;
	bool tyreBlanketsOn;
	float flatSpotK;
	ITyreModel * tyreModel;
	SCTM scTM;
	float explosionTemperature;
	float blanketTemperature;
	float pressureTemperatureGain;
	float localMX;
	inline Tyre()  { }
	inline void dtor() { typedef void (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2609952); _f(this); }
	inline void init(ISuspension * ihub, IRayTrackCollisionProvider * rcp, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * dataPath, int index, int carID, Car * car) { typedef void (*_fpt)(Tyre *pthis, ISuspension *, IRayTrackCollisionProvider *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, int, int, Car *); _fpt _f=(_fpt)_drva(2623056); return _f(this, ihub, rcp, dataPath, index, carID, car); }
	inline void step(float dt) { typedef void (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2635776); return _f(this, dt); }
	inline mat44f getFinalTyreRotation() { typedef mat44f (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2622864); return _f(this); }
	inline void stepRotationMatrix(float dt) { typedef void (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2640768); return _f(this, dt); }
	inline void reset() { typedef void (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2634624); return _f(this); }
	inline vec3f getWorldPosition() { typedef vec3f (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2623024); return _f(this); }
	inline float getDY(float load) { typedef float (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2622416); return _f(this, load); }
	inline float getDX(float load) { typedef float (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2622016); return _f(this, load); }
	inline bool setCompound(int cindex) { typedef bool (*_fpt)(Tyre *pthis, int); _fpt _f=(_fpt)_drva(2634976); return _f(this, cindex); }
	inline float getDynamicK() { typedef float (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2622816); return _f(this); }
	inline float getCorrectedD(float d, float * wear_mult) { typedef float (*_fpt)(Tyre *pthis, float, float *); _fpt _f=(_fpt)_drva(2621840); return _f(this, d, wear_mult); }
	inline float getCamberedDy(float camberRAD, float dy) { typedef float (*_fpt)(Tyre *pthis, float, float); _fpt _f=(_fpt)_drva(2621568); return _f(this, camberRAD, dy); }
	inline void addGroundContact(vec3f & pos, vec3f & normal) { typedef void (*_fpt)(Tyre *pthis, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(2611584); return _f(this, pos, normal); }
	inline void addTyreForces(vec3f & pos, vec3f & normal, SurfaceDef * surfaceDef, float dt) { typedef void (*_fpt)(Tyre *pthis, vec3f &, vec3f &, SurfaceDef *, float); _fpt _f=(_fpt)_drva(2613664); return _f(this, pos, normal, surfaceDef, dt); }
	inline void addTyreForcesV10(vec3f & pos, vec3f & normal, SurfaceDef * surfaceDef, float dt) { typedef void (*_fpt)(Tyre *pthis, vec3f &, vec3f &, SurfaceDef *, float); _fpt _f=(_fpt)_drva(2616672); return _f(this, pos, normal, surfaceDef, dt); }
	inline void updateAngularSpeed(float dt) { typedef void (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2641824); return _f(this, dt); }
	inline void updateLockedState(float dt) { typedef void (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2642032); return _f(this, dt); }
	inline void stepRelaxationLength(float svx, float svy, float hubVelocity, float dt) { typedef void (*_fpt)(Tyre *pthis, float, float, float, float); _fpt _f=(_fpt)_drva(2640448); return _f(this, svx, svy, hubVelocity, dt); }
	inline void stepDirtyLevel(float dt, float hubSpeed) { typedef void (*_fpt)(Tyre *pthis, float, float); _fpt _f=(_fpt)_drva(2638800); return _f(this, dt, hubSpeed); }
	inline void stepThermalModel(float dt) { typedef void (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2641056); return _f(this, dt); }
	inline void initCompounds(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataPath, int index) { typedef void (*_fpt)(Tyre *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, int); _fpt _f=(_fpt)_drva(2623488); return _f(this, dataPath, index); }
	inline void stepGrainBlister(float dt, float hubVelocity) { typedef void (*_fpt)(Tyre *pthis, float, float); _fpt _f=(_fpt)_drva(2639360); return _f(this, dt, hubVelocity); }
	inline void stepPuncture(float dt, float hubSpeed) { typedef void (*_fpt)(Tyre *pthis, float, float); _fpt _f=(_fpt)_drva(2640304); return _f(this, dt, hubSpeed); }
	inline void stepTyreBlankets(float dt) { typedef void (*_fpt)(Tyre *pthis, float); _fpt _f=(_fpt)_drva(2641680); return _f(this, dt); }
	inline void stepFlatSpot(float dt, float hubVelocity) { typedef void (*_fpt)(Tyre *pthis, float, float); _fpt _f=(_fpt)_drva(2639104); return _f(this, dt, hubVelocity); }
	inline bool generateCompoundNames() { typedef bool (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2619296); return _f(this); }
	inline void addTyreForceToHub(vec3f & pos, vec3f & force) { typedef void (*_fpt)(Tyre *pthis, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(2612224); return _f(this, pos, force); }
	inline void ctor() { typedef void (*_fpt)(Tyre *pthis); _fpt _f=(_fpt)_drva(2546640); _f(this); }
};

class RaceManager : public GameObject {
public:
	Event<OnRaceInitEvent> evOnRaceInit;
	Event<OnSessionEndEvent> evOnSessionEnd;
	Event<OnLapCompletedEvent> evOnLapCompleted;
	bool penaltiesEnabled;
	bool officialSpecialEvent;
	bool fixedSetup;
	bool pauseMode;
	int currentSessionIndex;
	float overTimeMult;
	bool timeRaceEnded;
	bool isTimedLastLap;
	bool hasAdditionalLap;
	float pitWindowStartTime;
	float pitWindowEndTime;
	WindSettings windSettings;
	MultiplayerStatus multiplayerStatus;
	bool leaderLastLap;
	Session currentSession;
	Trigger debugTrigger;
	Sim * sim;
	RaceTimingServices * raceTimingServices;
	std::shared_ptr<Font> font;
	std::vector<Session,std::allocator<Session> > sessions;
	std::vector<RealTimeCarDesc,std::allocator<RealTimeCarDesc> > carsRealTimePosition;
	std::vector<RaceStatusCarDesc,std::allocator<RaceStatusCarDesc> > carsRaceStatus;
	std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > mpCacheLeaderboard;
	std::vector<std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> >,std::allocator<std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > > > leaderboardHistory;
	std::vector<OnSessionEndEvent,std::allocator<OnSessionEndEvent> > sessionHistoryList;
	inline RaceManager()  { }
	inline void ctor(Sim * isim) { typedef void (*_fpt)(RaceManager *pthis, Sim *); _fpt _f=(_fpt)_drva(1256880); _f(this, isim); }
	virtual ~RaceManager();
	inline void dtor() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1258224); _f(this); }
	inline bool isOfficialSpecialEvent() { typedef bool (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1680464); return _f(this); }
	virtual void update_vf1(float deltaT);
	inline void update_impl(float deltaT) { typedef void (*_fpt)(RaceManager *pthis, float); _fpt _f=(_fpt)_drva(1316176); return _f(this, deltaT); }
	inline void update(float deltaT) { return update_vf1(deltaT); }
	inline void startRace(float millisecondsToStart) { typedef void (*_fpt)(RaceManager *pthis, float); _fpt _f=(_fpt)_drva(1315952); return _f(this, millisecondsToStart); }
	inline bool isRaceOver(unsigned int carIndex) { typedef bool (*_fpt)(RaceManager *pthis, unsigned int); _fpt _f=(_fpt)_drva(1305312); return _f(this, carIndex); }
	virtual void renderHUD_vf3(float dt);
	inline void renderHUD_impl(float dt) { typedef void (*_fpt)(RaceManager *pthis, float); _fpt _f=(_fpt)_drva(1309024); return _f(this, dt); }
	inline void renderHUD(float dt) { return renderHUD_vf3(dt); }
	virtual void shutdown_vf5();
	inline void shutdown_impl() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1315616); return _f(this); }
	inline void shutdown() { return shutdown_vf5(); }
	inline bool skipCurrentSession() { typedef bool (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1315664); return _f(this); }
	inline int getCurrentSessionIndex() { typedef int (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1278672); return _f(this); }
	inline SessionType getCurrentSessionType() { typedef SessionType (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1278720); return _f(this); }
	inline Session getCurrentSession() { typedef Session (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1278496); return _f(this); }
	inline Session getSessionInfo(int sessionIndex) { typedef Session (*_fpt)(RaceManager *pthis, int); _fpt _f=(_fpt)_drva(1285488); return _f(this, sessionIndex); }
	inline void restartCurrentSession() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1314048); return _f(this); }
	inline bool loadSessions() { typedef bool (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1305776); return _f(this); }
	inline bool hasSessionStarted() { typedef bool (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1286512); return _f(this); }
	inline double getTimeToSessionStart() { typedef double (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1286304); return _f(this); }
	inline unsigned int getSessionCount() { typedef unsigned int (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1285280); return _f(this); }
	inline double getSessionTimeLeft() { typedef double (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1285728); return _f(this); }
	inline bool hasExtraLap() { typedef bool (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1286480); return _f(this); }
	inline unsigned short getPitWindowStart() { typedef unsigned short (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1281376); return _f(this); }
	inline unsigned short getPitWindowEnd() { typedef unsigned short (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1281344); return _f(this); }
	inline std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > getLeaderboard() { typedef std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1280688); return _f(this); }
	inline std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > getLeaderboardFromSession(int sessionID) { typedef std::vector<LeaderboardEntry,std::allocator<LeaderboardEntry> > (*_fpt)(RaceManager *pthis, int); _fpt _f=(_fpt)_drva(1280912); return _f(this, sessionID); }
	inline int getCarLeaderboardPosition(CarAvatar * car) { typedef int (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1277664); return _f(this, car); }
	inline int getCarRealTimePosition(CarAvatar * car) { typedef int (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1277840); return _f(this, car); }
	inline int getSessionTotalTime(unsigned int carIndex) { typedef int (*_fpt)(RaceManager *pthis, unsigned int); _fpt _f=(_fpt)_drva(1285856); return _f(this, carIndex); }
	inline Lap getLastLap(CarAvatar * car) { typedef Lap (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1279936); return _f(this, car); }
	inline unsigned int getBestLap() { typedef unsigned int (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1277568); return _f(this); }
	inline Lap getBestLap(CarAvatar * car) { typedef Lap (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1276928); return _f(this, car); }
	inline unsigned int getLapCount(CarAvatar * car) { typedef unsigned int (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1279344); return _f(this, car); }
	inline unsigned int getSplit(CarAvatar * car, int & sector) { typedef unsigned int (*_fpt)(RaceManager *pthis, CarAvatar *, int &); _fpt _f=(_fpt)_drva(1286256); return _f(this, car, sector); }
	inline int getSplit(CarAvatar * car) { typedef int (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1286160); return _f(this, car); }
	inline Lap getCurrentLap(CarAvatar * car) { typedef Lap (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1278128); return _f(this, car); }
	inline std::vector<Lap,std::allocator<Lap> > getLaps(CarAvatar * car, bool readReplayModeIfActive) { typedef std::vector<Lap,std::allocator<Lap> > (*_fpt)(RaceManager *pthis, CarAvatar *, bool); _fpt _f=(_fpt)_drva(1279664); return _f(this, car, readReplayModeIfActive); }
	inline int getInstanceBestLap(CarAvatar * car) { typedef int (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1278976); return _f(this, car); }
	inline unsigned int getLastSplit(CarAvatar * car, int & sector) { typedef unsigned int (*_fpt)(RaceManager *pthis, CarAvatar *, int &); _fpt _f=(_fpt)_drva(1280464); return _f(this, car, sector); }
	inline unsigned int getBestSplit(int & sector, bool & isGlobal, CarAvatar * car) { typedef unsigned int (*_fpt)(RaceManager *pthis, int &, bool &, CarAvatar *); _fpt _f=(_fpt)_drva(1277616); return _f(this, sector, isGlobal, car); }
	inline bool isBestSplit(int & sector, int & t, bool & isGlobal, CarAvatar * car) { typedef bool (*_fpt)(RaceManager *pthis, int &, int &, bool &, CarAvatar *); _fpt _f=(_fpt)_drva(1305248); return _f(this, sector, t, isGlobal, car); }
	inline unsigned int getLeaderLapCount() { typedef unsigned int (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1280512); return _f(this); }
	inline OnSessionEndEvent getSessionHistory(int & sessionID) { typedef OnSessionEndEvent (*_fpt)(RaceManager *pthis, int &); _fpt _f=(_fpt)_drva(1285360); return _f(this, sessionID); }
	inline double getTimetoWait() { typedef double (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1286448); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getPracticeQualifyText() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1281408); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getRaceText() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1282384); return _f(this); }
	inline bool getHasCompletedFlag(CarAvatar * car) { typedef bool (*_fpt)(RaceManager *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1278864); return _f(this, car); }
	inline int getInvertedGridPositions() { typedef int (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1279312); return _f(this); }
	inline void checkMandatoryPit(int carIndex, float entry, float exit) { typedef void (*_fpt)(RaceManager *pthis, int, float, float); _fpt _f=(_fpt)_drva(1271856); return _f(this, carIndex, entry, exit); }
	inline std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > getModelSkins(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & model) { typedef std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > (*_fpt)(RaceManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1281056); return _f(this, model); }
	inline void initLighting(INIReaderDocuments & ini) { typedef void (*_fpt)(RaceManager *pthis, INIReaderDocuments &); _fpt _f=(_fpt)_drva(1287120); return _f(this, ini); }
	inline void initGhostcar(INIReaderDocuments & ini) { typedef void (*_fpt)(RaceManager *pthis, INIReaderDocuments &); _fpt _f=(_fpt)_drva(1286672); return _f(this, ini); }
	inline bool initOnline(INIReaderDocuments & ini) { typedef bool (*_fpt)(RaceManager *pthis, INIReaderDocuments &); _fpt _f=(_fpt)_drva(1298784); return _f(this, ini); }
	inline bool initOffline(INIReaderDocuments & anIniReader) { typedef bool (*_fpt)(RaceManager *pthis, INIReaderDocuments &); _fpt _f=(_fpt)_drva(1287872); return _f(this, anIniReader); }
	inline bool initReplay(INIReaderDocuments & r) { typedef bool (*_fpt)(RaceManager *pthis, INIReaderDocuments &); _fpt _f=(_fpt)_drva(1304608); return _f(this, r); }
	inline void updateCarsRealTimePositions(float dt) { typedef void (*_fpt)(RaceManager *pthis, float); _fpt _f=(_fpt)_drva(1321136); return _f(this, dt); }
	inline void finalizeCurrentSession() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1272624); return _f(this); }
	inline void updateCarsEndRace(float dt) { typedef void (*_fpt)(RaceManager *pthis, float); _fpt _f=(_fpt)_drva(1320528); return _f(this, dt); }
	inline void setCurrentSession(unsigned int index) { typedef void (*_fpt)(RaceManager *pthis, unsigned int); _fpt _f=(_fpt)_drva(1314288); return _f(this, index); }
	inline void onLapCompleted(OnLapCompletedEvent & message) { typedef void (*_fpt)(RaceManager *pthis, OnLapCompletedEvent &); _fpt _f=(_fpt)_drva(1307520); return _f(this, message); }
	inline void resetInvalidStateCarsToPits(float dt) { typedef void (*_fpt)(RaceManager *pthis, float); _fpt _f=(_fpt)_drva(1311552); return _f(this, dt); }
	inline void resetCurrentLaps() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1311488); return _f(this); }
	inline void resetMandatoryPit() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1312256); return _f(this); }
	inline Session convertRemoteSession(RemoteSession & rs) { typedef Session (*_fpt)(RaceManager *pthis, RemoteSession &); _fpt _f=(_fpt)_drva(1272400); return _f(this, rs); }
	inline bool isSkinAvailable(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & model, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin) { typedef bool (*_fpt)(RaceManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1305536); return _f(this, model, skin); }
	inline void setHasCompletedFlag(CarAvatar * car, bool value) { typedef void (*_fpt)(RaceManager *pthis, CarAvatar *, bool); _fpt _f=(_fpt)_drva(1315440); return _f(this, car, value); }
	inline void generateWind() { typedef void (*_fpt)(RaceManager *pthis); _fpt _f=(_fpt)_drva(1276832); return _f(this); }
};

class ksgui_GUI : public IKeyEventListener {
public:
	bool isActive;
	GraphicsManager * graphics;
	std::shared_ptr<Font> font;
	INIReader skinReader;
	int version;
	ksgui_Taskbar * taskbar;
	std::vector<ksgui_Control *,std::allocator<ksgui_Control *> > controls;
	bool isPointerVisible;
	bool allowFormSerialization;
	AudioEngine * audioEngine;
	ksgui_Form * capturedForm;
	vec2f capturedPos;
	vec2f oldGoodPos;
	bool moveApp;
	bool renderDrawingArea;
	bool allowOverlappingForms;
	ksgui_EventReplicator<OnMouseDownEvent> onMouseDownReplicator;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > applicationName;
	RenderTarget * renderTarget;
	Camera * camera;
	Node * referenceFrameForOrientation;
	Texture txPointer;
	vec2f mousePos;
	float mouseSpeed;
	vec2f oldMousePos;
	vec3f positionOffset3D;
	int virtualDesktop;
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,ksgui_GUI_FormData,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,ksgui_GUI_FormData> > > formDataMap;
	inline ksgui_GUI()  { }
	inline void ctor(GraphicsManager * graphics, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aapplicationName, KeyboardManager * keyManager) { typedef void (*_fpt)(ksgui_GUI *pthis, GraphicsManager *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, KeyboardManager *); _fpt _f=(_fpt)_drva(2431680); _f(this, graphics, aapplicationName, keyManager); }
	virtual ~ksgui_GUI();
	inline void dtor() { typedef void (*_fpt)(ksgui_GUI *pthis); _fpt _f=(_fpt)_drva(2433760); _f(this); }
	inline void preRender3D(float dt) { typedef void (*_fpt)(ksgui_GUI *pthis, float); _fpt _f=(_fpt)_drva(2443200); return _f(this, dt); }
	inline void render3D(float dt) { typedef void (*_fpt)(ksgui_GUI *pthis, float); _fpt _f=(_fpt)_drva(2443520); return _f(this, dt); }
	inline void render(float dt) { typedef void (*_fpt)(ksgui_GUI *pthis, float); _fpt _f=(_fpt)_drva(2444304); return _f(this, dt); }
	inline void addControl(ksgui_Control * c) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_Control *); _fpt _f=(_fpt)_drva(2436688); return _f(this, c); }
	inline vec4f getSkinColor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * key, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * value) { typedef vec4f (*_fpt)(ksgui_GUI *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2440800); return _f(this, key, value); }
	inline void setCapturedForm(ksgui_Form * form, int wx, int wy) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_Form *, int, int); _fpt _f=(_fpt)_drva(2447040); return _f(this, form, wx, wy); }
	inline void serializeForms(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename) { typedef void (*_fpt)(ksgui_GUI *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2444640); return _f(this, filename); }
	inline void deserializeForm(ksgui_Form * form) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_Form *); _fpt _f=(_fpt)_drva(2437008); return _f(this, form); }
	inline void attachToRenderWindowEvents(RenderWindow & window) { typedef void (*_fpt)(ksgui_GUI *pthis, RenderWindow &); _fpt _f=(_fpt)_drva(2436784); return _f(this, window); }
	inline void setAudioEngine(AudioEngine * engine) { typedef void (*_fpt)(ksgui_GUI *pthis, AudioEngine *); _fpt _f=(_fpt)_drva(2447024); return _f(this, engine); }
	inline void playClick() { typedef void (*_fpt)(ksgui_GUI *pthis); _fpt _f=(_fpt)_drva(96368); return _f(this); }
	inline void init3DMode(unsigned int width, unsigned int height, Node * referenceFrame, Camera * camera, vec3f posOffset) { typedef void (*_fpt)(ksgui_GUI *pthis, unsigned int, unsigned int, Node *, Camera *, vec3f); _fpt _f=(_fpt)_drva(2440976); return _f(this, width, height, referenceFrame, camera, posOffset); }
	inline void onMouseUp(OnMouseUpEvent & message) { typedef void (*_fpt)(ksgui_GUI *pthis, OnMouseUpEvent &); _fpt _f=(_fpt)_drva(2442688); return _f(this, message); }
	inline void onMouseMove(OnMouseMoveEvent & message) { typedef void (*_fpt)(ksgui_GUI *pthis, OnMouseMoveEvent &); _fpt _f=(_fpt)_drva(2442016); return _f(this, message); }
	inline void onMouseDown(OnMouseDownEvent & message) { typedef void (*_fpt)(ksgui_GUI *pthis, OnMouseDownEvent &); _fpt _f=(_fpt)_drva(2441872); return _f(this, message); }
	inline void onMouseWheelMoved(OnMouseWheelMovedEvent & message) { typedef void (*_fpt)(ksgui_GUI *pthis, OnMouseWheelMovedEvent &); _fpt _f=(_fpt)_drva(2443040); return _f(this, message); }
	virtual void onKeyChar_vf1(unsigned int key);
	inline void onKeyChar_impl(unsigned int key) { typedef void (*_fpt)(ksgui_GUI *pthis, unsigned int); _fpt _f=(_fpt)_drva(2441696); return _f(this, key); }
	inline void onKeyChar(unsigned int key) { return onKeyChar_vf1(key); }
	virtual void onKeyDown_vf0(OnKeyEvent & message);
	inline void onKeyDown_impl(OnKeyEvent & message) { typedef void (*_fpt)(ksgui_GUI *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(2441776); return _f(this, message); }
	inline void onKeyDown(OnKeyEvent & message) { return onKeyDown_vf0(message); }
	inline void renderVPointer() { typedef void (*_fpt)(ksgui_GUI *pthis); _fpt _f=(_fpt)_drva(2444336); return _f(this); }
	inline void doRender2D(float dt) { typedef void (*_fpt)(ksgui_GUI *pthis, float); _fpt _f=(_fpt)_drva(2439632); return _f(this, dt); }
	inline void deserializeOldForm(ksgui_Form * form) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_Form *); _fpt _f=(_fpt)_drva(2439312); return _f(this, form); }
	inline void deserializeFormDesktop(ksgui_Form * form, int idDesktop) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_Form *, int); _fpt _f=(_fpt)_drva(2438048); return _f(this, form, idDesktop); }
	inline void serializeMemoryForm(ksgui_Form * form) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_Form *); _fpt _f=(_fpt)_drva(2446304); return _f(this, form); }
	inline void fromIni(ksgui_GUI_FormData & data, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section) { typedef void (*_fpt)(ksgui_GUI *pthis, ksgui_GUI_FormData &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2439968); return _f(this, data, section); }
};

class PhysicsEngine : public ICollisionCallback {
public:
	std::vector<Car *,std::allocator<Car *> > cars;
	double physicsTime;
	bool validated;
	Concurrency::concurrent_queue<ACPhysicsEvent,std::allocator<ACPhysicsEvent> > eventQueue;
	Event<OnFlagEvent> evOnFlagEvent;
	Event<double> evOnStepCompleted;
	Event<double> evOnPreStep;
	Event<SessionInfo> evOnNewSessionPhysics;
	bool allowTyreBlankets;
	double lockGearboxAtStartTimeMS;
	float fuelConsumptionRate;
	float tyreConsumptionRate;
	int allowedTyresOut;
	PenaltyMode penaltyMode;
	PenaltyRules penaltyRules;
	std::vector<SlipStream *,std::allocator<SlipStream *> > slipStreams;
	double gameTime;
	float ambientTemperature;
	float roadTemperature;
	float mechanicalDamageRate;
	PhysicsCPUTimes physicsCPUTimes;
	float flatSpotFFGain;
	SteerMzLowSpeedReduction mzLowSpeedReduction;
	bool isEngineStallEnabled;
	float gyroWheelGain;
	float spinTorqueGain;
	float damperMinValue;
	float damperGain;
	Wind wind;
	SessionInfo sessionInfo;
	IPhysicsCore * core;
	Track * track;
	IDebugVisualizer * debugVisualizer;
	unsigned int stepCounter;
	std::vector<ICarPhysicsStateProvider *,std::allocator<ICarPhysicsStateProvider *> > additionalPhysicsProviders;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > legalTyreList;
	std::unique_ptr<ThreadPool,std::default_delete<ThreadPool> > pool;
	DynamicTempData dynamicTemp;
	inline PhysicsEngine()  { }
	inline void ctor() { typedef void (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2499632); _f(this); }
	virtual ~PhysicsEngine();
	inline void dtor() { typedef void (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2501872); _f(this); }
	inline void setWind(Speed speed, float directionDEG) { typedef void (*_fpt)(PhysicsEngine *pthis, Speed, float); _fpt _f=(_fpt)_drva(2508192); return _f(this, speed, directionDEG); }
	inline void setCurrentSessionStartTime(double st) { typedef void (*_fpt)(PhysicsEngine *pthis, double); _fpt _f=(_fpt)_drva(2508176); return _f(this, st); }
	inline void removeCar(Car * c) { typedef void (*_fpt)(PhysicsEngine *pthis, Car *); _fpt _f=(_fpt)_drva(2507328); return _f(this, c); }
	inline void step(float dt, double currentTime, double gt) { typedef void (*_fpt)(PhysicsEngine *pthis, float, double, double); _fpt _f=(_fpt)_drva(2508640); return _f(this, dt, currentTime, gt); }
	inline void stepPaused() { typedef void (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2511648); return _f(this); }
	inline IPhysicsCore * getCore() { typedef IPhysicsCore * (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2505344); return _f(this); }
	virtual void onCollisionCallBack_vf1(void * userData0, void * shape0, void * userData1, void * shape1, vec3f normal, vec3f pos, float depth);
	inline void onCollisionCallBack_impl(void * userData0, void * shape0, void * userData1, void * shape1, vec3f normal, vec3f pos, float depth) { typedef void (*_fpt)(PhysicsEngine *pthis, void *, void *, void *, void *, vec3f, vec3f, float); _fpt _f=(_fpt)_drva(2506784); return _f(this, userData0, shape0, userData1, shape1, normal, pos, depth); }
	inline void onCollisionCallBack(void * userData0, void * shape0, void * userData1, void * shape1, vec3f normal, vec3f pos, float depth) { return onCollisionCallBack_vf1(userData0, shape0, userData1, shape1, normal, pos, depth); }
	inline bool hasSessionStarted(double lag) { typedef bool (*_fpt)(PhysicsEngine *pthis, double); _fpt _f=(_fpt)_drva(2505840); return _f(this, lag); }
	inline double getOddTimeOffset() { typedef double (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2505360); return _f(this); }
	inline void addAdditionalPhysicsProvider(ICarPhysicsStateProvider * p) { typedef void (*_fpt)(PhysicsEngine *pthis, ICarPhysicsStateProvider *); _fpt _f=(_fpt)_drva(2505216); return _f(this, p); }
	inline void setSessionInfo(SessionInfo & info) { typedef void (*_fpt)(PhysicsEngine *pthis, SessionInfo &); _fpt _f=(_fpt)_drva(2508128); return _f(this, info); }
	inline void setSessionStartTimeMS(double new_start_time) { typedef void (*_fpt)(PhysicsEngine *pthis, double); _fpt _f=(_fpt)_drva(2508176); return _f(this, new_start_time); }
	inline void addLegalTyre(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & short_name) { typedef void (*_fpt)(PhysicsEngine *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2505248); return _f(this, short_name); }
	inline bool isTyreLegal(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & short_name) { typedef bool (*_fpt)(PhysicsEngine *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2506544); return _f(this, short_name); }
	inline float getAirDensity() { typedef float (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2505312); return _f(this); }
	inline void writeTestResult(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & result) { typedef void (*_fpt)(PhysicsEngine *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2512176); return _f(this, result); }
	inline void getPhysicsStates(std::vector<CarPhysicsState,std::allocator<CarPhysicsState> > & states, std::vector<std::vector<WingState,std::allocator<WingState> >,std::allocator<std::vector<WingState,std::allocator<WingState> > > > & wingStates) { typedef void (*_fpt)(PhysicsEngine *pthis, std::vector<CarPhysicsState,std::allocator<CarPhysicsState> > &, std::vector<std::vector<WingState,std::allocator<WingState> >,std::allocator<std::vector<WingState,std::allocator<WingState> > > > &); _fpt _f=(_fpt)_drva(2505408); return _f(this, states, wingStates); }
	inline void setDynamicTempData(DynamicTempData & data) { typedef void (*_fpt)(PhysicsEngine *pthis, DynamicTempData &); _fpt _f=(_fpt)_drva(1315344); return _f(this, data); }
	inline void initLowSpeedFF() { typedef void (*_fpt)(PhysicsEngine *pthis); _fpt _f=(_fpt)_drva(2505872); return _f(this); }
	inline void stepWind(float dt) { typedef void (*_fpt)(PhysicsEngine *pthis, float); _fpt _f=(_fpt)_drva(2511744); return _f(this, dt); }
};

class Sim : public GameObject, public IKeyEventListener {
public:
	Event<OnNewCarLoadedEvent> evNewCarLoaded;
	Event<OnReplayStatusChanged> evOnReplayStatusChanged;
	Event<bool> evOnPauseModeChanged;
	Event<CollisionEvent> evOnCollisionEvent;
	Event<OnNewSessionEvent> evOnNewSession;
	HDRLevels hdrLevels;
	CommandManager commandManager;
	OptionsManager optionsManager;
	ksgui_GameScreen * gameScreen;
	PauseMenu * pauseMenu;
	ESCMenu * escMenu;
	Node * rootNode;
	Node * trackNode;
	CarNodeSorter * carsNode;
	Node * skidMarkNode;
	Node * particlesNode;
	NodeEvent * carFakeShadowsNode;
	TrackAvatar * track;
	Node * blurredNode;
	Node * unblurredNode;
	DrivingAssistManager * drivingAidsManager;
	NodeEvent * renderFinishedNodeEvent;
	NodeEvent * beforeCarsNode;
	MirrorTextureRenderer * mirrorTextureRenderer;
	ACCameraManager * cameraManager;
	NodeDirtCamera * cameraDirtNode;
	SystemMessage * systemMessage;
	SystemNotification * systemNotification;
	RaceManager * raceManager;
	ReplayManager * replayManager;
	PhysicsAvatar * physicsAvatar;
	ScreenCapturer * screenCapturer;
	ACClient * client;
	PitStop * pitStop;
	TimeLimitedTest * timeLimitedTest;
	bool useProView;
	bool isVrConnected;
	bool isRoomVR;
	bool isVirtualMirrorForced;
	bool useMousePitstop;
	QuickMenu * quickMenu;
	MicroSectors * microSectors;
	bool serializeForms;
	int connectedCarsCount;
	bool allowFreeCamera;
	bool tripleScreenMode;
	bool audioPerformanceSpew;
	std::vector<CarAvatar *,std::allocator<CarAvatar *> > cars;
	unsigned int focusedCarIndex;
	float cameraFadeTime;
	Console * console;
	SimScreen * simScreen;
	CameraForward * sceneCamera;
	SkyBox * skyBox;
	ACErrorHandler * errorHandler;
	DebugVisualizer * debugVisualizer;
	ksgui_FormRenderStats * formRenderStats;
	EndSessionDisplayer * endSessionDisplayer;
	SessionLeaderboard * leaderboard;
	Trigger changingCameraTrigger;
	VirtualMirrorRenderer * virtualMirrorRenderer;
	BufferedChannel<std::function<void __cdecl(void)> > chFunctions;
	float lastDT;
	NodeEvent * node3DGUI;
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,Texture,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,Texture> > > flagMapCache;
	inline Sim()  { }
	inline void ctor(Game * igame) { typedef void (*_fpt)(Sim *pthis, Game *); _fpt _f=(_fpt)_drva(1646704); _f(this, igame); }
	virtual ~Sim();
	inline void dtor() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1660624); _f(this); }
	inline void applyCustomWeather(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef void (*_fpt)(Sim *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1671184); return _f(this, name); }
	inline void startGame() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1698560); return _f(this); }
	inline Camera * getSceneCamera() { typedef Camera * (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1676848); return _f(this); }
	virtual void update_vf1(float deltaT);
	inline void update_impl(float deltaT) { typedef void (*_fpt)(Sim *pthis, float); _fpt _f=(_fpt)_drva(1699728); return _f(this, deltaT); }
	inline void update(float deltaT) { return update_vf1(deltaT); }
	virtual void render_vf2(float deltaT);
	inline void render_impl(float deltaT) { typedef void (*_fpt)(Sim *pthis, float); _fpt _f=(_fpt)_drva(1696944); return _f(this, deltaT); }
	inline void render(float deltaT) { return render_vf2(deltaT); }
	virtual void renderHUD_vf3(float deltaT);
	inline void renderHUD_impl(float deltaT) { typedef void (*_fpt)(Sim *pthis, float); _fpt _f=(_fpt)_drva(1697008); return _f(this, deltaT); }
	inline void renderHUD(float deltaT) { return renderHUD_vf3(deltaT); }
	inline CarAvatar * addCar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & model, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin) { typedef CarAvatar * (*_fpt)(Sim *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1669280); return _f(this, model, config, skin); }
	inline CarAvatar * addNetCar(ClientRemoteCarDef & desc, ACClient * client) { typedef CarAvatar * (*_fpt)(Sim *pthis, ClientRemoteCarDef &, ACClient *); _fpt _f=(_fpt)_drva(1670192); return _f(this, desc, client); }
	inline void renderScene(float deltaT) { typedef void (*_fpt)(Sim *pthis, float); _fpt _f=(_fpt)_drva(1697136); return _f(this, deltaT); }
	inline void loadTrack(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & tname, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, int playerPitPosition) { typedef void (*_fpt)(Sim *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, int); _fpt _f=(_fpt)_drva(1680576); return _f(this, tname, config, playerPitPosition); }
	inline CarAvatar * getCar(unsigned int index) { typedef CarAvatar * (*_fpt)(Sim *pthis, unsigned int); _fpt _f=(_fpt)_drva(1676000); return _f(this, index); }
	inline unsigned int getCarsCount() { typedef unsigned int (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(210640); return _f(this); }
	inline void onPostLoad() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1695776); return _f(this); }
	inline CameraForward * createCamera(INIReaderDocuments & videoIni, float useBlur) { typedef CameraForward * (*_fpt)(Sim *pthis, INIReaderDocuments &, float); _fpt _f=(_fpt)_drva(1671904); return _f(this, videoIni, useBlur); }
	inline void setSplashMessage(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & msg) { typedef void (*_fpt)(Sim *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1697888); return _f(this, msg); }
	inline void setSplashLoadingCar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & carUnixName) { typedef void (*_fpt)(Sim *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1697760); return _f(this, carUnixName); }
	virtual void shutdown_vf5();
	inline void shutdown_impl() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1698272); return _f(this); }
	inline void shutdown() { return shutdown_vf5(); }
	inline bool isInTripleScreenMode() { typedef bool (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1680448); return _f(this); }
	inline unsigned int nextCar(unsigned int anIndex) { typedef unsigned int (*_fpt)(Sim *pthis, unsigned int); _fpt _f=(_fpt)_drva(1681216); return _f(this, anIndex); }
	inline unsigned int previousCar(unsigned int anIndex) { typedef unsigned int (*_fpt)(Sim *pthis, unsigned int); _fpt _f=(_fpt)_drva(1696128); return _f(this, anIndex); }
	inline void writeOutputJson() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1702224); return _f(this); }
	inline bool isDisplayingResults() { typedef bool (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(503088); return _f(this); }
	inline void displayLeaderboard(bool value) { typedef void (*_fpt)(Sim *pthis, bool); _fpt _f=(_fpt)_drva(280848); return _f(this, value); }
	virtual void onKeyDown_vf0(OnKeyEvent & message);
	inline void onKeyDown_impl(OnKeyEvent & message) { typedef void (*_fpt)(Sim *pthis, OnKeyEvent &); _fpt _f=(_fpt)_drva(1681728); return _f(this, message); }
	inline void onKeyDown(OnKeyEvent & message) { return onKeyDown_vf0(message); }
	virtual void onKeyChar_vf1(unsigned int key);
	inline void onKeyChar_impl(unsigned int key) { typedef void (*_fpt)(Sim *pthis, unsigned int); _fpt _f=(_fpt)_drva(96368); return _f(this, key); }
	inline void onKeyChar(unsigned int key) { return onKeyChar_vf1(key); }
	inline void onReplayModeChanged(OnReplayStatusChanged & message) { typedef void (*_fpt)(Sim *pthis, OnReplayStatusChanged &); _fpt _f=(_fpt)_drva(1695936); return _f(this, message); }
	inline Console & getConsole() { typedef Console & (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1676048); return _f(this); }
	inline TrackData getTrackData() { typedef TrackData (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1676864); return _f(this); }
	inline void setPauseMode(bool mode) { typedef void (*_fpt)(Sim *pthis, bool); _fpt _f=(_fpt)_drva(1697488); return _f(this, mode); }
	inline void activateGameGui() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1669088); return _f(this); }
	inline void activatePauseMenu() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1669184); return _f(this); }
	inline void activateEscMenu() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1668992); return _f(this); }
	inline void setFocusedCarIndex(unsigned int aCarIndex) { typedef void (*_fpt)(Sim *pthis, unsigned int); _fpt _f=(_fpt)_drva(1697456); return _f(this, aCarIndex); }
	inline unsigned int getFocusedCarIndex() { typedef unsigned int (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1676080); return _f(this); }
	inline void executeOnMainThread(std::function<void __cdecl(void)> * f) { typedef void (*_fpt)(Sim *pthis, std::function<void __cdecl(void)> *); _fpt _f=(_fpt)_drva(1675840); return _f(this, f); }
	inline void unloadMeshResources(Node * node) { typedef void (*_fpt)(Sim *pthis, Node *); _fpt _f=(_fpt)_drva(1699280); return _f(this, node); }
	inline Texture * getNationFlag(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & nationCode) { typedef Texture * (*_fpt)(Sim *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(1676112); return _f(this, nationCode); }
	inline bool isVrRoom() { typedef bool (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(888336); return _f(this); }
	inline void initSceneGraph() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1678704); return _f(this); }
	inline void stepPhysicsEvent() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1698768); return _f(this); }
	inline void initCubemaps() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1677216); return _f(this); }
	inline void initHDRLevels() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1678080); return _f(this); }
	inline void addForm(ksgui_Form * aForm, bool isInDevMode) { typedef void (*_fpt)(Sim *pthis, ksgui_Form *, bool); _fpt _f=(_fpt)_drva(1669856); return _f(this, aForm, isInDevMode); }
	inline void initStaticCubemap() { typedef void (*_fpt)(Sim *pthis); _fpt _f=(_fpt)_drva(1680032); return _f(this); }
};

struct RenderContext {
public:
	MaterialFilter & materialFilter;
	IMeshRenderFilter & meshFilter;
	GraphicsManager * graphics;
	Camera * camera;
	inline RenderContext()  : materialFilter(*((MaterialFilter*)NULL)), meshFilter(*((IMeshRenderFilter*)NULL)) { }
};

class PhysicsCarStateProvider : public ICarPhysicsStateProvider {
public:
	Car * car;
	inline PhysicsCarStateProvider()  { }
	inline void ctor(Car * car) { typedef void (*_fpt)(PhysicsCarStateProvider *pthis, Car *); _fpt _f=(_fpt)_drva(1190384); _f(this, car); }
	virtual ~PhysicsCarStateProvider();
	virtual void getPhysicsState_vf1(CarPhysicsState & physicsState);
	inline void getPhysicsState_impl(CarPhysicsState & physicsState) { typedef void (*_fpt)(PhysicsCarStateProvider *pthis, CarPhysicsState &); _fpt _f=(_fpt)_drva(1190464); return _f(this, physicsState); }
	inline void getPhysicsState(CarPhysicsState & physicsState) { return getPhysicsState_vf1(physicsState); }
	virtual void getWingState_vf2(std::vector<WingState,std::allocator<WingState> > & wingStatus);
	inline void getWingState_impl(std::vector<WingState,std::allocator<WingState> > & wingStatus) { typedef void (*_fpt)(PhysicsCarStateProvider *pthis, std::vector<WingState,std::allocator<WingState> > &); _fpt _f=(_fpt)_drva(1190480); return _f(this, wingStatus); }
	inline void getWingState(std::vector<WingState,std::allocator<WingState> > & wingStatus) { return getWingState_vf2(wingStatus); }
};

class ksgui_Graph : public ksgui_Control {
public:
	float minY;
	float maxY;
	unsigned int maxValuesCount;
	bool autoAdjustMaxValue;
	std::vector<ksgui_ValueSerie *,std::allocator<ksgui_ValueSerie *> > series;
	std::vector<ksgui_GraphReferenceAxis,std::allocator<ksgui_GraphReferenceAxis> > axes;
	inline ksgui_Graph()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui) { typedef void (*_fpt)(ksgui_Graph *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, ksgui_GUI *); _fpt _f=(_fpt)_drva(2388336); _f(this, iname, igui); }
	virtual ~ksgui_Graph();
	inline void dtor() { typedef void (*_fpt)(ksgui_Graph *pthis); _fpt _f=(_fpt)_drva(2388544); _f(this); }
	virtual void render_vf3(float dt);
	inline void render_impl(float dt) { typedef void (*_fpt)(ksgui_Graph *pthis, float); _fpt _f=(_fpt)_drva(2390240); return _f(this, dt); }
	inline void render(float dt) { return render_vf3(dt); }
	inline void addSerie(vec3f color) { typedef void (*_fpt)(ksgui_Graph *pthis, vec3f); _fpt _f=(_fpt)_drva(2389536); return _f(this, color); }
	inline void addValue(unsigned int serieIndex, float v) { typedef void (*_fpt)(ksgui_Graph *pthis, unsigned int, float); _fpt _f=(_fpt)_drva(2389760); return _f(this, serieIndex, v); }
	inline void clearSerie(unsigned int serieIndex) { typedef void (*_fpt)(ksgui_Graph *pthis, unsigned int); _fpt _f=(_fpt)_drva(2389856); return _f(this, serieIndex); }
	inline void addReferenceAxis(ksgui_GraphReferenceAxis & axis) { typedef void (*_fpt)(ksgui_Graph *pthis, ksgui_GraphReferenceAxis &); _fpt _f=(_fpt)_drva(2389520); return _f(this, axis); }
	inline int getSeriesCount() { typedef int (*_fpt)(ksgui_Graph *pthis); _fpt _f=(_fpt)_drva(2389936); return _f(this); }
	inline float getValuesAtPercentage(int serieIndex, float perc) { typedef float (*_fpt)(ksgui_Graph *pthis, int, float); _fpt _f=(_fpt)_drva(2389968); return _f(this, serieIndex, perc); }
	inline void renderAxes(GLRenderer & gl) { typedef void (*_fpt)(ksgui_Graph *pthis, GLRenderer &); _fpt _f=(_fpt)_drva(2390944); return _f(this, gl); }
};

struct BrakeSystem {
public:
	float frontBias;
	float brakePowerMultiplier;
	float electronicOverride;
	float handBrakeTorque;
	float ebbInstant;
	BrakeDisc discs[0x4];
	float limitDown;
	float limitUp;
	float rearCorrectionTorque;
	Car * car;
	float brakePower;
	float biasOverride;
	bool hasCockpitBias;
	float biasStep;
	EBBMode ebbMode;
	float ebbFrontMultiplier;
	SteerBrake steerBrake;
	bool hasBrakeTempsData;
	std::basic_ofstream<char,std::char_traits<char> > tempRunFile;
	DynamicController ebbController;
	inline BrakeSystem()  { }
	inline void init(Car * car) { typedef void (*_fpt)(BrakeSystem *pthis, Car *); _fpt _f=(_fpt)_drva(2676368); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(BrakeSystem *pthis, float); _fpt _f=(_fpt)_drva(2680384); return _f(this, dt); }
	inline float getBrakePower() { typedef float (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2676256); return _f(this); }
	inline void setManualFrontBias(int value) { typedef void (*_fpt)(BrakeSystem *pthis, int); _fpt _f=(_fpt)_drva(2680272); return _f(this, value); }
	inline float getFrontBias() { typedef float (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2676272); return _f(this); }
	inline void reset() { typedef void (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2679952); return _f(this); }
	inline bool isUsingEBB() { typedef bool (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2879392); return _f(this); }
	inline void activateTempRunFile(bool mode) { typedef void (*_fpt)(BrakeSystem *pthis, bool); _fpt _f=(_fpt)_drva(2675712); return _f(this, mode); }
	inline void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * dataPath) { typedef void (*_fpt)(BrakeSystem *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2676848); return _f(this, dataPath); }
	inline void stepTemps(float dt) { typedef void (*_fpt)(BrakeSystem *pthis, float); _fpt _f=(_fpt)_drva(2681120); return _f(this, dt); }
	inline void saveTempsRunFame() { typedef void (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2680032); return _f(this); }
	inline void ctor() { typedef void (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2539040); _f(this); }
	inline void dtor() { typedef void (*_fpt)(BrakeSystem *pthis); _fpt _f=(_fpt)_drva(2547824); _f(this); }
};

class ERS : public ITorqueGenerator, public ICoastGenerator {
public:
	bool present;
	float kineticRecovery;
	ERSStatus status;
	bool isHeatCharginBattery;
	std::vector<ERSPowerController,std::allocator<ERSPowerController> > ersPowerControllers;
	std::vector<ERSPowerController,std::allocator<ERSPowerController> > ersPowerControllersFront;
	int defaultPowerControllerIndex;
	bool isCharging;
	ERSCockpitControls cockpitControls;
	Car * car;
	double chargeK;
	double dischargeK;
	double dischargeKFront;
	bool hasButtonOverride;
	Curve torqueLUT;
	Curve coastLUT;
	DynamicController controller;
	DynamicController controllerFront;
	double charge;
	float maxJ;
	float currentJ;
	float input;
	double heatChargeK;
	float heatTorque;
	float rearCorrectionTorque;
	DynamicController frontController;
	Curve frontTorqueLUT;
	float frontTorqueVectoringBias;
	inline ERS()  { }
	inline void ctor() { typedef void (*_fpt)(ERS *pthis); _fpt _f=(_fpt)_drva(2692352); _f(this); }
	virtual ~ERS();
	inline void dtor() { typedef void (*_fpt)(ERS *pthis); _fpt _f=(_fpt)_drva(2692816); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(ERS *pthis, Car *); _fpt _f=(_fpt)_drva(2694192); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(ERS *pthis, float); _fpt _f=(_fpt)_drva(2699488); return _f(this, dt); }
	virtual float getOutputTorque_vf1();
	inline float getOutputTorque_impl() { typedef float (*_fpt)(ERS *pthis); _fpt _f=(_fpt)_drva(2694096); return _f(this); }
	inline float getOutputTorque() { return getOutputTorque_vf1(); }
	virtual float getCoastTorque_vf1();
	inline float getCoastTorque_impl() { typedef float (*_fpt)(ERS *pthis); _fpt _f=(_fpt)_drva(2693952); return _f(this); }
	inline float getCoastTorque() { return getCoastTorque_vf1(); }
	inline void reset() { typedef void (*_fpt)(ERS *pthis); _fpt _f=(_fpt)_drva(2699168); return _f(this); }
	inline void setPowerController(int index) { typedef void (*_fpt)(ERS *pthis, int); _fpt _f=(_fpt)_drva(2699200); return _f(this, index); }
};

class Wing {
public:
	WingData data;
	WingState status;
	std::vector<DynamicWingController,std::allocator<DynamicWingController> > dynamicControllers;
	Car * car;
	RaceEngineer engineer;
	float damageCL[0x5];
	float damageCD[0x5];
	bool hasDamage;
	WingOverrideDef overrideStatus;
	float SPEED_DAMAGE_COEFF;
	float SURFACE_DAMAGE_COEFF;
	inline Wing()  { }
	inline void ctor(Wing & __that) { typedef void (*_fpt)(Wing *pthis, Wing &); _fpt _f=(_fpt)_drva(2839136); _f(this, __that); }
	inline void ctor(Car * a_car, INIReader & ini, int index, bool isVertical) { typedef void (*_fpt)(Wing *pthis, Car *, INIReader &, int, bool); _fpt _f=(_fpt)_drva(2822976); _f(this, a_car, ini, index, isVertical); }
	inline void dtor() { typedef void (*_fpt)(Wing *pthis); _fpt _f=(_fpt)_drva(2827120); _f(this); }
	inline void step(float dt) { typedef void (*_fpt)(Wing *pthis, float); _fpt _f=(_fpt)_drva(2829248); return _f(this, dt); }
	inline void setOverrideAngle(float iangle) { typedef void (*_fpt)(Wing *pthis, float); _fpt _f=(_fpt)_drva(2829232); return _f(this, iangle); }
	inline void clearOverrides() { typedef void (*_fpt)(Wing *pthis); _fpt _f=(_fpt)_drva(2829168); return _f(this); }
	inline float getCurrentModifiedAngle() { typedef float (*_fpt)(Wing *pthis); _fpt _f=(_fpt)_drva(2829184); return _f(this); }
	inline void stepDynamicControllers(float dt) { typedef void (*_fpt)(Wing *pthis, float); _fpt _f=(_fpt)_drva(2829776); return _f(this, dt); }
	inline void addDrag(vec3f & lv) { typedef void (*_fpt)(Wing *pthis, vec3f &); _fpt _f=(_fpt)_drva(2827296); return _f(this, lv); }
	inline void addLift(vec3f & lv) { typedef void (*_fpt)(Wing *pthis, vec3f &); _fpt _f=(_fpt)_drva(2828080); return _f(this, lv); }
};

struct Engine {
public:
	acEngineData data;
	EngineStatus status;
	float coastTorqueMultiplier;
	float limiterMultiplier;
	float fuelPressure;
	float bov;
	std::vector<Turbo,std::allocator<Turbo> > turbos;
	bool isEngineStallEnabled;
	float starterTorque;
	float rpmDamageThreshold;
	float restrictor;
	PushToPass p2p;
	std::vector<ITorqueGenerator *,std::allocator<ITorqueGenerator *> > torqueGenerators;
	std::vector<ICoastGenerator *,std::allocator<ICoastGenerator *> > coastGenerators;
	bool turboAdjustableFromCockpit;
	SACEngineInput lastInput;
	int defaultEngineLimiter;
	float inertia;
	int limiterOn;
	float electronicOverride;
	float maxPowerW_Dynamic;
	float maxPowerW;
	float maxTorqueNM;
	float maxPowerRPM;
	float maxTorqueRPM;
	PhysicsEngine * physicsEngine;
	Curve throttleResponseCurve;
	Curve throttleResponseCurveMax;
	float throttleResponseCurveMaxRef;
	float gasUsage;
	double lifeLeft;
	float turboBoostDamageThreshold;
	float turboBoostDamageK;
	float rpmDamageK;
	float bovThreshold;
	Car * car;
	std::vector<TurboDynamicController,std::allocator<TurboDynamicController> > turboControllers;
	float gasCoastOffset;
	Curve gasCoastOffsetCurve;
	int coastSettingsDefaultIndex;
	int coastEntryRpm;
	inline Engine()  { }
	inline void ctor() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2642608); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2643264); _f(this); }
	virtual bool init_vf0(char * carModel);
	inline bool init_impl(char * carModel) { typedef bool (*_fpt)(Engine *pthis, char *); _fpt _f=(_fpt)_drva(706688); return _f(this, carModel); }
	inline bool init(char * carModel) { return init_vf0(carModel); }
	inline void init(Car * car) { typedef void (*_fpt)(Engine *pthis, Car *); _fpt _f=(_fpt)_drva(2645520); return _f(this, car); }
	inline void addTorqueGenerator(ITorqueGenerator * generator) { typedef void (*_fpt)(Engine *pthis, ITorqueGenerator *); _fpt _f=(_fpt)_drva(2644496); return _f(this, generator); }
	inline void addCoastGenerator(ICoastGenerator * generator) { typedef void (*_fpt)(Engine *pthis, ICoastGenerator *); _fpt _f=(_fpt)_drva(2644464); return _f(this, generator); }
	inline void setTurboBoostLevel(float value) { typedef void (*_fpt)(Engine *pthis, float); _fpt _f=(_fpt)_drva(2654352); return _f(this, value); }
	inline float getTurboBoostLevel() { typedef float (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2645440); return _f(this); }
	inline void step(SACEngineInput & input, float dt) { typedef void (*_fpt)(Engine *pthis, SACEngineInput &, float); _fpt _f=(_fpt)_drva(2654432); return _f(this, input, dt); }
	virtual int getLimiterRPM_vf1();
	inline int getLimiterRPM_impl() { typedef int (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644560); return _f(this); }
	inline int getLimiterRPM() { return getLimiterRPM_vf1(); }
	virtual bool isLimiterOn_vf2();
	inline bool isLimiterOn_impl() { typedef bool (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2645648); return _f(this); }
	inline bool isLimiterOn() { return isLimiterOn_vf2(); }
	inline float getMaxPowerW() { typedef float (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644608); return _f(this); }
	inline float getMaxTorqueNM() { typedef float (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644688); return _f(this); }
	inline float getMaxPowerRPM() { typedef float (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644592); return _f(this); }
	inline float getMaxTorqueRPM() { typedef float (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644704); return _f(this); }
	inline float getTorqueAtRPM(float rpm, float gas) { typedef float (*_fpt)(Engine *pthis, float, float); _fpt _f=(_fpt)_drva(2645264); return _f(this, rpm, gas); }
	inline void reset() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2654096); return _f(this); }
	inline float getMaxTurboBoost(bool with_wastegate) { typedef float (*_fpt)(Engine *pthis, bool); _fpt _f=(_fpt)_drva(2644720); return _f(this, with_wastegate); }
	inline float getSafeTurboLevel() { typedef float (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644816); return _f(this); }
	inline void blowUp() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2644528); return _f(this); }
	inline float getThrottleResponseGas(float gas, float rpm) { typedef float (*_fpt)(Engine *pthis, float, float); _fpt _f=(_fpt)_drva(2644880); return _f(this, gas, rpm); }
	inline void setCoastSettings(int s) { typedef void (*_fpt)(Engine *pthis, int); _fpt _f=(_fpt)_drva(2654224); return _f(this, s); }
	inline void stepP2P(float dt) { typedef void (*_fpt)(Engine *pthis, float); _fpt _f=(_fpt)_drva(2656080); return _f(this, dt); }
	inline void loadINI() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2646272); return _f(this); }
	inline void precalculatePowerAndTorque() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2653456); return _f(this); }
	inline void stepTurbos() { typedef void (*_fpt)(Engine *pthis); _fpt _f=(_fpt)_drva(2656512); return _f(this); }
	inline CoastSettings loadCoastSettings(INIReader & r, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section) { typedef CoastSettings (*_fpt)(Engine *pthis, INIReader &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2645664); return _f(this, r, section); }
};

class Game {
public:
	bool isRenderingGui;
	bool isSwappingBuffer;
	bool isSuppressingAudioUpdate;
	GameTime gameTime;
	RenderWindow window;
	GraphicsManager * graphics;
	float masterVolume;
	AudioEngine * audioEngine;
	GameStats stats;
	GameObject * root;
	ksgui_GUI * gui;
	KeyboardManager keyboardManager;
	JoypadManager joypadManager;
	int sleepTime;
	Event<float> evOnPreGUI;
	Event<float> evOnPostGui;
	Event<float> evOnRenderFinished;
	Event<float> evOnPostUpdate;
	Event<double> evOnBeginFrame;
	Event<double> evOnEndFrame;
	Event<int> evOnShutdownRequested;
	bool isClosing;
	inline Game()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, VideoSettings & videoSettings) { typedef void (*_fpt)(Game *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, VideoSettings &); _fpt _f=(_fpt)_drva(2365568); _f(this, name, videoSettings); }
	virtual ~Game();
	inline void dtor() { typedef void (*_fpt)(Game *pthis); _fpt _f=(_fpt)_drva(2367152); _f(this); }
	inline void onIdle() { typedef void (*_fpt)(Game *pthis); _fpt _f=(_fpt)_drva(2369328); return _f(this); }
	inline void run() { typedef void (*_fpt)(Game *pthis); _fpt _f=(_fpt)_drva(2371248); return _f(this); }
	virtual void shutdown_vf1();
	inline void shutdown_impl() { typedef void (*_fpt)(Game *pthis); _fpt _f=(_fpt)_drva(2371504); return _f(this); }
	inline void shutdown() { return shutdown_vf1(); }
	inline void shutdownObject(GameObject * o) { typedef void (*_fpt)(Game *pthis, GameObject *); _fpt _f=(_fpt)_drva(2371520); return _f(this, o); }
	inline void renderHUDOnDemand(float dt) { typedef void (*_fpt)(Game *pthis, float); _fpt _f=(_fpt)_drva(2371232); return _f(this, dt); }
	inline void update(GameObject * o, float dt) { typedef void (*_fpt)(Game *pthis, GameObject *, float); _fpt _f=(_fpt)_drva(2371600); return _f(this, o, dt); }
	inline void render(GameObject * o, float dt) { typedef void (*_fpt)(Game *pthis, GameObject *, float); _fpt _f=(_fpt)_drva(2370896); return _f(this, o, dt); }
	inline void renderHUD(GameObject * o, float dt) { typedef void (*_fpt)(Game *pthis, GameObject *, float); _fpt _f=(_fpt)_drva(2371120); return _f(this, o, dt); }
	inline void renderAudio(GameObject * o, float dt) { typedef void (*_fpt)(Game *pthis, GameObject *, float); _fpt _f=(_fpt)_drva(2371008); return _f(this, o, dt); }
	inline void deleteGameObjectRec(GameObject * go) { typedef void (*_fpt)(Game *pthis, GameObject *); _fpt _f=(_fpt)_drva(2369200); return _f(this, go); }
};

class NetCarStateProvider : public GameObject, public ICarPhysicsStateProvider {
public:
	Event<OnSectorSplitEvent> evOnSectorSplit;
	unsigned char sessionID;
	DriverInfo driverInfo;
	vec3f errorVector;
	int guid;
	int ping;
	unsigned int lastLap;
	unsigned int bestLap;
	std::vector<unsigned int,std::allocator<unsigned int> > currentSplits;
	std::vector<unsigned int,std::allocator<unsigned int> > personalBestSplits;
	double lastUpdateTime;
	double lastTime;
	std::vector<Lap,std::allocator<Lap> > laps;
	NetCarPushToPass p2p;
	bool hasEverReceivedAPacket;
	float ballastKG;
	float restrictor;
	float hasCollisionInThisStep;
	NetCarQoS qos;
	ACClient * client;
	mat44f bodyMatrix;
	mat44f wheelMatrix[0x4];
	mat44f suspMatrix[0x4];
	NetCarState netStates[0x3];
	vec3f lastScreenPos;
	mat44f wheelBasePosLS[0x4];
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > unixName;
	IRayTrackCollisionProvider * rayCastProvider;
	vec3f lastVelocity;
	vec3f instantVelocity;
	vec3f currentVelocity;
	CarPhysicsState state;
	std::vector<WingState,std::allocator<WingState> > wingStates;
	std::vector<std::vector<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> >,std::allocator<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> > > >,std::allocator<std::vector<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> >,std::allocator<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> > > > > > wingControllerLists;
	std::unique_ptr<Node,std::default_delete<Node> > colliderModel;
	mat44f pitPosition;
	SlipStream slipStream;
	DriverInfo oldDriverInfo;
	bool isDisconnected;
	float lastStepSpeedkmh;
	float lastStepGas;
	std::vector<DRSWingSetting,std::allocator<DRSWingSetting> > drsWingSettings;
	vec3f smoothVelocity;
	Concurrency::concurrent_queue<NetCarState,std::allocator<NetCarState> > incomingStateQueue;
	std::vector<CarPhysicsState,std::allocator<CarPhysicsState> > stateCache;
	double lastIntegrationTime;
	float tyreRadius[0x4];
	vec3f graphicsOffset;
	float graphicsPitchRotation;
	mat44f tyreLocalRotation[0x4];
	CarAvatar * avatar;
	IRigidBody * body;
	vec3f lastDistanceV;
	vec3f lastAxis;
	SplineLocationData splineLocationData;
	double lastSliceTimeStamp;
	float currentAOA;
	float carVerticalOffset;
	bool useLog;
	NetCarStateProvider_LagDebug lagDebug;
	inline NetCarStateProvider()  { }
	inline void ctor(NetCarStateProviderDef & def) { typedef void (*_fpt)(NetCarStateProvider *pthis, NetCarStateProviderDef &); _fpt _f=(_fpt)_drva(1141872); _f(this, def); }
	virtual ~NetCarStateProvider();
	inline void dtor() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1147072); _f(this); }
	virtual void update_vf1(float dt);
	inline void update_impl(float dt) { typedef void (*_fpt)(NetCarStateProvider *pthis, float); _fpt _f=(_fpt)_drva(1169744); return _f(this, dt); }
	inline void update(float dt) { return update_vf1(dt); }
	virtual void getPhysicsState_vf1(CarPhysicsState & physicsState);
	inline void getPhysicsState_impl(CarPhysicsState & physicsState) { typedef void (*_fpt)(NetCarStateProvider *pthis, CarPhysicsState &); _fpt _f=(_fpt)_drva(1151520); return _f(this, physicsState); }
	inline void getPhysicsState(CarPhysicsState & physicsState) { return getPhysicsState_vf1(physicsState); }
	virtual void getWingState_vf2(std::vector<WingState,std::allocator<WingState> > & ws);
	inline void getWingState_impl(std::vector<WingState,std::allocator<WingState> > & ws) { typedef void (*_fpt)(NetCarStateProvider *pthis, std::vector<WingState,std::allocator<WingState> > &); _fpt _f=(_fpt)_drva(1153600); return _f(this, ws); }
	inline void getWingState(std::vector<WingState,std::allocator<WingState> > & ws) { return getWingState_vf2(ws); }
	inline void onRemoteStateReceived(NetCarState & s) { typedef void (*_fpt)(NetCarStateProvider *pthis, NetCarState &); _fpt _f=(_fpt)_drva(1159648); return _f(this, s); }
	inline void setCarAvatar(CarAvatar * av) { typedef void (*_fpt)(NetCarStateProvider *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(1162368); return _f(this, av); }
	inline void resetDamage() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1162048); return _f(this); }
	inline void setDamageZoneLevel(float * zones) { typedef void (*_fpt)(NetCarStateProvider *pthis, float *); _fpt _f=(_fpt)_drva(1162464); return _f(this, zones); }
	inline void activateP2P() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1150672); return _f(this); }
	inline mat44f getSmoothBodyMatrix(double physics_time, CarPhysicsState & state) { typedef mat44f (*_fpt)(NetCarStateProvider *pthis, double, CarPhysicsState &); _fpt _f=(_fpt)_drva(1151760); return _f(this, physics_time, state); }
	inline vec3f projectNetStatePos(int ti, double t) { typedef vec3f (*_fpt)(NetCarStateProvider *pthis, int, double); _fpt _f=(_fpt)_drva(1161472); return _f(this, ti, t); }
	inline void initPhysicsValues() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1155040); return _f(this); }
	inline void step(double physics_time) { typedef void (*_fpt)(NetCarStateProvider *pthis, double); _fpt _f=(_fpt)_drva(1162496); return _f(this, physics_time); }
	inline void stepLagging(double physics_time) { typedef void (*_fpt)(NetCarStateProvider *pthis, double); _fpt _f=(_fpt)_drva(1167200); return _f(this, physics_time); }
	inline void debugLag(bool isLagging, double physics_time) { typedef void (*_fpt)(NetCarStateProvider *pthis, bool, double); _fpt _f=(_fpt)_drva(1151312); return _f(this, isLagging, physics_time); }
	inline void initWings() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1157760); return _f(this); }
	inline void stepWings() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1169344); return _f(this); }
	inline void initP2P() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1154352); return _f(this); }
	inline void stepP2P() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1169232); return _f(this); }
	inline void initDRS() { typedef void (*_fpt)(NetCarStateProvider *pthis); _fpt _f=(_fpt)_drva(1153632); return _f(this); }
};

class GraphicsManager {
public:
	bool useCustomSunDirection;
	float exposureMultiplier;
	VideoSettings videoSettings;
	RenderStats stats;
	LightingSettings lightingSettings;
	RendererFlags renderFlags;
	bool suspendViewportUpdateOnSetRenderTarget;
	GPUProfiler * gpuProfiler;
	PvsProcessor * pvsProcessor;
	CubeMap * currentCubeMap;
	Event<OnWindowResize> evWindowResize;
	Event<OnWindowResize> evWindowPreResize;
	std::unique_ptr<ResourceStore,std::default_delete<ResourceStore> > resourceStore;
	RenderState state;
	GLRenderer * gl;
	vec3f customSunDirection;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > errorStrings;
	std::vector<GLRenderer *,std::allocator<GLRenderer *> > glRenderers;
	int multiSampleQuality;
	SamplerStates samplerStates;
	SystemCBuffers sysBuffers;
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,CBuffer *,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,CBuffer *> > > cBuffersMap;
	ShaderManager shaderManager;
	inline GraphicsManager()  { }
	inline void ctor(VideoSettings & ivideoSettings) { typedef void (*_fpt)(GraphicsManager *pthis, VideoSettings &); _fpt _f=(_fpt)_drva(2102816); _f(this, ivideoSettings); }
	virtual ~GraphicsManager();
	inline void dtor() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2105552); _f(this); }
	inline std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > getErrorStrings() { typedef std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2107968); return _f(this); }
	inline void beginMainRenderPass() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2106736); return _f(this); }
	inline void endMainRenderPass() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2107456); return _f(this); }
	inline GLRenderer & getGL() { typedef GLRenderer & (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(148688); return _f(this); }
	inline GLRenderer * createGLRenderer(unsigned int maxVertices) { typedef GLRenderer * (*_fpt)(GraphicsManager *pthis, unsigned int); _fpt _f=(_fpt)_drva(2107152); return _f(this, maxVertices); }
	inline void clearRenderTarget(vec4f & color) { typedef void (*_fpt)(GraphicsManager *pthis, vec4f &); _fpt _f=(_fpt)_drva(2106944); return _f(this, color); }
	inline void clearRenderTargetDepth(float value) { typedef void (*_fpt)(GraphicsManager *pthis, float); _fpt _f=(_fpt)_drva(2106960); return _f(this, value); }
	inline void beginScene() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2106752); return _f(this); }
	inline void endScene() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2107472); return _f(this); }
	inline void setViewport(int left, int top, int width, int height) { typedef void (*_fpt)(GraphicsManager *pthis, int, int, int, int); _fpt _f=(_fpt)_drva(2117616); return _f(this, left, top, width, height); }
	inline void drawPrimitive(unsigned int indicesCount, unsigned int baseIndex, unsigned int baseVertex, unsigned int numVertices) { typedef void (*_fpt)(GraphicsManager *pthis, unsigned int, unsigned int, unsigned int, unsigned int); _fpt _f=(_fpt)_drva(2107376); return _f(this, indicesCount, baseIndex, baseVertex, numVertices); }
	inline void compile(Node * root) { typedef void (*_fpt)(GraphicsManager *pthis, Node *); _fpt _f=(_fpt)_drva(2107104); return _f(this, root); }
	inline void setScreenSpaceMode() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2115808); return _f(this); }
	inline void onResize(int width, int height) { typedef void (*_fpt)(GraphicsManager *pthis, int, int); _fpt _f=(_fpt)_drva(2114352); return _f(this, width, height); }
	inline void updateLightingSetttings() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2118032); return _f(this); }
	inline void loadLightingSettings(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef void (*_fpt)(GraphicsManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2110032); return _f(this, filename); }
	inline void loadLightingSettings() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2114224); return _f(this); }
	inline void resetRenderStates() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2114624); return _f(this); }
	inline void overrideSamplerState(SamplerState sampler) { typedef void (*_fpt)(GraphicsManager *pthis, SamplerState); _fpt _f=(_fpt)_drva(2114576); return _f(this, sampler); }
	inline Shader * getShader(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef Shader * (*_fpt)(GraphicsManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2108512); return _f(this, name); }
	inline void setShader(Shader * sh) { typedef void (*_fpt)(GraphicsManager *pthis, Shader *); _fpt _f=(_fpt)_drva(2116096); return _f(this, sh); }
	inline void setBlendMode(BlendMode mode) { typedef void (*_fpt)(GraphicsManager *pthis, BlendMode); _fpt _f=(_fpt)_drva(2114784); return _f(this, mode); }
	inline void commitShaderChanges() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2107040); return _f(this); }
	inline void setDepthMode(DepthMode mode) { typedef void (*_fpt)(GraphicsManager *pthis, DepthMode); _fpt _f=(_fpt)_drva(2114944); return _f(this, mode); }
	inline void setCullMode(CullMode mode) { typedef void (*_fpt)(GraphicsManager *pthis, CullMode); _fpt _f=(_fpt)_drva(2114832); return _f(this, mode); }
	inline void setShadowMapBias(float b0, float b1, float b2) { typedef void (*_fpt)(GraphicsManager *pthis, float, float, float); _fpt _f=(_fpt)_drva(2116160); return _f(this, b0, b1, b2); }
	inline vec3f getShadowMapBias() { typedef vec3f (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2108560); return _f(this); }
	inline void setProjectionMatrix(mat44f & matrix) { typedef void (*_fpt)(GraphicsManager *pthis, mat44f &); _fpt _f=(_fpt)_drva(2115168); return _f(this, matrix); }
	inline void setViewMatrix(mat44f & matrix, Camera * camera) { typedef void (*_fpt)(GraphicsManager *pthis, mat44f &, Camera *); _fpt _f=(_fpt)_drva(2116864); return _f(this, matrix, camera); }
	inline void setWorldMatrix(mat44f & matrix) { typedef void (*_fpt)(GraphicsManager *pthis, mat44f &); _fpt _f=(_fpt)_drva(2117792); return _f(this, matrix); }
	inline mat44f getWorldMatrix() { typedef mat44f (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2108656); return _f(this); }
	inline mat44f getProjectionMatrix() { typedef mat44f (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2108464); return _f(this); }
	inline mat44f getViewMatrix() { typedef mat44f (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2108608); return _f(this); }
	inline void setVB(IVertexBuffer * vb) { typedef void (*_fpt)(GraphicsManager *pthis, IVertexBuffer *); _fpt _f=(_fpt)_drva(2116848); return _f(this, vb); }
	inline void setIB(IndexBuffer * ib) { typedef void (*_fpt)(GraphicsManager *pthis, IndexBuffer *); _fpt _f=(_fpt)_drva(2115152); return _f(this, ib); }
	inline CBuffer * getCBuffer(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef CBuffer * (*_fpt)(GraphicsManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2107632); return _f(this, name); }
	inline void setRenderTarget(RenderTarget * rt) { typedef void (*_fpt)(GraphicsManager *pthis, RenderTarget *); _fpt _f=(_fpt)_drva(2115424); return _f(this, rt); }
	inline void setTextureRT(unsigned int slot, RenderTarget * rt) { typedef void (*_fpt)(GraphicsManager *pthis, unsigned int, RenderTarget *); _fpt _f=(_fpt)_drva(2116816); return _f(this, slot, rt); }
	inline void setDepthTextureRT(unsigned int slot, RenderTarget * rt) { typedef void (*_fpt)(GraphicsManager *pthis, unsigned int, RenderTarget *); _fpt _f=(_fpt)_drva(2115120); return _f(this, slot, rt); }
	inline void setScreenRenderTargets() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2115712); return _f(this); }
	inline void setTexture(int slot, Texture & tex) { typedef void (*_fpt)(GraphicsManager *pthis, int, Texture &); _fpt _f=(_fpt)_drva(2116720); return _f(this, slot, tex); }
	inline void setShadowMapTexture(int level, RenderTarget * rt) { typedef void (*_fpt)(GraphicsManager *pthis, int, RenderTarget *); _fpt _f=(_fpt)_drva(2116592); return _f(this, level, rt); }
	inline void setShadowMapMatrix(int level, mat44f & matrix) { typedef void (*_fpt)(GraphicsManager *pthis, int, mat44f &); _fpt _f=(_fpt)_drva(2116336); return _f(this, level, matrix); }
	inline void setCustomSunDirection(vec3f sunDirection) { typedef void (*_fpt)(GraphicsManager *pthis, vec3f); _fpt _f=(_fpt)_drva(2114912); return _f(this, sunDirection); }
	inline void setSamplerState() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2115584); return _f(this); }
	inline void clearTextureSlot(int slot) { typedef void (*_fpt)(GraphicsManager *pthis, int); _fpt _f=(_fpt)_drva(2106976); return _f(this, slot); }
	inline vec3f getLDRColor(vec3f & c) { typedef vec3f (*_fpt)(GraphicsManager *pthis, vec3f &); _fpt _f=(_fpt)_drva(2108016); return _f(this, c); }
	inline float getLDRScale(vec3f & c) { typedef float (*_fpt)(GraphicsManager *pthis, vec3f &); _fpt _f=(_fpt)_drva(2108336); return _f(this, c); }
	inline void initRenderFlags() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2109280); return _f(this); }
	inline void initSamplerStates() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2109824); return _f(this); }
	inline void initCBuffers() { typedef void (*_fpt)(GraphicsManager *pthis); _fpt _f=(_fpt)_drva(2108704); return _f(this); }
};

class ACClient : public GameObject {
public:
	Event<OnChatMessageEvent> evOnChatMessage;
	Event<RemoteSession> evOnOnlineNewSession;
	Event<RemoteSessionResume> evOnOnlineEndSession;
	Event<OnLapCompletedEvent> evOnLapCompleted;
	Event<ReceivedVoteDef> evOnVoteReceived;
	Event<bool> evOnVoteNotPassed;
	Event<bool> evOnMandatoryPitDone;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > serverName;
	float remoteSpring;
	float remoteDamper;
	float remoteFactor;
	DriverInfo driverInfo;
	ServerInfo serverInfo;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > guid;
	ClientRules rules;
	ServerDrivingAssists serverDrivingAssists;
	bool isTVMode;
	double handshakeServerTimeS;
	std::vector<unsigned char,std::allocator<unsigned char> > playerCarMD5;
	bool debugStartingLights;
	bool isAssociated;
	ACClient_ClientSessionTransition transitionInfo;
	RemoteSessionResult sessionResultsGT;
	UDPSocket sok;
	std::vector<std::vector<unsigned char,std::allocator<unsigned char> >,std::allocator<std::vector<unsigned char,std::allocator<unsigned char> > > > checksumResults;
	Sim * sim;
	Car * car;
	ACClientVotingManager * votingManager;
	CarAvatar * avatar;
	double lastSendTime;
	double sendInterval;
	double lastChatMessage;
	double lastSpectatorPulse;
	IPAddress serverIP;
	unsigned char sessionID;
	std::vector<NetCarStateProvider *,std::allocator<NetCarStateProvider *> > netCars;
	unsigned char pakSequenceIndex;
	int handshakeCarPosition;
	WrongWayIndicator * wrongWayIndicator;
	int numWrongWayInfractions;
	TCPSocket tcpSock;
	RemoteSession currentSession;
	int finishPosition;
	Font * font;
	std::vector<RemoteSession,std::allocator<RemoteSession> > sessions;
	DisconnectCountdown dcCountdown;
	double lastSessionCheckTime;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > welcomeMessage;
	DamageReportDef damageReport;
	bool isFlashingCache;
	WreckerProtection wreckerProtection;
	unsigned int resultScreenTime;
	unsigned int raceOverTime;
	bool isGasPenaltyDisabled;
	unsigned int pitWindowStart;
	unsigned int pitWindowEnd;
	bool hasLeaderFinished;
	bool hasPlayerFinished;
	double raceClosingTime;
	bool isResultScreenOn;
	bool hasExtraLap;
	bool isLastLap;
	bool ignoreResultTeleport;
	int invertedGridPositions;
	std::vector<unsigned int,std::allocator<unsigned int> > localCarCurrentSplits;
	std::vector<unsigned int,std::allocator<unsigned int> > localCarBestSplits;
	std::vector<unsigned int,std::allocator<unsigned int> > bestSplits;
	unsigned int globalBestlap;
	std::vector<Lap,std::allocator<Lap> > localCarLaps;
	unsigned int localCarBestLap;
	unsigned int instanceLocalCarBestLap;
	Lap localCarLastLap;
	bool useLog;
	bool playerIsSpectator;
	std::basic_ofstream<wchar_t,std::char_traits<wchar_t> > log;
	ClientQOSData qos;
	ACClient_ClientEndSession endSession;
	std::vector<ClientCollisionEvent,std::allocator<ClientCollisionEvent> > clientColissionEvents;
	double lastClientEventSendTime;
	inline ACClient()  { }
	inline void ctor(Sim * isim) { typedef void (*_fpt)(ACClient *pthis, Sim *); _fpt _f=(_fpt)_drva(238272); _f(this, isim); }
	virtual ~ACClient();
	inline void dtor() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(244880); _f(this); }
	virtual void renderHUD_vf3(float dt);
	inline void renderHUD_impl(float dt) { typedef void (*_fpt)(ACClient *pthis, float); _fpt _f=(_fpt)_drva(333392); return _f(this, dt); }
	inline void renderHUD(float dt) { return renderHUD_vf3(dt); }
	inline ClientHandshakeResult handshakeTCP() { typedef ClientHandshakeResult (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(297808); return _f(this); }
	inline bool getRemoteCarList(std::vector<ClientRemoteCarDef,std::allocator<ClientRemoteCarDef> > & remoteCars) { typedef bool (*_fpt)(ACClient *pthis, std::vector<ClientRemoteCarDef,std::allocator<ClientRemoteCarDef> > &); _fpt _f=(_fpt)_drva(290688); return _f(this, remoteCars); }
	inline void beginUpdateMode() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(280288); return _f(this); }
	inline void sendChat(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & message) { typedef void (*_fpt)(ACClient *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(338576); return _f(this, message); }
	inline void sendVote(VoteType aVote, bool aVoteValue, int targetAvatarGUID) { typedef void (*_fpt)(ACClient *pthis, VoteType, bool, int); _fpt _f=(_fpt)_drva(338848); return _f(this, aVote, aVoteValue, targetAvatarGUID); }
	inline bool hasVoted() { typedef bool (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(306432); return _f(this); }
	inline CarAvatar * getCurrentVotingTarget() { typedef CarAvatar * (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(287824); return _f(this); }
	inline float getVotingTimeLeft() { typedef float (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(296512); return _f(this); }
	inline VoteType getCurrentVoteType() { typedef VoteType (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(287808); return _f(this); }
	inline float getVotingMaxTime() { typedef float (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(296496); return _f(this); }
	inline void addNetCar(NetCarStateProvider * nc) { typedef void (*_fpt)(ACClient *pthis, NetCarStateProvider *); _fpt _f=(_fpt)_drva(279232); return _f(this, nc); }
	inline NetCarStateProvider * getNetCarFromSessionID(unsigned char id) { typedef NetCarStateProvider * (*_fpt)(ACClient *pthis, unsigned char); _fpt _f=(_fpt)_drva(290528); return _f(this, id); }
	virtual void update_vf1(float deltaT);
	inline void update_impl(float deltaT) { typedef void (*_fpt)(ACClient *pthis, float); _fpt _f=(_fpt)_drva(342832); return _f(this, deltaT); }
	inline void update(float deltaT) { return update_vf1(deltaT); }
	inline double getPhysicsTime() { typedef double (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(290576); return _f(this); }
	inline bool isAllowedToSendChatMessage() { typedef bool (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(307616); return _f(this); }
	inline int getPing(CarAvatar * anAvatar) { typedef int (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(290608); return _f(this, anAvatar); }
	inline RemoteSession getCurrentSession() { typedef RemoteSession (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(287648); return _f(this); }
	inline RemoteSessionResult & getCurrentSessionResults() { typedef RemoteSessionResult & (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(287792); return _f(this); }
	inline CarAvatar * getCarAvatarFromSessionID(unsigned char sesid) { typedef CarAvatar * (*_fpt)(ACClient *pthis, unsigned char); _fpt _f=(_fpt)_drva(286720); return _f(this, sesid); }
	inline int getSessionIDFromCarAvatar(CarAvatar * anAvatar) { typedef int (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(295696); return _f(this, anAvatar); }
	inline int getSessionCount() { typedef int (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(295664); return _f(this); }
	inline int getCurrentSessionIndex() { typedef int (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(287696); return _f(this); }
	inline RemoteSession getSessionInfo(int index) { typedef RemoteSession (*_fpt)(ACClient *pthis, int); _fpt _f=(_fpt)_drva(295936); return _f(this, index); }
	virtual void shutdown_vf5();
	inline void shutdown_impl() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(339248); return _f(this); }
	inline void shutdown() { return shutdown_vf5(); }
	inline double getSessionTimeLeft() { typedef double (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(296096); return _f(this); }
	inline int getHandshakeCarPosition() { typedef int (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(1278848); return _f(this); }
	inline unsigned int getCarPosition(CarAvatar * car) { typedef unsigned int (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(287168); return _f(this, car); }
	inline unsigned int getBestLap(CarAvatar * car) { typedef unsigned int (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(286272); return _f(this, car); }
	inline unsigned int getInstanceBestLap(CarAvatar * car) { typedef unsigned int (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(287856); return _f(this, car); }
	inline Lap getLastLap(CarAvatar * car) { typedef Lap (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(290000); return _f(this, car); }
	inline unsigned int getLapCount(CarAvatar * car) { typedef unsigned int (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(287968); return _f(this, car); }
	inline void getLaps(CarAvatar * car, std::vector<Lap,std::allocator<Lap> > & laps) { typedef void (*_fpt)(ACClient *pthis, CarAvatar *, std::vector<Lap,std::allocator<Lap> > &); _fpt _f=(_fpt)_drva(288160); return _f(this, car, laps); }
	inline bool isBestSplit(int & sector, int & t, bool & isGlobal, CarAvatar * car) { typedef bool (*_fpt)(ACClient *pthis, int &, int &, bool &, CarAvatar *); _fpt _f=(_fpt)_drva(307648); return _f(this, sector, t, isGlobal, car); }
	inline unsigned int getBestSplit(int & sector, bool & isGlobal, CarAvatar * car) { typedef unsigned int (*_fpt)(ACClient *pthis, int &, bool &, CarAvatar *); _fpt _f=(_fpt)_drva(286384); return _f(this, sector, isGlobal, car); }
	inline unsigned int getLastSplit(CarAvatar * car, int & sector) { typedef unsigned int (*_fpt)(ACClient *pthis, CarAvatar *, int &); _fpt _f=(_fpt)_drva(290336); return _f(this, car, sector); }
	inline unsigned int getSplit(CarAvatar * car, int & sector) { typedef unsigned int (*_fpt)(ACClient *pthis, CarAvatar *, int &); _fpt _f=(_fpt)_drva(296160); return _f(this, car, sector); }
	inline void addSplitToBest(std::vector<unsigned int,std::allocator<unsigned int> > & splits, std::vector<unsigned int,std::allocator<unsigned int> > & personalSplits, unsigned int & sector, unsigned int & time, unsigned int & cuts) { typedef void (*_fpt)(ACClient *pthis, std::vector<unsigned int,std::allocator<unsigned int> > &, std::vector<unsigned int,std::allocator<unsigned int> > &, unsigned int &, unsigned int &, unsigned int &); _fpt _f=(_fpt)_drva(279264); return _f(this, splits, personalSplits, sector, time, cuts); }
	inline void storeSplitToLap(std::vector<unsigned int,std::allocator<unsigned int> > & splits, std::vector<unsigned int,std::allocator<unsigned int> > & personalSplits, std::vector<Lap,std::allocator<Lap> > & laps, unsigned int & laptime, unsigned int & cuts) { typedef void (*_fpt)(ACClient *pthis, std::vector<unsigned int,std::allocator<unsigned int> > &, std::vector<unsigned int,std::allocator<unsigned int> > &, std::vector<Lap,std::allocator<Lap> > &, unsigned int &, unsigned int &); _fpt _f=(_fpt)_drva(339488); return _f(this, splits, personalSplits, laps, laptime, cuts); }
	inline Lap getCurrentLap(CarAvatar * car) { typedef Lap (*_fpt)(ACClient *pthis, CarAvatar *); _fpt _f=(_fpt)_drva(287360); return _f(this, car); }
	inline void resetCurrentLaps() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(334128); return _f(this); }
	inline std::vector<unsigned char,std::allocator<unsigned char> > getCarMD5(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & unix_name) { typedef std::vector<unsigned char,std::allocator<unsigned char> > (*_fpt)(ACClient *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(286832); return _f(this, unix_name); }
	inline double getTimetoWait() { typedef double (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(296448); return _f(this); }
	inline void setInvertedGridPositions(int pos) { typedef void (*_fpt)(ACClient *pthis, int); _fpt _f=(_fpt)_drva(1315584); return _f(this, pos); }
	inline int getInvertedGridPositions() { typedef int (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(1279296); return _f(this); }
	inline void askForP2Pvalue() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(279664); return _f(this); }
	inline void sendCarPosition() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(336432); return _f(this); }
	inline void onMessage(UDPMessage & msg) { typedef void (*_fpt)(ACClient *pthis, UDPMessage &); _fpt _f=(_fpt)_drva(309712); return _f(this, msg); }
	inline void onLapCompleted(OnLapCompletedEvent & ev) { typedef void (*_fpt)(ACClient *pthis, OnLapCompletedEvent &); _fpt _f=(_fpt)_drva(309296); return _f(this, ev); }
	inline void onNewSession(UDPPacket & pak, bool isHandShake) { typedef void (*_fpt)(ACClient *pthis, UDPPacket &, bool); _fpt _f=(_fpt)_drva(320672); return _f(this, pak, isHandShake); }
	inline void onRemoteLapCompleted(UDPPacket & pak) { typedef void (*_fpt)(ACClient *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(324720); return _f(this, pak); }
	inline void onRemoteSectorSplit(UDPPacket & pak) { typedef void (*_fpt)(ACClient *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(327104); return _f(this, pak); }
	inline void onPhysicsStep(double pt) { typedef void (*_fpt)(ACClient *pthis, double); _fpt _f=(_fpt)_drva(324048); return _f(this, pt); }
	inline void logMessage(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & message) { typedef void (*_fpt)(ACClient *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(307872); return _f(this, message); }
	inline unsigned char getSessionIdFromCarAvatarID(int carAvatarID) { typedef unsigned char (*_fpt)(ACClient *pthis, int); _fpt _f=(_fpt)_drva(295776); return _f(this, carAvatarID); }
	inline void updateQOS(double pt) { typedef void (*_fpt)(ACClient *pthis, double); _fpt _f=(_fpt)_drva(344528); return _f(this, pt); }
	inline void associate() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(280128); return _f(this); }
	inline void onMessageTCP(UDPMessage & msg) { typedef void (*_fpt)(ACClient *pthis, UDPMessage &); _fpt _f=(_fpt)_drva(313408); return _f(this, msg); }
	inline bool handleLocalAdminMessages(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * msg) { typedef bool (*_fpt)(ACClient *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(296560); return _f(this, msg); }
	inline void onTyreCompoundChanged(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & shortName) { typedef void (*_fpt)(ACClient *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(328656); return _f(this, shortName); }
	inline void addCollisionEvent(NetCarStateProvider * netCar, float speed, vec3f & worldPos, vec3f & relPos) { typedef void (*_fpt)(ACClient *pthis, NetCarStateProvider *, float, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(277152); return _f(this, netCar, speed, worldPos, relPos); }
	inline void onWelcomeMessageReceived(UDPPacket & pak) { typedef void (*_fpt)(ACClient *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(328800); return _f(this, pak); }
	inline void onSetupReceived(UDPPacket & pak) { typedef void (*_fpt)(ACClient *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(327824); return _f(this, pak); }
	inline void onDRSZoneReceived(UDPPacket & pak) { typedef void (*_fpt)(ACClient *pthis, UDPPacket &); _fpt _f=(_fpt)_drva(308512); return _f(this, pak); }
	inline void onRemoteSunAngleReceived(float angle) { typedef void (*_fpt)(ACClient *pthis, float); _fpt _f=(_fpt)_drva(327408); return _f(this, angle); }
	inline void updateDamageReport() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(344160); return _f(this); }
	inline void onSectorSplit(OnSectorSplitEvent & ev) { typedef void (*_fpt)(ACClient *pthis, OnSectorSplitEvent &); _fpt _f=(_fpt)_drva(327536); return _f(this, ev); }
	inline void onCollisionWithCar() { typedef void (*_fpt)(ACClient *pthis); _fpt _f=(_fpt)_drva(308080); return _f(this); }
};

struct AeroMap {
public:
	float referenceArea;
	float CD;
	float CL;
	float frontShare;
	float CDX;
	float CDY;
	float CDA;
	IRigidBody * carBody;
	float dynamicCD;
	float dynamicCL;
	float airDensity;
	std::vector<Wing,std::allocator<Wing> > wings;
	vec3f frontApplicationPoint;
	vec3f rearApplicationPoint;
	Car * car;
	inline AeroMap()  { }
	inline void dtor() { typedef void (*_fpt)(AeroMap *pthis); _fpt _f=(_fpt)_drva(2839440); _f(this); }
	inline void init(Car * a_car, vec3f & frontAP, vec3f & rearAP, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * dataPath) { typedef void (*_fpt)(AeroMap *pthis, Car *, vec3f &, vec3f &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(2841760); return _f(this, a_car, frontAP, rearAP, dataPath); }
	inline void step(float dt) { typedef void (*_fpt)(AeroMap *pthis, float); _fpt _f=(_fpt)_drva(2847056); return _f(this, dt); }
	inline std::vector<WingState,std::allocator<WingState> > getWingStatus() { typedef std::vector<WingState,std::allocator<WingState> > (*_fpt)(AeroMap *pthis); _fpt _f=(_fpt)_drva(2841536); return _f(this); }
	inline float getCurrentLiftKG() { typedef float (*_fpt)(AeroMap *pthis); _fpt _f=(_fpt)_drva(2841488); return _f(this); }
	inline float getCurrentDragKG() { typedef float (*_fpt)(AeroMap *pthis); _fpt _f=(_fpt)_drva(2841440); return _f(this); }
	inline void addDrag(vec3f & lv) { typedef void (*_fpt)(AeroMap *pthis, vec3f &); _fpt _f=(_fpt)_drva(2840672); return _f(this, lv); }
	inline void addLift(vec3f & localVelocity) { typedef void (*_fpt)(AeroMap *pthis, vec3f &); _fpt _f=(_fpt)_drva(2841232); return _f(this, localVelocity); }
	inline void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataPath) { typedef void (*_fpt)(AeroMap *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2841936); return _f(this, dataPath); }
};

struct Drivetrain {
public:
	bool isGearGrinding;
	float finalRatio;
	GearElement engine;
	GearElement drive;
	GearElement outShaftL;
	GearElement outShaftR;
	GearElement outShaftLF;
	GearElement outShaftRF;
	std::vector<SGearRatio,std::allocator<SGearRatio> > gears;
	double rootVelocity;
	bool clutchOpenState;
	double ratio;
	float diffPowerRamp;
	float diffCoastRamp;
	float diffPreLoad;
	DifferentialType diffType;
	double cutOff;
	Engine acEngine;
	bool isShifterSupported;
	double clutchMaxTorque;
	float totalTorque;
	float awdFrontShare;
	DifferentialSetting awdFrontDiff;
	DifferentialSetting awdRearDiff;
	DifferentialSetting awdCenterDiff;
	DownshiftProtection downshiftProtection;
	AWD2Data awd2;
	float currentClutchTorque;
	std::function<void __cdecl(void)> downshiftProtectionFunction;
	Event<OnGearRequestEvent> evOnGearRequest;
	Car * car;
	TractionType tractionType;
	int currentGear;
	double lastRatio;
	Tyre * tyreLeft;
	Tyre * tyreRight;
	GearRequestStatus gearRequest;
	double gearUpTime;
	double gearDnTime;
	double autoCutOffTime;
	double validShiftRPMWindow;
	double controlsWindowGain;
	std::vector<ITorqueGenerator *,std::allocator<ITorqueGenerator *> > wheelTorqueGenerators;
	double damageRpmWindow;
	double orgRpmWindow;
	int lockCounter[0x4];
	DrivetrainControllers controllers;
	float clutchInertia;
	float locClutch;
	inline Drivetrain()  { }
	inline void dtor() { typedef void (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2514976); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(Drivetrain *pthis, Car *); _fpt _f=(_fpt)_drva(2518464); return _f(this, car); }
	inline void reset() { typedef void (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2527872); return _f(this); }
	inline void addGear(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, double ratio) { typedef void (*_fpt)(Drivetrain *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, double); _fpt _f=(_fpt)_drva(2516336); return _f(this, name, ratio); }
	inline void step(float dt) { typedef void (*_fpt)(Drivetrain *pthis, float); _fpt _f=(_fpt)_drva(2535728); return _f(this, dt); }
	inline bool hasDynamicControllers() { typedef bool (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2518416); return _f(this); }
	inline SGearRatio getCurrentGear() { typedef SGearRatio (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2517776); return _f(this); }
	inline void setCurrentGear(int index, bool force) { typedef void (*_fpt)(Drivetrain *pthis, int, bool); _fpt _f=(_fpt)_drva(2527968); return _f(this, index, force); }
	inline bool gearUp() { typedef bool (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2517488); return _f(this); }
	inline bool gearDown() { typedef bool (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2516576); return _f(this); }
	inline bool isChangingGear() { typedef bool (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2519984); return _f(this); }
	inline TractionType getTractionType() { typedef TractionType (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2608000); return _f(this); }
	inline float getEngineRPM() { typedef float (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2517888); return _f(this); }
	inline float getDrivetrainSpeed() { typedef float (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2517872); return _f(this); }
	inline void setGearRatio(int index, float value) { typedef void (*_fpt)(Drivetrain *pthis, int, float); _fpt _f=(_fpt)_drva(2528368); return _f(this, index, value); }
	inline void forceAutoCutoffTime(double value) { typedef void (*_fpt)(Drivetrain *pthis, double); _fpt _f=(_fpt)_drva(864032); return _f(this, value); }
	inline float getRpmWindowStatus() { typedef float (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2518384); return _f(this); }
	inline double getGearDnTime() { typedef double (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2741824); return _f(this); }
	inline float projectRPMAtDownshift() { typedef float (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2526944); return _f(this); }
	inline double getInertiaFromEngine() { typedef double (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2517920); return _f(this); }
	inline double getInertiaFromWheels() { typedef double (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2518048); return _f(this); }
	inline void reallignSpeeds(float dt) { typedef void (*_fpt)(Drivetrain *pthis, float); _fpt _f=(_fpt)_drva(2527488); return _f(this, dt); }
	inline void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataPath) { typedef void (*_fpt)(Drivetrain *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2520128); return _f(this, dataPath); }
	inline bool isGearboxLocked() { typedef bool (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2520000); return _f(this); }
	inline void accelerateDrivetrainBlock(double acc, bool fromEngine) { typedef void (*_fpt)(Drivetrain *pthis, double, bool); _fpt _f=(_fpt)_drva(2516160); return _f(this, acc, fromEngine); }
	inline void step2WD(float dt) { typedef void (*_fpt)(Drivetrain *pthis, float); _fpt _f=(_fpt)_drva(2528480); return _f(this, dt); }
	inline void step4WD(float dt) { typedef void (*_fpt)(Drivetrain *pthis, float); _fpt _f=(_fpt)_drva(2531872); return _f(this, dt); }
	inline void step4WD_new(float dt) { typedef void (*_fpt)(Drivetrain *pthis, float); _fpt _f=(_fpt)_drva(2534784); return _f(this, dt); }
	inline void initControllers() { typedef void (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2519152); return _f(this); }
	inline void stepControllers(float dt) { typedef void (*_fpt)(Drivetrain *pthis, float); _fpt _f=(_fpt)_drva(2535936); return _f(this, dt); }
	inline void ctor() { typedef void (*_fpt)(Drivetrain *pthis); _fpt _f=(_fpt)_drva(2545776); _f(this); }
};

class CarAvatar : public GameObject {
public:
	EventTriggerOnChange<int> evOnGearChanged;
	Event<OnLapCompletedEvent> evOnLapCompleted;
	Event<OnSectorSplitEvent> evOnSectorSplit;
	Event<OnTyreCompoundChanged> evOnTyreCompoundChanged;
	Event<bool> evOnBackfireTriggered;
	Event<bool> evOnDownshiftProtection;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > externalTyreCompoundShortName;
	vec3f mirrorPosition;
	Sim * sim;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > unixName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > configName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > guiName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > guiShortName;
	Node * bodyTransform;
	Node * steerTransformHR;
	Node * steerTransformLR;
	mat44f orgSteerMatrix;
	NodeBoundingSphere * carNode;
	vec3f driverEyesPosition;
	mat44f bodyMatrix;
	CarPhysicsState physicsState;
	AIState aiState;
	CarAudioFMOD * carAudioFMOD;
	SkidMarkBuffer * skidMarkBuffers[0x4];
	CarPhysicsInfo physicsInfo;
	std::unique_ptr<RaceEngineer,std::default_delete<RaceEngineer> > raceEngineer;
	ModelBoundariesCoordinates modelBoundaries;
	std::vector<WingState,std::allocator<WingState> > wingsStatus;
	float onBoardExposure;
	float outBoardExposure;
	float dashBoardExposure;
	std::vector<CameraCarDefinition,std::allocator<CameraCarDefinition> > cameras;
	bool isDriverHR;
	DriverModel * driverModel_HR;
	DriverModel * driverModel_LR;
	ISuspensionAvatar * suspensionAvatar;
	bool isBlackFlagged;
	NetCarStateProvider * netCarStateProvider;
	CarLodManager * lodManager;
	ConstrainedObjectsManager * constrainedObjectManager;
	BufferedChannel<std::pair<int,int> > chTyreCompound;
	float fuelInExhaust;
	vec3f graphicsOffset;
	float graphicsPitchRotation;
	float userFFGain;
	float graphicSteerLockDegrees;
	BackfireParams * backfireParams;
	Node * meshCollider;
	mat44f pitPosition;
	DriverInfo driverInfo;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > currentSkin;
	int guid;
	Node * modelLink;
	Car * physics;
	vec3f lastSkidPosition[0x4];
	std::vector<IEventTrigger *,std::allocator<IEventTrigger *> > eventTriggers;
	ICarPhysicsStateProvider * physicsStateProvider;
	bool lockVirtualSteer;
	bool hideArmsInCockpit;
	bool hideSteer;
	CarRaceInfo raceInfo;
	std::unique_ptr<PhysicsCarStateProvider,std::default_delete<PhysicsCarStateProvider> > physicsCarStateProvider;
	BufferedChannel<OnLapCompletedEvent> lapQueue;
	BufferedChannel<OnSectorSplitEvent> splitQueue;
	ICarPhysicsStateProvider * nonReplayPhysicsStateProvider;
	Node * beltOffNode;
	Node * beltOnNode;
	int currentTyreCompoundIndex[0x4];
	CarAnimations * carAnimations;
	float filteredSpeed;
	int currentEngineBrakeSetting;
	bool isHeatChargingBatteries;
	int currentERSPowerIndex;
	int currentERSRecovery;
	float lastERSBatteryCharge;
	float currentERSNormalizedRecharge;
	bool showDownShiftProtetion;
	bool mandatoryPitstopDone;
	float lastPitstopTime;
	bool inPitlane;
	bool wasInPitlane;
	float pitLaneEntryTime;
	float pitLaneExitTime;
	inline CarAvatar()  { }
	inline void ctor(Sim * isim, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & unixName, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin) { typedef void (*_fpt)(CarAvatar *pthis, Sim *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(840096); _f(this, isim, unixName, config, skin); }
	inline void ctor(Sim * isim, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & unixName, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin, ICarPhysicsStateProvider * physicsStateProvider) { typedef void (*_fpt)(CarAvatar *pthis, Sim *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, ICarPhysicsStateProvider *); _fpt _f=(_fpt)_drva(843200); _f(this, isim, unixName, config, skin, physicsStateProvider); }
	virtual ~CarAvatar();
	inline void dtor() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(846608); _f(this); }
	inline void setP2PActivations(int activations) { typedef void (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(894352); return _f(this, activations); }
	inline void setP2PStartingActivations(int pos) { typedef void (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(894496); return _f(this, pos); }
	inline void setRestrictor(float v) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(894640); return _f(this, v); }
	inline float getRestrictor() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865792); return _f(this); }
	inline void makeBodyMatrix(mat44f & bm, mat44f & res) { typedef void (*_fpt)(CarAvatar *pthis, mat44f &, mat44f &); _fpt _f=(_fpt)_drva(888512); return _f(this, bm, res); }
	inline void setNewPhysicsState(CarPhysicsState & ps, float dt) { typedef void (*_fpt)(CarAvatar *pthis, CarPhysicsState &, float); _fpt _f=(_fpt)_drva(893984); return _f(this, ps, dt); }
	inline DriverModel * getActiveDriverModel() { typedef DriverModel * (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864432); return _f(this); }
	inline int getAILapsToComplete() { typedef int (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864400); return _f(this); }
	inline PitStopTime getPitstopTime(float fuel_requested, bool changeTyres, bool repairBody, bool repairEngine, bool repairSus, bool useRandomizer) { typedef PitStopTime (*_fpt)(CarAvatar *pthis, float, bool, bool, bool, bool, bool); _fpt _f=(_fpt)_drva(865648); return _f(this, fuel_requested, changeTyres, repairBody, repairEngine, repairSus, useRandomizer); }
	inline bool isRequestingPitStop() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(888176); return _f(this); }
	inline float modifyUserFFGain(float offset) { typedef float (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(889072); return _f(this, offset); }
	inline void setUserFFGain(float gain) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(896336); return _f(this, gain); }
	inline float getUserFFGain() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(866960); return _f(this); }
	inline bool isCurrentLapValid() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887632); return _f(this); }
	inline void setFFMult(float mult) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(893280); return _f(this, mult); }
	inline float getFFMult() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865424); return _f(this); }
	inline void setTurboBoost(float value) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(895456); return _f(this, value); }
	inline std::pair<unsigned int,unsigned int> getTCMode() { typedef std::pair<unsigned int,unsigned int> (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(866336); return _f(this); }
	inline std::pair<unsigned int,unsigned int> getABSMode() { typedef std::pair<unsigned int,unsigned int> (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864336); return _f(this); }
	inline bool isConnected() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887616); return _f(this); }
	inline bool isInPit() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887760); return _f(this); }
	inline void armFirstLap() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(860928); return _f(this); }
	inline void setTyreCompound(unsigned int index, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name) { typedef void (*_fpt)(CarAvatar *pthis, unsigned int, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(895952); return _f(this, index, name); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getTyreCompound(unsigned int index, bool fullname) { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(CarAvatar *pthis, unsigned int, bool); _fpt _f=(_fpt)_drva(866688); return _f(this, index, fullname); }
	inline unsigned int getTyreCompoundIndex(unsigned int tyre_index) { typedef unsigned int (*_fpt)(CarAvatar *pthis, unsigned int); _fpt _f=(_fpt)_drva(866944); return _f(this, tyre_index); }
	inline float getWingAngle(int wingIndex) { typedef float (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(866976); return _f(this, wingIndex); }
	inline CarColliderManager * getColliderManager() { typedef CarColliderManager * (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864576); return _f(this); }
	inline void setDamageLevel(float lvl) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(892960); return _f(this, lvl); }
	inline int getGuid() { typedef int (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865536); return _f(this); }
	virtual void update_vf1(float deltaT);
	inline void update_impl(float deltaT) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(899120); return _f(this, deltaT); }
	inline void update(float deltaT) { return update_vf1(deltaT); }
	inline void goToSpawnPosition(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName) { typedef void (*_fpt)(CarAvatar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(867040); return _f(this, setName); }
	inline bool isInSpawnPosition(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName) { typedef bool (*_fpt)(CarAvatar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(887872); return _f(this, setName); }
	inline void setSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName, int index) { typedef void (*_fpt)(CarAvatar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, int); _fpt _f=(_fpt)_drva(894816); return _f(this, setName, index); }
	inline void setControlsProvider(ICarControlsProvider * controls) { typedef void (*_fpt)(CarAvatar *pthis, ICarControlsProvider *); _fpt _f=(_fpt)_drva(892816); return _f(this, controls); }
	inline vec3f getGraphicsOffset() { typedef vec3f (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865504); return _f(this); }
	inline void setGraphicsPitchRotation(float v) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(893744); return _f(this, v); }
	inline void setGraphicsOffset(vec3f np) { typedef void (*_fpt)(CarAvatar *pthis, vec3f); _fpt _f=(_fpt)_drva(893712); return _f(this, np); }
	inline ICarControlsProvider * getControlsProvider() { typedef ICarControlsProvider * (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864608); return _f(this); }
	inline std::vector<DebugLine,std::allocator<DebugLine> > getSuspensionDebugLines(int index) { typedef std::vector<DebugLine,std::allocator<DebugLine> > (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(866256); return _f(this, index); }
	inline vec3f getRoadDirection() { typedef vec3f (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865840); return _f(this); }
	inline SetupManager * getSetupManager() { typedef SetupManager * (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(866064); return _f(this); }
	inline void onPostLoad() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(889520); return _f(this); }
	inline void onStartReplay(bool & mode) { typedef void (*_fpt)(CarAvatar *pthis, bool &); _fpt _f=(_fpt)_drva(889856); return _f(this, mode); }
	inline void onStopReplay(bool & mode) { typedef void (*_fpt)(CarAvatar *pthis, bool &); _fpt _f=(_fpt)_drva(890144); return _f(this, mode); }
	inline bool isAbsInAction() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887520); return _f(this); }
	inline void setAbsEnabled(bool mode, bool force) { typedef void (*_fpt)(CarAvatar *pthis, bool, bool); _fpt _f=(_fpt)_drva(891840); return _f(this, mode, force); }
	inline void cycleAbsMode(int value) { typedef void (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(862288); return _f(this, value); }
	inline bool isAbsAvailable() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887440); return _f(this); }
	inline bool isAbsEnabled() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887472); return _f(this); }
	inline bool isTcInAction() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(888288); return _f(this); }
	inline void setTcEnabled(bool mode, bool force) { typedef void (*_fpt)(CarAvatar *pthis, bool, bool); _fpt _f=(_fpt)_drva(895312); return _f(this, mode, force); }
	inline void cycleTcMode(int value) { typedef void (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(863584); return _f(this, value); }
	inline bool isTcEnabled() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(888240); return _f(this); }
	inline bool isTcAvailable() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(888208); return _f(this); }
	inline bool isEdlEnabled() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887712); return _f(this); }
	inline float getEdlOutLevel() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865392); return _f(this); }
	inline void setAutoBlip(bool mode) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(891984); return _f(this, mode); }
	inline bool getAutoBlip() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864464); return _f(this); }
	inline float getStabilityControl() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(866224); return _f(this); }
	inline void setStabilityControl(float gain) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(895168); return _f(this, gain); }
	inline void setFrontBias(int value) { typedef void (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(893424); return _f(this, value); }
	inline void setAutoShifter(bool mode) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(892240); return _f(this, mode); }
	inline bool getAutoShifter() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864496); return _f(this); }
	inline void setVisible(bool vis) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(896480); return _f(this, vis); }
	inline bool isVisible() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(888320); return _f(this); }
	inline void updateERSCharge() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(906032); return _f(this); }
	inline void setHeadlights(bool value) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(893760); return _f(this, value); }
	inline void resetPenalty(bool isTotalReset) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(891616); return _f(this, isTotalReset); }
	inline TimeTransponder * getTimeTransponder() { typedef TimeTransponder * (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(866656); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getScreenName() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865952); return _f(this); }
	inline void setAutoClutchEnabled(bool mode) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(892128); return _f(this, mode); }
	inline float getPackerRange(int index) { typedef float (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(865600); return _f(this, index); }
	inline void initGhostCar(bool isRecording, bool isPlaying) { typedef void (*_fpt)(CarAvatar *pthis, bool, bool); _fpt _f=(_fpt)_drva(881936); return _f(this, isRecording, isPlaying); }
	inline void setControlsLock(bool value) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(892672); return _f(this, value); }
	inline void setGentleStop(bool mode) { typedef void (*_fpt)(CarAvatar *pthis, bool); _fpt _f=(_fpt)_drva(893568); return _f(this, mode); }
	inline double hasPenalty() { typedef double (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(867184); return _f(this); }
	inline bool isMinSpeedPenaltyClearDisabled() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(888128); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getCurrentSkin() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(618160); return _f(this); }
	inline void resetTimeTransponder() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(891760); return _f(this); }
	inline void resetFlames() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(891440); return _f(this); }
	inline void setDriverInfo(DriverInfo & info) { typedef void (*_fpt)(CarAvatar *pthis, DriverInfo &); _fpt _f=(_fpt)_drva(893104); return _f(this, info); }
	inline DriverInfo & getDriverInfo() { typedef DriverInfo & (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(287840); return _f(this); }
	inline float getGraphicSteerDeg() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865456); return _f(this); }
	inline int getSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * setName) { typedef int (*_fpt)(CarAvatar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(866096); return _f(this, setName); }
	inline void setSlipStreamEffects(float receive, float generationSpeedFactor) { typedef void (*_fpt)(CarAvatar *pthis, float, float); _fpt _f=(_fpt)_drva(894784); return _f(this, receive, generationSpeedFactor); }
	inline double getKmPerLiter() { typedef double (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865552); return _f(this); }
	inline void setBlackFlag(PenaltyDescription descr) { typedef void (*_fpt)(CarAvatar *pthis, PenaltyDescription); _fpt _f=(_fpt)_drva(892528); return _f(this, descr); }
	inline void lockControlsUntilTime(double time, double start, bool forceToPits) { typedef void (*_fpt)(CarAvatar *pthis, double, double, bool); _fpt _f=(_fpt)_drva(888352); return _f(this, time, start, forceToPits); }
	inline bool isInPitlane() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(887856); return _f(this); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getCurrentCompoundShortName() { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864768); return _f(this); }
	inline int getCurrentCompoundIdealPressure(unsigned short tyreIndex) { typedef int (*_fpt)(CarAvatar *pthis, unsigned short); _fpt _f=(_fpt)_drva(864640); return _f(this, tyreIndex); }
	inline int getCurrentCompoundStaticPressure(unsigned short tyreIndex) { typedef int (*_fpt)(CarAvatar *pthis, unsigned short); _fpt _f=(_fpt)_drva(864944); return _f(this, tyreIndex); }
	inline void getDistancesOnSpline(CarAvatar & car, float * frontDistance, float * backDistance) { typedef void (*_fpt)(CarAvatar *pthis, CarAvatar &, float *, float *); _fpt _f=(_fpt)_drva(865072); return _f(this, car, frontDistance, backDistance); }
	inline double getTimeDifferenceOnSpline(CarAvatar & car) { typedef double (*_fpt)(CarAvatar *pthis, CarAvatar &); _fpt _f=(_fpt)_drva(866400); return _f(this, car); }
	inline float getBallastKG() { typedef float (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(864528); return _f(this); }
	inline void setBallastKG(float ballast) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(892384); return _f(this, ballast); }
	inline void setMultVolume(int value, bool active) { typedef void (*_fpt)(CarAvatar *pthis, int, bool); _fpt _f=(_fpt)_drva(893904); return _f(this, value, active); }
	inline void resetMultVolume() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(891568); return _f(this); }
	inline INIReader openINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef INIReader (*_fpt)(CarAvatar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(890416); return _f(this, filename); }
	inline int cycleEngineBrake(int value) { typedef int (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(863376); return _f(this, value); }
	inline bool cycleERSHeatCharging() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(862688); return _f(this); }
	inline std::pair<int,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > cycleERSPower(int value) { typedef std::pair<int,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(862848); return _f(this, value); }
	inline int cycleERSRecovery(int value) { typedef int (*_fpt)(CarAvatar *pthis, int); _fpt _f=(_fpt)_drva(863168); return _f(this, value); }
	inline std::pair<int,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > getERSPower() { typedef std::pair<int,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(865248); return _f(this); }
	inline bool getERSHeatCharging() { typedef bool (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(1243840); return _f(this); }
	inline void forcePosition(mat44f & matrix) { typedef void (*_fpt)(CarAvatar *pthis, mat44f &); _fpt _f=(_fpt)_drva(864048); return _f(this, matrix); }
	inline void initCommon() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(874208); return _f(this); }
	inline void initCommonPostPhysics() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(876944); return _f(this); }
	inline void init3D(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * skin) { typedef void (*_fpt)(CarAvatar *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *); _fpt _f=(_fpt)_drva(867216); return _f(this, skin); }
	inline void initPhysics() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(882272); return _f(this); }
	inline void updateSkidMarks(float dt) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(907136); return _f(this, dt); }
	inline void initDriver() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(881536); return _f(this); }
	inline void initCameraCar() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(870336); return _f(this); }
	inline void initMirrorMaterials() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(882160); return _f(this); }
	inline void initControls() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(880432); return _f(this); }
	inline void onNewSession() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(889280); return _f(this); }
	inline void updateFromChannels(float dt) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(906128); return _f(this, dt); }
	inline void checkACD() { typedef void (*_fpt)(CarAvatar *pthis); _fpt _f=(_fpt)_drva(861184); return _f(this); }
	inline void updateInPitlaneState(float dt) { typedef void (*_fpt)(CarAvatar *pthis, float); _fpt _f=(_fpt)_drva(906960); return _f(this, dt); }
};

class Car {
public:
	float finalSteerAngleSignal;
	float powerClassIndex;
	Event<OnStepCompleteEvent> evOnStepComplete;
	Event<OnControlsProviderChanged> evOnControlsProviderChanged;
	Event<OnLapCompletedEvent> evOnLapCompleted;
	Event<OnSectorSplitEvent> evOnSectorSplit;
	Event<vec3f> evOnForcedPositionCompleted;
	Event<std::pair<int,int> > evOnTyreCompoundChanged;
	Event<OnCollisionEvent> evOnCollisionEvent;
	Event<double> evOnJumpStartEvent;
	Event<std::pair<int,int> > evOnPush2Pass;
	float carHalfWidth;
	float userFFGain;
	bool isRetired;
	std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > tyreCompounds;
	void * tag;
	IRigidBody * body;
	IRigidBody * fuelTankBody;
	IRigidBody * rigidAxle;
	IJoint * fuelTankJoint;
	PhysicsEngine * ksPhysics;
	CarControls controls;
	float steerLock;
	float steerRatio;
	float steerLinearRatio;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > unixName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > configName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > carDataPath;
	float mass;
	float ffMult;
	vec3f accG;
	TimeTransponder transponder;
	CarColliderManager colliderManager;
	Drivetrain drivetrain;
	ABS abs;
	TractionControl tractionControl;
	SpeedLimiter speedLimiter;
	AeroMap aeroMap;
	Tyre tyres[0x4];
	std::vector<ISuspension *,std::allocator<ISuspension *> > suspensions;
	BrakeSystem brakeSystem;
	Autoclutch autoClutch;
	unsigned int physicsGUID;
	Telemetry telemetry;
	AutoBlip autoBlip;
	AutoShifter autoShift;
	GearChanger gearChanger;
	EDL edl;
	AntirollBar antirollBars[0x2];
	StabilityControl stabilityControl;
	double lastCollisionTime;
	DriftModeComponent driftMode;
	PerformanceMeter performanceMeter;
	SetupManager setupManager;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > screenName;
	DRS drs;
	Kers kers;
	ERS ers;
	LapInvalidator lapInvalidator;
	PenaltyManager penaltyManager;
	bool isGearboxLocked;
	bool isGentleStopping;
	double penaltyPerfTarget;
	mat44f pitPosition;
	SplineLocatorData splineLocatorData;
	FuelLapEvaluator fuelLapEvaluator;
	HeaveSpring heaveSprings[0x2];
	SteeringSystem steeringSystem;
	double lastCollisionWithCarTime;
	SuspensionType suspensionTypeF;
	SuspensionType suspensionTypeR;
	float axleTorqueReaction;
	float lastGyroFF;
	float lastFF;
	float lastPureMZFF;
	ThermalObject water;
	float steerAssist;
	bool isRequestingPitStop;
	int aiLapsToComplete;
	bool lightsOn;
	CarCollisionBounds bounds;
	SlipStream slipStream;
	TorqueModeEX torqueModeEx;
	bool isControlsLocked;
	double lockControlsTime;
	bool blackFlagged;
	double penaltyTimeAccumulator;
	double penaltyTime;
	float expectedFuelPerLap;
	mat44f meshColliderBodyMatrix;
	ICarControlsProvider * controlsProvider;
	vec3f lastVelocity;
	int sleepingFrames;
	float mzCurrent;
	vec3f bodyInertia;
	vec3f explicitInertia;
	double fuel;
	double fuelConsumptionK;
	double maxFuel;
	float requestedFuel;
	double lastBodyMassUpdateTime;
	float damageZoneLevel[0x5];
	vec3f ridePickupPoint[0x2];
	vec3f fuelTankPos;
	float vibrationPhase;
	float slipVibrationPhase;
	float flatSpotPhase;
	float slipStreamEffectGain;
	int framesToSleep;
	bool disableMinSpeedPenaltyClear;
	float ballastKG;
	float lastSteerPosition;
	SplineLocator splineLocator;
	bool isCollisionOffForPits;
	PitStopTimings pitTimings;
	vec3f gridPosition;
	bool hasGridPosition;
	bool lastLigthSwitchState;
	PhysicsValueCache valueCache;
	float fuelKG;
	bool externalControl;
	inline Car()  { }
	inline void ctor(PhysicsEngine * iengine, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iunixName, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config) { typedef void (*_fpt)(Car *pthis, PhysicsEngine *, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2539264); _f(this, iengine, iunixName, config); }
	virtual ~Car();
	inline void dtor() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2547920); _f(this); }
	inline void stepPreCacheValues(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2582720); return _f(this, dt); }
	inline void setBlackFlag(bool isflagged, PenaltyDescription type) { typedef void (*_fpt)(Car *pthis, bool, PenaltyDescription); _fpt _f=(_fpt)_drva(2578928); return _f(this, isflagged, type); }
	inline int setFuelForLaps(int laps, float mult) { typedef int (*_fpt)(Car *pthis, int, float); _fpt _f=(_fpt)_drva(2579296); return _f(this, laps, mult); }
	inline float getFuelPerLap() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2558928); return _f(this); }
	inline float getBackDistanceFromCar(Car * car) { typedef float (*_fpt)(Car *pthis, Car *); _fpt _f=(_fpt)_drva(2556768); return _f(this, car); }
	inline bool isInPitLane() { typedef bool (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2573536); return _f(this); }
	inline bool isInPits() { typedef bool (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2573616); return _f(this); }
	inline void setDamageLevel(float v, int index) { typedef void (*_fpt)(Car *pthis, float, int); _fpt _f=(_fpt)_drva(2579280); return _f(this, v, index); }
	inline void setDamageLevel(float v) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2579232); return _f(this, v); }
	inline void resetSuspensionDamageLevel() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2578800); return _f(this); }
	inline void onCollisionCallBack(void * userData0, void * shape0, void * userData1, void * shape1, vec3f normal, vec3f pos, float depth) { typedef void (*_fpt)(Car *pthis, void *, void *, void *, void *, vec3f, vec3f, float); _fpt _f=(_fpt)_drva(2573904); return _f(this, userData0, shape0, userData1, shape1, normal, pos, depth); }
	inline void reset() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2578656); return _f(this); }
	inline void getTyreThermalState(int index, TyreThermalState * state) { typedef void (*_fpt)(Car *pthis, int, TyreThermalState *); _fpt _f=(_fpt)_drva(2566064); return _f(this, index, state); }
	inline bool isSleeping() { typedef bool (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2573792); return _f(this); }
	inline void step(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2579872); return _f(this, dt); }
	inline void postStep(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2577456); return _f(this, dt); }
	inline void getPhysicsState(CarPhysicsState & state) { typedef void (*_fpt)(Car *pthis, CarPhysicsState &); _fpt _f=(_fpt)_drva(2559344); return _f(this, state); }
	inline void getAIState(AIState * state) { typedef void (*_fpt)(Car *pthis, AIState *); _fpt _f=(_fpt)_drva(2556480); return _f(this, state); }
	inline void setControllerProvider(ICarControlsProvider * cp) { typedef void (*_fpt)(Car *pthis, ICarControlsProvider *); _fpt _f=(_fpt)_drva(2579056); return _f(this, cp); }
	inline Speed getSpeed() { typedef Speed (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2564448); return _f(this); }
	inline float getCGHeight() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2556848); return _f(this); }
	inline vec3f getVelocity() { typedef vec3f (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2566464); return _f(this); }
	inline vec3f getLocalVelocity() { typedef vec3f (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2559264); return _f(this); }
	inline float getSteerFF() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2564480); return _f(this); }
	inline vec3f getLocalAngularVelocity() { typedef vec3f (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2559216); return _f(this); }
	inline ICarControlsProvider * getControlsProvider() { typedef ICarControlsProvider * (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2557520); return _f(this); }
	inline float getTotalMass(bool withFuel) { typedef float (*_fpt)(Car *pthis, bool); _fpt _f=(_fpt)_drva(2565488); return _f(this, withFuel); }
	inline mat44f getTyreMatrix(int index) { typedef mat44f (*_fpt)(Car *pthis, int); _fpt _f=(_fpt)_drva(2565792); return _f(this, index); }
	inline mat44f getSuspensionMatrix(unsigned int index) { typedef mat44f (*_fpt)(Car *pthis, unsigned int); _fpt _f=(_fpt)_drva(2565360); return _f(this, index); }
	inline void getWingState(std::vector<WingState,std::allocator<WingState> > & ws) { typedef void (*_fpt)(Car *pthis, std::vector<WingState,std::allocator<WingState> > &); _fpt _f=(_fpt)_drva(2566688); return _f(this, ws); }
	inline void forcePosition(vec3f & pos, bool invalidateTransponder) { typedef void (*_fpt)(Car *pthis, vec3f &, bool); _fpt _f=(_fpt)_drva(2555408); return _f(this, pos, invalidateTransponder); }
	inline void forceRotation(vec3f & ihed) { typedef void (*_fpt)(Car *pthis, vec3f &); _fpt _f=(_fpt)_drva(2555968); return _f(this, ihed); }
	inline void setRequestedFuel(float value, bool changeValue) { typedef void (*_fpt)(Car *pthis, float, bool); _fpt _f=(_fpt)_drva(2579632); return _f(this, value, changeValue); }
	inline void initColliderMesh(Mesh * mesh, mat44f & bodyMatrix) { typedef void (*_fpt)(Car *pthis, Mesh *, mat44f &); _fpt _f=(_fpt)_drva(2571040); return _f(this, mesh, bodyMatrix); }
	inline float computeRideHeight(int zone, plane4f & groundPlane) { typedef float (*_fpt)(Car *pthis, int, plane4f &); _fpt _f=(_fpt)_drva(2555024); return _f(this, zone, groundPlane); }
	inline void setSlipStreamEffects(float receive, float generationSpeedFactor) { typedef void (*_fpt)(Car *pthis, float, float); _fpt _f=(_fpt)_drva(2579776); return _f(this, receive, generationSpeedFactor); }
	inline float getFrontWheelAngle() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2558352); return _f(this); }
	inline void addPenalty(double ptime) { typedef void (*_fpt)(Car *pthis, double); _fpt _f=(_fpt)_drva(2553504); return _f(this, ptime); }
	inline void clearPenalty() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2554960); return _f(this); }
	inline bool isMinSpeedPenaltyClearDisabled() { typedef bool (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2573776); return _f(this); }
	inline double getPenaltyTime() { typedef double (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2559312); return _f(this); }
	inline void lockControlsUntil(double time, double start) { typedef void (*_fpt)(Car *pthis, double, double); _fpt _f=(_fpt)_drva(2573824); return _f(this, time, start); }
	inline void lockControls(bool lock) { typedef void (*_fpt)(Car *pthis, bool); _fpt _f=(_fpt)_drva(2573808); return _f(this, lock); }
	inline void setBallastKG(float kg) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2578912); return _f(this, kg); }
	inline float getRestrictor() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2564416); return _f(this); }
	inline void setRestrictor(float value) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2579728); return _f(this, value); }
	inline std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getConfigPath(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename) { typedef std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > (*_fpt)(Car *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2556960); return _f(this, filename); }
	inline PitStopTime getPitstopTime(float fuel_to_add, bool changeTyres, bool repairBody, bool repairEngine, bool repairSus, bool useRandomizer) { typedef PitStopTime (*_fpt)(Car *pthis, float, bool, bool, bool, bool, bool); _fpt _f=(_fpt)_drva(2563680); return _f(this, fuel_to_add, changeTyres, repairBody, repairEngine, repairSus, useRandomizer); }
	inline float getWheelSterAngleDEG(int index) { typedef float (*_fpt)(Car *pthis, int); _fpt _f=(_fpt)_drva(2566496); return _f(this, index); }
	inline float getFinalFF() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2558304); return _f(this); }
	inline void getEngagement(float engagement_length, float * left, float * right) { typedef void (*_fpt)(Car *pthis, float, float *, float *); _fpt _f=(_fpt)_drva(2557536); return _f(this, engagement_length, left, right); }
	inline void resetSplineLocator() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2578784); return _f(this); }
	inline float getTotalKM() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2565456); return _f(this); }
	inline vec3f getGroundWindVector() { typedef vec3f (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2558976); return _f(this); }
	inline void pollControls(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2575984); return _f(this, dt); }
	inline void initCarData() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2566960); return _f(this); }
	inline void initCarDataPath() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2570688); return _f(this); }
	inline void buildARBS() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2553680); return _f(this); }
	inline void initAeroMap() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2566784); return _f(this); }
	inline float calcBodyMass() { typedef float (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2554736); return _f(this); }
	inline void updateBodyMass() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2583664); return _f(this); }
	inline void updateAirPressure() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2583264); return _f(this); }
	inline void initHeaveSprings() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2572096); return _f(this); }
	inline void onTyresStepCompleted() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2575568); return _f(this); }
	inline void onNewSession(SessionInfo & si) { typedef void (*_fpt)(Car *pthis, SessionInfo &); _fpt _f=(_fpt)_drva(2575408); return _f(this, si); }
	inline void stepThermalObjects(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2583024); return _f(this, dt); }
	inline void stepComponents(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2581712); return _f(this, dt); }
	inline void updateColliderStatus(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2584048); return _f(this, dt); }
	inline void initPitstopTimings() { typedef void (*_fpt)(Car *pthis); _fpt _f=(_fpt)_drva(2572320); return _f(this); }
	inline void stepJumpStart(float dt) { typedef void (*_fpt)(Car *pthis, float); _fpt _f=(_fpt)_drva(2582400); return _f(this, dt); }
};

