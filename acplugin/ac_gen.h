// ### AUTO-GENERATED ###

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

enum class eTimeLineCheckResponse {
	eOutOfRange = 0x0,
	eNegativeSide = 0x1,
	ePositiveSide = 0x2,
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

enum class DifferentialType {
	LSD = 0x0,
	Spool = 0x1,
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

enum class MouseButton {
	Unknown = 0xFF,
	Left = 0x0,
	Middle = 0x1,
	Right = 0x2,
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

enum class DRWWingConnectionMode {
	UseEffect = 0x0,
	UseAngle = 0x1,
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

enum class ksgui_eArrowsDirection {
	eLeft = 0x0,
	eRight = 0x1,
	eUp = 0x2,
	eDown = 0x3,
};

enum class FindTyreCompoundLogic {
	Random = 0x0,
	Fastest = 0x1,
	Preferred = 0x2,
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
class Car;
struct SVar;
class ESCMenu;
class ksgui_Spinner;
struct TyreThermalPatch;
class CarAvatar;
class ksgui_Control;
class ksgui_CheckBox;
class ksgui_ScrollBar;
class ksgui_Slider;
class ICarControlsProvider;
class Wing;
struct HWND__;
struct HINSTANCE__;
class ksgui_ListBox;
struct ksgui_ListBoxRowData;
class GraphicsManager;
class Shader;
class Game;
class GameObject;
class RaceManager;
class ICollisionObject;
class NetCarStateProvider;
class IRigidBody;
class Task;
class Suspension;
class AISpline;
class Track;
class SetupItem;
class Sim;
struct ACCarState;
struct ACPluginContext;
class Joypad;
struct TimeTransponder;
class Font;
class ConsoleCommand;
class IVarCallback;
class PhysicsEngine;
class ISphereCollisionCallback;
class IRayTrackCollisionProvider;
class ACClient;
class PhysicsAvatar;
class Material;
struct CarPhysicsState;
class IJoint;
class ICollisionCallback;
class IRayCaster;
class Node;
struct CompileContext;
struct RenderContext;
class ksgui_GUI;
class GLRenderer;
class Turbo;
class ISuspension;
class IKeyEventListener;
class ksgui_TextBox;
class ksgui_Label;
class ksgui_ActiveButton;
class ksgui_MovingBar;
struct Tyre;
class ksgui_Form;
class ksgui_TaskBarIcon;
class ksgui_PopOver;
class IndexBuffer;
struct MeshVertex;
class ksgui_ListBoxRow;
class ITyreModel;
struct SurfaceDef;
struct RayCastResult;
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
class CBuffer;
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

//UDT: struct ACClient::ClientEndSession @len=12
	//_Data: this+0x0, Member, Type: bool, isConnectionFinished
	//_Data: this+0x4, Member, Type: float, sendToPitsTimer
	//_Data: this+0x8, Member, Type: float, shutdownTimer
	//_Func: public void ClientEndSession(); @loc=optimized @len=0 @rva=0
//UDT;

struct ACClient_ClientEndSession {
public:
	bool isConnectionFinished;
	float sendToPitsTimer;
	float shutdownTimer;
	inline ACClient_ClientEndSession()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ACClient_ClientEndSession)==12),"bad size");
		static_assert((offsetof(ACClient_ClientEndSession,isConnectionFinished)==0x0),"bad off");
		static_assert((offsetof(ACClient_ClientEndSession,sendToPitsTimer)==0x4),"bad off");
		static_assert((offsetof(ACClient_ClientEndSession,shutdownTimer)==0x8),"bad off");
	};
};

//UDT: struct OnSectorSplitEvent @len=16
	//_Data: this+0x0, Member, Type: unsigned int, carIndex
	//_Data: this+0x4, Member, Type: unsigned int, sectorIndex
	//_Data: this+0x8, Member, Type: unsigned int, sectorTime
	//_Data: this+0xC, Member, Type: unsigned int, cuts
//UDT;

struct OnSectorSplitEvent {
public:
	unsigned int carIndex;
	unsigned int sectorIndex;
	unsigned int sectorTime;
	unsigned int cuts;
	inline OnSectorSplitEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnSectorSplitEvent)==16),"bad size");
		static_assert((offsetof(OnSectorSplitEvent,carIndex)==0x0),"bad off");
		static_assert((offsetof(OnSectorSplitEvent,sectorIndex)==0x4),"bad off");
		static_assert((offsetof(OnSectorSplitEvent,sectorTime)==0x8),"bad off");
		static_assert((offsetof(OnSectorSplitEvent,cuts)==0xC),"bad off");
	};
};

//UDT: struct BrushTyreModelData @len=28
	//_Func: public void BrushTyreModelData(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, CF
	//_Data: this+0x4, Member, Type: float, xu
	//_Data: this+0x8, Member, Type: float, CF1
	//_Data: this+0xC, Member, Type: float, Fz0
	//_Data: this+0x10, Member, Type: float, maxSlip0
	//_Data: this+0x14, Member, Type: float, maxSlip1
	//_Data: this+0x18, Member, Type: float, falloffSpeed
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(BrushTyreModelData)==28),"bad size");
		static_assert((offsetof(BrushTyreModelData,CF)==0x0),"bad off");
		static_assert((offsetof(BrushTyreModelData,xu)==0x4),"bad off");
		static_assert((offsetof(BrushTyreModelData,CF1)==0x8),"bad off");
		static_assert((offsetof(BrushTyreModelData,Fz0)==0xC),"bad off");
		static_assert((offsetof(BrushTyreModelData,maxSlip0)==0x10),"bad off");
		static_assert((offsetof(BrushTyreModelData,maxSlip1)==0x14),"bad off");
		static_assert((offsetof(BrushTyreModelData,falloffSpeed)==0x18),"bad off");
	};
};

//UDT: struct NetCarStateProvider::LagDebug @len=24
	//_Data: this+0x0, Member, Type: double, rcvTime
	//_Data: this+0x8, Member, Type: double, physicsTime
	//_Data: this+0x10, Member, Type: bool, wasLagging
	//_Func: public void LagDebug(); @loc=optimized @len=0 @rva=0
//UDT;

struct NetCarStateProvider_LagDebug {
public:
	double rcvTime;
	double physicsTime;
	bool wasLagging;
	inline NetCarStateProvider_LagDebug()  { }
	inline void _guard_obj() {
		static_assert((sizeof(NetCarStateProvider_LagDebug)==24),"bad size");
		static_assert((offsetof(NetCarStateProvider_LagDebug,rcvTime)==0x0),"bad off");
		static_assert((offsetof(NetCarStateProvider_LagDebug,physicsTime)==0x8),"bad off");
		static_assert((offsetof(NetCarStateProvider_LagDebug,wasLagging)==0x10),"bad off");
	};
};

//UDT: struct DRSWingSetting @len=8
	//_Data: this+0x0, Member, Type: int, index
	//_Data: this+0x4, Member, Type: float, angle
//UDT;

struct DRSWingSetting {
public:
	int index;
	float angle;
	inline DRSWingSetting()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DRSWingSetting)==8),"bad size");
		static_assert((offsetof(DRSWingSetting,index)==0x0),"bad off");
		static_assert((offsetof(DRSWingSetting,angle)==0x4),"bad off");
	};
};

//UDT: struct RendererFlags @len=8
	//_Func: public void RendererFlags(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, maxFrameLatency
	//_Data: this+0x4, Member, Type: float, mipLodBias
//UDT;

struct RendererFlags {
public:
	int maxFrameLatency;
	float mipLodBias;
	inline RendererFlags()  { }
	inline void _guard_obj() {
		static_assert((sizeof(RendererFlags)==8),"bad size");
		static_assert((offsetof(RendererFlags,maxFrameLatency)==0x0),"bad off");
		static_assert((offsetof(RendererFlags,mipLodBias)==0x4),"bad off");
	};
};

//UDT: struct RenderStats @len=20
	//_Func: public void RenderStats(); @loc=optimized @len=0 @rva=0
	//_Func: public void reset(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, dipCalls
	//_Data: this+0x4, Member, Type: int, sceneDipCalls
	//_Data: this+0x8, Member, Type: int, triangles
	//_Data: this+0xC, Member, Type: int, sceneTriangles
	//_Data: this+0x10, Member, Type: bool, isInMainRenderPass
//UDT;

struct RenderStats {
public:
	int dipCalls;
	int sceneDipCalls;
	int triangles;
	int sceneTriangles;
	bool isInMainRenderPass;
	inline RenderStats()  { }
	inline void _guard_obj() {
		static_assert((sizeof(RenderStats)==20),"bad size");
		static_assert((offsetof(RenderStats,dipCalls)==0x0),"bad off");
		static_assert((offsetof(RenderStats,sceneDipCalls)==0x4),"bad off");
		static_assert((offsetof(RenderStats,triangles)==0x8),"bad off");
		static_assert((offsetof(RenderStats,sceneTriangles)==0xC),"bad off");
		static_assert((offsetof(RenderStats,isInMainRenderPass)==0x10),"bad off");
	};
};

//UDT: struct ksgui::ksRect @len=16
	//_Func: public void ksRect(); @loc=optimized @len=0 @rva=0
	//_Func: public void ksRect(float  _arg0, float  _arg1, float  _arg2, float  _arg3); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, left
	//_Data: this+0x4, Member, Type: float, right
	//_Data: this+0x8, Member, Type: float, top
	//_Data: this+0xC, Member, Type: float, bottom
	//_Func: public float getWidth(); @loc=static @len=10 @rva=694320
	//_Func: public float getHeight(); @loc=static @len=11 @rva=220096
	//_Func: public void scaleByMult(float  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct ksgui_ksRect {
public:
	float left;
	float right;
	float top;
	float bottom;
	inline ksgui_ksRect()  { }
	inline float getWidth() { typedef float (*_fpt)(ksgui_ksRect *pthis); _fpt _f=(_fpt)_drva(694320); return _f(this); }
	inline float getHeight() { typedef float (*_fpt)(ksgui_ksRect *pthis); _fpt _f=(_fpt)_drva(220096); return _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ksRect)==16),"bad size");
		static_assert((offsetof(ksgui_ksRect,left)==0x0),"bad off");
		static_assert((offsetof(ksgui_ksRect,right)==0x4),"bad off");
		static_assert((offsetof(ksgui_ksRect,top)==0x8),"bad off");
		static_assert((offsetof(ksgui_ksRect,bottom)==0xC),"bad off");
	};
};

//UDT: struct SACEngineInput @len=16
	//_Data: this+0x0, Member, Type: float, gasInput
	//_Data: this+0x4, Member, Type: float, carSpeed
	//_Data: this+0x8, Member, Type: float, altitude
	//_Data: this+0xC, Member, Type: float, rpm
	//_Func: public void SACEngineInput(); @loc=optimized @len=0 @rva=0
//UDT;

struct SACEngineInput {
public:
	float gasInput;
	float carSpeed;
	float altitude;
	float rpm;
	inline SACEngineInput()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SACEngineInput)==16),"bad size");
		static_assert((offsetof(SACEngineInput,gasInput)==0x0),"bad off");
		static_assert((offsetof(SACEngineInput,carSpeed)==0x4),"bad off");
		static_assert((offsetof(SACEngineInput,altitude)==0x8),"bad off");
		static_assert((offsetof(SACEngineInput,rpm)==0xC),"bad off");
	};
};

//UDT: struct TyrePatchData @len=20
	//_Func: public void TyrePatchData(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, surfaceTransfer
	//_Data: this+0x4, Member, Type: float, patchTransfer
	//_Data: this+0x8, Member, Type: float, patchCoreTransfer
	//_Data: this+0xC, Member, Type: float, internalCoreTransfer
	//_Data: this+0x10, Member, Type: float, coolFactorGain
//UDT;

struct TyrePatchData {
public:
	float surfaceTransfer;
	float patchTransfer;
	float patchCoreTransfer;
	float internalCoreTransfer;
	float coolFactorGain;
	inline TyrePatchData()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TyrePatchData)==20),"bad size");
		static_assert((offsetof(TyrePatchData,surfaceTransfer)==0x0),"bad off");
		static_assert((offsetof(TyrePatchData,patchTransfer)==0x4),"bad off");
		static_assert((offsetof(TyrePatchData,patchCoreTransfer)==0x8),"bad off");
		static_assert((offsetof(TyrePatchData,internalCoreTransfer)==0xC),"bad off");
		static_assert((offsetof(TyrePatchData,coolFactorGain)==0x10),"bad off");
	};
};

//UDT: struct ClientQOSData @len=24
	//_Data: this+0x0, Member, Type: bool, usingMegapackets
	//_Data: this+0x4, Member, Type: int, counter
	//_Data: this+0x8, Member, Type: double, startTime
	//_Data: this+0x10, Member, Type: int, lastQOS
	//_Func: public void ClientQOSData(); @loc=optimized @len=0 @rva=0
//UDT;

struct ClientQOSData {
public:
	bool usingMegapackets;
	int counter;
	double startTime;
	int lastQOS;
	inline ClientQOSData()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ClientQOSData)==24),"bad size");
		static_assert((offsetof(ClientQOSData,usingMegapackets)==0x0),"bad off");
		static_assert((offsetof(ClientQOSData,counter)==0x4),"bad off");
		static_assert((offsetof(ClientQOSData,startTime)==0x8),"bad off");
		static_assert((offsetof(ClientQOSData,lastQOS)==0x10),"bad off");
	};
};

//UDT: struct ACPluginContext @len=1
//UDT;

struct ACPluginContext {
public:
	inline ACPluginContext()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ACPluginContext)==1),"bad size");
	};
};

//UDT: struct WheelValues @len=16
	//_Data: this+0x0, Member, Type: float, lf
	//_Data: this+0x4, Member, Type: float, rf
	//_Data: this+0x8, Member, Type: float, lr
	//_Data: this+0xC, Member, Type: float, rr
	//_Func: public void WheelValues(); @loc=optimized @len=0 @rva=0
//UDT;

struct WheelValues {
public:
	float lf;
	float rf;
	float lr;
	float rr;
	inline WheelValues()  { }
	inline void _guard_obj() {
		static_assert((sizeof(WheelValues)==16),"bad size");
		static_assert((offsetof(WheelValues,lf)==0x0),"bad off");
		static_assert((offsetof(WheelValues,rf)==0x4),"bad off");
		static_assert((offsetof(WheelValues,lr)==0x8),"bad off");
		static_assert((offsetof(WheelValues,rr)==0xC),"bad off");
	};
};

//UDT: struct KPI @len=8
	//_Data: this+0x0, Member, Type: float, angleRAD
	//_Data: this+0x4, Member, Type: float, scrubRadius
	//_Func: public void KPI(); @loc=optimized @len=0 @rva=0
//UDT;

struct KPI {
public:
	float angleRAD;
	float scrubRadius;
	inline KPI()  { }
	inline void _guard_obj() {
		static_assert((sizeof(KPI)==8),"bad size");
		static_assert((offsetof(KPI,angleRAD)==0x0),"bad off");
		static_assert((offsetof(KPI,scrubRadius)==0x4),"bad off");
	};
};

//UDT: struct CoastSettings @len=8
	//_Data: this+0x0, Member, Type: float, coast1
	//_Data: this+0x4, Member, Type: float, coast2
//UDT;

struct CoastSettings {
public:
	float coast1;
	float coast2;
	inline CoastSettings()  { }
	inline void _guard_obj() {
		static_assert((sizeof(CoastSettings)==8),"bad size");
		static_assert((offsetof(CoastSettings,coast1)==0x0),"bad off");
		static_assert((offsetof(CoastSettings,coast2)==0x4),"bad off");
	};
};

//UDT: struct AWD2Data @len=24
	//_Data: this+0x0, Member, Type: double, ramp
	//_Data: this+0x8, Member, Type: double, maxTorque
	//_Data: this+0x10, Member, Type: float, currentLockTorque
	//_Func: public void AWD2Data(); @loc=optimized @len=0 @rva=0
//UDT;

struct AWD2Data {
public:
	double ramp;
	double maxTorque;
	float currentLockTorque;
	inline AWD2Data()  { }
	inline void _guard_obj() {
		static_assert((sizeof(AWD2Data)==24),"bad size");
		static_assert((offsetof(AWD2Data,ramp)==0x0),"bad off");
		static_assert((offsetof(AWD2Data,maxTorque)==0x8),"bad off");
		static_assert((offsetof(AWD2Data,currentLockTorque)==0x10),"bad off");
	};
};

//UDT: struct SplineLocatorData @len=40
	//_Data: this+0x0, Member, Type: float, npos
	//_Data: this+0x4, Member, Type: unsigned int, currentIndex
	//_Data: this+0x8, Member, Type: float, lateralOffset
	//_Data: this+0xC, Member, Type: float, splineLength
	//_Data: this+0x10, Member, Type: float[0x2], sides
	//_Data: this+0x18, Member, Type: float[0x2], sidesFromIL
	//_Data: this+0x20, Member, Type: float, sideVelocity
	//_Data: this+0x24, Member, Type: bool, isOutsideTrackLimits
	//_Func: public void SplineLocatorData(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SplineLocatorData)==40),"bad size");
		static_assert((offsetof(SplineLocatorData,npos)==0x0),"bad off");
		static_assert((offsetof(SplineLocatorData,currentIndex)==0x4),"bad off");
		static_assert((offsetof(SplineLocatorData,lateralOffset)==0x8),"bad off");
		static_assert((offsetof(SplineLocatorData,splineLength)==0xC),"bad off");
		static_assert((offsetof(SplineLocatorData,sides)==0x10),"bad off");
		static_assert((offsetof(SplineLocatorData,sidesFromIL)==0x18),"bad off");
		static_assert((offsetof(SplineLocatorData,sideVelocity)==0x20),"bad off");
		static_assert((offsetof(SplineLocatorData,isOutsideTrackLimits)==0x24),"bad off");
	};
};

//UDT: struct PerformancePair @len=8
	//_Func: public void PerformancePair(unsigned int  _arg0, float  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, t
	//_Data: this+0x4, Member, Type: float, speedMS
//UDT;

struct PerformancePair {
public:
	unsigned int t;
	float speedMS;
	inline PerformancePair()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PerformancePair)==8),"bad size");
		static_assert((offsetof(PerformancePair,t)==0x0),"bad off");
		static_assert((offsetof(PerformancePair,speedMS)==0x4),"bad off");
	};
};

//UDT: struct PitStopTime @len=16
	//_Data: this+0x0, Member, Type: float, total
	//_Data: this+0x4, Member, Type: float, tyres
	//_Data: this+0x8, Member, Type: float, repair
	//_Data: this+0xC, Member, Type: float, fuel
	//_Func: public void PitStopTime(); @loc=optimized @len=0 @rva=0
//UDT;

struct PitStopTime {
public:
	float total;
	float tyres;
	float repair;
	float fuel;
	inline PitStopTime()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PitStopTime)==16),"bad size");
		static_assert((offsetof(PitStopTime,total)==0x0),"bad off");
		static_assert((offsetof(PitStopTime,tyres)==0x4),"bad off");
		static_assert((offsetof(PitStopTime,repair)==0x8),"bad off");
		static_assert((offsetof(PitStopTime,fuel)==0xC),"bad off");
	};
};

//UDT: struct SplineIndexBound @len=8
	//_Func: public void SplineIndexBound(unsigned int  _arg0, unsigned int  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, minIndex
	//_Data: this+0x4, Member, Type: unsigned int, maxIndex
//UDT;

struct SplineIndexBound {
public:
	unsigned int minIndex;
	unsigned int maxIndex;
	inline SplineIndexBound()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SplineIndexBound)==8),"bad size");
		static_assert((offsetof(SplineIndexBound,minIndex)==0x0),"bad off");
		static_assert((offsetof(SplineIndexBound,maxIndex)==0x4),"bad off");
	};
};

//UDT: struct SplineLocationData @len=4
	//_Func: public void SplineLocationData(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, currentIndex
//UDT;

struct SplineLocationData {
public:
	int currentIndex;
	inline SplineLocationData()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SplineLocationData)==4),"bad size");
		static_assert((offsetof(SplineLocationData,currentIndex)==0x0),"bad off");
	};
};

//UDT: struct WreckerProtection @len=16
	//_Data: this+0x0, Member, Type: float, maxContactsPerKM
	//_Data: this+0x4, Member, Type: int, warningCount
	//_Data: this+0x8, Member, Type: int, contacts
	//_Data: this+0xC, Member, Type: bool, blackListRequested
	//_Func: public void WreckerProtection(); @loc=optimized @len=0 @rva=0
//UDT;

struct WreckerProtection {
public:
	float maxContactsPerKM;
	int warningCount;
	int contacts;
	bool blackListRequested;
	inline WreckerProtection()  { }
	inline void _guard_obj() {
		static_assert((sizeof(WreckerProtection)==16),"bad size");
		static_assert((offsetof(WreckerProtection,maxContactsPerKM)==0x0),"bad off");
		static_assert((offsetof(WreckerProtection,warningCount)==0x4),"bad off");
		static_assert((offsetof(WreckerProtection,contacts)==0x8),"bad off");
		static_assert((offsetof(WreckerProtection,blackListRequested)==0xC),"bad off");
	};
};

//UDT: struct OnKeyCharEvent @len=4
	//_Func: public void OnKeyCharEvent(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, key
//UDT;

struct OnKeyCharEvent {
public:
	unsigned int key;
	inline OnKeyCharEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnKeyCharEvent)==4),"bad size");
		static_assert((offsetof(OnKeyCharEvent,key)==0x0),"bad off");
	};
};

//UDT: struct PushToPass @len=40
	//_Data: this+0x0, Member, Type: bool, enabled
	//_Data: this+0x1, Member, Type: bool, active
	//_Data: this+0x4, Member, Type: float, overboost
	//_Data: this+0x8, Member, Type: float, timeS
	//_Data: this+0xC, Member, Type: float, coolDownS
	//_Data: this+0x10, Member, Type: float, timeAccum
	//_Data: this+0x14, Member, Type: int, activations
	//_Data: this+0x18, Member, Type: float, baseWastegate
	//_Data: this+0x1C, Member, Type: int, baseActivations
	//_Data: this+0x20, Member, Type: int, basePositionCoeff
	//_Data: this+0x24, Member, Type: int, maxActivations
	//_Func: public void PushToPass(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(PushToPass)==40),"bad size");
		static_assert((offsetof(PushToPass,enabled)==0x0),"bad off");
		static_assert((offsetof(PushToPass,active)==0x1),"bad off");
		static_assert((offsetof(PushToPass,overboost)==0x4),"bad off");
		static_assert((offsetof(PushToPass,timeS)==0x8),"bad off");
		static_assert((offsetof(PushToPass,coolDownS)==0xC),"bad off");
		static_assert((offsetof(PushToPass,timeAccum)==0x10),"bad off");
		static_assert((offsetof(PushToPass,activations)==0x14),"bad off");
		static_assert((offsetof(PushToPass,baseWastegate)==0x18),"bad off");
		static_assert((offsetof(PushToPass,baseActivations)==0x1C),"bad off");
		static_assert((offsetof(PushToPass,basePositionCoeff)==0x20),"bad off");
		static_assert((offsetof(PushToPass,maxActivations)==0x24),"bad off");
	};
};

//UDT: struct DamageReportDef @len=32
	//_Func: public void DamageReportDef(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: double, lastSendTime
	//_Data: this+0x8, Member, Type: float[0x5], damageZoneLevel
//UDT;

struct DamageReportDef {
public:
	double lastSendTime;
	float damageZoneLevel[0x5];
	inline DamageReportDef()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DamageReportDef)==32),"bad size");
		static_assert((offsetof(DamageReportDef,lastSendTime)==0x0),"bad off");
		static_assert((offsetof(DamageReportDef,damageZoneLevel)==0x8),"bad off");
	};
};

//UDT: struct VideoSettings @len=80
	//_Func: public void VideoSettings(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, aaSamples
	//_Data: this+0x4, Member, Type: int, width
	//_Data: this+0x8, Member, Type: int, height
	//_Data: this+0x10, Member, Type: void *, hWnd
	//_Data: this+0x18, Member, Type: bool, isFullscreen
	//_Data: this+0x19, Member, Type: bool, vSync
	//_Data: this+0x1C, Member, Type: int, anisotropic
	//_Data: this+0x20, Member, Type: int, aaQuality
	//_Data: this+0x24, Member, Type: int, shadowMapSize
	//_Data: this+0x28, Member, Type: double, fpsCapMS
	//_Data: this+0x30, Member, Type: int, dxgiModeIndex
	//_Data: this+0x34, Member, Type: int, worldDetail
	//_Data: this+0x38, Member, Type: bool, ppHDREnabled
	//_Data: this+0x39, Member, Type: bool, ppHeatShimmer
	//_Data: this+0x3A, Member, Type: bool, ppFXAA
	//_Data: this+0x3B, Member, Type: bool, ppRaysOfGod
	//_Data: this+0x3C, Member, Type: int, ppQuality
	//_Data: this+0x40, Member, Type: int, ppGlare
	//_Data: this+0x44, Member, Type: int, ppDof
	//_Data: this+0x48, Member, Type: bool, tripleBuffer
	//_Data: this+0x4C, Member, Type: float, refresh
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(VideoSettings)==80),"bad size");
		static_assert((offsetof(VideoSettings,aaSamples)==0x0),"bad off");
		static_assert((offsetof(VideoSettings,width)==0x4),"bad off");
		static_assert((offsetof(VideoSettings,height)==0x8),"bad off");
		static_assert((offsetof(VideoSettings,hWnd)==0x10),"bad off");
		static_assert((offsetof(VideoSettings,isFullscreen)==0x18),"bad off");
		static_assert((offsetof(VideoSettings,vSync)==0x19),"bad off");
		static_assert((offsetof(VideoSettings,anisotropic)==0x1C),"bad off");
		static_assert((offsetof(VideoSettings,aaQuality)==0x20),"bad off");
		static_assert((offsetof(VideoSettings,shadowMapSize)==0x24),"bad off");
		static_assert((offsetof(VideoSettings,fpsCapMS)==0x28),"bad off");
		static_assert((offsetof(VideoSettings,dxgiModeIndex)==0x30),"bad off");
		static_assert((offsetof(VideoSettings,worldDetail)==0x34),"bad off");
		static_assert((offsetof(VideoSettings,ppHDREnabled)==0x38),"bad off");
		static_assert((offsetof(VideoSettings,ppHeatShimmer)==0x39),"bad off");
		static_assert((offsetof(VideoSettings,ppFXAA)==0x3A),"bad off");
		static_assert((offsetof(VideoSettings,ppRaysOfGod)==0x3B),"bad off");
		static_assert((offsetof(VideoSettings,ppQuality)==0x3C),"bad off");
		static_assert((offsetof(VideoSettings,ppGlare)==0x40),"bad off");
		static_assert((offsetof(VideoSettings,ppDof)==0x44),"bad off");
		static_assert((offsetof(VideoSettings,tripleBuffer)==0x48),"bad off");
		static_assert((offsetof(VideoSettings,refresh)==0x4C),"bad off");
	};
};

//UDT: struct GameStats @len=40
	//_Func: public void GameStats(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: double, cpuTime
	//_Data: this+0x8, Member, Type: double, updateTime
	//_Data: this+0x10, Member, Type: double, renderHUDTime
	//_Data: this+0x18, Member, Type: double, renderTime
	//_Data: this+0x20, Member, Type: double, renderAudioTime
//UDT;

struct GameStats {
public:
	double cpuTime;
	double updateTime;
	double renderHUDTime;
	double renderTime;
	double renderAudioTime;
	inline GameStats()  { }
	inline void _guard_obj() {
		static_assert((sizeof(GameStats)==40),"bad size");
		static_assert((offsetof(GameStats,cpuTime)==0x0),"bad off");
		static_assert((offsetof(GameStats,updateTime)==0x8),"bad off");
		static_assert((offsetof(GameStats,renderHUDTime)==0x10),"bad off");
		static_assert((offsetof(GameStats,renderTime)==0x18),"bad off");
		static_assert((offsetof(GameStats,renderAudioTime)==0x20),"bad off");
	};
};

//UDT: struct OnKeyEvent @len=4
	//_Func: public void OnKeyEvent(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, keyCode
//UDT;

struct OnKeyEvent {
public:
	unsigned int keyCode;
	inline OnKeyEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnKeyEvent)==4),"bad size");
		static_assert((offsetof(OnKeyEvent,keyCode)==0x0),"bad off");
	};
};

//UDT: class ITorqueGenerator @len=8 @vfcount=2
	//_VTable: 
	//_Func: public void ~ITorqueGenerator(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2549840
	//_Func: public float getOutputTorque(); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void ITorqueGenerator(ITorqueGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ITorqueGenerator(); @loc=optimized @len=0 @rva=0
	//_Func: public ITorqueGenerator & operator=(ITorqueGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class ITorqueGenerator {
public:
	inline ITorqueGenerator()  { }
	virtual ~ITorqueGenerator();
	inline void dtor() { typedef void (*_fpt)(ITorqueGenerator *pthis); _fpt _f=(_fpt)_drva(2549840); _f(this); }
	virtual float getOutputTorque_vf1() = 0;
	inline float getOutputTorque() { return getOutputTorque_vf1(); }
	inline void _guard_obj() {
		static_assert((sizeof(ITorqueGenerator)==8),"bad size");
	};
};

//UDT: struct TyreInputs @len=12
	//_Func: public void TyreInputs(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, brakeTorque
	//_Data: this+0x4, Member, Type: float, handBrakeTorque
	//_Data: this+0x8, Member, Type: float, electricTorque
//UDT;

struct TyreInputs {
public:
	float brakeTorque;
	float handBrakeTorque;
	float electricTorque;
	inline TyreInputs()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TyreInputs)==12),"bad size");
		static_assert((offsetof(TyreInputs,brakeTorque)==0x0),"bad off");
		static_assert((offsetof(TyreInputs,handBrakeTorque)==0x4),"bad off");
		static_assert((offsetof(TyreInputs,electricTorque)==0x8),"bad off");
	};
};

//UDT: struct TyreStatus @len=184
	//_Func: public void TyreStatus(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, depth
	//_Data: this+0x4, Member, Type: float, load
	//_Data: this+0x8, Member, Type: float, camberRAD
	//_Data: this+0xC, Member, Type: float, slipAngleRAD
	//_Data: this+0x10, Member, Type: float, slipRatio
	//_Data: this+0x14, Member, Type: float, angularVelocity
	//_Data: this+0x18, Member, Type: float, Fy
	//_Data: this+0x1C, Member, Type: float, Fx
	//_Data: this+0x20, Member, Type: float, Mz
	//_Data: this+0x24, Member, Type: bool, isLocked
	//_Data: this+0x28, Member, Type: float, slipFactor
	//_Data: this+0x2C, Member, Type: float, ndSlip
	//_Data: this+0x30, Member, Type: float, distToGround
	//_Data: this+0x34, Member, Type: float, Dy
	//_Data: this+0x38, Member, Type: float, Dx
	//_Data: this+0x3C, Member, Type: float, D
	//_Data: this+0x40, Member, Type: float, dirtyLevel
	//_Data: this+0x44, Member, Type: float, rollingResistence
	//_Data: this+0x48, Member, Type: float, thermalInput
	//_Data: this+0x4C, Member, Type: float, feedbackTorque
	//_Data: this+0x50, Member, Type: float, loadedRadius
	//_Data: this+0x54, Member, Type: float, effectiveRadius
	//_Data: this+0x58, Member, Type: float, liveRadius
	//_Data: this+0x5C, Member, Type: float, pressureStatic
	//_Data: this+0x60, Member, Type: float, pressureDynamic
	//_Data: this+0x68, Member, Type: double, virtualKM
	//_Data: this+0x70, Member, Type: float[0x3], lastTempIMO
	//_Data: this+0x7C, Member, Type: float, peakSA
	//_Data: this+0x80, Member, Type: double, grain
	//_Data: this+0x88, Member, Type: double, blister
	//_Data: this+0x90, Member, Type: float, inflation
	//_Data: this+0x98, Member, Type: double, flatSpot
	//_Data: this+0xA0, Member, Type: float, lastGrain
	//_Data: this+0xA4, Member, Type: float, lastBlister
	//_Data: this+0xA8, Member, Type: float, normalizedSlideX
	//_Data: this+0xAC, Member, Type: float, normalizedSlideY
	//_Data: this+0xB0, Member, Type: float, finalDY
	//_Data: this+0xB4, Member, Type: float, wearMult
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreStatus)==184),"bad size");
		static_assert((offsetof(TyreStatus,depth)==0x0),"bad off");
		static_assert((offsetof(TyreStatus,load)==0x4),"bad off");
		static_assert((offsetof(TyreStatus,camberRAD)==0x8),"bad off");
		static_assert((offsetof(TyreStatus,slipAngleRAD)==0xC),"bad off");
		static_assert((offsetof(TyreStatus,slipRatio)==0x10),"bad off");
		static_assert((offsetof(TyreStatus,angularVelocity)==0x14),"bad off");
		static_assert((offsetof(TyreStatus,Fy)==0x18),"bad off");
		static_assert((offsetof(TyreStatus,Fx)==0x1C),"bad off");
		static_assert((offsetof(TyreStatus,Mz)==0x20),"bad off");
		static_assert((offsetof(TyreStatus,isLocked)==0x24),"bad off");
		static_assert((offsetof(TyreStatus,slipFactor)==0x28),"bad off");
		static_assert((offsetof(TyreStatus,ndSlip)==0x2C),"bad off");
		static_assert((offsetof(TyreStatus,distToGround)==0x30),"bad off");
		static_assert((offsetof(TyreStatus,Dy)==0x34),"bad off");
		static_assert((offsetof(TyreStatus,Dx)==0x38),"bad off");
		static_assert((offsetof(TyreStatus,D)==0x3C),"bad off");
		static_assert((offsetof(TyreStatus,dirtyLevel)==0x40),"bad off");
		static_assert((offsetof(TyreStatus,rollingResistence)==0x44),"bad off");
		static_assert((offsetof(TyreStatus,thermalInput)==0x48),"bad off");
		static_assert((offsetof(TyreStatus,feedbackTorque)==0x4C),"bad off");
		static_assert((offsetof(TyreStatus,loadedRadius)==0x50),"bad off");
		static_assert((offsetof(TyreStatus,effectiveRadius)==0x54),"bad off");
		static_assert((offsetof(TyreStatus,liveRadius)==0x58),"bad off");
		static_assert((offsetof(TyreStatus,pressureStatic)==0x5C),"bad off");
		static_assert((offsetof(TyreStatus,pressureDynamic)==0x60),"bad off");
		static_assert((offsetof(TyreStatus,virtualKM)==0x68),"bad off");
		static_assert((offsetof(TyreStatus,lastTempIMO)==0x70),"bad off");
		static_assert((offsetof(TyreStatus,peakSA)==0x7C),"bad off");
		static_assert((offsetof(TyreStatus,grain)==0x80),"bad off");
		static_assert((offsetof(TyreStatus,blister)==0x88),"bad off");
		static_assert((offsetof(TyreStatus,inflation)==0x90),"bad off");
		static_assert((offsetof(TyreStatus,flatSpot)==0x98),"bad off");
		static_assert((offsetof(TyreStatus,lastGrain)==0xA0),"bad off");
		static_assert((offsetof(TyreStatus,lastBlister)==0xA4),"bad off");
		static_assert((offsetof(TyreStatus,normalizedSlideX)==0xA8),"bad off");
		static_assert((offsetof(TyreStatus,normalizedSlideY)==0xAC),"bad off");
		static_assert((offsetof(TyreStatus,finalDY)==0xB0),"bad off");
		static_assert((offsetof(TyreStatus,wearMult)==0xB4),"bad off");
	};
};

//UDT: struct SCarStateAero @len=12
	//_Func: public void SCarStateAero(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, CD
	//_Data: this+0x4, Member, Type: float, CL_Front
	//_Data: this+0x8, Member, Type: float, CL_Rear
//UDT;

struct SCarStateAero {
public:
	float CD;
	float CL_Front;
	float CL_Rear;
	inline SCarStateAero()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SCarStateAero)==12),"bad size");
		static_assert((offsetof(SCarStateAero,CD)==0x0),"bad off");
		static_assert((offsetof(SCarStateAero,CL_Front)==0x4),"bad off");
		static_assert((offsetof(SCarStateAero,CL_Rear)==0x8),"bad off");
	};
};

//UDT: struct AccelerationProfile @len=8
	//_Data: this+0x0, Member, Type: float, zero
	//_Data: this+0x4, Member, Type: float, maxTyres
//UDT;

struct AccelerationProfile {
public:
	float zero;
	float maxTyres;
	inline AccelerationProfile()  { }
	inline void _guard_obj() {
		static_assert((sizeof(AccelerationProfile)==8),"bad size");
		static_assert((offsetof(AccelerationProfile,zero)==0x0),"bad off");
		static_assert((offsetof(AccelerationProfile,maxTyres)==0x4),"bad off");
	};
};

//UDT: struct TyreData @len=72
	//_Func: public void TyreData(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, width
	//_Data: this+0x4, Member, Type: float, radius
	//_Data: this+0x8, Member, Type: float, k
	//_Data: this+0xC, Member, Type: float, d
	//_Data: this+0x10, Member, Type: float, angularInertia
	//_Data: this+0x14, Member, Type: float, thermalFrictionK
	//_Data: this+0x18, Member, Type: float, thermalRollingK
	//_Data: this+0x1C, Member, Type: float, thermalRollingSurfaceK
	//_Data: this+0x20, Member, Type: float, grainThreshold
	//_Data: this+0x24, Member, Type: float, blisterThreshold
	//_Data: this+0x28, Member, Type: float, grainGamma
	//_Data: this+0x2C, Member, Type: float, blisterGamma
	//_Data: this+0x30, Member, Type: float, grainGain
	//_Data: this+0x34, Member, Type: float, blisterGain
	//_Data: this+0x38, Member, Type: float, rimRadius
	//_Data: this+0x3C, Member, Type: float, optimumTemp
	//_Data: this+0x40, Member, Type: float, softnessIndex
	//_Data: this+0x44, Member, Type: float, radiusRaiseK
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreData)==72),"bad size");
		static_assert((offsetof(TyreData,width)==0x0),"bad off");
		static_assert((offsetof(TyreData,radius)==0x4),"bad off");
		static_assert((offsetof(TyreData,k)==0x8),"bad off");
		static_assert((offsetof(TyreData,d)==0xC),"bad off");
		static_assert((offsetof(TyreData,angularInertia)==0x10),"bad off");
		static_assert((offsetof(TyreData,thermalFrictionK)==0x14),"bad off");
		static_assert((offsetof(TyreData,thermalRollingK)==0x18),"bad off");
		static_assert((offsetof(TyreData,thermalRollingSurfaceK)==0x1C),"bad off");
		static_assert((offsetof(TyreData,grainThreshold)==0x20),"bad off");
		static_assert((offsetof(TyreData,blisterThreshold)==0x24),"bad off");
		static_assert((offsetof(TyreData,grainGamma)==0x28),"bad off");
		static_assert((offsetof(TyreData,blisterGamma)==0x2C),"bad off");
		static_assert((offsetof(TyreData,grainGain)==0x30),"bad off");
		static_assert((offsetof(TyreData,blisterGain)==0x34),"bad off");
		static_assert((offsetof(TyreData,rimRadius)==0x38),"bad off");
		static_assert((offsetof(TyreData,optimumTemp)==0x3C),"bad off");
		static_assert((offsetof(TyreData,softnessIndex)==0x40),"bad off");
		static_assert((offsetof(TyreData,radiusRaiseK)==0x44),"bad off");
	};
};

//UDT: struct TyreThermalState @len=192
	//_Data: this+0x0, Member, Type: float[0x3][0xC], temps
	//_Data: this+0x90, Member, Type: float, coreTemp
	//_Data: this+0x94, Member, Type: float, thermalInput
	//_Data: this+0x98, Member, Type: float, dynamicPressure
	//_Data: this+0x9C, Member, Type: float, staticPressure
	//_Data: this+0xA0, Member, Type: float[0x3], lastSetIMO
	//_Data: this+0xAC, Member, Type: float, cpTemperature
	//_Data: this+0xB0, Member, Type: float, lastGrain
	//_Data: this+0xB4, Member, Type: float, lastBlister
	//_Data: this+0xB8, Member, Type: float, mult
	//_Data: this+0xBC, Member, Type: bool, isHot
	//_Func: public void TyreThermalState(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreThermalState)==192),"bad size");
		static_assert((offsetof(TyreThermalState,temps)==0x0),"bad off");
		static_assert((offsetof(TyreThermalState,coreTemp)==0x90),"bad off");
		static_assert((offsetof(TyreThermalState,thermalInput)==0x94),"bad off");
		static_assert((offsetof(TyreThermalState,dynamicPressure)==0x98),"bad off");
		static_assert((offsetof(TyreThermalState,staticPressure)==0x9C),"bad off");
		static_assert((offsetof(TyreThermalState,lastSetIMO)==0xA0),"bad off");
		static_assert((offsetof(TyreThermalState,cpTemperature)==0xAC),"bad off");
		static_assert((offsetof(TyreThermalState,lastGrain)==0xB0),"bad off");
		static_assert((offsetof(TyreThermalState,lastBlister)==0xB4),"bad off");
		static_assert((offsetof(TyreThermalState,mult)==0xB8),"bad off");
		static_assert((offsetof(TyreThermalState,isHot)==0xBC),"bad off");
	};
};

//UDT: struct BrushOutput @len=8
	//_Func: public void BrushOutput(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, force
	//_Data: this+0x4, Member, Type: float, slip
//UDT;

struct BrushOutput {
public:
	float force;
	float slip;
	inline BrushOutput()  { }
	inline void _guard_obj() {
		static_assert((sizeof(BrushOutput)==8),"bad size");
		static_assert((offsetof(BrushOutput,force)==0x0),"bad off");
		static_assert((offsetof(BrushOutput,slip)==0x4),"bad off");
	};
};

//UDT: struct NetCarQoS @len=8
	//_Data: this+0x0, Member, Type: int, goodPackets
	//_Data: this+0x4, Member, Type: int, badPackets
	//_Func: public void NetCarQoS(); @loc=optimized @len=0 @rva=0
//UDT;

struct NetCarQoS {
public:
	int goodPackets;
	int badPackets;
	inline NetCarQoS()  { }
	inline void _guard_obj() {
		static_assert((sizeof(NetCarQoS)==8),"bad size");
		static_assert((offsetof(NetCarQoS,goodPackets)==0x0),"bad off");
		static_assert((offsetof(NetCarQoS,badPackets)==0x4),"bad off");
	};
};

//UDT: struct TyreExternalInputs @len=16
	//_Data: this+0x0, Member, Type: bool, isActive
	//_Data: this+0x4, Member, Type: float, load
	//_Data: this+0x8, Member, Type: float, slipAngle
	//_Data: this+0xC, Member, Type: float, slipRatio
	//_Func: public void TyreExternalInputs(); @loc=optimized @len=0 @rva=0
//UDT;

struct TyreExternalInputs {
public:
	bool isActive;
	float load;
	float slipAngle;
	float slipRatio;
	inline TyreExternalInputs()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TyreExternalInputs)==16),"bad size");
		static_assert((offsetof(TyreExternalInputs,isActive)==0x0),"bad off");
		static_assert((offsetof(TyreExternalInputs,load)==0x4),"bad off");
		static_assert((offsetof(TyreExternalInputs,slipAngle)==0x8),"bad off");
		static_assert((offsetof(TyreExternalInputs,slipRatio)==0xC),"bad off");
	};
};

//UDT: class SignalGenerator @len=16 @vfcount=3
	//_VTable: 
	//_Func: public void SignalGenerator(SignalGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SignalGenerator(); @loc=static @len=22 @rva=2282048
	//_Func: public void ~SignalGenerator(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2282080
	//_Data: this+0x8, Member, Type: float, freqScale
	//_Func: public void step(float dt); @intro @virtual vtpo=0 vfid=1 @loc=static @len=21 @rva=2282240
	//_Func: public float getValue(); @intro @pure @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Data: this+0xC, Member, Type: int, value
	//_Func: public SignalGenerator & operator=(SignalGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SignalGenerator)==16),"bad size");
		static_assert((offsetof(SignalGenerator,freqScale)==0x8),"bad off");
		static_assert((offsetof(SignalGenerator,value)==0xC),"bad off");
	};
};

//UDT: class ICoastGenerator @len=8 @vfcount=2
	//_VTable: 
	//_Func: public void ~ICoastGenerator(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2693184
	//_Func: public float getCoastTorque(); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void ICoastGenerator(ICoastGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ICoastGenerator(); @loc=optimized @len=0 @rva=0
	//_Func: public ICoastGenerator & operator=(ICoastGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class ICoastGenerator {
public:
	inline ICoastGenerator()  { }
	virtual ~ICoastGenerator();
	inline void dtor() { typedef void (*_fpt)(ICoastGenerator *pthis); _fpt _f=(_fpt)_drva(2693184); _f(this); }
	virtual float getCoastTorque_vf1() = 0;
	inline float getCoastTorque() { return getCoastTorque_vf1(); }
	inline void _guard_obj() {
		static_assert((sizeof(ICoastGenerator)==8),"bad size");
	};
};

//UDT: struct OnWindowResize @len=8
	//_Data: this+0x0, Member, Type: int, width
	//_Data: this+0x4, Member, Type: int, height
//UDT;

struct OnWindowResize {
public:
	int width;
	int height;
	inline OnWindowResize()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnWindowResize)==8),"bad size");
		static_assert((offsetof(OnWindowResize,width)==0x0),"bad off");
		static_assert((offsetof(OnWindowResize,height)==0x4),"bad off");
	};
};

//UDT: struct SamplerStates @len=56
	//_Data: this+0x0, Member, Type: void *, samplerAniso
	//_Data: this+0x8, Member, Type: void *, samplerLinearShadow
	//_Data: this+0x10, Member, Type: void *, samplerShadow
	//_Data: this+0x18, Member, Type: void *, samplerPoint
	//_Data: this+0x20, Member, Type: void *, samplerPointClamp
	//_Data: this+0x28, Member, Type: void *, samplerLinearSimple
	//_Data: this+0x30, Member, Type: void *, samplerLinearClamp
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SamplerStates)==56),"bad size");
		static_assert((offsetof(SamplerStates,samplerAniso)==0x0),"bad off");
		static_assert((offsetof(SamplerStates,samplerLinearShadow)==0x8),"bad off");
		static_assert((offsetof(SamplerStates,samplerShadow)==0x10),"bad off");
		static_assert((offsetof(SamplerStates,samplerPoint)==0x18),"bad off");
		static_assert((offsetof(SamplerStates,samplerPointClamp)==0x20),"bad off");
		static_assert((offsetof(SamplerStates,samplerLinearSimple)==0x28),"bad off");
		static_assert((offsetof(SamplerStates,samplerLinearClamp)==0x30),"bad off");
	};
};

//UDT: struct ERSStatus @len=8
	//_Data: this+0x0, Member, Type: float, kineticRecovery
	//_Data: this+0x4, Member, Type: float, heatRecovery
	//_Func: public void ERSStatus(); @loc=optimized @len=0 @rva=0
//UDT;

struct ERSStatus {
public:
	float kineticRecovery;
	float heatRecovery;
	inline ERSStatus()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ERSStatus)==8),"bad size");
		static_assert((offsetof(ERSStatus,kineticRecovery)==0x0),"bad off");
		static_assert((offsetof(ERSStatus,heatRecovery)==0x4),"bad off");
	};
};

//UDT: struct TyreSlipOutput @len=8
	//_Data: this+0x0, Member, Type: float, normalizedForce
	//_Data: this+0x4, Member, Type: float, slip
//UDT;

struct TyreSlipOutput {
public:
	float normalizedForce;
	float slip;
	inline TyreSlipOutput()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TyreSlipOutput)==8),"bad size");
		static_assert((offsetof(TyreSlipOutput,normalizedForce)==0x0),"bad off");
		static_assert((offsetof(TyreSlipOutput,slip)==0x4),"bad off");
	};
};

//UDT: class GameTime @len=56 @vfcount=1
	//_VTable: 
	//_Func: public void GameTime(GameTime &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void GameTime(); @loc=static @len=62 @rva=4506064
	//_Func: public void ~GameTime(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=4506128
	//_Data: this+0x8, Member, Type: double, now
	//_Data: this+0x10, Member, Type: float, deltaT
	//_Data: this+0x14, Member, Type: float, smoothDeltaT
	//_Data: this+0x18, Member, Type: float, fps
	//_Data: this+0x1C, Member, Type: bool, useStabilizer
	//_Data: this+0x20, Member, Type: double, cappedFPS
	//_Data: this+0x28, Member, Type: double, startTime
	//_Func: public void update(); @loc=static @len=341 @rva=4506192
	//_Func: public bool isUsingHRT(); @loc=optimized @len=0 @rva=0
	//_Func: protected void initTimer(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x30, Member, Type: bool, m_isUsingHRT
	//_Func: public GameTime & operator=(GameTime &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(GameTime)==56),"bad size");
		static_assert((offsetof(GameTime,now)==0x8),"bad off");
		static_assert((offsetof(GameTime,deltaT)==0x10),"bad off");
		static_assert((offsetof(GameTime,smoothDeltaT)==0x14),"bad off");
		static_assert((offsetof(GameTime,fps)==0x18),"bad off");
		static_assert((offsetof(GameTime,useStabilizer)==0x1C),"bad off");
		static_assert((offsetof(GameTime,cappedFPS)==0x20),"bad off");
		static_assert((offsetof(GameTime,startTime)==0x28),"bad off");
		static_assert((offsetof(GameTime,m_isUsingHRT)==0x30),"bad off");
	};
};

//UDT: struct ERSCockpitControls @len=3
	//_Data: this+0x0, Member, Type: bool, recovery
	//_Data: this+0x1, Member, Type: bool, mguHMode
	//_Data: this+0x2, Member, Type: bool, deliveryProfile
	//_Func: public void ERSCockpitControls(); @loc=optimized @len=0 @rva=0
//UDT;

struct ERSCockpitControls {
public:
	bool recovery;
	bool mguHMode;
	bool deliveryProfile;
	inline ERSCockpitControls()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ERSCockpitControls)==3),"bad size");
		static_assert((offsetof(ERSCockpitControls,recovery)==0x0),"bad off");
		static_assert((offsetof(ERSCockpitControls,mguHMode)==0x1),"bad off");
		static_assert((offsetof(ERSCockpitControls,deliveryProfile)==0x2),"bad off");
	};
};

//UDT: class CarControls @len=52
	//_Func: public void CarControls(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: bool, gearUp
	//_Data: this+0x1, Member, Type: bool, gearDn
	//_Data: this+0x2, Member, Type: bool, drs
	//_Data: this+0x3, Member, Type: bool, kers
	//_Data: this+0x4, Member, Type: bool, brakeBalanceUp
	//_Data: this+0x5, Member, Type: bool, brakeBalanceDn
	//_Data: this+0x8, Member, Type: int, requestedGearIndex
	//_Data: this+0xC, Member, Type: bool, isShifterSupported
	//_Data: this+0x10, Member, Type: float, handBrake
	//_Data: this+0x14, Member, Type: bool, absUp
	//_Data: this+0x15, Member, Type: bool, absDn
	//_Data: this+0x16, Member, Type: bool, tcUp
	//_Data: this+0x17, Member, Type: bool, tcDn
	//_Data: this+0x18, Member, Type: bool, turboUp
	//_Data: this+0x19, Member, Type: bool, turboDn
	//_Data: this+0x1A, Member, Type: bool, engineBrakeUp
	//_Data: this+0x1B, Member, Type: bool, engineBrakeDn
	//_Data: this+0x1C, Member, Type: bool, MGUKDeliveryUp
	//_Data: this+0x1D, Member, Type: bool, MGUKDeliveryDn
	//_Data: this+0x1E, Member, Type: bool, MGUKRecoveryUp
	//_Data: this+0x1F, Member, Type: bool, MGUKRecoveryDn
	//_Data: this+0x20, Member, Type: bool, MGUHMode
	//_Func: public void setSteer(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getSteer(); @loc=optimized @len=0 @rva=0
	//_Func: public void setGas(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getGas(); @loc=optimized @len=0 @rva=0
	//_Func: public void setBrake(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getBrake(); @loc=optimized @len=0 @rva=0
	//_Func: public void setClutch(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getClutch(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x24, Member, Type: float, gas
	//_Data: this+0x28, Member, Type: float, brake
	//_Data: this+0x2C, Member, Type: float, steer
	//_Data: this+0x30, Member, Type: float, clutch
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CarControls)==52),"bad size");
		static_assert((offsetof(CarControls,gearUp)==0x0),"bad off");
		static_assert((offsetof(CarControls,gearDn)==0x1),"bad off");
		static_assert((offsetof(CarControls,drs)==0x2),"bad off");
		static_assert((offsetof(CarControls,kers)==0x3),"bad off");
		static_assert((offsetof(CarControls,brakeBalanceUp)==0x4),"bad off");
		static_assert((offsetof(CarControls,brakeBalanceDn)==0x5),"bad off");
		static_assert((offsetof(CarControls,requestedGearIndex)==0x8),"bad off");
		static_assert((offsetof(CarControls,isShifterSupported)==0xC),"bad off");
		static_assert((offsetof(CarControls,handBrake)==0x10),"bad off");
		static_assert((offsetof(CarControls,absUp)==0x14),"bad off");
		static_assert((offsetof(CarControls,absDn)==0x15),"bad off");
		static_assert((offsetof(CarControls,tcUp)==0x16),"bad off");
		static_assert((offsetof(CarControls,tcDn)==0x17),"bad off");
		static_assert((offsetof(CarControls,turboUp)==0x18),"bad off");
		static_assert((offsetof(CarControls,turboDn)==0x19),"bad off");
		static_assert((offsetof(CarControls,engineBrakeUp)==0x1A),"bad off");
		static_assert((offsetof(CarControls,engineBrakeDn)==0x1B),"bad off");
		static_assert((offsetof(CarControls,MGUKDeliveryUp)==0x1C),"bad off");
		static_assert((offsetof(CarControls,MGUKDeliveryDn)==0x1D),"bad off");
		static_assert((offsetof(CarControls,MGUKRecoveryUp)==0x1E),"bad off");
		static_assert((offsetof(CarControls,MGUKRecoveryDn)==0x1F),"bad off");
		static_assert((offsetof(CarControls,MGUHMode)==0x20),"bad off");
		static_assert((offsetof(CarControls,gas)==0x24),"bad off");
		static_assert((offsetof(CarControls,brake)==0x28),"bad off");
		static_assert((offsetof(CarControls,steer)==0x2C),"bad off");
		static_assert((offsetof(CarControls,clutch)==0x30),"bad off");
	};
};

//UDT: struct TyreModelInput @len=48
	//_Data: this+0x0, Member, Type: float, load
	//_Data: this+0x4, Member, Type: float, slipAngleRAD
	//_Data: this+0x8, Member, Type: float, slipRatio
	//_Data: this+0xC, Member, Type: float, camberRAD
	//_Data: this+0x10, Member, Type: float, speed
	//_Data: this+0x14, Member, Type: float, u
	//_Data: this+0x18, Member, Type: int, tyreIndex
	//_Data: this+0x1C, Member, Type: float, cpLength
	//_Data: this+0x20, Member, Type: float, grain
	//_Data: this+0x24, Member, Type: float, blister
	//_Data: this+0x28, Member, Type: float, pressureRatio
	//_Data: this+0x2C, Member, Type: bool, useSimpleModel
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreModelInput)==48),"bad size");
		static_assert((offsetof(TyreModelInput,load)==0x0),"bad off");
		static_assert((offsetof(TyreModelInput,slipAngleRAD)==0x4),"bad off");
		static_assert((offsetof(TyreModelInput,slipRatio)==0x8),"bad off");
		static_assert((offsetof(TyreModelInput,camberRAD)==0xC),"bad off");
		static_assert((offsetof(TyreModelInput,speed)==0x10),"bad off");
		static_assert((offsetof(TyreModelInput,u)==0x14),"bad off");
		static_assert((offsetof(TyreModelInput,tyreIndex)==0x18),"bad off");
		static_assert((offsetof(TyreModelInput,cpLength)==0x1C),"bad off");
		static_assert((offsetof(TyreModelInput,grain)==0x20),"bad off");
		static_assert((offsetof(TyreModelInput,blister)==0x24),"bad off");
		static_assert((offsetof(TyreModelInput,pressureRatio)==0x28),"bad off");
		static_assert((offsetof(TyreModelInput,useSimpleModel)==0x2C),"bad off");
	};
};

//UDT: class CBuffer @len=40
	//_Func: public void CBuffer(); @loc=static @len=27 @rva=2201856
	//_Func: public void CBuffer(int islot, int isize); @loc=static @len=104 @rva=2201744
	//_Func: public void ~CBuffer(); @loc=static @len=55 @rva=2202128
	//_Data: this+0x0, Member, Type: int, size
	//_Data: this+0x4, Member, Type: int, slot
	//_Data: this+0x8, Member, Type: bool, isSystem
	//_Data: this+0x9, Member, Type: bool, isPS
	//_Data: this+0xA, Member, Type: bool, isVS
	//_Data: this+0x10, Member, Type: void *, kid
	//_Func: public void set(int * value, int offset, int size); @loc=static @len=37 @rva=2202192
	//_Func: public void set(float * value, int offset, int size); @loc=static @len=37 @rva=2202192
	//_Func: public void get(int *  _arg0, int  _arg1, int  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public void get(float * value, int offset, int size); @loc=static @len=24 @rva=2201968
	//_Func: public void commit(); @loc=static @len=77 @rva=2201888
	//_Func: public void touch(); @loc=static @len=5 @rva=2202240
	//_Func: public void map(void * bdata, int bsize); @loc=static @len=28 @rva=2202096
	//_Func: public void init(int islot, int isize); @loc=static @len=82 @rva=2202000
	//_Func: public void release(); @loc=static @len=55 @rva=2202128
	//_Data: this+0x18, Member, Type: unsigned char *, data
	//_Data: this+0x20, Member, Type: bool, touched
	//_Func: public void __autoclassinit2(unsigned __int64  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CBuffer)==40),"bad size");
		static_assert((offsetof(CBuffer,size)==0x0),"bad off");
		static_assert((offsetof(CBuffer,slot)==0x4),"bad off");
		static_assert((offsetof(CBuffer,isSystem)==0x8),"bad off");
		static_assert((offsetof(CBuffer,isPS)==0x9),"bad off");
		static_assert((offsetof(CBuffer,isVS)==0xA),"bad off");
		static_assert((offsetof(CBuffer,kid)==0x10),"bad off");
		static_assert((offsetof(CBuffer,data)==0x18),"bad off");
		static_assert((offsetof(CBuffer,touched)==0x20),"bad off");
	};
};

//UDT: struct EngineStatus @len=24
	//_Data: this+0x0, Member, Type: double, outTorque
	//_Data: this+0x8, Member, Type: double, externalCoastTorque
	//_Data: this+0x10, Member, Type: float, turboBoost
	//_Data: this+0x14, Member, Type: bool, isLimiterOn
	//_Func: public void EngineStatus(); @loc=optimized @len=0 @rva=0
//UDT;

struct EngineStatus {
public:
	double outTorque;
	double externalCoastTorque;
	float turboBoost;
	bool isLimiterOn;
	inline EngineStatus()  { }
	inline void _guard_obj() {
		static_assert((sizeof(EngineStatus)==24),"bad size");
		static_assert((offsetof(EngineStatus,outTorque)==0x0),"bad off");
		static_assert((offsetof(EngineStatus,externalCoastTorque)==0x8),"bad off");
		static_assert((offsetof(EngineStatus,turboBoost)==0x10),"bad off");
		static_assert((offsetof(EngineStatus,isLimiterOn)==0x14),"bad off");
	};
};

//UDT: struct HeaveSpringStatus @len=4
	//_Func: public void HeaveSpringStatus(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, travel
//UDT;

struct HeaveSpringStatus {
public:
	float travel;
	inline HeaveSpringStatus()  { }
	inline void _guard_obj() {
		static_assert((sizeof(HeaveSpringStatus)==4),"bad size");
		static_assert((offsetof(HeaveSpringStatus,travel)==0x0),"bad off");
	};
};

//UDT: struct TyreSlipInput @len=24
	//_Func: public void TyreSlipInput(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, slip
	//_Data: this+0x4, Member, Type: float, friction
	//_Data: this+0x8, Member, Type: float, load
	//_Data: this+0xC, Member, Type: float, normalizedSlipX
	//_Data: this+0x10, Member, Type: float, normalizedSlipY
	//_Data: this+0x14, Member, Type: float, D
//UDT;

struct TyreSlipInput {
public:
	float slip;
	float friction;
	float load;
	float normalizedSlipX;
	float normalizedSlipY;
	float D;
	inline TyreSlipInput()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TyreSlipInput)==24),"bad size");
		static_assert((offsetof(TyreSlipInput,slip)==0x0),"bad off");
		static_assert((offsetof(TyreSlipInput,friction)==0x4),"bad off");
		static_assert((offsetof(TyreSlipInput,load)==0x8),"bad off");
		static_assert((offsetof(TyreSlipInput,normalizedSlipX)==0xC),"bad off");
		static_assert((offsetof(TyreSlipInput,normalizedSlipY)==0x10),"bad off");
		static_assert((offsetof(TyreSlipInput,D)==0x14),"bad off");
	};
};

//UDT: class Damper @len=24
	//_Func: public void Damper(); @loc=static @len=45 @rva=2830928
	//_Func: public void ~Damper(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: float, reboundSlow
	//_Data: this+0x4, Member, Type: float, reboundFast
	//_Data: this+0x8, Member, Type: float, bumpSlow
	//_Data: this+0xC, Member, Type: float, bumpFast
	//_Data: this+0x10, Member, Type: float, fastThresholdBump
	//_Data: this+0x14, Member, Type: float, fastThresholdRebound
	//_Func: public float getForce(float v); @loc=static @len=119 @rva=2830976
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Damper)==24),"bad size");
		static_assert((offsetof(Damper,reboundSlow)==0x0),"bad off");
		static_assert((offsetof(Damper,reboundFast)==0x4),"bad off");
		static_assert((offsetof(Damper,bumpSlow)==0x8),"bad off");
		static_assert((offsetof(Damper,bumpFast)==0xC),"bad off");
		static_assert((offsetof(Damper,fastThresholdBump)==0x10),"bad off");
		static_assert((offsetof(Damper,fastThresholdRebound)==0x14),"bad off");
	};
};

//UDT: struct ServerDrivingAssists @len=12
	//_Data: this+0x0, Member, Type: int, tc
	//_Data: this+0x4, Member, Type: int, abs
	//_Data: this+0x8, Member, Type: bool, stability
	//_Data: this+0x9, Member, Type: bool, autoClutch
	//_Func: public void ServerDrivingAssists(); @loc=optimized @len=0 @rva=0
//UDT;

struct ServerDrivingAssists {
public:
	int tc;
	int abs;
	bool stability;
	bool autoClutch;
	inline ServerDrivingAssists()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ServerDrivingAssists)==12),"bad size");
		static_assert((offsetof(ServerDrivingAssists,tc)==0x0),"bad off");
		static_assert((offsetof(ServerDrivingAssists,abs)==0x4),"bad off");
		static_assert((offsetof(ServerDrivingAssists,stability)==0x8),"bad off");
		static_assert((offsetof(ServerDrivingAssists,autoClutch)==0x9),"bad off");
	};
};

//UDT: struct PerformanceSplit @len=16
	//_Data: this+0x0, Member, Type: double, t
	//_Data: this+0x8, Member, Type: float, speedMS
	//_Func: public void PerformanceSplit(); @loc=optimized @len=0 @rva=0
//UDT;

struct PerformanceSplit {
public:
	double t;
	float speedMS;
	inline PerformanceSplit()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PerformanceSplit)==16),"bad size");
		static_assert((offsetof(PerformanceSplit,t)==0x0),"bad off");
		static_assert((offsetof(PerformanceSplit,speedMS)==0x8),"bad off");
	};
};

//UDT: struct NetCarPushToPass @len=20
	//_Data: this+0x0, Member, Type: bool, enabled
	//_Data: this+0x1, Member, Type: bool, active
	//_Data: this+0x4, Member, Type: float, coolDownS
	//_Data: this+0x8, Member, Type: float, timeS
	//_Data: this+0xC, Member, Type: float, timeAccum
	//_Data: this+0x10, Member, Type: int, activations
	//_Func: public void NetCarPushToPass(); @loc=optimized @len=0 @rva=0
//UDT;

struct NetCarPushToPass {
public:
	bool enabled;
	bool active;
	float coolDownS;
	float timeS;
	float timeAccum;
	int activations;
	inline NetCarPushToPass()  { }
	inline void _guard_obj() {
		static_assert((sizeof(NetCarPushToPass)==20),"bad size");
		static_assert((offsetof(NetCarPushToPass,enabled)==0x0),"bad off");
		static_assert((offsetof(NetCarPushToPass,active)==0x1),"bad off");
		static_assert((offsetof(NetCarPushToPass,coolDownS)==0x4),"bad off");
		static_assert((offsetof(NetCarPushToPass,timeS)==0x8),"bad off");
		static_assert((offsetof(NetCarPushToPass,timeAccum)==0xC),"bad off");
		static_assert((offsetof(NetCarPushToPass,activations)==0x10),"bad off");
	};
};

//UDT: class Speed @len=4
	//_Func: private void Speed(float v); @loc=static @len=8 @rva=2333072
	//_Func: public void Speed(); @loc=static @len=10 @rva=2333024
	//_Func: public void ~Speed(); @loc=static @len=3 @rva=96368
	//_Func: public Speed fromMS(float ms); @pure @loc=static @len=8 @rva=2333072
	//_Func: public Speed fromKMH(float ms); @pure @loc=static @len=16 @rva=2333040
	//_Func: public Speed fromMPH(float ms); @pure @loc=static @len=16 @rva=2333056
	//_Data: static, [0155A5AE][0003:000475AE], Static Member, Type: bool, useMPH
	//_Func: public float ms(); @loc=optimized @len=0 @rva=0
	//_Func: public float kmh(); @loc=static @len=13 @rva=364368
	//_Func: public float mph(); @loc=optimized @len=0 @rva=0
	//_Func: public float toSystemDefault(); @loc=optimized @len=0 @rva=0
	//_Func: public Speed operator-(Speed &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Speed operator+(Speed &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Speed operator*(float v2); @loc=static @len=41 @rva=2502224
	//_Func: public Speed operator/(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, value
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class Speed {
public:
	float value;
	inline Speed()  { }
	inline void ctor(float v) { typedef void (*_fpt)(Speed *pthis, float); _fpt _f=(_fpt)_drva(2333072); _f(this, v); }
	inline void ctor() { typedef void (*_fpt)(Speed *pthis); _fpt _f=(_fpt)_drva(2333024); _f(this); }
	inline void dtor() { typedef void (*_fpt)(Speed *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline float kmh() { typedef float (*_fpt)(Speed *pthis); _fpt _f=(_fpt)_drva(364368); return _f(this); }
	inline Speed operator*(float v2) { typedef Speed (*_fpt)(Speed *pthis, float); _fpt _f=(_fpt)_drva(2502224); return _f(this, v2); }
	inline void _guard_obj() {
		static_assert((sizeof(Speed)==4),"bad size");
		static_assert((offsetof(Speed,value)==0x0),"bad off");
	};
};

//UDT: struct CommandItem @len=4
	//_Func: public void CommandItem(int akey); @loc=static @len=6 @rva=953184
	//_Func: public void CommandItem(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, key
//UDT;

struct CommandItem {
public:
	int key;
	inline CommandItem()  { }
	inline void ctor(int akey) { typedef void (*_fpt)(CommandItem *pthis, int); _fpt _f=(_fpt)_drva(953184); _f(this, akey); }
	inline void _guard_obj() {
		static_assert((sizeof(CommandItem)==4),"bad size");
		static_assert((offsetof(CommandItem,key)==0x0),"bad off");
	};
};

//UDT: struct GearElement @len=24
	//_Data: this+0x0, Member, Type: double, velocity
	//_Data: this+0x8, Member, Type: double, inertia
	//_Data: this+0x10, Member, Type: double, oldVelocity
	//_Func: public void GearElement(); @loc=optimized @len=0 @rva=0
//UDT;

struct GearElement {
public:
	double velocity;
	double inertia;
	double oldVelocity;
	inline GearElement()  { }
	inline void _guard_obj() {
		static_assert((sizeof(GearElement)==24),"bad size");
		static_assert((offsetof(GearElement,velocity)==0x0),"bad off");
		static_assert((offsetof(GearElement,inertia)==0x8),"bad off");
		static_assert((offsetof(GearElement,oldVelocity)==0x10),"bad off");
	};
};

//UDT: struct HDRLevels @len=8
	//_Func: public void HDRLevels(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, minExposure
	//_Data: this+0x4, Member, Type: float, maxExposure
//UDT;

struct HDRLevels {
public:
	float minExposure;
	float maxExposure;
	inline HDRLevels()  { }
	inline void _guard_obj() {
		static_assert((sizeof(HDRLevels)==8),"bad size");
		static_assert((offsetof(HDRLevels,minExposure)==0x0),"bad off");
		static_assert((offsetof(HDRLevels,maxExposure)==0x4),"bad off");
	};
};

//UDT: struct ACCarState @len=328
	//_Data: this+0x0, Member, Type: float[0x3], wheelLF_localPos
	//_Data: this+0xC, Member, Type: float[0x3], wheelRF_localPos
	//_Data: this+0x18, Member, Type: float[0x3], wheelLR_localPos
	//_Data: this+0x24, Member, Type: float[0x3], wheelRR_localPos
	//_Data: this+0x30, Member, Type: float[0x3], localVelocity
	//_Data: this+0x3C, Member, Type: float[0x3], worldVelocity
	//_Data: this+0x48, Member, Type: float[0x3], accG
	//_Data: this+0x54, Member, Type: float, engineRPMS
	//_Data: this+0x58, Member, Type: float[0x3], worldPosition
	//_Data: this+0x64, Member, Type: float[0x10], bodyMatrix
	//_Data: this+0xA4, Member, Type: int, gear
	//_Data: this+0xA8, Member, Type: bool, isEngineLimiterOn
	//_Data: this+0xAC, Member, Type: float[0x4], wheelAngularSpeed
	//_Data: this+0xBC, Member, Type: float, steer
	//_Data: this+0xC0, Member, Type: float, gas
	//_Data: this+0xC4, Member, Type: float, brake
	//_Data: this+0xC8, Member, Type: float, clutch
	//_Data: this+0xCC, Member, Type: float[0x3], localAngularVelocity
	//_Data: this+0xD8, Member, Type: float[0x4], ndSlip
	//_Data: this+0xE8, Member, Type: float[0x4], load
	//_Data: this+0xF8, Member, Type: float[0x4], Mz
	//_Data: this+0x108, Member, Type: float[0x4], tyreDirtyLevel
	//_Data: this+0x118, Member, Type: float, lastFF
	//_Data: this+0x11C, Member, Type: float, drivetrainSpeed
	//_Data: this+0x120, Member, Type: float, turboBoost
	//_Data: this+0x124, Member, Type: float, performanceMeter
	//_Data: this+0x128, Member, Type: bool, isGearGrinding
	//_Data: this+0x12C, Member, Type: float[0x5], damageZoneLevel
	//_Data: this+0x140, Member, Type: int, limiterRPM
	//_Data: this+0x144, Member, Type: float, speedMS
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ACCarState)==328),"bad size");
		static_assert((offsetof(ACCarState,wheelLF_localPos)==0x0),"bad off");
		static_assert((offsetof(ACCarState,wheelRF_localPos)==0xC),"bad off");
		static_assert((offsetof(ACCarState,wheelLR_localPos)==0x18),"bad off");
		static_assert((offsetof(ACCarState,wheelRR_localPos)==0x24),"bad off");
		static_assert((offsetof(ACCarState,localVelocity)==0x30),"bad off");
		static_assert((offsetof(ACCarState,worldVelocity)==0x3C),"bad off");
		static_assert((offsetof(ACCarState,accG)==0x48),"bad off");
		static_assert((offsetof(ACCarState,engineRPMS)==0x54),"bad off");
		static_assert((offsetof(ACCarState,worldPosition)==0x58),"bad off");
		static_assert((offsetof(ACCarState,bodyMatrix)==0x64),"bad off");
		static_assert((offsetof(ACCarState,gear)==0xA4),"bad off");
		static_assert((offsetof(ACCarState,isEngineLimiterOn)==0xA8),"bad off");
		static_assert((offsetof(ACCarState,wheelAngularSpeed)==0xAC),"bad off");
		static_assert((offsetof(ACCarState,steer)==0xBC),"bad off");
		static_assert((offsetof(ACCarState,gas)==0xC0),"bad off");
		static_assert((offsetof(ACCarState,brake)==0xC4),"bad off");
		static_assert((offsetof(ACCarState,clutch)==0xC8),"bad off");
		static_assert((offsetof(ACCarState,localAngularVelocity)==0xCC),"bad off");
		static_assert((offsetof(ACCarState,ndSlip)==0xD8),"bad off");
		static_assert((offsetof(ACCarState,load)==0xE8),"bad off");
		static_assert((offsetof(ACCarState,Mz)==0xF8),"bad off");
		static_assert((offsetof(ACCarState,tyreDirtyLevel)==0x108),"bad off");
		static_assert((offsetof(ACCarState,lastFF)==0x118),"bad off");
		static_assert((offsetof(ACCarState,drivetrainSpeed)==0x11C),"bad off");
		static_assert((offsetof(ACCarState,turboBoost)==0x120),"bad off");
		static_assert((offsetof(ACCarState,performanceMeter)==0x124),"bad off");
		static_assert((offsetof(ACCarState,isGearGrinding)==0x128),"bad off");
		static_assert((offsetof(ACCarState,damageZoneLevel)==0x12C),"bad off");
		static_assert((offsetof(ACCarState,limiterRPM)==0x140),"bad off");
		static_assert((offsetof(ACCarState,speedMS)==0x144),"bad off");
	};
};

//UDT: struct WingOverrideDef @len=8
	//_Data: this+0x0, Member, Type: float, overrideAngle
	//_Data: this+0x4, Member, Type: bool, isActive
	//_Func: public void WingOverrideDef(); @loc=optimized @len=0 @rva=0
//UDT;

struct WingOverrideDef {
public:
	float overrideAngle;
	bool isActive;
	inline WingOverrideDef()  { }
	inline void _guard_obj() {
		static_assert((sizeof(WingOverrideDef)==8),"bad size");
		static_assert((offsetof(WingOverrideDef,overrideAngle)==0x0),"bad off");
		static_assert((offsetof(WingOverrideDef,isActive)==0x4),"bad off");
	};
};

//UDT: struct SurfaceDef @len=200
	//_Func: public void SurfaceDef(SurfaceDef &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SurfaceDef(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: wchar_t[0x40], wavString
	//_Data: this+0x80, Member, Type: float, wavPitchSpeed
	//_Data: this+0x88, Member, Type: void *, userPointer
	//_Data: this+0x90, Member, Type: float, gripMod
	//_Data: this+0x94, Member, Type: int, sectorID
	//_Data: this+0x98, Member, Type: float, dirtAdditiveK
	//_Data: this+0x9C, Member, Type: unsigned int, collisionCategory
	//_Data: this+0xA0, Member, Type: bool, isValidTrack
	//_Data: this+0xA4, Member, Type: float, blackFlagTime
	//_Data: this+0xA8, Member, Type: float, sinHeight
	//_Data: this+0xAC, Member, Type: float, sinLength
	//_Data: this+0xB0, Member, Type: bool, isPitlane
	//_Data: this+0xB4, Member, Type: float, damping
	//_Data: this+0xB8, Member, Type: float, granularity
	//_Data: this+0xBC, Member, Type: float, vibrationGain
	//_Data: this+0xC0, Member, Type: float, vibrationLength
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SurfaceDef)==200),"bad size");
		static_assert((offsetof(SurfaceDef,wavString)==0x0),"bad off");
		static_assert((offsetof(SurfaceDef,wavPitchSpeed)==0x80),"bad off");
		static_assert((offsetof(SurfaceDef,userPointer)==0x88),"bad off");
		static_assert((offsetof(SurfaceDef,gripMod)==0x90),"bad off");
		static_assert((offsetof(SurfaceDef,sectorID)==0x94),"bad off");
		static_assert((offsetof(SurfaceDef,dirtAdditiveK)==0x98),"bad off");
		static_assert((offsetof(SurfaceDef,collisionCategory)==0x9C),"bad off");
		static_assert((offsetof(SurfaceDef,isValidTrack)==0xA0),"bad off");
		static_assert((offsetof(SurfaceDef,blackFlagTime)==0xA4),"bad off");
		static_assert((offsetof(SurfaceDef,sinHeight)==0xA8),"bad off");
		static_assert((offsetof(SurfaceDef,sinLength)==0xAC),"bad off");
		static_assert((offsetof(SurfaceDef,isPitlane)==0xB0),"bad off");
		static_assert((offsetof(SurfaceDef,damping)==0xB4),"bad off");
		static_assert((offsetof(SurfaceDef,granularity)==0xB8),"bad off");
		static_assert((offsetof(SurfaceDef,vibrationGain)==0xBC),"bad off");
		static_assert((offsetof(SurfaceDef,vibrationLength)==0xC0),"bad off");
	};
};

//UDT: class vec2f @len=8
	//_Data: this+0x0, Member, Type: float, x
	//_Data: this+0x4, Member, Type: float, y
	//_Func: public void vec2f(float ix, float iy); @loc=static @len=13 @rva=216944
	//_Func: public void vec2f(); @loc=optimized @len=0 @rva=0
	//_Func: public float length(); @loc=optimized @len=0 @rva=0
	//_Func: public void normalize(); @loc=optimized @len=0 @rva=0
//UDT;

class vec2f {
public:
	float x;
	float y;
	inline vec2f()  { }
	inline void ctor(float ix, float iy) { typedef void (*_fpt)(vec2f *pthis, float, float); _fpt _f=(_fpt)_drva(216944); _f(this, ix, iy); }
	inline void _guard_obj() {
		static_assert((sizeof(vec2f)==8),"bad size");
		static_assert((offsetof(vec2f,x)==0x0),"bad off");
		static_assert((offsetof(vec2f,y)==0x4),"bad off");
	};
};

//UDT: struct TyreModelOutput @len=28
	//_Data: this+0x0, Member, Type: float, Fy
	//_Data: this+0x4, Member, Type: float, Fx
	//_Data: this+0x8, Member, Type: float, Mz
	//_Data: this+0xC, Member, Type: float, trail
	//_Data: this+0x10, Member, Type: float, ndSlip
	//_Data: this+0x14, Member, Type: float, Dy
	//_Data: this+0x18, Member, Type: float, Dx
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreModelOutput)==28),"bad size");
		static_assert((offsetof(TyreModelOutput,Fy)==0x0),"bad off");
		static_assert((offsetof(TyreModelOutput,Fx)==0x4),"bad off");
		static_assert((offsetof(TyreModelOutput,Mz)==0x8),"bad off");
		static_assert((offsetof(TyreModelOutput,trail)==0xC),"bad off");
		static_assert((offsetof(TyreModelOutput,ndSlip)==0x10),"bad off");
		static_assert((offsetof(TyreModelOutput,Dy)==0x14),"bad off");
		static_assert((offsetof(TyreModelOutput,Dx)==0x18),"bad off");
	};
};

//UDT: struct ClientRules @len=4
	//_Data: this+0x0, Member, Type: float, maxMetersWrongWay
	//_Func: public void ClientRules(); @loc=optimized @len=0 @rva=0
//UDT;

struct ClientRules {
public:
	float maxMetersWrongWay;
	inline ClientRules()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ClientRules)==4),"bad size");
		static_assert((offsetof(ClientRules,maxMetersWrongWay)==0x0),"bad off");
	};
};

//UDT: struct PitStopTimings @len=20
	//_Data: this+0x0, Member, Type: float, tyreChangeTimeSec
	//_Data: this+0x4, Member, Type: float, fuelChangeTimeSec
	//_Data: this+0x8, Member, Type: float, bodyRepairTimeSec
	//_Data: this+0xC, Member, Type: float, engineRepairTimeSec
	//_Data: this+0x10, Member, Type: float, suspRepairTimeSec
	//_Func: public void PitStopTimings(); @loc=optimized @len=0 @rva=0
//UDT;

struct PitStopTimings {
public:
	float tyreChangeTimeSec;
	float fuelChangeTimeSec;
	float bodyRepairTimeSec;
	float engineRepairTimeSec;
	float suspRepairTimeSec;
	inline PitStopTimings()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PitStopTimings)==20),"bad size");
		static_assert((offsetof(PitStopTimings,tyreChangeTimeSec)==0x0),"bad off");
		static_assert((offsetof(PitStopTimings,fuelChangeTimeSec)==0x4),"bad off");
		static_assert((offsetof(PitStopTimings,bodyRepairTimeSec)==0x8),"bad off");
		static_assert((offsetof(PitStopTimings,engineRepairTimeSec)==0xC),"bad off");
		static_assert((offsetof(PitStopTimings,suspRepairTimeSec)==0x10),"bad off");
	};
};

//UDT: struct ksgui::GUI::FormData @len=16
	//_Data: this+0x0, Member, Type: float, x
	//_Data: this+0x4, Member, Type: float, y
	//_Data: this+0x8, Member, Type: bool, visible
	//_Data: this+0x9, Member, Type: bool, blocked
	//_Data: this+0xC, Member, Type: float, scale
	//_Func: public void FormData(); @loc=optimized @len=0 @rva=0
//UDT;

struct ksgui_GUI_FormData {
public:
	float x;
	float y;
	bool visible;
	bool blocked;
	float scale;
	inline ksgui_GUI_FormData()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_GUI_FormData)==16),"bad size");
		static_assert((offsetof(ksgui_GUI_FormData,x)==0x0),"bad off");
		static_assert((offsetof(ksgui_GUI_FormData,y)==0x4),"bad off");
		static_assert((offsetof(ksgui_GUI_FormData,visible)==0x8),"bad off");
		static_assert((offsetof(ksgui_GUI_FormData,blocked)==0x9),"bad off");
		static_assert((offsetof(ksgui_GUI_FormData,scale)==0xC),"bad off");
	};
};

//UDT: class Trigger @len=12
	//_Func: public void Trigger(); @loc=static @len=13 @rva=2339728
	//_Func: public void ~Trigger(); @loc=static @len=3 @rva=96368
	//_Func: public bool hasSwitchedState(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool ignoreSubsequentTrue(bool value); @loc=static @len=32 @rva=2339744
	//_Func: public bool keepSteady(float dt, bool value); @loc=static @len=60 @rva=2339776
	//_Func: public bool turnOn(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool turnOff(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setAccumulatorLimit(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: bool, state
	//_Data: this+0x1, Member, Type: bool, lastState
	//_Data: this+0x4, Member, Type: float, accumulator
	//_Data: this+0x8, Member, Type: float, accumulatorLimit
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Trigger)==12),"bad size");
		static_assert((offsetof(Trigger,state)==0x0),"bad off");
		static_assert((offsetof(Trigger,lastState)==0x1),"bad off");
		static_assert((offsetof(Trigger,accumulator)==0x4),"bad off");
		static_assert((offsetof(Trigger,accumulatorLimit)==0x8),"bad off");
	};
};

//UDT: struct SteerMzLowSpeedReduction @len=8
	//_Data: this+0x0, Member, Type: float, speedKMH
	//_Data: this+0x4, Member, Type: float, minValue
	//_Func: public void SteerMzLowSpeedReduction(); @loc=optimized @len=0 @rva=0
//UDT;

struct SteerMzLowSpeedReduction {
public:
	float speedKMH;
	float minValue;
	inline SteerMzLowSpeedReduction()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SteerMzLowSpeedReduction)==8),"bad size");
		static_assert((offsetof(SteerMzLowSpeedReduction,speedKMH)==0x0),"bad off");
		static_assert((offsetof(SteerMzLowSpeedReduction,minValue)==0x4),"bad off");
	};
};

//UDT: struct CoreCPUTimes @len=24
	//_Data: this+0x0, Member, Type: double, solverTime
	//_Data: this+0x8, Member, Type: double, collisionTime
	//_Data: this+0x10, Member, Type: int, contactPoints
	//_Data: this+0x14, Member, Type: int, narrowPhaseTests
	//_Func: public void CoreCPUTimes(); @loc=optimized @len=0 @rva=0
//UDT;

struct CoreCPUTimes {
public:
	double solverTime;
	double collisionTime;
	int contactPoints;
	int narrowPhaseTests;
	inline CoreCPUTimes()  { }
	inline void _guard_obj() {
		static_assert((sizeof(CoreCPUTimes)==24),"bad size");
		static_assert((offsetof(CoreCPUTimes,solverTime)==0x0),"bad off");
		static_assert((offsetof(CoreCPUTimes,collisionTime)==0x8),"bad off");
		static_assert((offsetof(CoreCPUTimes,contactPoints)==0x10),"bad off");
		static_assert((offsetof(CoreCPUTimes,narrowPhaseTests)==0x14),"bad off");
	};
};

//UDT: struct DownshiftProtection @len=12
	//_Data: this+0x0, Member, Type: bool, isActive
	//_Data: this+0x1, Member, Type: bool, isDebug
	//_Data: this+0x4, Member, Type: int, overrev
	//_Data: this+0x8, Member, Type: bool, lockN
	//_Func: public void DownshiftProtection(); @loc=optimized @len=0 @rva=0
//UDT;

struct DownshiftProtection {
public:
	bool isActive;
	bool isDebug;
	int overrev;
	bool lockN;
	inline DownshiftProtection()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DownshiftProtection)==12),"bad size");
		static_assert((offsetof(DownshiftProtection,isActive)==0x0),"bad off");
		static_assert((offsetof(DownshiftProtection,isDebug)==0x1),"bad off");
		static_assert((offsetof(DownshiftProtection,overrev)==0x4),"bad off");
		static_assert((offsetof(DownshiftProtection,lockN)==0x8),"bad off");
	};
};

//UDT: struct TurboDef @len=28
	//_Func: public void TurboDef(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, maxBoost
	//_Data: this+0x4, Member, Type: float, lagUP
	//_Data: this+0x8, Member, Type: float, lagDN
	//_Data: this+0xC, Member, Type: float, rpmRef
	//_Data: this+0x10, Member, Type: float, gamma
	//_Data: this+0x14, Member, Type: float, wastegate
	//_Data: this+0x18, Member, Type: bool, isAdjustable
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TurboDef)==28),"bad size");
		static_assert((offsetof(TurboDef,maxBoost)==0x0),"bad off");
		static_assert((offsetof(TurboDef,lagUP)==0x4),"bad off");
		static_assert((offsetof(TurboDef,lagDN)==0x8),"bad off");
		static_assert((offsetof(TurboDef,rpmRef)==0xC),"bad off");
		static_assert((offsetof(TurboDef,gamma)==0x10),"bad off");
		static_assert((offsetof(TurboDef,wastegate)==0x14),"bad off");
		static_assert((offsetof(TurboDef,isAdjustable)==0x18),"bad off");
	};
};

//UDT: struct OnWindowClosedEvent @len=8
	//_Func: public void OnWindowClosedEvent(RenderWindow *  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class RenderWindow *, renderWindow
//UDT;

struct OnWindowClosedEvent {
public:
	RenderWindow * renderWindow;
	inline OnWindowClosedEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnWindowClosedEvent)==8),"bad size");
		static_assert((offsetof(OnWindowClosedEvent,renderWindow)==0x0),"bad off");
	};
};

//UDT: struct ClientRemoteCarDef @len=216
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, model
	//_Data: this+0x20, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, config
	//_Data: this+0x40, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, skin
	//_Data: this+0x60, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, driverName
	//_Data: this+0x80, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, team
	//_Data: this+0xA0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, nationCode
	//_Data: this+0xC0, Member, Type: unsigned char, sessionID
	//_Data: this+0xC4, Member, Type: float[0x5], damageZoneLevel
	//_Func: public void ClientRemoteCarDef(ClientRemoteCarDef & __that); @loc=static @len=314 @rva=241776
	//_Func: public void ClientRemoteCarDef(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~ClientRemoteCarDef(); @loc=static @len=246 @rva=246128
	//_Func: public ClientRemoteCarDef & operator=(ClientRemoteCarDef &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ClientRemoteCarDef)==216),"bad size");
		static_assert((offsetof(ClientRemoteCarDef,model)==0x0),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,config)==0x20),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,skin)==0x40),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,driverName)==0x60),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,team)==0x80),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,nationCode)==0xA0),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,sessionID)==0xC0),"bad off");
		static_assert((offsetof(ClientRemoteCarDef,damageZoneLevel)==0xC4),"bad off");
	};
};

//UDT: struct DriftModeComponent @len=80
	//_Func: public void ~DriftModeComponent(); @loc=static @len=3 @rva=96368
	//_Func: public double getPoints(); @loc=optimized @len=0 @rva=0
	//_Func: public double getInstantPoints(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isExtremeDrifting(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isValid(); @loc=optimized @len=0 @rva=0
	//_Func: public double getCurrentSpeedMult(); @loc=optimized @len=0 @rva=0
	//_Func: public double getCurrentDriftAngle(); @loc=optimized @len=0 @rva=0
	//_Func: public int getComboCounter(); @loc=optimized @len=0 @rva=0
	//_Func: public void init(Car * car); @loc=static @len=380 @rva=2882976
	//_Func: public void step(float dt); @loc=static @len=846 @rva=2883360
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: float, driftStraightTimer
	//_Data: this+0x10, Member, Type: double, points
	//_Data: this+0x18, Member, Type: double, instantDrift
	//_Data: this+0x20, Member, Type: double, currentSpeedMultiplier
	//_Data: this+0x28, Member, Type: double, currentDriftAngle
	//_Data: this+0x30, Member, Type: int, comboCounter
	//_Data: this+0x34, Member, Type: int, lastDriftDirection
	//_Data: this+0x38, Member, Type: bool, extremeDrifting
	//_Data: this+0x39, Member, Type: bool, drifting
	//_Data: this+0x3A, Member, Type: bool, invalid
	//_Data: this+0x3C, Member, Type: float[0x5], oldDamageZones
	//_Func: private bool checkExtremeDrifting(); @loc=static @len=321 @rva=2882640
	//_Func: private bool hasCarBeenDamaged(); @loc=optimized @len=0 @rva=0
	//_Func: private void validateDrift(); @loc=static @len=347 @rva=2884208
	//_Func: public void DriftModeComponent(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DriftModeComponent)==80),"bad size");
		static_assert((offsetof(DriftModeComponent,car)==0x0),"bad off");
		static_assert((offsetof(DriftModeComponent,driftStraightTimer)==0x8),"bad off");
		static_assert((offsetof(DriftModeComponent,points)==0x10),"bad off");
		static_assert((offsetof(DriftModeComponent,instantDrift)==0x18),"bad off");
		static_assert((offsetof(DriftModeComponent,currentSpeedMultiplier)==0x20),"bad off");
		static_assert((offsetof(DriftModeComponent,currentDriftAngle)==0x28),"bad off");
		static_assert((offsetof(DriftModeComponent,comboCounter)==0x30),"bad off");
		static_assert((offsetof(DriftModeComponent,lastDriftDirection)==0x34),"bad off");
		static_assert((offsetof(DriftModeComponent,extremeDrifting)==0x38),"bad off");
		static_assert((offsetof(DriftModeComponent,drifting)==0x39),"bad off");
		static_assert((offsetof(DriftModeComponent,invalid)==0x3A),"bad off");
		static_assert((offsetof(DriftModeComponent,oldDamageZones)==0x3C),"bad off");
	};
};

//UDT: struct AutoShifter @len=40
	//_Func: public void ~AutoShifter(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: bool, isActive
	//_Data: this+0x4, Member, Type: int, changeUpRpm
	//_Data: this+0x8, Member, Type: int, changeDnRpm
	//_Data: this+0xC, Member, Type: float, slipThreshold
	//_Func: public void init(Car * car); @loc=static @len=44 @rva=2859040
	//_Func: public void step(float dt); @loc=static @len=843 @rva=2861040
	//_Data: this+0x10, Member, Type: class Car *, car
	//_Data: this+0x18, Member, Type: bool, butGearUp
	//_Data: this+0x19, Member, Type: bool, butGearDn
	//_Data: this+0x1C, Member, Type: float, gasCutoff
	//_Data: this+0x20, Member, Type: float, gasCutoffTime
	//_Func: private void loadINI(); @loc=static @len=1946 @rva=2859088
	//_Func: public void AutoShifter(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(AutoShifter)==40),"bad size");
		static_assert((offsetof(AutoShifter,isActive)==0x0),"bad off");
		static_assert((offsetof(AutoShifter,changeUpRpm)==0x4),"bad off");
		static_assert((offsetof(AutoShifter,changeDnRpm)==0x8),"bad off");
		static_assert((offsetof(AutoShifter,slipThreshold)==0xC),"bad off");
		static_assert((offsetof(AutoShifter,car)==0x10),"bad off");
		static_assert((offsetof(AutoShifter,butGearUp)==0x18),"bad off");
		static_assert((offsetof(AutoShifter,butGearDn)==0x19),"bad off");
		static_assert((offsetof(AutoShifter,gasCutoff)==0x1C),"bad off");
		static_assert((offsetof(AutoShifter,gasCutoffTime)==0x20),"bad off");
	};
};

//UDT: struct ClientHandshakeResult @len=144
	//_Data: this+0x0, Member, Type: bool, success
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, model
	//_Data: this+0x28, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, skin
	//_Data: this+0x48, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, track
	//_Data: this+0x68, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, track_config
	//_Data: this+0x88, Member, Type: float, sunAngle
	//_Data: this+0x8C, Member, Type: unsigned char, sessionID
	//_Func: public void ClientHandshakeResult(ClientHandshakeResult & __that); @loc=static @len=229 @rva=241536
	//_Func: public void ClientHandshakeResult(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~ClientHandshakeResult(); @loc=static @len=160 @rva=245968
	//_Func: public ClientHandshakeResult & operator=(ClientHandshakeResult &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ClientHandshakeResult)==144),"bad size");
		static_assert((offsetof(ClientHandshakeResult,success)==0x0),"bad off");
		static_assert((offsetof(ClientHandshakeResult,model)==0x8),"bad off");
		static_assert((offsetof(ClientHandshakeResult,skin)==0x28),"bad off");
		static_assert((offsetof(ClientHandshakeResult,track)==0x48),"bad off");
		static_assert((offsetof(ClientHandshakeResult,track_config)==0x68),"bad off");
		static_assert((offsetof(ClientHandshakeResult,sunAngle)==0x88),"bad off");
		static_assert((offsetof(ClientHandshakeResult,sessionID)==0x8C),"bad off");
	};
};

//UDT: struct EDL @len=56
	//_Func: public void ~EDL(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: bool, isPresent
	//_Data: this+0x1, Member, Type: bool, isActive
	//_Data: this+0x4, Member, Type: float, wheelSpeedGainPower
	//_Data: this+0x8, Member, Type: float, wheelSpeedGainCoast
	//_Data: this+0xC, Member, Type: float, deadZonePower
	//_Data: this+0x10, Member, Type: float, deadZoneCoast
	//_Data: this+0x14, Member, Type: float, brakeTorquePower
	//_Data: this+0x18, Member, Type: float, brakeTorqueCoast
	//_Data: this+0x1C, Member, Type: float, outLevel
	//_Data: this+0x20, Member, Type: float, outBrakeTorque
	//_Data: this+0x24, Member, Type: float, speedDiff
	//_Func: public void step(float td); @loc=static @len=414 @rva=2864224
	//_Func: public void init(Car * a_car); @loc=static @len=2145 @rva=2862064
	//_Data: this+0x28, Member, Type: class Car *, car
	//_Data: this+0x30, Member, Type: int, leftTyreIndex
	//_Data: this+0x34, Member, Type: int, rightTyreIndex
	//_Func: public void EDL(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(EDL)==56),"bad size");
		static_assert((offsetof(EDL,isPresent)==0x0),"bad off");
		static_assert((offsetof(EDL,isActive)==0x1),"bad off");
		static_assert((offsetof(EDL,wheelSpeedGainPower)==0x4),"bad off");
		static_assert((offsetof(EDL,wheelSpeedGainCoast)==0x8),"bad off");
		static_assert((offsetof(EDL,deadZonePower)==0xC),"bad off");
		static_assert((offsetof(EDL,deadZoneCoast)==0x10),"bad off");
		static_assert((offsetof(EDL,brakeTorquePower)==0x14),"bad off");
		static_assert((offsetof(EDL,brakeTorqueCoast)==0x18),"bad off");
		static_assert((offsetof(EDL,outLevel)==0x1C),"bad off");
		static_assert((offsetof(EDL,outBrakeTorque)==0x20),"bad off");
		static_assert((offsetof(EDL,speedDiff)==0x24),"bad off");
		static_assert((offsetof(EDL,car)==0x28),"bad off");
		static_assert((offsetof(EDL,leftTyreIndex)==0x30),"bad off");
		static_assert((offsetof(EDL,rightTyreIndex)==0x34),"bad off");
	};
};

//UDT: class FileChangeObserver @len=48
	//_Func: public void FileChangeObserver(FileChangeObserver &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void FileChangeObserver(); @loc=optimized @len=0 @rva=0
	//_Func: public void FileChangeObserver(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & a_filename); @loc=static @len=55 @rva=2339840
	//_Func: public void ~FileChangeObserver(); @loc=static @len=49 @rva=2180672
	//_Func: public bool hasChanged(); @loc=static @len=47 @rva=2339904
	//_Func: public void reset(); @loc=static @len=162 @rva=2340144
	//_Func: public void observe(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & a_filename); @loc=static @len=191 @rva=2339952
	//_Data: this+0x0, Member, Type: struct _FILETIME, lastFileTime
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, filename
	//_Data: this+0x28, Member, Type: unsigned long, lastChanged
	//_Func: private bool getFileTime(_FILETIME &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public FileChangeObserver & operator=(FileChangeObserver &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(FileChangeObserver)==48),"bad size");
		static_assert((offsetof(FileChangeObserver,lastFileTime)==0x0),"bad off");
		static_assert((offsetof(FileChangeObserver,filename)==0x8),"bad off");
		static_assert((offsetof(FileChangeObserver,lastChanged)==0x28),"bad off");
	};
};

//UDT: struct sockaddr_in @len=16
	//_Data: this+0x0, Member, Type: unsigned short, sin_family
	//_Data: this+0x2, Member, Type: unsigned short, sin_port
	//_Data: this+0x4, Member, Type: struct in_addr, sin_addr
	//_Data: this+0x8, Member, Type: char[0x8], sin_zero
//UDT;

struct sockaddr_in {
public:
	unsigned short sin_family;
	unsigned short sin_port;
	in_addr sin_addr;
	char sin_zero[0x8];
	inline sockaddr_in()  { }
	inline void _guard_obj() {
		static_assert((sizeof(sockaddr_in)==16),"bad size");
		static_assert((offsetof(sockaddr_in,sin_family)==0x0),"bad off");
		static_assert((offsetof(sockaddr_in,sin_port)==0x2),"bad off");
		static_assert((offsetof(sockaddr_in,sin_addr)==0x4),"bad off");
		static_assert((offsetof(sockaddr_in,sin_zero)==0x8),"bad off");
	};
};

//UDT: struct OnReplayStatusChanged @len=12
	//_Func: public void OnReplayStatusChanged(eReplayStatus  _arg0, float  _arg1, float  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: enum eReplayStatus, status
	//_Data: this+0x4, Member, Type: float, timeMult
	//_Data: this+0x8, Member, Type: float, slowMotionLevel
//UDT;

struct OnReplayStatusChanged {
public:
	eReplayStatus status;
	float timeMult;
	float slowMotionLevel;
	inline OnReplayStatusChanged()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnReplayStatusChanged)==12),"bad size");
		static_assert((offsetof(OnReplayStatusChanged,status)==0x0),"bad off");
		static_assert((offsetof(OnReplayStatusChanged,timeMult)==0x4),"bad off");
		static_assert((offsetof(OnReplayStatusChanged,slowMotionLevel)==0x8),"bad off");
	};
};

//UDT: struct GearChanger @len=24
	//_Func: public void ~GearChanger(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: bool, wasGearUpTriggered
	//_Data: this+0x1, Member, Type: bool, wasGearDnTriggered
	//_Func: public void init(Car * car); @loc=static @len=11 @rva=2861888
	//_Func: public void step(float dt); @loc=static @len=146 @rva=2861904
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: bool, lastGearUp
	//_Data: this+0x11, Member, Type: bool, lastGearDn
	//_Func: public void GearChanger(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(GearChanger)==24),"bad size");
		static_assert((offsetof(GearChanger,wasGearUpTriggered)==0x0),"bad off");
		static_assert((offsetof(GearChanger,wasGearDnTriggered)==0x1),"bad off");
		static_assert((offsetof(GearChanger,car)==0x8),"bad off");
		static_assert((offsetof(GearChanger,lastGearUp)==0x10),"bad off");
		static_assert((offsetof(GearChanger,lastGearDn)==0x11),"bad off");
	};
};

//UDT: struct DisconnectCountdown @len=40
	//_Data: this+0x0, Member, Type: bool, isSignaled
	//_Data: this+0x4, Member, Type: float, timeToDisconnection
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, message
	//_Func: public void DisconnectCountdown(DisconnectCountdown &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void DisconnectCountdown(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~DisconnectCountdown(); @loc=static @len=49 @rva=2180672
	//_Func: public DisconnectCountdown & operator=(DisconnectCountdown &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct DisconnectCountdown {
public:
	bool isSignaled;
	float timeToDisconnection;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > message;
	inline DisconnectCountdown()  { }
	inline void dtor() { typedef void (*_fpt)(DisconnectCountdown *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(DisconnectCountdown)==40),"bad size");
		static_assert((offsetof(DisconnectCountdown,isSignaled)==0x0),"bad off");
		static_assert((offsetof(DisconnectCountdown,timeToDisconnection)==0x4),"bad off");
		static_assert((offsetof(DisconnectCountdown,message)==0x8),"bad off");
	};
};

//UDT: struct LapInvalidator @len=32
	//_Func: public void ~LapInvalidator(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: double, collisionSafeTime
	//_Func: public void init(Car * car); @loc=static @len=5 @rva=2865408
	//_Func: public int getCurrentTyresOut(); @loc=optimized @len=0 @rva=0
	//_Func: public void step(float dt); @loc=static @len=402 @rva=2884992
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: int, currentTyresOut
	//_Data: this+0x14, Member, Type: bool, isInPenaltyZone
	//_Data: this+0x18, Member, Type: float, lastBlackFlagTime
	//_Func: private void onEnterPenaltyZone(int tyre_count, float black_flag_time); @loc=static @len=431 @rva=2884560
	//_Func: private void onInPenaltyZone(int  _arg0, float  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void LapInvalidator(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(LapInvalidator)==32),"bad size");
		static_assert((offsetof(LapInvalidator,collisionSafeTime)==0x0),"bad off");
		static_assert((offsetof(LapInvalidator,car)==0x8),"bad off");
		static_assert((offsetof(LapInvalidator,currentTyresOut)==0x10),"bad off");
		static_assert((offsetof(LapInvalidator,isInPenaltyZone)==0x14),"bad off");
		static_assert((offsetof(LapInvalidator,lastBlackFlagTime)==0x18),"bad off");
	};
};

//UDT: struct OnStepCompleteEvent @len=16
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: double, physicsTime
//UDT;

struct OnStepCompleteEvent {
public:
	Car * car;
	double physicsTime;
	inline OnStepCompleteEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnStepCompleteEvent)==16),"bad size");
		static_assert((offsetof(OnStepCompleteEvent,car)==0x0),"bad off");
		static_assert((offsetof(OnStepCompleteEvent,physicsTime)==0x8),"bad off");
	};
};

//UDT: class TCPQueue @len=65540
	//_Func: public void TCPQueue(); @loc=optimized @len=0 @rva=0
	//_Func: public void push(unsigned char * data, unsigned int size); @loc=static @len=215 @rva=2482992
	//_Func: public std::vector<unsigned char,std::allocator<unsigned char> > getPacket(); @loc=static @len=238 @rva=2482752
	//_Data: this+0x0, Member, Type: unsigned char[0x10000], buffer
	//_Data: this+0x10000, Member, Type: unsigned int, cursor
//UDT;

class TCPQueue {
public:
	unsigned char buffer[0x10000];
	unsigned int cursor;
	inline TCPQueue()  { }
	inline void push(unsigned char * data, unsigned int size) { typedef void (*_fpt)(TCPQueue *pthis, unsigned char *, unsigned int); _fpt _f=(_fpt)_drva(2482992); return _f(this, data, size); }
	inline std::vector<unsigned char,std::allocator<unsigned char> > getPacket() { typedef std::vector<unsigned char,std::allocator<unsigned char> > (*_fpt)(TCPQueue *pthis); _fpt _f=(_fpt)_drva(2482752); return _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(TCPQueue)==65540),"bad size");
		static_assert((offsetof(TCPQueue,buffer)==0x0),"bad off");
		static_assert((offsetof(TCPQueue,cursor)==0x10000),"bad off");
	};
};

//UDT: class vec3f @len=12
	//_Data: this+0x0, Member, Type: float, x
	//_Data: this+0x4, Member, Type: float, y
	//_Data: this+0x8, Member, Type: float, z
	//_Func: public void vec3f(double *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void vec3f(float *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void vec3f(float ix, float iy, float iz); @loc=static @len=18 @rva=147424
	//_Func: public void vec3f(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void vec3f(); @loc=optimized @len=0 @rva=0
	//_Func: public void normalize(); @loc=static @len=136 @rva=147456
	//_Func: public bool isZero(); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > toString(); @loc=static @len=322 @rva=340976
	//_Func: public float length(); @loc=optimized @len=0 @rva=0
	//_Func: public float lengthSquared(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isFinite(); @loc=static @len=105 @rva=418800
	//_Func: public vec3f negate(); @loc=optimized @len=0 @rva=0
	//_Func: public void operator+=(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void operator-=(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void operator*=(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void operator/=(float m); @loc=static @len=47 @rva=908800
	//_Func: public void print(char * name); @loc=static @len=61 @rva=918176
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(vec3f)==12),"bad size");
		static_assert((offsetof(vec3f,x)==0x0),"bad off");
		static_assert((offsetof(vec3f,y)==0x4),"bad off");
		static_assert((offsetof(vec3f,z)==0x8),"bad off");
	};
};

//UDT: struct FuelLapEvaluator @len=72
	//_Func: public void ~FuelLapEvaluator(); @loc=static @len=3 @rva=96368
	//_Func: public float getFuelLaps(); @loc=static @len=43 @rva=2675296
	//_Func: public void setIgnoreLap(bool value); @loc=static @len=4 @rva=2675536
	//_Func: public void init(Car * icar); @loc=static @len=145 @rva=2675376
	//_Func: public float getFuelPerLap(); @loc=static @len=26 @rva=2675344
	//_Func: public float getKmPerLiter(); @loc=optimized @len=0 @rva=0
	//_Func: public float getTotalKM(); @loc=optimized @len=0 @rva=0
	//_Func: public void step(float dt); @loc=static @len=152 @rva=2675552
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: int, lapCount
	//_Data: this+0x10, Member, Type: double, lapStartFuel
	//_Data: this+0x18, Member, Type: double, fuelPerLap
	//_Data: this+0x20, Member, Type: double, totalM
	//_Data: this+0x28, Member, Type: double, totalLiters
	//_Data: this+0x30, Member, Type: double, startFuel
	//_Data: this+0x38, Member, Type: double, oldFuelPerLap
	//_Data: this+0x40, Member, Type: bool, ignoreLap
	//_Func: public void FuelLapEvaluator(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(FuelLapEvaluator)==72),"bad size");
		static_assert((offsetof(FuelLapEvaluator,car)==0x0),"bad off");
		static_assert((offsetof(FuelLapEvaluator,lapCount)==0x8),"bad off");
		static_assert((offsetof(FuelLapEvaluator,lapStartFuel)==0x10),"bad off");
		static_assert((offsetof(FuelLapEvaluator,fuelPerLap)==0x18),"bad off");
		static_assert((offsetof(FuelLapEvaluator,totalM)==0x20),"bad off");
		static_assert((offsetof(FuelLapEvaluator,totalLiters)==0x28),"bad off");
		static_assert((offsetof(FuelLapEvaluator,startFuel)==0x30),"bad off");
		static_assert((offsetof(FuelLapEvaluator,oldFuelPerLap)==0x38),"bad off");
		static_assert((offsetof(FuelLapEvaluator,ignoreLap)==0x40),"bad off");
	};
};

//UDT: struct DriverActionsState @len=4
	//_Data: this+0x0, Member, Type: int, state
	//_Func: public void reset(); @loc=optimized @len=0 @rva=0
	//_Func: public void setState(DriverActions  _arg0, bool  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public bool getState(DriverActions  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void DriverActionsState(); @loc=optimized @len=0 @rva=0
//UDT;

struct DriverActionsState {
public:
	int state;
	inline DriverActionsState()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DriverActionsState)==4),"bad size");
		static_assert((offsetof(DriverActionsState,state)==0x0),"bad off");
	};
};

//UDT: struct TimeLineStatus @len=12
	//_Func: public void TimeLineStatus(); @loc=optimized @len=0 @rva=0
	//_Func: public void reset(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: bool, isValid
	//_Data: this+0x4, Member, Type: enum eTimeLineCheckResponse, lastResponse
	//_Data: this+0x8, Member, Type: unsigned int, lastTime
//UDT;

struct TimeLineStatus {
public:
	bool isValid;
	eTimeLineCheckResponse lastResponse;
	unsigned int lastTime;
	inline TimeLineStatus()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TimeLineStatus)==12),"bad size");
		static_assert((offsetof(TimeLineStatus,isValid)==0x0),"bad off");
		static_assert((offsetof(TimeLineStatus,lastResponse)==0x4),"bad off");
		static_assert((offsetof(TimeLineStatus,lastTime)==0x8),"bad off");
	};
};

//UDT: struct SpeedLimiter @len=16
	//_Func: public void ~SpeedLimiter(); @loc=static @len=3 @rva=96368
	//_Func: public void init(Car * car); @loc=static @len=5 @rva=2865408
	//_Func: public void step(float dt); @loc=static @len=247 @rva=2865424
	//_Func: public bool isLimitingSpeed(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: bool, shoudLimit
	//_Data: this+0x1, Member, Type: bool, isLimiting
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Func: public void SpeedLimiter(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct SpeedLimiter {
public:
	bool shoudLimit;
	bool isLimiting;
	Car * car;
	inline SpeedLimiter()  { }
	inline void dtor() { typedef void (*_fpt)(SpeedLimiter *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void init(Car * car) { typedef void (*_fpt)(SpeedLimiter *pthis, Car *); _fpt _f=(_fpt)_drva(2865408); return _f(this, car); }
	inline void step(float dt) { typedef void (*_fpt)(SpeedLimiter *pthis, float); _fpt _f=(_fpt)_drva(2865424); return _f(this, dt); }
	inline void _guard_obj() {
		static_assert((sizeof(SpeedLimiter)==16),"bad size");
		static_assert((offsetof(SpeedLimiter,shoudLimit)==0x0),"bad off");
		static_assert((offsetof(SpeedLimiter,isLimiting)==0x1),"bad off");
		static_assert((offsetof(SpeedLimiter,car)==0x8),"bad off");
	};
};

//UDT: struct OnWindowResizeEvent @len=16
	//_Func: public void OnWindowResizeEvent(int  _arg0, int  _arg1, RenderWindow *  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, width
	//_Data: this+0x4, Member, Type: int, height
	//_Data: this+0x8, Member, Type: class RenderWindow *, renderWindow
//UDT;

struct OnWindowResizeEvent {
public:
	int width;
	int height;
	RenderWindow * renderWindow;
	inline OnWindowResizeEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnWindowResizeEvent)==16),"bad size");
		static_assert((offsetof(OnWindowResizeEvent,width)==0x0),"bad off");
		static_assert((offsetof(OnWindowResizeEvent,height)==0x4),"bad off");
		static_assert((offsetof(OnWindowResizeEvent,renderWindow)==0x8),"bad off");
	};
};

//UDT: struct ServerInfo @len=72
	//_Func: public void ServerInfo(ServerInfo &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ServerInfo(); @loc=static @len=127 @rva=243344
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, ip
	//_Data: this+0x40, Member, Type: unsigned short, httpPort
	//_Data: this+0x42, Member, Type: unsigned short, udpPort
	//_Data: this+0x44, Member, Type: unsigned short, tcpPort
	//_Func: public void ~ServerInfo(); @loc=static @len=88 @rva=776352
	//_Func: public ServerInfo & operator=(ServerInfo &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ServerInfo)==72),"bad size");
		static_assert((offsetof(ServerInfo,name)==0x0),"bad off");
		static_assert((offsetof(ServerInfo,ip)==0x20),"bad off");
		static_assert((offsetof(ServerInfo,httpPort)==0x40),"bad off");
		static_assert((offsetof(ServerInfo,udpPort)==0x42),"bad off");
		static_assert((offsetof(ServerInfo,tcpPort)==0x44),"bad off");
	};
};

//UDT: struct DriverInfo @len=128
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, team
	//_Data: this+0x40, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, nationality
	//_Data: this+0x60, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, nationCode
	//_Func: public void DriverInfo(DriverInfo & __that); @loc=static @len=192 @rva=242096
	//_Func: public void DriverInfo(); @loc=static @len=69 @rva=242288
	//_Func: public void ~DriverInfo(); @loc=static @len=152 @rva=246464
	//_Func: public DriverInfo & operator=(DriverInfo & __that); @loc=static @len=122 @rva=848192
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DriverInfo)==128),"bad size");
		static_assert((offsetof(DriverInfo,name)==0x0),"bad off");
		static_assert((offsetof(DriverInfo,team)==0x20),"bad off");
		static_assert((offsetof(DriverInfo,nationality)==0x40),"bad off");
		static_assert((offsetof(DriverInfo,nationCode)==0x60),"bad off");
	};
};

//UDT: struct SessionInfo @len=32
	//_Func: public void SessionInfo(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: enum SessionType, type
	//_Data: this+0x8, Member, Type: double, startTimeMS
	//_Data: this+0x10, Member, Type: double, timeSecs
	//_Data: this+0x18, Member, Type: int, laps
	//_Data: this+0x1C, Member, Type: int, index
//UDT;

struct SessionInfo {
public:
	SessionType type;
	double startTimeMS;
	double timeSecs;
	int laps;
	int index;
	inline SessionInfo()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SessionInfo)==32),"bad size");
		static_assert((offsetof(SessionInfo,type)==0x0),"bad off");
		static_assert((offsetof(SessionInfo,startTimeMS)==0x8),"bad off");
		static_assert((offsetof(SessionInfo,timeSecs)==0x10),"bad off");
		static_assert((offsetof(SessionInfo,laps)==0x18),"bad off");
		static_assert((offsetof(SessionInfo,index)==0x1C),"bad off");
	};
};

//UDT: struct DifferentialSetting @len=16
	//_Data: this+0x0, Member, Type: float, power
	//_Data: this+0x4, Member, Type: float, coast
	//_Data: this+0x8, Member, Type: float, preload
	//_Data: this+0xC, Member, Type: enum DifferentialType, type
	//_Func: public void DifferentialSetting(); @loc=optimized @len=0 @rva=0
//UDT;

struct DifferentialSetting {
public:
	float power;
	float coast;
	float preload;
	DifferentialType type;
	inline DifferentialSetting()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DifferentialSetting)==16),"bad size");
		static_assert((offsetof(DifferentialSetting,power)==0x0),"bad off");
		static_assert((offsetof(DifferentialSetting,coast)==0x4),"bad off");
		static_assert((offsetof(DifferentialSetting,preload)==0x8),"bad off");
		static_assert((offsetof(DifferentialSetting,type)==0xC),"bad off");
	};
};

//UDT: struct ReceivedVoteDef @len=28
	//_Data: this+0x0, Member, Type: enum VoteType, voteType
	//_Data: this+0x4, Member, Type: int, quorum
	//_Data: this+0x8, Member, Type: int, votes
	//_Data: this+0xC, Member, Type: int, timeLeftMS
	//_Data: this+0x10, Member, Type: int, lastVoterSessionID
	//_Data: this+0x14, Member, Type: int, lastVoterVote
	//_Data: this+0x18, Member, Type: unsigned char, targetSessionID
	//_Data: this+0x19, Member, Type: bool, isMe
	//_Func: public void ReceivedVoteDef(); @loc=static @len=25 @rva=242528
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ReceivedVoteDef)==28),"bad size");
		static_assert((offsetof(ReceivedVoteDef,voteType)==0x0),"bad off");
		static_assert((offsetof(ReceivedVoteDef,quorum)==0x4),"bad off");
		static_assert((offsetof(ReceivedVoteDef,votes)==0x8),"bad off");
		static_assert((offsetof(ReceivedVoteDef,timeLeftMS)==0xC),"bad off");
		static_assert((offsetof(ReceivedVoteDef,lastVoterSessionID)==0x10),"bad off");
		static_assert((offsetof(ReceivedVoteDef,lastVoterVote)==0x14),"bad off");
		static_assert((offsetof(ReceivedVoteDef,targetSessionID)==0x18),"bad off");
		static_assert((offsetof(ReceivedVoteDef,isMe)==0x19),"bad off");
	};
};

//UDT: struct OnLapCompletedEvent @len=56
	//_Func: public void OnLapCompletedEvent(OnLapCompletedEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void OnLapCompletedEvent(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, carIndex
	//_Data: this+0x4, Member, Type: unsigned int, lapTime
	//_Data: this+0x8, Member, Type: unsigned int, lapCount
	//_Data: this+0x10, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, splits
	//_Data: this+0x28, Member, Type: double, eventTime
	//_Data: this+0x30, Member, Type: bool, isValid
	//_Data: this+0x34, Member, Type: int, cuts
	//_Func: public void ~OnLapCompletedEvent(); @loc=static @len=53 @rva=246720
	//_Func: public OnLapCompletedEvent & operator=(OnLapCompletedEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(OnLapCompletedEvent)==56),"bad size");
		static_assert((offsetof(OnLapCompletedEvent,carIndex)==0x0),"bad off");
		static_assert((offsetof(OnLapCompletedEvent,lapTime)==0x4),"bad off");
		static_assert((offsetof(OnLapCompletedEvent,lapCount)==0x8),"bad off");
		static_assert((offsetof(OnLapCompletedEvent,splits)==0x10),"bad off");
		static_assert((offsetof(OnLapCompletedEvent,eventTime)==0x28),"bad off");
		static_assert((offsetof(OnLapCompletedEvent,isValid)==0x30),"bad off");
		static_assert((offsetof(OnLapCompletedEvent,cuts)==0x34),"bad off");
	};
};

//UDT: struct OnChatMessageEvent @len=40
	//_Data: this+0x0, Member, Type: int, sessionID
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, message
	//_Func: public void OnChatMessageEvent(OnChatMessageEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void OnChatMessageEvent(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~OnChatMessageEvent(); @loc=static @len=49 @rva=2180672
	//_Func: public OnChatMessageEvent & operator=(OnChatMessageEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct OnChatMessageEvent {
public:
	int sessionID;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > message;
	inline OnChatMessageEvent()  { }
	inline void dtor() { typedef void (*_fpt)(OnChatMessageEvent *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(OnChatMessageEvent)==40),"bad size");
		static_assert((offsetof(OnChatMessageEvent,sessionID)==0x0),"bad off");
		static_assert((offsetof(OnChatMessageEvent,message)==0x8),"bad off");
	};
};

//UDT: struct PenaltyRecord @len=12
	//_Func: public void PenaltyRecord(unsigned int  _arg0, unsigned int  _arg1, PenaltyDescription  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, lap
	//_Data: this+0x4, Member, Type: unsigned int, seconds
	//_Data: this+0x8, Member, Type: enum PenaltyDescription, descr
//UDT;

struct PenaltyRecord {
public:
	unsigned int lap;
	unsigned int seconds;
	PenaltyDescription descr;
	inline PenaltyRecord()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PenaltyRecord)==12),"bad size");
		static_assert((offsetof(PenaltyRecord,lap)==0x0),"bad off");
		static_assert((offsetof(PenaltyRecord,seconds)==0x4),"bad off");
		static_assert((offsetof(PenaltyRecord,descr)==0x8),"bad off");
	};
};

//UDT: class IVarCallback @len=8 @vfcount=1
	//_VTable: 
	//_Func: public void onSetVar(SVar *  _arg0, float  _arg1); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void IVarCallback(IVarCallback &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IVarCallback(); @loc=optimized @len=0 @rva=0
	//_Func: public IVarCallback & operator=(IVarCallback &  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class IVarCallback {
public:
	inline IVarCallback()  { }
	virtual void onSetVar_vf0(SVar *  _arg0, float  _arg1) = 0;
	inline void onSetVar(SVar *  _arg0, float  _arg1) { return onSetVar_vf0( _arg0,  _arg1); }
	inline void _guard_obj() {
		static_assert((sizeof(IVarCallback)==8),"bad size");
	};
};

//UDT: struct GearRequestStatus @len=32
	//_Func: public void GearRequestStatus(); @loc=optimized @len=0 @rva=0
	//_Func: public bool setRequest(GearChangeRequest  _arg0, double  _arg1, int  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: enum GearChangeRequest, request
	//_Data: this+0x8, Member, Type: double, timeAccumulator
	//_Data: this+0x10, Member, Type: double, timeout
	//_Data: this+0x18, Member, Type: int, requestedGear
//UDT;

struct GearRequestStatus {
public:
	GearChangeRequest request;
	double timeAccumulator;
	double timeout;
	int requestedGear;
	inline GearRequestStatus()  { }
	inline void _guard_obj() {
		static_assert((sizeof(GearRequestStatus)==32),"bad size");
		static_assert((offsetof(GearRequestStatus,request)==0x0),"bad off");
		static_assert((offsetof(GearRequestStatus,timeAccumulator)==0x8),"bad off");
		static_assert((offsetof(GearRequestStatus,timeout)==0x10),"bad off");
		static_assert((offsetof(GearRequestStatus,requestedGear)==0x18),"bad off");
	};
};

//UDT: struct PenaltyRules @len=8
	//_Data: this+0x0, Member, Type: enum JumpStartPenaltyMode, jumpStartPenaltyMode
	//_Data: this+0x4, Member, Type: short, basePitPenaltyLaps
	//_Func: public bool isFirstGearLocked(); @loc=optimized @len=0 @rva=0
	//_Func: public void PenaltyRules(); @loc=optimized @len=0 @rva=0
//UDT;

struct PenaltyRules {
public:
	JumpStartPenaltyMode jumpStartPenaltyMode;
	short basePitPenaltyLaps;
	inline PenaltyRules()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PenaltyRules)==8),"bad size");
		static_assert((offsetof(PenaltyRules,jumpStartPenaltyMode)==0x0),"bad off");
		static_assert((offsetof(PenaltyRules,basePitPenaltyLaps)==0x4),"bad off");
	};
};

//UDT: struct OnESCMenuTriggered @len=16
	//_Func: public void OnESCMenuTriggered(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class ESCMenu *, menu
	//_Data: this+0x8, Member, Type: bool, visible
	//_Data: this+0x9, Member, Type: bool, startReplay
//UDT;

struct OnESCMenuTriggered {
public:
	ESCMenu * menu;
	bool visible;
	bool startReplay;
	inline OnESCMenuTriggered()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnESCMenuTriggered)==16),"bad size");
		static_assert((offsetof(OnESCMenuTriggered,menu)==0x0),"bad off");
		static_assert((offsetof(OnESCMenuTriggered,visible)==0x8),"bad off");
		static_assert((offsetof(OnESCMenuTriggered,startReplay)==0x9),"bad off");
	};
};

//UDT: struct ksgui::OnSpinnerValueChanged @len=16
	//_Data: this+0x0, Member, Type: class ksgui::Spinner *, spinner
	//_Data: this+0x8, Member, Type: int, value
//UDT;

struct ksgui_OnSpinnerValueChanged {
public:
	ksgui_Spinner * spinner;
	int value;
	inline ksgui_OnSpinnerValueChanged()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnSpinnerValueChanged)==16),"bad size");
		static_assert((offsetof(ksgui_OnSpinnerValueChanged,spinner)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnSpinnerValueChanged,value)==0x8),"bad off");
	};
};

//UDT: struct TyreThermalPatch @len=40
	//_Func: public void TyreThermalPatch(TyreThermalPatch &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TyreThermalPatch(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class std::vector<TyreThermalPatch *,std::allocator<TyreThermalPatch *> >, connections
	//_Data: this+0x18, Member, Type: float, T
	//_Data: this+0x1C, Member, Type: float, inputT
	//_Data: this+0x20, Member, Type: int, elementIndex
	//_Data: this+0x24, Member, Type: int, stripeIndex
	//_Func: public void ~TyreThermalPatch(); @loc=optimized @len=0 @rva=0
	//_Func: public TyreThermalPatch & operator=(TyreThermalPatch &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct TyreThermalPatch {
public:
	std::vector<TyreThermalPatch *,std::allocator<TyreThermalPatch *> > connections;
	float T;
	float inputT;
	int elementIndex;
	int stripeIndex;
	inline TyreThermalPatch()  { }
	inline void _guard_obj() {
		static_assert((sizeof(TyreThermalPatch)==40),"bad size");
		static_assert((offsetof(TyreThermalPatch,connections)==0x0),"bad off");
		static_assert((offsetof(TyreThermalPatch,T)==0x18),"bad off");
		static_assert((offsetof(TyreThermalPatch,inputT)==0x1C),"bad off");
		static_assert((offsetof(TyreThermalPatch,elementIndex)==0x20),"bad off");
		static_assert((offsetof(TyreThermalPatch,stripeIndex)==0x24),"bad off");
	};
};

//UDT: class Task @len=40
	//_Func: public void Task(Task &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Task(); @loc=optimized @len=0 @rva=0
	//_Func: public void Task(std::function<void __cdecl(void)>  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void run(); @loc=optimized @len=0 @rva=0
	//_Func: public void spinWait(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: volatile bool, isDone
	//_Data: this+0x8, Member, Type: class std::function<void __cdecl(void)>, function
	//_Func: public void ~Task(); @loc=static @len=74 @rva=175552
	//_Func: public Task & operator=(Task &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class Task {
public:
	bool isDone;
	std::function<void __cdecl(void)> function;
	inline Task()  { }
	inline void dtor() { typedef void (*_fpt)(Task *pthis); _fpt _f=(_fpt)_drva(175552); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(Task)==40),"bad size");
		static_assert((offsetof(Task,isDone)==0x0),"bad off");
		static_assert((offsetof(Task,function)==0x8),"bad off");
	};
};

//UDT: struct StabilityControl @len=24
	//_Func: public void ~StabilityControl(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: float, gain
	//_Data: this+0x4, Member, Type: bool, useBeta
	//_Func: public void init(Car * car); @loc=static @len=22 @rva=2882096
	//_Func: public void step(float dt); @loc=static @len=511 @rva=2882128
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: float, maxGain
	//_Func: public void StabilityControl(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(StabilityControl)==24),"bad size");
		static_assert((offsetof(StabilityControl,gain)==0x0),"bad off");
		static_assert((offsetof(StabilityControl,useBeta)==0x4),"bad off");
		static_assert((offsetof(StabilityControl,car)==0x8),"bad off");
		static_assert((offsetof(StabilityControl,maxGain)==0x10),"bad off");
	};
};

//UDT: struct TrackData @len=72
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, configuration
	//_Data: this+0x40, Member, Type: int, gridPlaces
	//_Func: public void TrackData(TrackData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TrackData(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~TrackData(); @loc=static @len=88 @rva=776352
	//_Func: public TrackData & operator=(TrackData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct TrackData {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > configuration;
	int gridPlaces;
	inline TrackData()  { }
	inline void dtor() { typedef void (*_fpt)(TrackData *pthis); _fpt _f=(_fpt)_drva(776352); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(TrackData)==72),"bad size");
		static_assert((offsetof(TrackData,name)==0x0),"bad off");
		static_assert((offsetof(TrackData,configuration)==0x20),"bad off");
		static_assert((offsetof(TrackData,gridPlaces)==0x40),"bad off");
	};
};

//UDT: struct OnTyreCompoundChanged @len=72
	//_Data: this+0x0, Member, Type: int, tyreIndex
	//_Data: this+0x4, Member, Type: int, compoundIndex
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, compoundName
	//_Data: this+0x28, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, shortName
	//_Func: public void OnTyreCompoundChanged(OnTyreCompoundChanged &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void OnTyreCompoundChanged(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~OnTyreCompoundChanged(); @loc=static @len=90 @rva=847584
	//_Func: public OnTyreCompoundChanged & operator=(OnTyreCompoundChanged &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct OnTyreCompoundChanged {
public:
	int tyreIndex;
	int compoundIndex;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > compoundName;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > shortName;
	inline OnTyreCompoundChanged()  { }
	inline void dtor() { typedef void (*_fpt)(OnTyreCompoundChanged *pthis); _fpt _f=(_fpt)_drva(847584); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(OnTyreCompoundChanged)==72),"bad size");
		static_assert((offsetof(OnTyreCompoundChanged,tyreIndex)==0x0),"bad off");
		static_assert((offsetof(OnTyreCompoundChanged,compoundIndex)==0x4),"bad off");
		static_assert((offsetof(OnTyreCompoundChanged,compoundName)==0x8),"bad off");
		static_assert((offsetof(OnTyreCompoundChanged,shortName)==0x28),"bad off");
	};
};

//UDT: struct SGearRatio @len=40
	//_Func: public void SGearRatio(SGearRatio &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SGearRatio(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, double iratio); @loc=static @len=173 @rva=2514800
	//_Data: this+0x0, Member, Type: double, ratio
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Func: public void ~SGearRatio(); @loc=static @len=49 @rva=2180672
	//_Func: public SGearRatio & operator=(SGearRatio &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct SGearRatio {
public:
	double ratio;
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	inline SGearRatio()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, double iratio) { typedef void (*_fpt)(SGearRatio *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, double); _fpt _f=(_fpt)_drva(2514800); _f(this, iname, iratio); }
	inline void dtor() { typedef void (*_fpt)(SGearRatio *pthis); _fpt _f=(_fpt)_drva(2180672); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(SGearRatio)==40),"bad size");
		static_assert((offsetof(SGearRatio,ratio)==0x0),"bad off");
		static_assert((offsetof(SGearRatio,name)==0x8),"bad off");
	};
};

//UDT: struct OnGearRequestEvent @len=8
	//_Func: public void OnGearRequestEvent(GearChangeRequest  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: enum GearChangeRequest, request
	//_Data: this+0x4, Member, Type: int, nextGear
//UDT;

struct OnGearRequestEvent {
public:
	GearChangeRequest request;
	int nextGear;
	inline OnGearRequestEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnGearRequestEvent)==8),"bad size");
		static_assert((offsetof(OnGearRequestEvent,request)==0x0),"bad off");
		static_assert((offsetof(OnGearRequestEvent,nextGear)==0x4),"bad off");
	};
};

//UDT: struct OnNewCarLoadedEvent @len=8
	//_Data: this+0x0, Member, Type: class CarAvatar *, car
//UDT;

struct OnNewCarLoadedEvent {
public:
	CarAvatar * car;
	inline OnNewCarLoadedEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnNewCarLoadedEvent)==8),"bad size");
		static_assert((offsetof(OnNewCarLoadedEvent,car)==0x0),"bad off");
	};
};

//UDT: struct ksgui::OnControlClicked @len=16
	//_Data: this+0x0, Member, Type: class ksgui::Control *, control
	//_Data: this+0x8, Member, Type: int, localx
	//_Data: this+0xC, Member, Type: int, localy
//UDT;

struct ksgui_OnControlClicked {
public:
	ksgui_Control * control;
	int localx;
	int localy;
	inline ksgui_OnControlClicked()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnControlClicked)==16),"bad size");
		static_assert((offsetof(ksgui_OnControlClicked,control)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnControlClicked,localx)==0x8),"bad off");
		static_assert((offsetof(ksgui_OnControlClicked,localy)==0xC),"bad off");
	};
};

//UDT: struct MouseEvent @len=12
	//_Func: public void MouseEvent(int  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: int, x
	//_Data: this+0x4, Member, Type: int, y
	//_Data: this+0x8, Member, Type: enum MouseButton, button
//UDT;

struct MouseEvent {
public:
	int x;
	int y;
	MouseButton button;
	inline MouseEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(MouseEvent)==12),"bad size");
		static_assert((offsetof(MouseEvent,x)==0x0),"bad off");
		static_assert((offsetof(MouseEvent,y)==0x4),"bad off");
		static_assert((offsetof(MouseEvent,button)==0x8),"bad off");
	};
};

//UDT: struct ksgui::OnCheckBoxChanged @len=16
	//_Data: this+0x0, Member, Type: class ksgui::CheckBox *, checkBox
	//_Data: this+0x8, Member, Type: bool, value
//UDT;

struct ksgui_OnCheckBoxChanged {
public:
	ksgui_CheckBox * checkBox;
	bool value;
	inline ksgui_OnCheckBoxChanged()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnCheckBoxChanged)==16),"bad size");
		static_assert((offsetof(ksgui_OnCheckBoxChanged,checkBox)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnCheckBoxChanged,value)==0x8),"bad off");
	};
};

//UDT: struct ksgui::OnScrollBarValueChanged @len=16
	//_Data: this+0x0, Member, Type: class ksgui::ScrollBar *, scrollBar
	//_Data: this+0x8, Member, Type: int, value
//UDT;

struct ksgui_OnScrollBarValueChanged {
public:
	ksgui_ScrollBar * scrollBar;
	int value;
	inline ksgui_OnScrollBarValueChanged()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnScrollBarValueChanged)==16),"bad size");
		static_assert((offsetof(ksgui_OnScrollBarValueChanged,scrollBar)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnScrollBarValueChanged,value)==0x8),"bad off");
	};
};

//UDT: struct ksgui::OnCutExtremesChanged @len=16
	//_Data: this+0x0, Member, Type: class ksgui::Slider *, slider
	//_Data: this+0x8, Member, Type: float, cutIn
	//_Data: this+0xC, Member, Type: float, cutOut
//UDT;

struct ksgui_OnCutExtremesChanged {
public:
	ksgui_Slider * slider;
	float cutIn;
	float cutOut;
	inline ksgui_OnCutExtremesChanged()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnCutExtremesChanged)==16),"bad size");
		static_assert((offsetof(ksgui_OnCutExtremesChanged,slider)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnCutExtremesChanged,cutIn)==0x8),"bad off");
		static_assert((offsetof(ksgui_OnCutExtremesChanged,cutOut)==0xC),"bad off");
	};
};

//UDT: struct ksgui::OnSliderInteraction @len=16
	//_Data: this+0x0, Member, Type: class ksgui::Slider *, slider
	//_Data: this+0x8, Member, Type: float, value
//UDT;

struct ksgui_OnSliderInteraction {
public:
	ksgui_Slider * slider;
	float value;
	inline ksgui_OnSliderInteraction()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnSliderInteraction)==16),"bad size");
		static_assert((offsetof(ksgui_OnSliderInteraction,slider)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnSliderInteraction,value)==0x8),"bad off");
	};
};

//UDT: struct INISection @len=16
	//_Data: this+0x0, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > >, keys
	//_Func: public bool hasKey(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & k); @loc=static @len=105 @rva=2322496
	//_Func: public void INISection(INISection &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void INISection(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~INISection(); @loc=static @len=41 @rva=4512704
	//_Func: public INISection & operator=(INISection &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct INISection {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > > keys;
	inline INISection()  { }
	inline bool hasKey(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & k) { typedef bool (*_fpt)(INISection *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(2322496); return _f(this, k); }
	inline void dtor() { typedef void (*_fpt)(INISection *pthis); _fpt _f=(_fpt)_drva(4512704); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(INISection)==16),"bad size");
		static_assert((offsetof(INISection,keys)==0x0),"bad off");
	};
};

//UDT: struct OnControlsProviderChanged @len=16
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: class ICarControlsProvider *, newControlsProvider
//UDT;

struct OnControlsProviderChanged {
public:
	Car * car;
	ICarControlsProvider * newControlsProvider;
	inline OnControlsProviderChanged()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnControlsProviderChanged)==16),"bad size");
		static_assert((offsetof(OnControlsProviderChanged,car)==0x0),"bad off");
		static_assert((offsetof(OnControlsProviderChanged,newControlsProvider)==0x8),"bad off");
	};
};

//UDT: class Session @len=112
	//_Func: public void Session(Session & __that); @loc=static @len=190 @rva=704528
	//_Func: public void Session(); @loc=static @len=145 @rva=610160
	//_Data: this+0x0, Member, Type: enum SessionType, sessionType
	//_Data: this+0x4, Member, Type: bool, isTimedRace
	//_Data: this+0x5, Member, Type: bool, hasAdditionalLap
	//_Data: this+0x8, Member, Type: int, laps
	//_Data: this+0xC, Member, Type: float, durationMinutes
	//_Data: this+0x10, Member, Type: float, overtime_ms
	//_Data: this+0x18, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, spawSet
	//_Data: this+0x38, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x58, Member, Type: double, startTime
	//_Data: this+0x60, Member, Type: int, forcedPosition
	//_Data: this+0x64, Member, Type: bool, isOver
	//_Data: this+0x68, Member, Type: unsigned int, leaderCompletedLaps
	//_Func: public void ~Session(); @loc=static @len=90 @rva=584992
	//_Func: public Session & operator=(Session & __that); @loc=static @len=143 @rva=612480
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Session)==112),"bad size");
		static_assert((offsetof(Session,sessionType)==0x0),"bad off");
		static_assert((offsetof(Session,isTimedRace)==0x4),"bad off");
		static_assert((offsetof(Session,hasAdditionalLap)==0x5),"bad off");
		static_assert((offsetof(Session,laps)==0x8),"bad off");
		static_assert((offsetof(Session,durationMinutes)==0xC),"bad off");
		static_assert((offsetof(Session,overtime_ms)==0x10),"bad off");
		static_assert((offsetof(Session,spawSet)==0x18),"bad off");
		static_assert((offsetof(Session,name)==0x38),"bad off");
		static_assert((offsetof(Session,startTime)==0x58),"bad off");
		static_assert((offsetof(Session,forcedPosition)==0x60),"bad off");
		static_assert((offsetof(Session,isOver)==0x64),"bad off");
		static_assert((offsetof(Session,leaderCompletedLaps)==0x68),"bad off");
	};
};

//UDT: class SetupItem @len=136 @vfcount=1
	//_VTable: 
	//_Func: public void SetupItem(SetupItem & __that); @loc=static @len=224 @rva=2657424
	//_Func: public void SetupItem(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aname, float & aconnectedFloat, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * units, bool isAttached, float multiplier, float labelMult); @loc=static @len=303 @rva=2929008
	//_Func: public void ~SetupItem(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=164 @rva=2929312
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x28, Member, Type: float &, connectedFloat
	//_Data: this+0x30, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, units
	//_Data: this+0x50, Member, Type: float, multiplier
	//_Data: this+0x54, Member, Type: float, newValue
	//_Data: this+0x58, Member, Type: bool, attached
	//_Data: this+0x60, Member, Type: class std::function<void __cdecl(SetupItem *)>, onValueChanged
	//_Data: this+0x80, Member, Type: float, labelMultiplier
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SetupItem)==136),"bad size");
		static_assert((offsetof(SetupItem,name)==0x8),"bad off");
		static_assert((offsetof(SetupItem,units)==0x30),"bad off");
		static_assert((offsetof(SetupItem,multiplier)==0x50),"bad off");
		static_assert((offsetof(SetupItem,newValue)==0x54),"bad off");
		static_assert((offsetof(SetupItem,attached)==0x58),"bad off");
		static_assert((offsetof(SetupItem,onValueChanged)==0x60),"bad off");
		static_assert((offsetof(SetupItem,labelMultiplier)==0x80),"bad off");
	};
};

//UDT: struct TelemetryChannelData @len=32
	//_Func: public void TelemetryChannelData(TelemetryChannelData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TelemetryChannelData(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class std::vector<float,std::allocator<float> >, values
	//_Data: this+0x18, Member, Type: enum TelemetryUnits, units
	//_Data: this+0x1C, Member, Type: int, frequency
	//_Func: public void ~TelemetryChannelData(); @loc=static @len=48 @rva=811392
	//_Func: public TelemetryChannelData & operator=(TelemetryChannelData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct TelemetryChannelData {
public:
	std::vector<float,std::allocator<float> > values;
	TelemetryUnits units;
	int frequency;
	inline TelemetryChannelData()  { }
	inline void dtor() { typedef void (*_fpt)(TelemetryChannelData *pthis); _fpt _f=(_fpt)_drva(811392); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(TelemetryChannelData)==32),"bad size");
		static_assert((offsetof(TelemetryChannelData,values)==0x0),"bad off");
		static_assert((offsetof(TelemetryChannelData,units)==0x18),"bad off");
		static_assert((offsetof(TelemetryChannelData,frequency)==0x1C),"bad off");
	};
};

//UDT: struct DRSWingConnection @len=24
	//_Func: public void DRSWingConnection(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class Wing *, wing
	//_Data: this+0x8, Member, Type: float, effect
	//_Data: this+0xC, Member, Type: float, angle
	//_Data: this+0x10, Member, Type: enum DRWWingConnectionMode, mode
//UDT;

struct DRSWingConnection {
public:
	Wing * wing;
	float effect;
	float angle;
	DRWWingConnectionMode mode;
	inline DRSWingConnection()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DRSWingConnection)==24),"bad size");
		static_assert((offsetof(DRSWingConnection,wing)==0x0),"bad off");
		static_assert((offsetof(DRSWingConnection,effect)==0x8),"bad off");
		static_assert((offsetof(DRSWingConnection,angle)==0xC),"bad off");
		static_assert((offsetof(DRSWingConnection,mode)==0x10),"bad off");
	};
};

//UDT: struct OnPenaltyEvent @len=16
	//_Func: public void OnPenaltyEvent(Car *  _arg0, PenaltyType  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: enum PenaltyType, ptype
//UDT;

struct OnPenaltyEvent {
public:
	Car * car;
	PenaltyType ptype;
	inline OnPenaltyEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnPenaltyEvent)==16),"bad size");
		static_assert((offsetof(OnPenaltyEvent,car)==0x0),"bad off");
		static_assert((offsetof(OnPenaltyEvent,ptype)==0x8),"bad off");
	};
};

//UDT: class IACPPluginHost @len=8 @vfcount=9
	//_VTable: 
	//_Func: public HWND__ * getHwnd(); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public HINSTANCE__ * getHInstance(); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void setABS(float  _arg0); @intro @pure @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Func: public void setTC(float  _arg0); @intro @pure @virtual vtpo=0 vfid=3 @loc=optimized @len=0 @rva=0
	//_Func: public void setStabilityControl(float  _arg0); @intro @pure @virtual vtpo=0 vfid=4 @loc=optimized @len=0 @rva=0
	//_Func: public void setIdealLine(bool  _arg0, bool  _arg1); @intro @pure @virtual vtpo=0 vfid=5 @loc=optimized @len=0 @rva=0
	//_Func: public void setAutoShift(bool  _arg0); @intro @pure @virtual vtpo=0 vfid=6 @loc=optimized @len=0 @rva=0
	//_Func: public void setBrakeBias(float  _arg0); @intro @pure @virtual vtpo=0 vfid=7 @loc=optimized @len=0 @rva=0
	//_Func: public void setSystemMessage(wchar_t *  _arg0, wchar_t *  _arg1, bool  _arg2); @intro @pure @virtual vtpo=0 vfid=8 @loc=optimized @len=0 @rva=0
	//_Func: public void IACPPluginHost(IACPPluginHost &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IACPPluginHost(); @loc=optimized @len=0 @rva=0
	//_Func: public IACPPluginHost & operator=(IACPPluginHost &  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(IACPPluginHost)==8),"bad size");
	};
};

//UDT: struct Lap @len=72
	//_Func: public void Lap(Lap & __that); @loc=static @len=113 @rva=242400
	//_Func: public void Lap(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Lap(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned int, time
	//_Data: this+0x8, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, splits
	//_Data: this+0x20, Member, Type: unsigned int, cuts
	//_Data: this+0x24, Member, Type: bool, isValid
	//_Data: this+0x28, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, compound
	//_Func: public void ~Lap(); @loc=static @len=95 @rva=246624
	//_Func: public Lap & operator=(Lap &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Lap)==72),"bad size");
		static_assert((offsetof(Lap,time)==0x0),"bad off");
		static_assert((offsetof(Lap,splits)==0x8),"bad off");
		static_assert((offsetof(Lap,cuts)==0x20),"bad off");
		static_assert((offsetof(Lap,isValid)==0x24),"bad off");
		static_assert((offsetof(Lap,compound)==0x28),"bad off");
	};
};

//UDT: struct ksgui::OnListBoxItemClickedEvent @len=24
	//_Data: this+0x0, Member, Type: class ksgui::ListBox *, listBox
	//_Data: this+0x8, Member, Type: struct ksgui::ListBoxRowData *, row
	//_Data: this+0x10, Member, Type: unsigned int, itemIndex
//UDT;

struct ksgui_OnListBoxItemClickedEvent {
public:
	ksgui_ListBox * listBox;
	ksgui_ListBoxRowData * row;
	unsigned int itemIndex;
	inline ksgui_OnListBoxItemClickedEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_OnListBoxItemClickedEvent)==24),"bad size");
		static_assert((offsetof(ksgui_OnListBoxItemClickedEvent,listBox)==0x0),"bad off");
		static_assert((offsetof(ksgui_OnListBoxItemClickedEvent,row)==0x8),"bad off");
		static_assert((offsetof(ksgui_OnListBoxItemClickedEvent,itemIndex)==0x10),"bad off");
	};
};

//UDT: struct RemoteSessionResult @len=112
	//_Func: public void RemoteSessionResult(RemoteSessionResult & __that); @loc=static @len=125 @rva=242752
	//_Func: public void RemoteSessionResult(int carsCount); @loc=static @len=211 @rva=242880
	//_Func: public void RemoteSessionResult(); @loc=static @len=127 @rva=243104
	//_Data: this+0x0, Member, Type: class std::vector<int,std::allocator<int> >, positions
	//_Data: this+0x18, Member, Type: class std::vector<int,std::allocator<int> >, times
	//_Data: this+0x30, Member, Type: class std::vector<int,std::allocator<int> >, lapCounter
	//_Data: this+0x48, Member, Type: class std::vector<bool,std::allocator<bool> >, hasFinished
	//_Data: this+0x68, Member, Type: unsigned int, leaderLapCount
	//_Func: public void ~RemoteSessionResult(); @loc=static @len=184 @rva=246784
	//_Func: public RemoteSessionResult & operator=(RemoteSessionResult &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(RemoteSessionResult)==112),"bad size");
		static_assert((offsetof(RemoteSessionResult,positions)==0x0),"bad off");
		static_assert((offsetof(RemoteSessionResult,times)==0x18),"bad off");
		static_assert((offsetof(RemoteSessionResult,lapCounter)==0x30),"bad off");
		static_assert((offsetof(RemoteSessionResult,hasFinished)==0x48),"bad off");
		static_assert((offsetof(RemoteSessionResult,leaderLapCount)==0x68),"bad off");
	};
};

//UDT: struct RemoteSession @len=64
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: enum SessionType, type
	//_Data: this+0x24, Member, Type: int, index
	//_Data: this+0x28, Member, Type: int, time
	//_Data: this+0x2C, Member, Type: int, laps
	//_Data: this+0x30, Member, Type: double, startTime
	//_Data: this+0x38, Member, Type: bool, isSpectator
	//_Data: this+0x39, Member, Type: bool, isTimedRace
	//_Data: this+0x3A, Member, Type: bool, hasExtraLap
	//_Func: public void RemoteSession(RemoteSession & __that); @loc=static @len=112 @rva=242560
	//_Func: public void RemoteSession(); @loc=static @len=79 @rva=242672
	//_Func: public void ~RemoteSession(); @loc=static @len=47 @rva=2334528
	//_Func: public RemoteSession & operator=(RemoteSession &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(RemoteSession)==64),"bad size");
		static_assert((offsetof(RemoteSession,name)==0x0),"bad off");
		static_assert((offsetof(RemoteSession,type)==0x20),"bad off");
		static_assert((offsetof(RemoteSession,index)==0x24),"bad off");
		static_assert((offsetof(RemoteSession,time)==0x28),"bad off");
		static_assert((offsetof(RemoteSession,laps)==0x2C),"bad off");
		static_assert((offsetof(RemoteSession,startTime)==0x30),"bad off");
		static_assert((offsetof(RemoteSession,isSpectator)==0x38),"bad off");
		static_assert((offsetof(RemoteSession,isTimedRace)==0x39),"bad off");
		static_assert((offsetof(RemoteSession,hasExtraLap)==0x3A),"bad off");
	};
};

//UDT: class RenderTarget @len=40 @vfcount=1
	//_VTable: 
	//_Func: public void RenderTarget(RenderTarget &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void RenderTarget(GraphicsManager * graphics, eRenderTargetFormat fmt, unsigned int iwidth, unsigned int iheight, bool hasColor, bool hasDepth, int mips); @loc=static @len=276 @rva=2211888
	//_Func: public void ~RenderTarget(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=58 @rva=2212176
	//_Data: this+0x8, Member, Type: enum eRenderTargetFormat, format
	//_Data: this+0xC, Member, Type: int, width
	//_Data: this+0x10, Member, Type: int, height
	//_Data: this+0x14, Member, Type: bool, hasNullDepth
	//_Data: this+0x18, Member, Type: void *, kidColor
	//_Data: this+0x20, Member, Type: void *, kidDepth
	//_Func: public void clear(); @loc=static @len=109 @rva=2212336
	//_Func: public RenderTarget & operator=(RenderTarget &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(RenderTarget)==40),"bad size");
		static_assert((offsetof(RenderTarget,format)==0x8),"bad off");
		static_assert((offsetof(RenderTarget,width)==0xC),"bad off");
		static_assert((offsetof(RenderTarget,height)==0x10),"bad off");
		static_assert((offsetof(RenderTarget,hasNullDepth)==0x14),"bad off");
		static_assert((offsetof(RenderTarget,kidColor)==0x18),"bad off");
		static_assert((offsetof(RenderTarget,kidDepth)==0x20),"bad off");
	};
};

//UDT: class OptionsManager @len=24 @vfcount=1
	//_VTable: 
	//_Func: public void OptionsManager(OptionsManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void OptionsManager(); @loc=static @len=1210 @rva=1627248
	//_Func: public void ~OptionsManager(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=55 @rva=1628464
	//_Data: this+0x8, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,float,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,float> > >, options
	//_Func: protected void loadOptions(); @loc=static @len=446 @rva=1628704
	//_Func: public OptionsManager & operator=(OptionsManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class OptionsManager {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,float,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,float> > > options;
	inline OptionsManager()  { }
	inline void ctor() { typedef void (*_fpt)(OptionsManager *pthis); _fpt _f=(_fpt)_drva(1627248); _f(this); }
	virtual ~OptionsManager();
	inline void dtor() { typedef void (*_fpt)(OptionsManager *pthis); _fpt _f=(_fpt)_drva(1628464); _f(this); }
	inline void loadOptions() { typedef void (*_fpt)(OptionsManager *pthis); _fpt _f=(_fpt)_drva(1628704); return _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(OptionsManager)==24),"bad size");
		static_assert((offsetof(OptionsManager,options)==0x8),"bad off");
	};
};

//UDT: class ShaderManager @len=40 @vfcount=1
	//_VTable: 
	//_Func: public void ShaderManager(ShaderManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ShaderManager(GraphicsManager * graphics); @loc=static @len=32 @rva=2259120
	//_Func: public void ~ShaderManager(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=62 @rva=2259152
	//_Func: public Shader * getShader(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=391 @rva=2259776
	//_Func: public void cleanup(); @loc=static @len=95 @rva=2259680
	//_Func: public std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > getShaders(); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getPermutationSignature(std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > &  _arg0); @pure @loc=optimized @len=0 @rva=0
	//_Data: this+0x8, Member, Type: class GraphicsManager *, graphics
	//_Data: this+0x10, Member, Type: class std::vector<Shader *,std::allocator<Shader *> >, shaders
	//_Func: public ShaderManager & operator=(ShaderManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ShaderManager)==40),"bad size");
		static_assert((offsetof(ShaderManager,graphics)==0x8),"bad off");
		static_assert((offsetof(ShaderManager,shaders)==0x10),"bad off");
	};
};

//UDT: class GameObject @len=88 @vfcount=6
	//_VTable: 
	//_Func: public void GameObject(GameObject &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void GameObject(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, Game * igame); @loc=static @len=161 @rva=2358320
	//_Func: public void ~GameObject(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=167 @rva=2358496
	//_Data: this+0x8, Member, Type: class Game *, game
	//_Data: this+0x10, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x30, Member, Type: bool, isActive
	//_Func: public void removeGameObject(GameObject *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public GameObject * getParent(); @loc=static @len=5 @rva=2358672
	//_Func: public void update(float deltaT); @intro @virtual vtpo=0 vfid=1 @loc=static @len=3 @rva=96368
	//_Func: public void render(float deltaT); @intro @virtual vtpo=0 vfid=2 @loc=static @len=3 @rva=96368
	//_Func: public void renderHUD(float deltaT); @intro @virtual vtpo=0 vfid=3 @loc=static @len=3 @rva=96368
	//_Func: public void renderAudio(float deltaT); @intro @virtual vtpo=0 vfid=4 @loc=static @len=3 @rva=96368
	//_Func: public void shutdown(); @intro @virtual vtpo=0 vfid=5 @loc=static @len=3 @rva=96368
	//_Func: public GameObject * findGameObject(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0, GameObject *  _arg1); @pure @loc=optimized @len=0 @rva=0
	//_Func: public GameObject * getGameObject(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name); @loc=static @len=242 @rva=1177456
	//_Data: this+0x38, Member, Type: class GameObject *, parent
	//_Data: this+0x40, Member, Type: class std::vector<GameObject *,std::allocator<GameObject *> >, gameObjects
	//_Func: public GameObject & operator=(GameObject &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(GameObject)==88),"bad size");
		static_assert((offsetof(GameObject,game)==0x8),"bad off");
		static_assert((offsetof(GameObject,name)==0x10),"bad off");
		static_assert((offsetof(GameObject,isActive)==0x30),"bad off");
		static_assert((offsetof(GameObject,parent)==0x38),"bad off");
		static_assert((offsetof(GameObject,gameObjects)==0x40),"bad off");
	};
};

//UDT: struct OnFlagEvent @len=16
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: enum FlagEventType, type
	//_Data: this+0xC, Member, Type: enum PenaltyDescription, description
//UDT;

struct OnFlagEvent {
public:
	Car * car;
	FlagEventType type;
	PenaltyDescription description;
	inline OnFlagEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnFlagEvent)==16),"bad size");
		static_assert((offsetof(OnFlagEvent,car)==0x0),"bad off");
		static_assert((offsetof(OnFlagEvent,type)==0x8),"bad off");
		static_assert((offsetof(OnFlagEvent,description)==0xC),"bad off");
	};
};

//UDT: class CarRaceInfo @len=40 @vfcount=1
	//_VTable: 
	//_Func: public void CarRaceInfo(CarRaceInfo &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CarRaceInfo(); @loc=static @len=78 @rva=948448
	//_Func: public void ~CarRaceInfo(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=55 @rva=948528
	//_Func: public void setSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName, int gp); @loc=static @len=26 @rva=951456
	//_Func: public int getSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName); @loc=static @len=20 @rva=950352
	//_Func: public void init(CarAvatar * acar); @loc=static @len=760 @rva=950384
	//_Data: this+0x8, Member, Type: class CarAvatar *, car
	//_Data: this+0x10, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,int,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,int> > >, spawnPositionIndex
	//_Data: this+0x20, Member, Type: class RaceManager *, raceManager
	//_Func: public CarRaceInfo & operator=(CarRaceInfo &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CarRaceInfo)==40),"bad size");
		static_assert((offsetof(CarRaceInfo,car)==0x8),"bad off");
		static_assert((offsetof(CarRaceInfo,spawnPositionIndex)==0x10),"bad off");
		static_assert((offsetof(CarRaceInfo,raceManager)==0x20),"bad off");
	};
};

//UDT: struct ModelBoundariesCoordinates @len=24
	//_Data: this+0x0, Member, Type: float, front
	//_Data: this+0x4, Member, Type: float, rear
	//_Data: this+0x8, Member, Type: float, left
	//_Data: this+0xC, Member, Type: float, right
	//_Data: this+0x10, Member, Type: float, top
	//_Data: this+0x14, Member, Type: float, bottom
	//_Func: public void update(vec3f  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ModelBoundariesCoordinates(); @loc=optimized @len=0 @rva=0
//UDT;

struct ModelBoundariesCoordinates {
public:
	float front;
	float rear;
	float left;
	float right;
	float top;
	float bottom;
	inline ModelBoundariesCoordinates()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ModelBoundariesCoordinates)==24),"bad size");
		static_assert((offsetof(ModelBoundariesCoordinates,front)==0x0),"bad off");
		static_assert((offsetof(ModelBoundariesCoordinates,rear)==0x4),"bad off");
		static_assert((offsetof(ModelBoundariesCoordinates,left)==0x8),"bad off");
		static_assert((offsetof(ModelBoundariesCoordinates,right)==0xC),"bad off");
		static_assert((offsetof(ModelBoundariesCoordinates,top)==0x10),"bad off");
		static_assert((offsetof(ModelBoundariesCoordinates,bottom)==0x14),"bad off");
	};
};

//UDT: struct PhysicsCPUTimes @len=40
	//_Data: this+0x0, Member, Type: double, carStep
	//_Data: this+0x8, Member, Type: struct CoreCPUTimes, coreCPUTimes
	//_Data: this+0x20, Member, Type: int, currentCPU
	//_Func: public void PhysicsCPUTimes(); @loc=optimized @len=0 @rva=0
//UDT;

struct PhysicsCPUTimes {
public:
	double carStep;
	CoreCPUTimes coreCPUTimes;
	int currentCPU;
	inline PhysicsCPUTimes()  { }
	inline void _guard_obj() {
		static_assert((sizeof(PhysicsCPUTimes)==40),"bad size");
		static_assert((offsetof(PhysicsCPUTimes,carStep)==0x0),"bad off");
		static_assert((offsetof(PhysicsCPUTimes,coreCPUTimes)==0x8),"bad off");
		static_assert((offsetof(PhysicsCPUTimes,currentCPU)==0x20),"bad off");
	};
};

//UDT: struct CollisionEvent @len=44
	//_Data: this+0x0, Member, Type: int, carIndex
	//_Data: this+0x4, Member, Type: float, normalForce
	//_Data: this+0x8, Member, Type: class vec3f, pos
	//_Data: this+0x14, Member, Type: class vec3f, normal
	//_Data: this+0x20, Member, Type: float, impactAngle
	//_Data: this+0x24, Member, Type: float, relativeSpeed
	//_Data: this+0x28, Member, Type: unsigned long, colliderCategory
	//_Func: public void CollisionEvent(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CollisionEvent)==44),"bad size");
		static_assert((offsetof(CollisionEvent,carIndex)==0x0),"bad off");
		static_assert((offsetof(CollisionEvent,normalForce)==0x4),"bad off");
		static_assert((offsetof(CollisionEvent,pos)==0x8),"bad off");
		static_assert((offsetof(CollisionEvent,normal)==0x14),"bad off");
		static_assert((offsetof(CollisionEvent,impactAngle)==0x20),"bad off");
		static_assert((offsetof(CollisionEvent,relativeSpeed)==0x24),"bad off");
		static_assert((offsetof(CollisionEvent,colliderCategory)==0x28),"bad off");
	};
};

//UDT: struct CarCollisionBox @len=32
	//_Func: public void CarCollisionBox(vec3f &  _arg0, vec3f &  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, centre
	//_Data: this+0xC, Member, Type: class vec3f, size
	//_Data: this+0x18, Member, Type: unsigned __int64, id
//UDT;

struct CarCollisionBox {
public:
	vec3f centre;
	vec3f size;
	unsigned __int64 id;
	inline CarCollisionBox()  { }
	inline void _guard_obj() {
		static_assert((sizeof(CarCollisionBox)==32),"bad size");
		static_assert((offsetof(CarCollisionBox,centre)==0x0),"bad off");
		static_assert((offsetof(CarCollisionBox,size)==0xC),"bad off");
		static_assert((offsetof(CarCollisionBox,id)==0x18),"bad off");
	};
};

//UDT: struct UDPMessage @len=32
	//_Data: this+0x0, Member, Type: void *, data
	//_Data: this+0x8, Member, Type: int, size
	//_Data: this+0xC, Member, Type: struct sockaddr_in, srcAddress
	//_Func: public void UDPMessage(); @loc=optimized @len=0 @rva=0
//UDT;

struct UDPMessage {
public:
	void * data;
	int size;
	sockaddr_in srcAddress;
	inline UDPMessage()  { }
	inline void _guard_obj() {
		static_assert((sizeof(UDPMessage)==32),"bad size");
		static_assert((offsetof(UDPMessage,data)==0x0),"bad off");
		static_assert((offsetof(UDPMessage,size)==0x8),"bad off");
		static_assert((offsetof(UDPMessage,srcAddress)==0xC),"bad off");
	};
};

//UDT: struct AIState @len=60
	//_Func: public void AIState(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, currentSteerSignal
	//_Data: this+0x4, Member, Type: class vec3f, steerTarget
	//_Data: this+0x10, Member, Type: float, gasBrakeTarget
	//_Data: this+0x14, Member, Type: bool, isActive
	//_Data: this+0x18, Member, Type: float, targetSpeed
	//_Data: this+0x1C, Member, Type: float, targetLateralOffset
	//_Data: this+0x20, Member, Type: float, brakeTargetSpeed
	//_Data: this+0x24, Member, Type: float, brakeTargetDist
	//_Data: this+0x28, Member, Type: float, currentNormalizedSplinePosition
	//_Data: this+0x2C, Member, Type: float, outsideOffset
	//_Data: this+0x30, Member, Type: float, projDNRPM
	//_Data: this+0x34, Member, Type: float, understeerFactor
	//_Data: this+0x38, Member, Type: float, currentPush
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(AIState)==60),"bad size");
		static_assert((offsetof(AIState,currentSteerSignal)==0x0),"bad off");
		static_assert((offsetof(AIState,steerTarget)==0x4),"bad off");
		static_assert((offsetof(AIState,gasBrakeTarget)==0x10),"bad off");
		static_assert((offsetof(AIState,isActive)==0x14),"bad off");
		static_assert((offsetof(AIState,targetSpeed)==0x18),"bad off");
		static_assert((offsetof(AIState,targetLateralOffset)==0x1C),"bad off");
		static_assert((offsetof(AIState,brakeTargetSpeed)==0x20),"bad off");
		static_assert((offsetof(AIState,brakeTargetDist)==0x24),"bad off");
		static_assert((offsetof(AIState,currentNormalizedSplinePosition)==0x28),"bad off");
		static_assert((offsetof(AIState,outsideOffset)==0x2C),"bad off");
		static_assert((offsetof(AIState,projDNRPM)==0x30),"bad off");
		static_assert((offsetof(AIState,understeerFactor)==0x34),"bad off");
		static_assert((offsetof(AIState,currentPush)==0x38),"bad off");
	};
};

//UDT: struct PhysicsValueCache @len=4
	//_Data: this+0x0, Member, Type: class Speed, speed
	//_Func: public void PhysicsValueCache(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~PhysicsValueCache(); @loc=static @len=5 @rva=1183184
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct PhysicsValueCache {
public:
	Speed speed;
	inline PhysicsValueCache()  { }
	inline void dtor() { typedef void (*_fpt)(PhysicsValueCache *pthis); _fpt _f=(_fpt)_drva(1183184); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(PhysicsValueCache)==4),"bad size");
		static_assert((offsetof(PhysicsValueCache,speed)==0x0),"bad off");
	};
};

//UDT: class plane4f @len=16
	//_Data: this+0x0, Member, Type: class vec3f, normal
	//_Data: this+0xC, Member, Type: float, d
	//_Func: public void plane4f(vec3f & point1, vec3f & point2, vec3f & point3); @loc=static @len=308 @rva=1146576
	//_Func: public void plane4f(vec3f &  _arg0, vec3f &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void plane4f(); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getRayIntersection(vec3f &  _arg0, vec3f &  _arg1); @loc=optimized @len=0 @rva=0
//UDT;

class plane4f {
public:
	vec3f normal;
	float d;
	inline plane4f()  { }
	inline void ctor(vec3f & point1, vec3f & point2, vec3f & point3) { typedef void (*_fpt)(plane4f *pthis, vec3f &, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(1146576); _f(this, point1, point2, point3); }
	inline void _guard_obj() {
		static_assert((sizeof(plane4f)==16),"bad size");
		static_assert((offsetof(plane4f,normal)==0x0),"bad off");
		static_assert((offsetof(plane4f,d)==0xC),"bad off");
	};
};

//UDT: struct NetCarState @len=136
	//_Func: public void NetCarState(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, pos
	//_Data: this+0xC, Member, Type: class vec3f, rotation
	//_Data: this+0x18, Member, Type: class vec3f, velocity
	//_Data: this+0x24, Member, Type: class vec3f, acceleration
	//_Data: this+0x30, Member, Type: double, timeStamp
	//_Data: this+0x38, Member, Type: double, rcvTime
	//_Data: this+0x40, Member, Type: unsigned char, pakSequenceId
	//_Data: this+0x44, Member, Type: float, targetHeight
	//_Data: this+0x48, Member, Type: float[0x4], tyreAngularSpeed
	//_Data: this+0x58, Member, Type: unsigned short, engineRPM
	//_Data: this+0x5C, Member, Type: float, steerAngle
	//_Data: this+0x60, Member, Type: float, wheelAngle
	//_Data: this+0x64, Member, Type: int, gearIndex
	//_Data: this+0x68, Member, Type: unsigned int, statusBytes
	//_Data: this+0x6C, Member, Type: float, aoa
	//_Data: this+0x70, Member, Type: class vec3f, aoaAxis
	//_Data: this+0x7C, Member, Type: int, ping
	//_Data: this+0x80, Member, Type: float, performanceDelta
	//_Data: this+0x84, Member, Type: float, gas
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(NetCarState)==136),"bad size");
		static_assert((offsetof(NetCarState,pos)==0x0),"bad off");
		static_assert((offsetof(NetCarState,rotation)==0xC),"bad off");
		static_assert((offsetof(NetCarState,velocity)==0x18),"bad off");
		static_assert((offsetof(NetCarState,acceleration)==0x24),"bad off");
		static_assert((offsetof(NetCarState,timeStamp)==0x30),"bad off");
		static_assert((offsetof(NetCarState,rcvTime)==0x38),"bad off");
		static_assert((offsetof(NetCarState,pakSequenceId)==0x40),"bad off");
		static_assert((offsetof(NetCarState,targetHeight)==0x44),"bad off");
		static_assert((offsetof(NetCarState,tyreAngularSpeed)==0x48),"bad off");
		static_assert((offsetof(NetCarState,engineRPM)==0x58),"bad off");
		static_assert((offsetof(NetCarState,steerAngle)==0x5C),"bad off");
		static_assert((offsetof(NetCarState,wheelAngle)==0x60),"bad off");
		static_assert((offsetof(NetCarState,gearIndex)==0x64),"bad off");
		static_assert((offsetof(NetCarState,statusBytes)==0x68),"bad off");
		static_assert((offsetof(NetCarState,aoa)==0x6C),"bad off");
		static_assert((offsetof(NetCarState,aoaAxis)==0x70),"bad off");
		static_assert((offsetof(NetCarState,ping)==0x7C),"bad off");
		static_assert((offsetof(NetCarState,performanceDelta)==0x80),"bad off");
		static_assert((offsetof(NetCarState,gas)==0x84),"bad off");
	};
};

//UDT: struct CarCollisionBounds @len=40
	//_Data: this+0x0, Member, Type: class vec3f, min
	//_Data: this+0xC, Member, Type: class vec3f, max
	//_Data: this+0x18, Member, Type: float, length
	//_Data: this+0x1C, Member, Type: float, width
	//_Data: this+0x20, Member, Type: float, lengthFront
	//_Data: this+0x24, Member, Type: float, lengthRear
	//_Func: public void CarCollisionBounds(); @loc=optimized @len=0 @rva=0
//UDT;

struct CarCollisionBounds {
public:
	vec3f min;
	vec3f max;
	float length;
	float width;
	float lengthFront;
	float lengthRear;
	inline CarCollisionBounds()  { }
	inline void _guard_obj() {
		static_assert((sizeof(CarCollisionBounds)==40),"bad size");
		static_assert((offsetof(CarCollisionBounds,min)==0x0),"bad off");
		static_assert((offsetof(CarCollisionBounds,max)==0xC),"bad off");
		static_assert((offsetof(CarCollisionBounds,length)==0x18),"bad off");
		static_assert((offsetof(CarCollisionBounds,width)==0x1C),"bad off");
		static_assert((offsetof(CarCollisionBounds,lengthFront)==0x20),"bad off");
		static_assert((offsetof(CarCollisionBounds,lengthRear)==0x24),"bad off");
	};
};

//UDT: class Turbo @len=36
	//_Func: public void Turbo(TurboDef & data); @loc=static @len=67 @rva=2811696
	//_Func: public void ~Turbo(); @loc=static @len=3 @rva=96368
	//_Func: public void step(float gas, float rpms, float dt); @loc=static @len=236 @rva=2811840
	//_Func: public float getBoost(); @loc=static @len=11 @rva=2811776
	//_Func: public void reset(); @loc=static @len=8 @rva=2811792
	//_Func: public void setTurboBoostLevel(float value); @loc=static @len=18 @rva=2811808
	//_Func: public float getTurboBoostLevel(); @loc=optimized @len=0 @rva=0
	//_Func: public void setWastegate(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getWastegate(); @loc=static @len=6 @rva=2645504
	//_Func: public float getMaxBoost(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setMaxBoost(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: float, userSetting
	//_Data: this+0x4, Member, Type: float, rotation
	//_Data: this+0x8, Member, Type: struct TurboDef, data
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Turbo)==36),"bad size");
		static_assert((offsetof(Turbo,userSetting)==0x0),"bad off");
		static_assert((offsetof(Turbo,rotation)==0x4),"bad off");
		static_assert((offsetof(Turbo,data)==0x8),"bad off");
	};
};

//UDT: struct OnMouseWheelMovedEvent @len=16
	//_Base: struct MouseEvent @off=0 @len=12
	//_Func: public void OnMouseWheelMovedEvent(int  _arg0, int  _arg1, float  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0xC, Member, Type: float, delta
//UDT;

struct OnMouseWheelMovedEvent : public MouseEvent {
public:
	float delta;
	inline OnMouseWheelMovedEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnMouseWheelMovedEvent)==16),"bad size");
		static_assert((offsetof(OnMouseWheelMovedEvent,delta)==0xC),"bad off");
	};
};

//UDT: struct SystemCBuffers @len=160
	//_Data: this+0x0, Member, Type: class CBuffer, cbCamera
	//_Data: this+0x28, Member, Type: class CBuffer, cbPerObject
	//_Data: this+0x50, Member, Type: class CBuffer, cbLighting
	//_Data: this+0x78, Member, Type: class CBuffer, cbShadowMap
	//_Func: public void SystemCBuffers(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~SystemCBuffers(); @loc=static @len=65 @rva=2106176
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct SystemCBuffers {
public:
	CBuffer cbCamera;
	CBuffer cbPerObject;
	CBuffer cbLighting;
	CBuffer cbShadowMap;
	inline SystemCBuffers()  { }
	inline void dtor() { typedef void (*_fpt)(SystemCBuffers *pthis); _fpt _f=(_fpt)_drva(2106176); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(SystemCBuffers)==160),"bad size");
		static_assert((offsetof(SystemCBuffers,cbCamera)==0x0),"bad off");
		static_assert((offsetof(SystemCBuffers,cbPerObject)==0x28),"bad off");
		static_assert((offsetof(SystemCBuffers,cbLighting)==0x50),"bad off");
		static_assert((offsetof(SystemCBuffers,cbShadowMap)==0x78),"bad off");
	};
};

//UDT: struct OnMouseMoveEvent @len=12
	//_Base: struct MouseEvent @off=0 @len=12
	//_Func: public void OnMouseMoveEvent(int  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
//UDT;

struct OnMouseMoveEvent : public MouseEvent {
public:
	inline OnMouseMoveEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnMouseMoveEvent)==12),"bad size");
	};
};

//UDT: class vec4f @len=16
	//_Data: this+0x0, Member, Type: float, x
	//_Data: this+0x4, Member, Type: float, y
	//_Data: this+0x8, Member, Type: float, z
	//_Data: this+0xC, Member, Type: float, w
	//_Func: public void vec4f(vec3f & v3, float vw); @loc=static @len=25 @rva=962960
	//_Func: public void vec4f(float *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void vec4f(float ix, float iy, float iz, float iw); @loc=static @len=29 @rva=3178496
	//_Func: public void vec4f(); @loc=static @len=13 @rva=508768
	//_Func: public vec3f toVec3fLinear(); @loc=optimized @len=0 @rva=0
	//_Func: public float length(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(vec4f)==16),"bad size");
		static_assert((offsetof(vec4f,x)==0x0),"bad off");
		static_assert((offsetof(vec4f,y)==0x4),"bad off");
		static_assert((offsetof(vec4f,z)==0x8),"bad off");
		static_assert((offsetof(vec4f,w)==0xC),"bad off");
	};
};

//UDT: class ThermalObject @len=32 @vfcount=1
	//_VTable: 
	//_Func: public void ThermalObject(ThermalObject &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ThermalObject(); @loc=static @len=42 @rva=2829952
	//_Func: public void ~ThermalObject(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2830000
	//_Data: this+0x8, Member, Type: float, tmass
	//_Data: this+0xC, Member, Type: float, coolSpeedK
	//_Data: this+0x10, Member, Type: float, coolFactor
	//_Data: this+0x14, Member, Type: float, heatFactor
	//_Data: this+0x18, Member, Type: float, t
	//_Func: public void step(float dt, float ambientTemp, Speed & speed); @loc=static @len=128 @rva=2830080
	//_Func: public void addHeadSource(float heat); @loc=static @len=11 @rva=2830064
	//_Data: this+0x1C, Member, Type: float, heatAccumulator
	//_Func: public ThermalObject & operator=(ThermalObject &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ThermalObject)==32),"bad size");
		static_assert((offsetof(ThermalObject,tmass)==0x8),"bad off");
		static_assert((offsetof(ThermalObject,coolSpeedK)==0xC),"bad off");
		static_assert((offsetof(ThermalObject,coolFactor)==0x10),"bad off");
		static_assert((offsetof(ThermalObject,heatFactor)==0x14),"bad off");
		static_assert((offsetof(ThermalObject,t)==0x18),"bad off");
		static_assert((offsetof(ThermalObject,heatAccumulator)==0x1C),"bad off");
	};
};

//UDT: class Joypad @len=8 @vfcount=7
	//_VTable: 
	//_Func: public void acquire(); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public vec2f getLeftStick(); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public vec2f getRightStick(); @intro @pure @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Func: public float getLeftTrigger(); @intro @pure @virtual vtpo=0 vfid=3 @loc=optimized @len=0 @rva=0
	//_Func: public float getRightTrigger(); @intro @pure @virtual vtpo=0 vfid=4 @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getButtonMask(); @intro @pure @virtual vtpo=0 vfid=5 @loc=optimized @len=0 @rva=0
	//_Func: public void setVibrations(float  _arg0, float  _arg1); @intro @pure @virtual vtpo=0 vfid=6 @loc=optimized @len=0 @rva=0
	//_Func: public void Joypad(Joypad &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Joypad(); @loc=optimized @len=0 @rva=0
	//_Func: public Joypad & operator=(Joypad &  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Joypad)==8),"bad size");
	};
};

//UDT: class ICollisionCallback @len=8 @vfcount=2
	//_VTable: 
	//_Func: public void ~ICollisionCallback(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2501856
	//_Func: public void onCollisionCallBack(void *  _arg0, void *  _arg1, void *  _arg2, void *  _arg3, vec3f  _arg4, vec3f  _arg5, float  _arg6); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void ICollisionCallback(ICollisionCallback &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ICollisionCallback(); @loc=optimized @len=0 @rva=0
	//_Func: public ICollisionCallback & operator=(ICollisionCallback &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class ICollisionCallback {
public:
	inline ICollisionCallback()  { }
	virtual ~ICollisionCallback();
	inline void dtor() { typedef void (*_fpt)(ICollisionCallback *pthis); _fpt _f=(_fpt)_drva(2501856); _f(this); }
	virtual void onCollisionCallBack_vf1(void *  _arg0, void *  _arg1, void *  _arg2, void *  _arg3, vec3f  _arg4, vec3f  _arg5, float  _arg6) = 0;
	inline void onCollisionCallBack(void *  _arg0, void *  _arg1, void *  _arg2, void *  _arg3, vec3f  _arg4, vec3f  _arg5, float  _arg6) { return onCollisionCallBack_vf1( _arg0,  _arg1,  _arg2,  _arg3,  _arg4,  _arg5,  _arg6); }
	inline void _guard_obj() {
		static_assert((sizeof(ICollisionCallback)==8),"bad size");
	};
};

//UDT: struct OnNewSessionEvent @len=120
	//_Data: this+0x0, Member, Type: class Session, newSession
	//_Data: this+0x70, Member, Type: short, index
	//_Func: public void OnNewSessionEvent(OnNewSessionEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void OnNewSessionEvent(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~OnNewSessionEvent(); @loc=static @len=90 @rva=584992
	//_Func: public OnNewSessionEvent & operator=(OnNewSessionEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct OnNewSessionEvent {
public:
	Session newSession;
	short index;
	inline OnNewSessionEvent()  { }
	inline void dtor() { typedef void (*_fpt)(OnNewSessionEvent *pthis); _fpt _f=(_fpt)_drva(584992); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(OnNewSessionEvent)==120),"bad size");
		static_assert((offsetof(OnNewSessionEvent,newSession)==0x0),"bad off");
		static_assert((offsetof(OnNewSessionEvent,index)==0x70),"bad off");
	};
};

//UDT: class SinSignalGenerator @len=16 @vfcount=3
	//_Base: class SignalGenerator @off=0 @len=16
	//_Func: public void SinSignalGenerator(SinSignalGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SinSignalGenerator(); @loc=static @len=33 @rva=2197680
	//_Func: public void ~SinSignalGenerator(); @virtual vtpo=0 vfid=0 @loc=static @len=15 @rva=2197728
	//_Func: public float getValue(); @virtual vtpo=0 vfid=2 @loc=static @len=81 @rva=2197808
	//_Func: public SinSignalGenerator & operator=(SinSignalGenerator &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class SinSignalGenerator : public SignalGenerator {
public:
	inline SinSignalGenerator()  { }
	inline void ctor() { typedef void (*_fpt)(SinSignalGenerator *pthis); _fpt _f=(_fpt)_drva(2197680); _f(this); }
	virtual ~SinSignalGenerator();
	inline void dtor() { typedef void (*_fpt)(SinSignalGenerator *pthis); _fpt _f=(_fpt)_drva(2197728); _f(this); }
	virtual float getValue_vf2();
	inline float getValue_impl() { typedef float (*_fpt)(SinSignalGenerator *pthis); _fpt _f=(_fpt)_drva(2197808); return _f(this); }
	inline float getValue() { return getValue_vf2(); }
	inline void _guard_obj() {
		static_assert((sizeof(SinSignalGenerator)==16),"bad size");
	};
};

//UDT: struct WingState @len=68
	//_Data: this+0x0, Member, Type: float, aoa
	//_Data: this+0x4, Member, Type: float, cd
	//_Data: this+0x8, Member, Type: float, cl
	//_Data: this+0xC, Member, Type: float, angle
	//_Data: this+0x10, Member, Type: float, inputAngle
	//_Data: this+0x14, Member, Type: float, groundHeight
	//_Data: this+0x18, Member, Type: float, frontShare
	//_Data: this+0x1C, Member, Type: float, dragKG
	//_Data: this+0x20, Member, Type: float, liftKG
	//_Data: this+0x24, Member, Type: float, angleMult
	//_Data: this+0x28, Member, Type: float, groundEffectLift
	//_Data: this+0x2C, Member, Type: float, groundEffectDrag
	//_Data: this+0x30, Member, Type: float, yawAngle
	//_Data: this+0x34, Member, Type: bool, isVertical
	//_Data: this+0x38, Member, Type: class vec3f, liftVector
	//_Func: public void WingState(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(WingState)==68),"bad size");
		static_assert((offsetof(WingState,aoa)==0x0),"bad off");
		static_assert((offsetof(WingState,cd)==0x4),"bad off");
		static_assert((offsetof(WingState,cl)==0x8),"bad off");
		static_assert((offsetof(WingState,angle)==0xC),"bad off");
		static_assert((offsetof(WingState,inputAngle)==0x10),"bad off");
		static_assert((offsetof(WingState,groundHeight)==0x14),"bad off");
		static_assert((offsetof(WingState,frontShare)==0x18),"bad off");
		static_assert((offsetof(WingState,dragKG)==0x1C),"bad off");
		static_assert((offsetof(WingState,liftKG)==0x20),"bad off");
		static_assert((offsetof(WingState,angleMult)==0x24),"bad off");
		static_assert((offsetof(WingState,groundEffectLift)==0x28),"bad off");
		static_assert((offsetof(WingState,groundEffectDrag)==0x2C),"bad off");
		static_assert((offsetof(WingState,yawAngle)==0x30),"bad off");
		static_assert((offsetof(WingState,isVertical)==0x34),"bad off");
		static_assert((offsetof(WingState,liftVector)==0x38),"bad off");
	};
};

//UDT: struct LightingSettings @len=180
	//_Func: public void LightingSettings(); @loc=static @len=323 @rva=2104736
	//_Data: this+0x0, Member, Type: class vec3f, lightDirection
	//_Data: this+0xC, Member, Type: class vec3f, lightColor
	//_Data: this+0x18, Member, Type: class vec3f, horizonLow
	//_Data: this+0x24, Member, Type: class vec3f, horizonHigh
	//_Data: this+0x30, Member, Type: class vec3f, skyLow
	//_Data: this+0x3C, Member, Type: class vec3f, skyHigh
	//_Data: this+0x48, Member, Type: class vec3f, sunLow
	//_Data: this+0x54, Member, Type: class vec3f, sunHigh
	//_Data: this+0x60, Member, Type: float, angle
	//_Data: this+0x64, Member, Type: float, headingAngle
	//_Data: this+0x68, Member, Type: float, pitchAngle
	//_Data: this+0x6C, Member, Type: class vec3f, ambientLow
	//_Data: this+0x78, Member, Type: class vec3f, ambientHigh
	//_Data: this+0x84, Member, Type: class vec3f, fogColor
	//_Data: this+0x90, Member, Type: float, fogLinear
	//_Data: this+0x94, Member, Type: float, fogBlend
	//_Data: this+0x98, Member, Type: float, cloudCover
	//_Data: this+0x9C, Member, Type: float, cloudCutoff
	//_Data: this+0xA0, Member, Type: float, cloudColor
	//_Data: this+0xA4, Member, Type: float, cloudOffset
	//_Data: this+0xA8, Member, Type: float, saturation
	//_Data: this+0xAC, Member, Type: float, gameTime
	//_Data: this+0xB0, Member, Type: float, sunAngleGamma
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(LightingSettings)==180),"bad size");
		static_assert((offsetof(LightingSettings,lightDirection)==0x0),"bad off");
		static_assert((offsetof(LightingSettings,lightColor)==0xC),"bad off");
		static_assert((offsetof(LightingSettings,horizonLow)==0x18),"bad off");
		static_assert((offsetof(LightingSettings,horizonHigh)==0x24),"bad off");
		static_assert((offsetof(LightingSettings,skyLow)==0x30),"bad off");
		static_assert((offsetof(LightingSettings,skyHigh)==0x3C),"bad off");
		static_assert((offsetof(LightingSettings,sunLow)==0x48),"bad off");
		static_assert((offsetof(LightingSettings,sunHigh)==0x54),"bad off");
		static_assert((offsetof(LightingSettings,angle)==0x60),"bad off");
		static_assert((offsetof(LightingSettings,headingAngle)==0x64),"bad off");
		static_assert((offsetof(LightingSettings,pitchAngle)==0x68),"bad off");
		static_assert((offsetof(LightingSettings,ambientLow)==0x6C),"bad off");
		static_assert((offsetof(LightingSettings,ambientHigh)==0x78),"bad off");
		static_assert((offsetof(LightingSettings,fogColor)==0x84),"bad off");
		static_assert((offsetof(LightingSettings,fogLinear)==0x90),"bad off");
		static_assert((offsetof(LightingSettings,fogBlend)==0x94),"bad off");
		static_assert((offsetof(LightingSettings,cloudCover)==0x98),"bad off");
		static_assert((offsetof(LightingSettings,cloudCutoff)==0x9C),"bad off");
		static_assert((offsetof(LightingSettings,cloudColor)==0xA0),"bad off");
		static_assert((offsetof(LightingSettings,cloudOffset)==0xA4),"bad off");
		static_assert((offsetof(LightingSettings,saturation)==0xA8),"bad off");
		static_assert((offsetof(LightingSettings,gameTime)==0xAC),"bad off");
		static_assert((offsetof(LightingSettings,sunAngleGamma)==0xB0),"bad off");
	};
};

//UDT: struct SplinePoint @len=20
	//_Func: public void SplinePoint(vec3f &  _arg0, float  _arg1, int  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, point
	//_Data: this+0xC, Member, Type: float, pointLength
	//_Data: this+0x10, Member, Type: int, tag
//UDT;

struct SplinePoint {
public:
	vec3f point;
	float pointLength;
	int tag;
	inline SplinePoint()  { }
	inline void _guard_obj() {
		static_assert((sizeof(SplinePoint)==20),"bad size");
		static_assert((offsetof(SplinePoint,point)==0x0),"bad off");
		static_assert((offsetof(SplinePoint,pointLength)==0xC),"bad off");
		static_assert((offsetof(SplinePoint,tag)==0x10),"bad off");
	};
};

//UDT: class IKeyEventListener @len=8 @vfcount=2
	//_VTable: 
	//_Func: public void onKeyDown(OnKeyEvent &  _arg0); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void onKeyChar(unsigned int  _arg0); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void IKeyEventListener(IKeyEventListener &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IKeyEventListener(); @loc=optimized @len=0 @rva=0
	//_Func: public IKeyEventListener & operator=(IKeyEventListener &  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class IKeyEventListener {
public:
	inline IKeyEventListener()  { }
	virtual void onKeyDown_vf0(OnKeyEvent &  _arg0) = 0;
	inline void onKeyDown(OnKeyEvent &  _arg0) { return onKeyDown_vf0( _arg0); }
	virtual void onKeyChar_vf1(unsigned int  _arg0) = 0;
	inline void onKeyChar(unsigned int  _arg0) { return onKeyChar_vf1( _arg0); }
	inline void _guard_obj() {
		static_assert((sizeof(IKeyEventListener)==8),"bad size");
	};
};

//UDT: struct ksgui::GraphReferenceAxis @len=20
	//_Func: public void GraphReferenceAxis(vec3f & acolor, float arefValue, bool bisVertical); @loc=static @len=35 @rva=2987152
	//_Data: this+0x0, Member, Type: class vec3f, color
	//_Data: this+0xC, Member, Type: float, refValue
	//_Data: this+0x10, Member, Type: bool, isVertical
//UDT;

struct ksgui_GraphReferenceAxis {
public:
	vec3f color;
	float refValue;
	bool isVertical;
	inline ksgui_GraphReferenceAxis()  { }
	inline void ctor(vec3f & acolor, float arefValue, bool bisVertical) { typedef void (*_fpt)(ksgui_GraphReferenceAxis *pthis, vec3f &, float, bool); _fpt _f=(_fpt)_drva(2987152); _f(this, acolor, arefValue, bisVertical); }
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_GraphReferenceAxis)==20),"bad size");
		static_assert((offsetof(ksgui_GraphReferenceAxis,color)==0x0),"bad off");
		static_assert((offsetof(ksgui_GraphReferenceAxis,refValue)==0xC),"bad off");
		static_assert((offsetof(ksgui_GraphReferenceAxis,isVertical)==0x10),"bad off");
	};
};

//UDT: struct Wind @len=20
	//_Data: this+0x0, Member, Type: class vec3f, vector
	//_Data: this+0xC, Member, Type: class Speed, speed
	//_Data: this+0x10, Member, Type: float, directionDeg
	//_Func: public void Wind(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~Wind(); @loc=static @len=9 @rva=2502208
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct Wind {
public:
	vec3f vector;
	Speed speed;
	float directionDeg;
	inline Wind()  { }
	inline void dtor() { typedef void (*_fpt)(Wind *pthis); _fpt _f=(_fpt)_drva(2502208); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(Wind)==20),"bad size");
		static_assert((offsetof(Wind,vector)==0x0),"bad off");
		static_assert((offsetof(Wind,speed)==0xC),"bad off");
		static_assert((offsetof(Wind,directionDeg)==0x10),"bad off");
	};
};

//UDT: struct ACPhysicsEvent @len=72
	//_Data: this+0x0, Member, Type: enum eACEventType, type
	//_Data: this+0x4, Member, Type: float, param1
	//_Data: this+0x8, Member, Type: float, param2
	//_Data: this+0xC, Member, Type: float, param3
	//_Data: this+0x10, Member, Type: float, param4
	//_Data: this+0x14, Member, Type: class vec3f, vParam1
	//_Data: this+0x20, Member, Type: class vec3f, vParam2
	//_Data: this+0x30, Member, Type: void *, voidParam0
	//_Data: this+0x38, Member, Type: void *, voidParam1
	//_Data: this+0x40, Member, Type: unsigned long, ulParam0
	//_Func: public void ACPhysicsEvent(); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ACPhysicsEvent)==72),"bad size");
		static_assert((offsetof(ACPhysicsEvent,type)==0x0),"bad off");
		static_assert((offsetof(ACPhysicsEvent,param1)==0x4),"bad off");
		static_assert((offsetof(ACPhysicsEvent,param2)==0x8),"bad off");
		static_assert((offsetof(ACPhysicsEvent,param3)==0xC),"bad off");
		static_assert((offsetof(ACPhysicsEvent,param4)==0x10),"bad off");
		static_assert((offsetof(ACPhysicsEvent,vParam1)==0x14),"bad off");
		static_assert((offsetof(ACPhysicsEvent,vParam2)==0x20),"bad off");
		static_assert((offsetof(ACPhysicsEvent,voidParam0)==0x30),"bad off");
		static_assert((offsetof(ACPhysicsEvent,voidParam1)==0x38),"bad off");
		static_assert((offsetof(ACPhysicsEvent,ulParam0)==0x40),"bad off");
	};
};

//UDT: struct RayCastHit @len=40
	//_Func: public void RayCastHit(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, pos
	//_Data: this+0xC, Member, Type: class vec3f, normal
	//_Data: this+0x18, Member, Type: class ICollisionObject *, collisionObject
	//_Data: this+0x20, Member, Type: bool, hasContact
//UDT;

struct RayCastHit {
public:
	vec3f pos;
	vec3f normal;
	ICollisionObject * collisionObject;
	bool hasContact;
	inline RayCastHit()  { }
	inline void _guard_obj() {
		static_assert((sizeof(RayCastHit)==40),"bad size");
		static_assert((offsetof(RayCastHit,pos)==0x0),"bad off");
		static_assert((offsetof(RayCastHit,normal)==0xC),"bad off");
		static_assert((offsetof(RayCastHit,collisionObject)==0x18),"bad off");
		static_assert((offsetof(RayCastHit,hasContact)==0x20),"bad off");
	};
};

//UDT: class ITyreModel @len=8 @vfcount=2
	//_VTable: 
	//_Func: public void ~ITyreModel(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=4503904
	//_Func: public TyreModelOutput solve(TyreModelInput &  _arg0); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void ITyreModel(ITyreModel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ITyreModel(); @loc=optimized @len=0 @rva=0
	//_Func: public ITyreModel & operator=(ITyreModel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class ITyreModel {
public:
	inline ITyreModel()  { }
	virtual ~ITyreModel();
	inline void dtor() { typedef void (*_fpt)(ITyreModel *pthis); _fpt _f=(_fpt)_drva(4503904); _f(this); }
	virtual TyreModelOutput solve_vf1(TyreModelInput &  _arg0) = 0;
	inline TyreModelOutput solve(TyreModelInput &  _arg0) { return solve_vf1( _arg0); }
	inline void _guard_obj() {
		static_assert((sizeof(ITyreModel)==8),"bad size");
	};
};

//UDT: class BrushTyreModel @len=28
	//_Func: public void BrushTyreModel(); @loc=static @len=46 @rva=2929488
	//_Func: public void ~BrushTyreModel(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: struct BrushTyreModelData, data
	//_Func: public BrushOutput solve(float slip, float friction, float load, float cf1_mix, float asy); @loc=static @len=286 @rva=2929600
	//_Func: public BrushOutput solveV5(float slip, float load, float asy); @loc=static @len=236 @rva=2929888
	//_Func: public float getCFFromSlipAngle(float angle); @loc=static @len=56 @rva=2929536
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class BrushTyreModel {
public:
	BrushTyreModelData data;
	inline BrushTyreModel()  { }
	inline void ctor() { typedef void (*_fpt)(BrushTyreModel *pthis); _fpt _f=(_fpt)_drva(2929488); _f(this); }
	inline void dtor() { typedef void (*_fpt)(BrushTyreModel *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline BrushOutput solve(float slip, float friction, float load, float cf1_mix, float asy) { typedef BrushOutput (*_fpt)(BrushTyreModel *pthis, float, float, float, float, float); _fpt _f=(_fpt)_drva(2929600); return _f(this, slip, friction, load, cf1_mix, asy); }
	inline BrushOutput solveV5(float slip, float load, float asy) { typedef BrushOutput (*_fpt)(BrushTyreModel *pthis, float, float, float); _fpt _f=(_fpt)_drva(2929888); return _f(this, slip, load, asy); }
	inline float getCFFromSlipAngle(float angle) { typedef float (*_fpt)(BrushTyreModel *pthis, float); _fpt _f=(_fpt)_drva(2929536); return _f(this, angle); }
	inline void _guard_obj() {
		static_assert((sizeof(BrushTyreModel)==28),"bad size");
		static_assert((offsetof(BrushTyreModel,data)==0x0),"bad off");
	};
};

//UDT: struct DRS @len=48
	//_Func: public void ~DRS(); @loc=static @len=53 @rva=650384
	//_Data: this+0x0, Member, Type: bool, isPresent
	//_Data: this+0x1, Member, Type: bool, isActive
	//_Data: this+0x2, Member, Type: bool, isAvailable
	//_Data: this+0x3, Member, Type: bool, ignoreZones
	//_Data: this+0x8, Member, Type: class std::vector<DRSWingConnection,std::allocator<DRSWingConnection> >, wings
	//_Func: public void init(Car * car); @loc=static @len=2665 @rva=2835248
	//_Func: public void step(float dt); @loc=static @len=312 @rva=2838112
	//_Data: this+0x20, Member, Type: class Car *, car
	//_Data: this+0x28, Member, Type: bool, lastState
	//_Data: this+0x2C, Member, Type: float, limitG
	//_Func: public void DRS(DRS &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void DRS(); @loc=optimized @len=0 @rva=0
	//_Func: public DRS & operator=(DRS &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DRS)==48),"bad size");
		static_assert((offsetof(DRS,isPresent)==0x0),"bad off");
		static_assert((offsetof(DRS,isActive)==0x1),"bad off");
		static_assert((offsetof(DRS,isAvailable)==0x2),"bad off");
		static_assert((offsetof(DRS,ignoreZones)==0x3),"bad off");
		static_assert((offsetof(DRS,wings)==0x8),"bad off");
		static_assert((offsetof(DRS,car)==0x20),"bad off");
		static_assert((offsetof(DRS,lastState)==0x28),"bad off");
		static_assert((offsetof(DRS,limitG)==0x2C),"bad off");
	};
};

//UDT: class IPAddress @len=16
	//_Func: public void IPAddress(); @loc=static @len=13 @rva=508768
	//_Func: public void IPAddress(IPAddress &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IPAddress(sockaddr_in & addr); @loc=static @len=10 @rva=2479584
	//_Func: public void IPAddress(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & addr, unsigned short port); @loc=static @len=143 @rva=2479600
	//_Func: public void ~IPAddress(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: struct sockaddr_in, sokaddr
	//_Func: public unsigned short getPort(); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getAddress(); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getAddressPort(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class IPAddress {
public:
	sockaddr_in sokaddr;
	inline IPAddress()  { }
	inline void ctor() { typedef void (*_fpt)(IPAddress *pthis); _fpt _f=(_fpt)_drva(508768); _f(this); }
	inline void ctor(sockaddr_in & addr) { typedef void (*_fpt)(IPAddress *pthis, sockaddr_in &); _fpt _f=(_fpt)_drva(2479584); _f(this, addr); }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & addr, unsigned short port) { typedef void (*_fpt)(IPAddress *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &, unsigned short); _fpt _f=(_fpt)_drva(2479600); _f(this, addr, port); }
	inline void dtor() { typedef void (*_fpt)(IPAddress *pthis); _fpt _f=(_fpt)_drva(96368); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(IPAddress)==16),"bad size");
		static_assert((offsetof(IPAddress,sokaddr)==0x0),"bad off");
	};
};

//UDT: struct RemoteSessionResume @len=176
	//_Data: this+0x0, Member, Type: struct RemoteSession, session
	//_Data: this+0x40, Member, Type: struct RemoteSessionResult, results
	//_Func: public void RemoteSessionResume(RemoteSessionResume &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void RemoteSessionResume(); @loc=static @len=103 @rva=243232
	//_Func: public void ~RemoteSessionResume(); @loc=optimized @len=0 @rva=0
	//_Func: public RemoteSessionResume & operator=(RemoteSessionResume &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct RemoteSessionResume {
public:
	RemoteSession session;
	RemoteSessionResult results;
	inline RemoteSessionResume()  { }
	inline void ctor() { typedef void (*_fpt)(RemoteSessionResume *pthis); _fpt _f=(_fpt)_drva(243232); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(RemoteSessionResume)==176),"bad size");
		static_assert((offsetof(RemoteSessionResume,session)==0x0),"bad off");
		static_assert((offsetof(RemoteSessionResume,results)==0x40),"bad off");
	};
};

//UDT: struct ClientCollisionEvent @len=40
	//_Data: this+0x0, Member, Type: class NetCarStateProvider *, netCar
	//_Data: this+0x8, Member, Type: float, speed
	//_Data: this+0xC, Member, Type: class vec3f, worldPos
	//_Data: this+0x18, Member, Type: class vec3f, relPos
	//_Func: public void ClientCollisionEvent(); @loc=optimized @len=0 @rva=0
//UDT;

struct ClientCollisionEvent {
public:
	NetCarStateProvider * netCar;
	float speed;
	vec3f worldPos;
	vec3f relPos;
	inline ClientCollisionEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(ClientCollisionEvent)==40),"bad size");
		static_assert((offsetof(ClientCollisionEvent,netCar)==0x0),"bad off");
		static_assert((offsetof(ClientCollisionEvent,speed)==0x8),"bad off");
		static_assert((offsetof(ClientCollisionEvent,worldPos)==0xC),"bad off");
		static_assert((offsetof(ClientCollisionEvent,relPos)==0x18),"bad off");
	};
};

//UDT: struct MeshVertex @len=44
	//_Func: public void MeshVertex(MeshVertex &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void MeshVertex(vec3f &  _arg0, vec3f &  _arg1, vec2f &  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public void MeshVertex(vec3f &  _arg0, vec3f &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void MeshVertex(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, pos
	//_Data: this+0xC, Member, Type: class vec3f, normal
	//_Data: this+0x18, Member, Type: class vec2f, texCoord
	//_Data: this+0x20, Member, Type: class vec3f, tangent
//UDT;

struct MeshVertex {
public:
	vec3f pos;
	vec3f normal;
	vec2f texCoord;
	vec3f tangent;
	inline MeshVertex()  { }
	inline void _guard_obj() {
		static_assert((sizeof(MeshVertex)==44),"bad size");
		static_assert((offsetof(MeshVertex,pos)==0x0),"bad off");
		static_assert((offsetof(MeshVertex,normal)==0xC),"bad off");
		static_assert((offsetof(MeshVertex,texCoord)==0x18),"bad off");
		static_assert((offsetof(MeshVertex,tangent)==0x20),"bad off");
	};
};

//UDT: struct OnCollisionEvent @len=40
	//_Data: this+0x0, Member, Type: class IRigidBody *, body
	//_Data: this+0x8, Member, Type: float, relativeSpeed
	//_Data: this+0xC, Member, Type: class vec3f, worldPos
	//_Data: this+0x18, Member, Type: class vec3f, relPos
	//_Data: this+0x24, Member, Type: unsigned long, colliderGroup
	//_Func: public void OnCollisionEvent(); @loc=optimized @len=0 @rva=0
//UDT;

struct OnCollisionEvent {
public:
	IRigidBody * body;
	float relativeSpeed;
	vec3f worldPos;
	vec3f relPos;
	unsigned long colliderGroup;
	inline OnCollisionEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnCollisionEvent)==40),"bad size");
		static_assert((offsetof(OnCollisionEvent,body)==0x0),"bad off");
		static_assert((offsetof(OnCollisionEvent,relativeSpeed)==0x8),"bad off");
		static_assert((offsetof(OnCollisionEvent,worldPos)==0xC),"bad off");
		static_assert((offsetof(OnCollisionEvent,relPos)==0x18),"bad off");
		static_assert((offsetof(OnCollisionEvent,colliderGroup)==0x24),"bad off");
	};
};

//UDT: struct OnMouseUpEvent @len=12
	//_Base: struct MouseEvent @off=0 @len=12
	//_Func: public void OnMouseUpEvent(int  _arg0, int  _arg1, MouseButton  _arg2); @loc=optimized @len=0 @rva=0
//UDT;

struct OnMouseUpEvent : public MouseEvent {
public:
	inline OnMouseUpEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnMouseUpEvent)==12),"bad size");
	};
};

//UDT: struct OnMouseDownEvent @len=12
	//_Base: struct MouseEvent @off=0 @len=12
	//_Func: public void OnMouseDownEvent(int  _arg0, int  _arg1, MouseButton  _arg2); @loc=optimized @len=0 @rva=0
//UDT;

struct OnMouseDownEvent : public MouseEvent {
public:
	inline OnMouseDownEvent()  { }
	inline void _guard_obj() {
		static_assert((sizeof(OnMouseDownEvent)==12),"bad size");
	};
};

//UDT: class CommandManager @len=16
	//_Func: public void CommandManager(CommandManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CommandManager(); @loc=static @len=8361 @rva=953200
	//_Func: public void ~CommandManager(); @loc=static @len=41 @rva=2105504
	//_Func: public int getCommand(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & commandName); @loc=static @len=16 @rva=962064
	//_Func: public bool isTriggered(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,CommandItem,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,CommandItem> > >, commands
	//_Func: public CommandManager & operator=(CommandManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class CommandManager {
public:
	std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,CommandItem,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,CommandItem> > > commands;
	inline CommandManager()  { }
	inline void ctor() { typedef void (*_fpt)(CommandManager *pthis); _fpt _f=(_fpt)_drva(953200); _f(this); }
	inline void dtor() { typedef void (*_fpt)(CommandManager *pthis); _fpt _f=(_fpt)_drva(2105504); _f(this); }
	inline int getCommand(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & commandName) { typedef int (*_fpt)(CommandManager *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &); _fpt _f=(_fpt)_drva(962064); return _f(this, commandName); }
	inline void _guard_obj() {
		static_assert((sizeof(CommandManager)==16),"bad size");
		static_assert((offsetof(CommandManager,commands)==0x0),"bad off");
	};
};

//UDT: class Texture @len=40
	//_Func: public void Texture(Texture &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Texture(RenderTarget & rt); @loc=static @len=51 @rva=2088512
	//_Func: public void Texture(unsigned char * buffer, unsigned int size); @loc=static @len=56 @rva=2088576
	//_Func: public void Texture(unsigned char * buffer, unsigned int width, unsigned int height, PixelFormat aFormat); @loc=static @len=79 @rva=2088640
	//_Func: public void Texture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=125 @rva=2088384
	//_Func: public void Texture(); @loc=static @len=25 @rva=205552
	//_Func: public void ~Texture(); @loc=static @len=49 @rva=2180672
	//_Data: this+0x0, Member, Type: void *, kid
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, fileName
	//_Func: public void reload(); @loc=optimized @len=0 @rva=0
	//_Func: public void save(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public PixelFormat getTexturePixelFormat(); @loc=optimized @len=0 @rva=0
	//_Func: public void release(); @loc=static @len=35 @rva=2088720
	//_Func: public bool isNull(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isValid(); @loc=optimized @len=0 @rva=0
	//_Func: public void getSize(unsigned int &  _arg0, unsigned int &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int height(); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int width(); @loc=optimized @len=0 @rva=0
	//_Func: public Texture & operator=(Texture & __that); @loc=static @len=49 @rva=206640
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Texture)==40),"bad size");
		static_assert((offsetof(Texture,kid)==0x0),"bad off");
		static_assert((offsetof(Texture,fileName)==0x8),"bad off");
	};
};

//UDT: class ThreadPool @len=104 @vfcount=1
	//_VTable: 
	//_Func: public void ThreadPool(ThreadPool &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ThreadPool(int inumThreads, std::function<void __cdecl(int)> * initFun); @loc=static @len=740 @rva=2950304
	//_Func: public void ~ThreadPool(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=323 @rva=2951328
	//_Func: public void join(); @loc=optimized @len=0 @rva=0
	//_Func: public void addTask(Task & task); @loc=static @len=131 @rva=2953008
	//_Func: public unsigned int workersCount(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x8, Member, Type: int, numThreads
	//_Data: this+0x10, Member, Type: class std::vector<std::thread,std::allocator<std::thread> >, workers
	//_Data: this+0x28, Member, Type: bool, stop
	//_Data: this+0x30, Member, Type: class std::deque<Task *,std::allocator<Task *> >, tasks
	//_Data: this+0x58, Member, Type: class std::mutex, queue_mutex
	//_Data: this+0x60, Member, Type: class std::condition_variable, condition
	//_Func: public ThreadPool & operator=(ThreadPool &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ThreadPool)==104),"bad size");
		static_assert((offsetof(ThreadPool,numThreads)==0x8),"bad off");
		static_assert((offsetof(ThreadPool,workers)==0x10),"bad off");
		static_assert((offsetof(ThreadPool,stop)==0x28),"bad off");
		static_assert((offsetof(ThreadPool,tasks)==0x30),"bad off");
		static_assert((offsetof(ThreadPool,queue_mutex)==0x58),"bad off");
		static_assert((offsetof(ThreadPool,condition)==0x60),"bad off");
	};
};

//UDT: struct PerformanceMeter @len=112
	//_Func: public void ~PerformanceMeter(); @loc=static @len=110 @rva=2536400
	//_Data: this+0x0, Member, Type: bool, isEnabled
	//_Func: public void init(Car * car); @loc=static @len=35 @rva=2537408
	//_Func: public void step(float dt); @loc=static @len=497 @rva=2537696
	//_Func: public PerformanceSplit getCurrentSplit(); @loc=static @len=157 @rva=2537216
	//_Func: public bool hasData(); @loc=static @len=19 @rva=2537376
	//_Func: public void reset(); @loc=static @len=87 @rva=2537600
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: class std::vector<PerformancePair,std::allocator<PerformancePair> >, currentLap
	//_Data: this+0x28, Member, Type: class std::vector<PerformancePair,std::allocator<PerformancePair> >, bestLap
	//_Data: this+0x40, Member, Type: double, bestLapTime
	//_Data: this+0x48, Member, Type: int, lastLapIndex
	//_Data: this+0x50, Member, Type: double, currentDistance
	//_Data: this+0x58, Member, Type: double, lastRecordedDistance
	//_Data: this+0x60, Member, Type: double, currentDiff
	//_Data: this+0x68, Member, Type: float, currentSpeedDiffMS
	//_Func: public void PerformanceMeter(PerformanceMeter &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void PerformanceMeter(); @loc=optimized @len=0 @rva=0
	//_Func: public PerformanceMeter & operator=(PerformanceMeter &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(PerformanceMeter)==112),"bad size");
		static_assert((offsetof(PerformanceMeter,isEnabled)==0x0),"bad off");
		static_assert((offsetof(PerformanceMeter,car)==0x8),"bad off");
		static_assert((offsetof(PerformanceMeter,currentLap)==0x10),"bad off");
		static_assert((offsetof(PerformanceMeter,bestLap)==0x28),"bad off");
		static_assert((offsetof(PerformanceMeter,bestLapTime)==0x40),"bad off");
		static_assert((offsetof(PerformanceMeter,lastLapIndex)==0x48),"bad off");
		static_assert((offsetof(PerformanceMeter,currentDistance)==0x50),"bad off");
		static_assert((offsetof(PerformanceMeter,lastRecordedDistance)==0x58),"bad off");
		static_assert((offsetof(PerformanceMeter,currentDiff)==0x60),"bad off");
		static_assert((offsetof(PerformanceMeter,currentSpeedDiffMS)==0x68),"bad off");
	};
};

//UDT: class TelemetryChannel @len=88
	//_Func: public void TelemetryChannel(TelemetryChannel & __that); @loc=static @len=148 @rva=2866288
	//_Func: public void TelemetryChannel(std::basic_string<char,std::char_traits<char>,std::allocator<char> > & name, float * adataSource, TelemetryUnits units, int frequency, float scale); @loc=static @len=156 @rva=2866448
	//_Data: this+0x0, Member, Type: class std::basic_string<char,std::char_traits<char>,std::allocator<char> >, name
	//_Data: this+0x20, Member, Type: struct TelemetryChannelData, data
	//_Data: this+0x40, Member, Type: float *, dataSource
	//_Data: this+0x48, Member, Type: double, lastTickTime
	//_Data: this+0x50, Member, Type: float, scale
	//_Func: public void ~TelemetryChannel(); @loc=static @len=103 @rva=2549872
	//_Func: public TelemetryChannel & operator=(TelemetryChannel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TelemetryChannel)==88),"bad size");
		static_assert((offsetof(TelemetryChannel,name)==0x0),"bad off");
		static_assert((offsetof(TelemetryChannel,data)==0x20),"bad off");
		static_assert((offsetof(TelemetryChannel,dataSource)==0x40),"bad off");
		static_assert((offsetof(TelemetryChannel,lastTickTime)==0x48),"bad off");
		static_assert((offsetof(TelemetryChannel,scale)==0x50),"bad off");
	};
};

//UDT: class Curve @len=128 @vfcount=1
	//_VTable: 
	//_Func: public void Curve(Curve & __that); @loc=static @len=158 @rva=844960
	//_Func: public void Curve(); @loc=static @len=85 @rva=2119856
	//_Func: public void ~Curve(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=216 @rva=2119952
	//_Data: static, [0155A278][0003:00047278], Static Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, openedFiles
	//_Func: public void load(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename); @loc=static @len=1276 @rva=2124368
	//_Func: public void loadEncrypted(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataFile); @loc=static @len=1013 @rva=2125648
	//_Func: public void save(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void scale(float v); @loc=static @len=39 @rva=2126816
	//_Func: public void scaleReferences(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getValue(float ref); @loc=static @len=186 @rva=2124176
	//_Func: public float getValueUncapped(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getValueCatmullRom(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getCubicSplineValue(float ref); @loc=static @len=61 @rva=2124016
	//_Func: public void addValue(float ref, float val); @loc=static @len=312 @rva=2120416
	//_Func: public void resample(float  _arg0, float  _arg1, float  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public void getMaxValue(float &  _arg0, float &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaxReference(); @loc=static @len=10 @rva=2124080
	//_Func: public int getCount(); @loc=static @len=13 @rva=2124000
	//_Func: public float getValueFast(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public std::pair<float,float> getPairAtIndex(int index); @loc=static @len=79 @rva=2124096
	//_Func: public void clear(); @loc=optimized @len=0 @rva=0
	//_Func: public void print(); @loc=static @len=137 @rva=2126672
	//_Data: this+0x8, Member, Type: class std::vector<float,std::allocator<float> >, references
	//_Data: this+0x20, Member, Type: class std::vector<float,std::allocator<float> >, values
	//_Data: this+0x38, Member, Type: float, fastStep
	//_Data: this+0x3C, Member, Type: bool, cubicSplineReady
	//_Data: this+0x40, Member, Type: class CubicSpline<float,float>, cSpline
	//_Data: this+0x60, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, filename
	//_Func: public Curve & operator=(Curve & __that); @loc=static @len=107 @rva=355408
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Curve)==128),"bad size");
		static_assert((offsetof(Curve,references)==0x8),"bad off");
		static_assert((offsetof(Curve,values)==0x20),"bad off");
		static_assert((offsetof(Curve,fastStep)==0x38),"bad off");
		static_assert((offsetof(Curve,cubicSplineReady)==0x3C),"bad off");
		static_assert((offsetof(Curve,cSpline)==0x40),"bad off");
		static_assert((offsetof(Curve,filename)==0x60),"bad off");
	};
};

//UDT: struct HeaveSpring @len=88
	//_Func: public void ~HeaveSpring(); @loc=static @len=9 @rva=2831104
	//_Data: this+0x0, Member, Type: bool, isPresent
	//_Data: this+0x4, Member, Type: float, rodLength
	//_Data: this+0x8, Member, Type: struct HeaveSpringStatus, status
	//_Data: this+0xC, Member, Type: float, k
	//_Data: this+0x10, Member, Type: float, progressiveK
	//_Data: this+0x14, Member, Type: float, packerRange
	//_Data: this+0x18, Member, Type: float, bumpStopRate
	//_Data: this+0x1C, Member, Type: float, bumpStopUp
	//_Data: this+0x20, Member, Type: float, bumpStopDn
	//_Data: this+0x24, Member, Type: class Damper, damper
	//_Data: this+0x3C, Member, Type: bool, isFront
	//_Func: public void init(Car * car, Suspension * s0, Suspension * s1, bool isFront); @loc=static @len=34 @rva=2831120
	//_Func: public void step(float dt); @loc=static @len=2039 @rva=2832736
	//_Data: this+0x40, Member, Type: class Suspension *[0x2], suspensions
	//_Data: this+0x50, Member, Type: class Car *, car
	//_Func: private void initData(); @loc=static @len=1565 @rva=2831168
	//_Func: public void HeaveSpring(); @loc=static @len=52 @rva=2546416
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(HeaveSpring)==88),"bad size");
		static_assert((offsetof(HeaveSpring,isPresent)==0x0),"bad off");
		static_assert((offsetof(HeaveSpring,rodLength)==0x4),"bad off");
		static_assert((offsetof(HeaveSpring,status)==0x8),"bad off");
		static_assert((offsetof(HeaveSpring,k)==0xC),"bad off");
		static_assert((offsetof(HeaveSpring,progressiveK)==0x10),"bad off");
		static_assert((offsetof(HeaveSpring,packerRange)==0x14),"bad off");
		static_assert((offsetof(HeaveSpring,bumpStopRate)==0x18),"bad off");
		static_assert((offsetof(HeaveSpring,bumpStopUp)==0x1C),"bad off");
		static_assert((offsetof(HeaveSpring,bumpStopDn)==0x20),"bad off");
		static_assert((offsetof(HeaveSpring,damper)==0x24),"bad off");
		static_assert((offsetof(HeaveSpring,isFront)==0x3C),"bad off");
		static_assert((offsetof(HeaveSpring,suspensions)==0x40),"bad off");
		static_assert((offsetof(HeaveSpring,car)==0x50),"bad off");
	};
};

//UDT: struct TimeTransponder @len=152
	//_Func: public void ~TimeTransponder(); @loc=static @len=184 @rva=2688112
	//_Data: this+0x0, Member, Type: unsigned int, t
	//_Data: this+0x4, Member, Type: unsigned int, lastLap
	//_Data: this+0x8, Member, Type: unsigned int, bestLap
	//_Data: this+0xC, Member, Type: unsigned int, lapCount
	//_Data: this+0x10, Member, Type: bool, finishLinePassed
	//_Data: this+0x11, Member, Type: bool, wasLastLapValid
	//_Func: public void init(Car * car); @loc=static @len=298 @rva=2688960
	//_Func: public bool isValid(); @loc=static @len=195 @rva=2689408
	//_Func: public void onTimeLinePassed(int index, bool isFinishLine); @loc=static @len=569 @rva=2690080
	//_Func: public TimeLineStatus & getStatus(int index); @loc=static @len=217 @rva=2688736
	//_Func: public std::vector<unsigned int,std::allocator<unsigned int> > getLastLapSplits(); @loc=static @len=38 @rva=2688688
	//_Func: public unsigned int getCurrentSplit(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void invalidate(); @loc=static @len=112 @rva=2689264
	//_Func: public void reset(); @loc=static @len=219 @rva=2690656
	//_Func: public void step(float dt); @loc=static @len=476 @rva=2691568
	//_Func: public void armFirstLap(); @loc=static @len=8 @rva=2688656
	//_Func: public void addCut(); @loc=static @len=7 @rva=2688640
	//_Func: public int getCuts(); @loc=static @len=7 @rva=2688672
	//_Func: public bool isInOpenTrackTransition(); @loc=static @len=24 @rva=2689376
	//_Data: this+0x18, Member, Type: class std::vector<TimeLineStatus,std::allocator<TimeLineStatus> >, status
	//_Data: this+0x30, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, lastLapSplits
	//_Data: this+0x48, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, bestLapSplits
	//_Data: this+0x60, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, currentSplits
	//_Data: this+0x78, Member, Type: class Car *, car
	//_Data: this+0x80, Member, Type: bool, isFirstLapArmed
	//_Data: this+0x84, Member, Type: int, cuts
	//_Data: this+0x88, Member, Type: bool, extInvalid
	//_Data: this+0x8C, Member, Type: enum OpenTrackTimeState, openTrackState
	//_Data: this+0x90, Member, Type: bool, isOpenTrack
	//_Func: protected void lap(bool valid); @loc=static @len=462 @rva=2689616
	//_Func: protected void split(int sectorIndex); @loc=static @len=155 @rva=2691408
	//_Func: public void TimeTransponder(TimeTransponder &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TimeTransponder(); @loc=optimized @len=0 @rva=0
	//_Func: public TimeTransponder & operator=(TimeTransponder &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TimeTransponder)==152),"bad size");
		static_assert((offsetof(TimeTransponder,t)==0x0),"bad off");
		static_assert((offsetof(TimeTransponder,lastLap)==0x4),"bad off");
		static_assert((offsetof(TimeTransponder,bestLap)==0x8),"bad off");
		static_assert((offsetof(TimeTransponder,lapCount)==0xC),"bad off");
		static_assert((offsetof(TimeTransponder,finishLinePassed)==0x10),"bad off");
		static_assert((offsetof(TimeTransponder,wasLastLapValid)==0x11),"bad off");
		static_assert((offsetof(TimeTransponder,status)==0x18),"bad off");
		static_assert((offsetof(TimeTransponder,lastLapSplits)==0x30),"bad off");
		static_assert((offsetof(TimeTransponder,bestLapSplits)==0x48),"bad off");
		static_assert((offsetof(TimeTransponder,currentSplits)==0x60),"bad off");
		static_assert((offsetof(TimeTransponder,car)==0x78),"bad off");
		static_assert((offsetof(TimeTransponder,isFirstLapArmed)==0x80),"bad off");
		static_assert((offsetof(TimeTransponder,cuts)==0x84),"bad off");
		static_assert((offsetof(TimeTransponder,extInvalid)==0x88),"bad off");
		static_assert((offsetof(TimeTransponder,openTrackState)==0x8C),"bad off");
		static_assert((offsetof(TimeTransponder,isOpenTrack)==0x90),"bad off");
	};
};

//UDT: struct SplineLocator @len=48
	//_Func: public void ~SplineLocator(); @loc=static @len=3 @rva=96368
	//_Data: this+0x0, Member, Type: class AISpline *, currentSpline
	//_Func: public void init(Car * car); @loc=static @len=73 @rva=2798240
	//_Func: public void step(float dt); @loc=static @len=370 @rva=2799040
	//_Func: public float getNormalizedPosition(); @loc=optimized @len=0 @rva=0
	//_Func: public int getCurrentIndex(); @loc=optimized @len=0 @rva=0
	//_Func: public float locateOnSpline(AISpline * spline, vec3f & pos, int & index); @pure @loc=static @len=258 @rva=2798320
	//_Func: public float locateOnSplineWithBounds(AISpline * spline, vec3f & pos, std::vector<SplineIndexBound,std::allocator<SplineIndexBound> > & bounds, int & index); @pure @loc=static @len=273 @rva=2798592
	//_Func: public void getSides(float * sides, float nsplinepos); @loc=static @len=355 @rva=2797872
	//_Func: public float getOffset(); @loc=optimized @len=0 @rva=0
	//_Func: public float getRoadLateralPosition(); @loc=optimized @len=0 @rva=0
	//_Func: public void reset(); @loc=static @len=15 @rva=2798880
	//_Func: public void resetToClosestPoint(); @loc=static @len=143 @rva=2798896
	//_Func: public bool isOutside(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: int, currentIndex
	//_Data: this+0x18, Member, Type: class Track *, track
	//_Data: this+0x20, Member, Type: float, normalizedPos
	//_Data: this+0x24, Member, Type: float, offset
	//_Data: this+0x28, Member, Type: bool, isOutsideLimits
	//_Func: public void SplineLocator(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SplineLocator)==48),"bad size");
		static_assert((offsetof(SplineLocator,currentSpline)==0x0),"bad off");
		static_assert((offsetof(SplineLocator,car)==0x8),"bad off");
		static_assert((offsetof(SplineLocator,currentIndex)==0x10),"bad off");
		static_assert((offsetof(SplineLocator,track)==0x18),"bad off");
		static_assert((offsetof(SplineLocator,normalizedPos)==0x20),"bad off");
		static_assert((offsetof(SplineLocator,offset)==0x24),"bad off");
		static_assert((offsetof(SplineLocator,isOutsideLimits)==0x28),"bad off");
	};
};

//UDT: struct SetupManager @len=80
	//_Func: public void ~SetupManager(); @loc=static @len=71 @rva=2657648
	//_Data: this+0x0, Member, Type: class std::vector<SetupItem,std::allocator<SetupItem> >, items
	//_Func: public void init(Car * acar); @loc=static @len=723 @rva=2658960
	//_Func: public void step(float dt); @loc=static @len=108 @rva=2674832
	//_Func: public SetupItem * getSetupItem(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=215 @rva=2658736
	//_Func: public bool isItemAttached(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public CarSetupState getSetupState(); @loc=optimized @len=0 @rva=0
	//_Func: public float getMinHeightM(); @loc=optimized @len=0 @rva=0
	//_Func: public void forceCheckRules(); @loc=optimized @len=0 @rva=0
	//_Func: public void load(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=806 @rva=2673808
	//_Data: this+0x18, Member, Type: bool, checkRules
	//_Data: this+0x20, Member, Type: class Car *, car
	//_Data: this+0x28, Member, Type: class std::vector<float,std::allocator<float> >, gearSettings
	//_Data: this+0x40, Member, Type: float, minimumHeight_m
	//_Data: this+0x44, Member, Type: const float, maxWaitTime
	//_Data: this+0x48, Member, Type: float, waitTime
	//_Data: this+0x4C, Member, Type: enum CarSetupState, setupState
	//_Func: protected void initItems(bool attached); @loc=static @len=13969 @rva=2659696
	//_Func: protected void writeTemplateForLocalization(); @loc=optimized @len=0 @rva=0
	//_Func: protected bool isSetupRespectingRules(); @loc=static @len=128 @rva=2673680
	//_Func: public void SetupManager(SetupManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SetupManager(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SetupManager)==80),"bad size");
		static_assert((offsetof(SetupManager,items)==0x0),"bad off");
		static_assert((offsetof(SetupManager,checkRules)==0x18),"bad off");
		static_assert((offsetof(SetupManager,car)==0x20),"bad off");
		static_assert((offsetof(SetupManager,gearSettings)==0x28),"bad off");
		static_assert((offsetof(SetupManager,minimumHeight_m)==0x40),"bad off");
		static_assert((offsetof(SetupManager,maxWaitTime)==0x44),"bad off");
		static_assert((offsetof(SetupManager,waitTime)==0x48),"bad off");
		static_assert((offsetof(SetupManager,setupState)==0x4C),"bad off");
	};
};

//UDT: class ACPlugin @len=120 @vfcount=10
	//_Base: class IACPPluginHost @off=0 @len=8
	//_Func: public void ACPlugin(ACPlugin &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ACPlugin(Sim * aSim, HINSTANCE__ * module); @loc=static @len=683 @rva=784032
	//_Func: public void ~ACPlugin(); @intro @virtual vtpo=0 vfid=9 @loc=static @len=134 @rva=784720
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Func: public void update(ACCarState * state, float dt); @loc=static @len=19 @rva=785920
	//_Func: public void onGui(ACPluginContext * cc); @loc=static @len=16 @rva=785184
	//_Func: public HWND__ * getHwnd(); @virtual vtpo=0 vfid=0 @loc=static @len=8 @rva=785168
	//_Func: public HINSTANCE__ * getHInstance(); @virtual vtpo=0 vfid=1 @loc=static @len=8 @rva=785152
	//_Func: public void setABS(float value); @virtual vtpo=0 vfid=2 @loc=static @len=62 @rva=785200
	//_Func: public void setTC(float value); @virtual vtpo=0 vfid=3 @loc=static @len=62 @rva=785856
	//_Func: public void setStabilityControl(float value); @virtual vtpo=0 vfid=4 @loc=static @len=13 @rva=785440
	//_Func: public void setIdealLine(bool value, bool isFromPhysicsThread); @virtual vtpo=0 vfid=5 @loc=static @len=137 @rva=785296
	//_Func: public void setAutoShift(bool value); @virtual vtpo=0 vfid=6 @loc=static @len=11 @rva=785264
	//_Func: public void setBrakeBias(float value); @virtual vtpo=0 vfid=7 @loc=static @len=13 @rva=785280
	//_Func: public void setSystemMessage(wchar_t * message, wchar_t * description, bool isFromPhysicsThread); @virtual vtpo=0 vfid=8 @loc=static @len=388 @rva=785456
	//_Data: this+0x28, Member, Type: struct HINSTANCE__ *, hModule
	//_Data: this+0x30, Member, Type: function  *, getName
	//_Data: this+0x38, Member, Type: function  *, init
	//_Data: this+0x40, Member, Type: function  *, shutdown
	//_Data: this+0x48, Member, Type: function  *, p_update
	//_Data: this+0x50, Member, Type: function  *, p_onGui
	//_Data: this+0x58, Member, Type: function  *, getControls
	//_Data: this+0x60, Member, Type: class CarAvatar *, carAvatar
	//_Data: this+0x68, Member, Type: class Car *, car
	//_Data: this+0x70, Member, Type: class Sim *, sim
	//_Func: public ACPlugin & operator=(ACPlugin &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=9 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ACPlugin)==120),"bad size");
		static_assert((offsetof(ACPlugin,name)==0x8),"bad off");
		static_assert((offsetof(ACPlugin,hModule)==0x28),"bad off");
		static_assert((offsetof(ACPlugin,getName)==0x30),"bad off");
		static_assert((offsetof(ACPlugin,init)==0x38),"bad off");
		static_assert((offsetof(ACPlugin,shutdown)==0x40),"bad off");
		static_assert((offsetof(ACPlugin,p_update)==0x48),"bad off");
		static_assert((offsetof(ACPlugin,p_onGui)==0x50),"bad off");
		static_assert((offsetof(ACPlugin,getControls)==0x58),"bad off");
		static_assert((offsetof(ACPlugin,carAvatar)==0x60),"bad off");
		static_assert((offsetof(ACPlugin,car)==0x68),"bad off");
		static_assert((offsetof(ACPlugin,sim)==0x70),"bad off");
	};
};

//UDT: struct ClutchSequence @len=136
	//_Func: public void ClutchSequence(ClutchSequence &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ClutchSequence(Curve & c); @loc=static @len=154 @rva=2851616
	//_Func: public void ClutchSequence(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class Curve, clutchCurve
	//_Data: this+0x80, Member, Type: float, currentTime
	//_Data: this+0x84, Member, Type: bool, isDone
	//_Func: public void ~ClutchSequence(); @loc=static @len=5 @rva=1258144
	//_Func: public ClutchSequence & operator=(ClutchSequence & __that); @loc=static @len=132 @rva=2851840
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct ClutchSequence {
public:
	Curve clutchCurve;
	float currentTime;
	bool isDone;
	inline ClutchSequence()  { }
	inline void ctor(Curve & c) { typedef void (*_fpt)(ClutchSequence *pthis, Curve &); _fpt _f=(_fpt)_drva(2851616); _f(this, c); }
	inline void dtor() { typedef void (*_fpt)(ClutchSequence *pthis); _fpt _f=(_fpt)_drva(1258144); _f(this); }
	inline ClutchSequence & operator=(ClutchSequence & __that) { typedef ClutchSequence & (*_fpt)(ClutchSequence *pthis, ClutchSequence &); _fpt _f=(_fpt)_drva(2851840); return _f(this, __that); }
	inline void _guard_obj() {
		static_assert((sizeof(ClutchSequence)==136),"bad size");
		static_assert((offsetof(ClutchSequence,clutchCurve)==0x0),"bad off");
		static_assert((offsetof(ClutchSequence,currentTime)==0x80),"bad off");
		static_assert((offsetof(ClutchSequence,isDone)==0x84),"bad off");
	};
};

//UDT: struct acEngineData @len=296
	//_Func: public void acEngineData(acEngineData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void acEngineData(); @loc=static @len=125 @rva=2643104
	//_Data: this+0x0, Member, Type: class Curve, powerCurve
	//_Data: this+0x80, Member, Type: class Curve, coastCurve
	//_Data: this+0x100, Member, Type: float, coast2
	//_Data: this+0x104, Member, Type: float, coast1
	//_Data: this+0x108, Member, Type: float, coast0
	//_Data: this+0x10C, Member, Type: bool, useCoastCurve
	//_Data: this+0x110, Member, Type: int, minimum
	//_Data: this+0x114, Member, Type: int, limiter
	//_Data: this+0x118, Member, Type: int, limiterCycles
	//_Data: this+0x11C, Member, Type: float, overlapFreq
	//_Data: this+0x120, Member, Type: float, overlapGain
	//_Data: this+0x124, Member, Type: float, overlapIdealRPM
	//_Func: public void ~acEngineData(); @loc=static @len=45 @rva=2643504
	//_Func: public acEngineData & operator=(acEngineData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(acEngineData)==296),"bad size");
		static_assert((offsetof(acEngineData,powerCurve)==0x0),"bad off");
		static_assert((offsetof(acEngineData,coastCurve)==0x80),"bad off");
		static_assert((offsetof(acEngineData,coast2)==0x100),"bad off");
		static_assert((offsetof(acEngineData,coast1)==0x104),"bad off");
		static_assert((offsetof(acEngineData,coast0)==0x108),"bad off");
		static_assert((offsetof(acEngineData,useCoastCurve)==0x10C),"bad off");
		static_assert((offsetof(acEngineData,minimum)==0x110),"bad off");
		static_assert((offsetof(acEngineData,limiter)==0x114),"bad off");
		static_assert((offsetof(acEngineData,limiterCycles)==0x118),"bad off");
		static_assert((offsetof(acEngineData,overlapFreq)==0x11C),"bad off");
		static_assert((offsetof(acEngineData,overlapGain)==0x120),"bad off");
		static_assert((offsetof(acEngineData,overlapIdealRPM)==0x124),"bad off");
	};
};

//UDT: struct TyreModelData @len=656
	//_Func: public void TyreModelData(TyreModelData & __that); @loc=static @len=398 @rva=2609552
	//_Func: public void TyreModelData(); @loc=static @len=289 @rva=2547280
	//_Data: this+0x0, Member, Type: int, version
	//_Data: this+0x4, Member, Type: float, Dy0
	//_Data: this+0x8, Member, Type: float, Dy1
	//_Data: this+0xC, Member, Type: float, Dx0
	//_Data: this+0x10, Member, Type: float, Dx1
	//_Data: this+0x14, Member, Type: float, Fz0
	//_Data: this+0x18, Member, Type: float, flexK
	//_Data: this+0x1C, Member, Type: float, speedSensitivity
	//_Data: this+0x20, Member, Type: float, relaxationLength
	//_Data: this+0x24, Member, Type: float, rr0
	//_Data: this+0x28, Member, Type: float, rr1
	//_Data: this+0x2C, Member, Type: float, rr_sa
	//_Data: this+0x30, Member, Type: float, rr_sr
	//_Data: this+0x34, Member, Type: float, rr_slip
	//_Data: this+0x38, Member, Type: float, camberGain
	//_Data: this+0x3C, Member, Type: float, pressureSpringGain
	//_Data: this+0x40, Member, Type: float, pressureFlexGain
	//_Data: this+0x44, Member, Type: float, pressureRRGain
	//_Data: this+0x48, Member, Type: float, pressureGainD
	//_Data: this+0x4C, Member, Type: float, idealPressure
	//_Data: this+0x50, Member, Type: float, pressureRef
	//_Data: this+0x58, Member, Type: class Curve, wearCurve
	//_Data: this+0xD8, Member, Type: float, dcamber0
	//_Data: this+0xDC, Member, Type: float, dcamber1
	//_Data: this+0xE0, Member, Type: class Curve, dyLoadCurve
	//_Data: this+0x160, Member, Type: class Curve, dxLoadCurve
	//_Data: this+0x1E0, Member, Type: float, lsMultY
	//_Data: this+0x1E4, Member, Type: float, lsExpY
	//_Data: this+0x1E8, Member, Type: float, lsMultX
	//_Data: this+0x1EC, Member, Type: float, lsExpX
	//_Data: this+0x1F0, Member, Type: float, maxWearKM
	//_Data: this+0x1F4, Member, Type: float, maxWearMult
	//_Data: this+0x1F8, Member, Type: float, asy
	//_Data: this+0x1FC, Member, Type: float, cfXmult
	//_Data: this+0x200, Member, Type: float, brakeDXMod
	//_Data: this+0x208, Member, Type: class Curve, dCamberCurve
	//_Data: this+0x288, Member, Type: bool, useSmoothDCamberCurve
	//_Data: this+0x28C, Member, Type: float, combinedFactor
	//_Func: public void ~TyreModelData(); @loc=static @len=75 @rva=2550176
	//_Func: public TyreModelData & operator=(TyreModelData & __that); @loc=static @len=751 @rva=2610288
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreModelData)==656),"bad size");
		static_assert((offsetof(TyreModelData,version)==0x0),"bad off");
		static_assert((offsetof(TyreModelData,Dy0)==0x4),"bad off");
		static_assert((offsetof(TyreModelData,Dy1)==0x8),"bad off");
		static_assert((offsetof(TyreModelData,Dx0)==0xC),"bad off");
		static_assert((offsetof(TyreModelData,Dx1)==0x10),"bad off");
		static_assert((offsetof(TyreModelData,Fz0)==0x14),"bad off");
		static_assert((offsetof(TyreModelData,flexK)==0x18),"bad off");
		static_assert((offsetof(TyreModelData,speedSensitivity)==0x1C),"bad off");
		static_assert((offsetof(TyreModelData,relaxationLength)==0x20),"bad off");
		static_assert((offsetof(TyreModelData,rr0)==0x24),"bad off");
		static_assert((offsetof(TyreModelData,rr1)==0x28),"bad off");
		static_assert((offsetof(TyreModelData,rr_sa)==0x2C),"bad off");
		static_assert((offsetof(TyreModelData,rr_sr)==0x30),"bad off");
		static_assert((offsetof(TyreModelData,rr_slip)==0x34),"bad off");
		static_assert((offsetof(TyreModelData,camberGain)==0x38),"bad off");
		static_assert((offsetof(TyreModelData,pressureSpringGain)==0x3C),"bad off");
		static_assert((offsetof(TyreModelData,pressureFlexGain)==0x40),"bad off");
		static_assert((offsetof(TyreModelData,pressureRRGain)==0x44),"bad off");
		static_assert((offsetof(TyreModelData,pressureGainD)==0x48),"bad off");
		static_assert((offsetof(TyreModelData,idealPressure)==0x4C),"bad off");
		static_assert((offsetof(TyreModelData,pressureRef)==0x50),"bad off");
		static_assert((offsetof(TyreModelData,wearCurve)==0x58),"bad off");
		static_assert((offsetof(TyreModelData,dcamber0)==0xD8),"bad off");
		static_assert((offsetof(TyreModelData,dcamber1)==0xDC),"bad off");
		static_assert((offsetof(TyreModelData,dyLoadCurve)==0xE0),"bad off");
		static_assert((offsetof(TyreModelData,dxLoadCurve)==0x160),"bad off");
		static_assert((offsetof(TyreModelData,lsMultY)==0x1E0),"bad off");
		static_assert((offsetof(TyreModelData,lsExpY)==0x1E4),"bad off");
		static_assert((offsetof(TyreModelData,lsMultX)==0x1E8),"bad off");
		static_assert((offsetof(TyreModelData,lsExpX)==0x1EC),"bad off");
		static_assert((offsetof(TyreModelData,maxWearKM)==0x1F0),"bad off");
		static_assert((offsetof(TyreModelData,maxWearMult)==0x1F4),"bad off");
		static_assert((offsetof(TyreModelData,asy)==0x1F8),"bad off");
		static_assert((offsetof(TyreModelData,cfXmult)==0x1FC),"bad off");
		static_assert((offsetof(TyreModelData,brakeDXMod)==0x200),"bad off");
		static_assert((offsetof(TyreModelData,dCamberCurve)==0x208),"bad off");
		static_assert((offsetof(TyreModelData,useSmoothDCamberCurve)==0x288),"bad off");
		static_assert((offsetof(TyreModelData,combinedFactor)==0x28C),"bad off");
	};
};

//UDT: struct BrakeDisc @len=144
	//_Data: this+0x0, Member, Type: float, t
	//_Data: this+0x4, Member, Type: float, coolTransfer
	//_Data: this+0x8, Member, Type: float, torqueK
	//_Data: this+0xC, Member, Type: float, coolSpeedFactor
	//_Data: this+0x10, Member, Type: class Curve, perfCurve
	//_Func: public void BrakeDisc(BrakeDisc &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void BrakeDisc(); @loc=static @len=54 @rva=2538976
	//_Func: public void ~BrakeDisc(); @loc=static @len=9 @rva=2547808
	//_Func: public BrakeDisc & operator=(BrakeDisc &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(BrakeDisc)==144),"bad size");
		static_assert((offsetof(BrakeDisc,t)==0x0),"bad off");
		static_assert((offsetof(BrakeDisc,coolTransfer)==0x4),"bad off");
		static_assert((offsetof(BrakeDisc,torqueK)==0x8),"bad off");
		static_assert((offsetof(BrakeDisc,coolSpeedFactor)==0xC),"bad off");
		static_assert((offsetof(BrakeDisc,perfCurve)==0x10),"bad off");
	};
};

//UDT: struct DynamicTempData @len=144
	//_Data: this+0x0, Member, Type: class Curve, temperatureCurve
	//_Data: this+0x80, Member, Type: double, temperatureStartTime
	//_Data: this+0x88, Member, Type: float, baseRoad
	//_Data: this+0x8C, Member, Type: float, baseAir
	//_Func: public void DynamicTempData(DynamicTempData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void DynamicTempData(); @loc=static @len=54 @rva=1256656
	//_Func: public void ~DynamicTempData(); @loc=static @len=5 @rva=1258144
	//_Func: public DynamicTempData & operator=(DynamicTempData & __that); @loc=static @len=145 @rva=1259120
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DynamicTempData)==144),"bad size");
		static_assert((offsetof(DynamicTempData,temperatureCurve)==0x0),"bad off");
		static_assert((offsetof(DynamicTempData,temperatureStartTime)==0x80),"bad off");
		static_assert((offsetof(DynamicTempData,baseRoad)==0x88),"bad off");
		static_assert((offsetof(DynamicTempData,baseAir)==0x8C),"bad off");
	};
};

//UDT: class mat44f @len=64
	//_Data: this+0x0, Member, Type: float, M11
	//_Data: this+0x4, Member, Type: float, M12
	//_Data: this+0x8, Member, Type: float, M13
	//_Data: this+0xC, Member, Type: float, M14
	//_Data: this+0x10, Member, Type: float, M21
	//_Data: this+0x14, Member, Type: float, M22
	//_Data: this+0x18, Member, Type: float, M23
	//_Data: this+0x1C, Member, Type: float, M24
	//_Data: this+0x20, Member, Type: float, M31
	//_Data: this+0x24, Member, Type: float, M32
	//_Data: this+0x28, Member, Type: float, M33
	//_Data: this+0x2C, Member, Type: float, M34
	//_Data: this+0x30, Member, Type: float, M41
	//_Data: this+0x34, Member, Type: float, M42
	//_Data: this+0x38, Member, Type: float, M43
	//_Data: this+0x3C, Member, Type: float, M44
	//_Func: public void mat44f(float  _arg0, float  _arg1, float  _arg2, float  _arg3, float  _arg4, float  _arg5, float  _arg6, float  _arg7, float  _arg8, float  _arg9, float  _arg10, float  _arg11, float  _arg12, float  _arg13, float  _arg14, float  _arg15); @loc=optimized @len=0 @rva=0
	//_Func: public void mat44f(float *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void mat44f(mat44f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void mat44f(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isFinite(); @loc=static @len=485 @rva=2072368
	//_Func: public mat44f createIdentity(); @pure @loc=optimized @len=0 @rva=0
	//_Func: public void loadIdentity(); @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createBillboard(vec3f & objectPosition, vec3f & cameraPosition, vec3f & cameraUpVector, vec3f & cameraForwardVector); @pure @loc=static @len=646 @rva=511792
	//_Func: public mat44f createBillboard(vec3f &  _arg0, mat44f &  _arg1); @pure @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createScale(vec3f &  _arg0); @pure @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createTarget(vec3f & cameraPosition, vec3f & cameraTarget); @pure @loc=static @len=682 @rva=393040
	//_Func: public vec3f toRollPitchYaw(); @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createFromRollPitchYaw(vec3f &  _arg0); @pure @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createLookAt(vec3f & cameraPosition, vec3f & cameraTarget, vec3f & cameraUpVector); @pure @loc=static @len=811 @rva=147600
	//_Func: public mat44f createPerspectiveOffCenter(float  _arg0, float  _arg1, float  _arg2, float  _arg3, float  _arg4, float  _arg5); @pure @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createPerspective(float  _arg0, float  _arg1, float  _arg2, float  _arg3); @pure @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createOrtho(float  _arg0, float  _arg1, float  _arg2, float  _arg3, float  _arg4, float  _arg5); @pure @loc=optimized @len=0 @rva=0
	//_Func: public void transpose(mat44f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getScale(); @loc=static @len=204 @rva=2071424
	//_Func: public void normalizeRotation(); @loc=optimized @len=0 @rva=0
	//_Func: public void setScale(vec3f & s); @loc=static @len=591 @rva=2134544
	//_Func: public void setStandardScale(); @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createFromQuaternion(vec4f &  _arg0); @pure @loc=optimized @len=0 @rva=0
	//_Func: public mat44f createFromAxisAngle(vec3f & axis, float angle); @pure @loc=static @len=310 @rva=356768
	//_Func: public void setTranslation(vec3f & value); @loc=static @len=18 @rva=394240
	//_Func: public vec3f getTranslation(); @loc=static @len=21 @rva=296464
	//_Func: public void setUp(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getUp(); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getForward(); @loc=optimized @len=0 @rva=0
	//_Func: public void setForward(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getBackward(); @loc=optimized @len=0 @rva=0
	//_Func: public void setRight(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getRight(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isIdentity(); @loc=static @len=112 @rva=1817520
	//_Func: public void setFromHeadingUp(vec3f & h, vec3f & u); @loc=static @len=264 @rva=393968
	//_Func: public vec4f toQuaternion(); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f toEuler(); @loc=static @len=166 @rva=340800
	//_Func: public mat44f createFromEuler(vec3f & angles, vec3f & translation); @pure @loc=static @len=597 @rva=1150704
	//_Func: public mat44f createFromEulerSafe(vec3f euler, vec3f translation); @pure @loc=static @len=337 @rva=1399856
	//_Func: public mat44f invert(); @loc=optimized @len=0 @rva=0
	//_Func: public void print(); @loc=static @len=278 @rva=938816
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(mat44f)==64),"bad size");
		static_assert((offsetof(mat44f,M11)==0x0),"bad off");
		static_assert((offsetof(mat44f,M12)==0x4),"bad off");
		static_assert((offsetof(mat44f,M13)==0x8),"bad off");
		static_assert((offsetof(mat44f,M14)==0xC),"bad off");
		static_assert((offsetof(mat44f,M21)==0x10),"bad off");
		static_assert((offsetof(mat44f,M22)==0x14),"bad off");
		static_assert((offsetof(mat44f,M23)==0x18),"bad off");
		static_assert((offsetof(mat44f,M24)==0x1C),"bad off");
		static_assert((offsetof(mat44f,M31)==0x20),"bad off");
		static_assert((offsetof(mat44f,M32)==0x24),"bad off");
		static_assert((offsetof(mat44f,M33)==0x28),"bad off");
		static_assert((offsetof(mat44f,M34)==0x2C),"bad off");
		static_assert((offsetof(mat44f,M41)==0x30),"bad off");
		static_assert((offsetof(mat44f,M42)==0x34),"bad off");
		static_assert((offsetof(mat44f,M43)==0x38),"bad off");
		static_assert((offsetof(mat44f,M44)==0x3C),"bad off");
	};
};

//UDT: class Triangle @len=64 @vfcount=1
	//_VTable: 
	//_Func: public void Triangle(Triangle &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Triangle(vec3f & p1, vec3f & p2, vec3f & p3); @loc=static @len=564 @rva=2144864
	//_Func: public void Triangle(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~Triangle(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2145440
	//_Data: this+0x8, Member, Type: class vec3f[0x3], points
	//_Data: this+0x2C, Member, Type: class plane4f, plane
	//_Func: public bool getRayIntersection(vec3f &  _arg0, vec3f &  _arg1, vec3f &  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public float computeArea(); @loc=static @len=175 @rva=2145456
	//_Func: public Triangle & operator=(Triangle &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class Triangle {
public:
	vec3f points[0x3];
	plane4f plane;
	inline Triangle()  { }
	inline void ctor(vec3f & p1, vec3f & p2, vec3f & p3) { typedef void (*_fpt)(Triangle *pthis, vec3f &, vec3f &, vec3f &); _fpt _f=(_fpt)_drva(2144864); _f(this, p1, p2, p3); }
	virtual ~Triangle();
	inline void dtor() { typedef void (*_fpt)(Triangle *pthis); _fpt _f=(_fpt)_drva(2145440); _f(this); }
	inline float computeArea() { typedef float (*_fpt)(Triangle *pthis); _fpt _f=(_fpt)_drva(2145456); return _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(Triangle)==64),"bad size");
		static_assert((offsetof(Triangle,points)==0x8),"bad off");
		static_assert((offsetof(Triangle,plane)==0x2C),"bad off");
	};
};

//UDT: struct ksgui::ListBoxRowData @len=104
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: unsigned int, category
	//_Data: this+0x24, Member, Type: class vec4f, backColor
	//_Func: public void ListBoxRowData(ksgui_ListBoxRowData & __that); @loc=static @len=116 @rva=2395072
	//_Func: public void ListBoxRowData(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >  _arg0, std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > &  _arg1, unsigned int  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public void ListBoxRowData(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * text); @loc=static @len=365 @rva=454352
	//_Func: public void addData(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >  _arg0, vec4f  _arg1, vec4f  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public void setColumn(unsigned int  _arg0, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getColumn(unsigned int columnIndex); @loc=static @len=65 @rva=2398208
	//_Func: public unsigned int columnCount(); @loc=optimized @len=0 @rva=0
	//_Func: public void setColumnForeColor(unsigned int  _arg0, vec4f  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public vec4f getForeColor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x38, Member, Type: class std::vector<vec4f,std::allocator<vec4f> >, forecolors
	//_Data: this+0x50, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, columns
	//_Func: public void ~ListBoxRowData(); @loc=static @len=112 @rva=454992
	//_Func: public ksgui_ListBoxRowData & operator=(ksgui_ListBoxRowData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ListBoxRowData)==104),"bad size");
		static_assert((offsetof(ksgui_ListBoxRowData,name)==0x0),"bad off");
		static_assert((offsetof(ksgui_ListBoxRowData,category)==0x20),"bad off");
		static_assert((offsetof(ksgui_ListBoxRowData,backColor)==0x24),"bad off");
		static_assert((offsetof(ksgui_ListBoxRowData,forecolors)==0x38),"bad off");
		static_assert((offsetof(ksgui_ListBoxRowData,columns)==0x50),"bad off");
	};
};

//UDT: class JoypadManager @len=8
	//_Func: public void JoypadManager(JoypadManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void JoypadManager(); @loc=static @len=80 @rva=2375584
	//_Func: public Joypad * getJoypad(); @loc=static @len=4 @rva=100192
	//_Data: this+0x0, Member, Type: class std::unique_ptr<Joypad,std::default_delete<Joypad> >, joypad
	//_Func: public void ~JoypadManager(); @loc=static @len=13 @rva=2367136
	//_Func: public JoypadManager & operator=(JoypadManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

class JoypadManager {
public:
	std::unique_ptr<Joypad,std::default_delete<Joypad> > joypad;
	inline JoypadManager()  { }
	inline void ctor() { typedef void (*_fpt)(JoypadManager *pthis); _fpt _f=(_fpt)_drva(2375584); _f(this); }
	inline Joypad * getJoypad() { typedef Joypad * (*_fpt)(JoypadManager *pthis); _fpt _f=(_fpt)_drva(100192); return _f(this); }
	inline void dtor() { typedef void (*_fpt)(JoypadManager *pthis); _fpt _f=(_fpt)_drva(2367136); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(JoypadManager)==8),"bad size");
		static_assert((offsetof(JoypadManager,joypad)==0x0),"bad off");
	};
};

//UDT: struct DebugLine @len=44
	//_Func: public void DebugLine(vec3f &  _arg0, vec3f &  _arg1, vec4f &  _arg2, float  _arg3); @loc=optimized @len=0 @rva=0
	//_Func: public void DebugLine(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, p0
	//_Data: this+0xC, Member, Type: class vec3f, p1
	//_Data: this+0x18, Member, Type: class vec4f, color
	//_Data: this+0x28, Member, Type: float, seconds
//UDT;

struct DebugLine {
public:
	vec3f p0;
	vec3f p1;
	vec4f color;
	float seconds;
	inline DebugLine()  { }
	inline void _guard_obj() {
		static_assert((sizeof(DebugLine)==44),"bad size");
		static_assert((offsetof(DebugLine,p0)==0x0),"bad off");
		static_assert((offsetof(DebugLine,p1)==0xC),"bad off");
		static_assert((offsetof(DebugLine,color)==0x18),"bad off");
		static_assert((offsetof(DebugLine,seconds)==0x28),"bad off");
	};
};

//UDT: struct WingData @len=592
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: float, chord
	//_Data: this+0x24, Member, Type: float, span
	//_Data: this+0x28, Member, Type: class vec3f, position
	//_Data: this+0x38, Member, Type: class Curve, lutAOA_CL
	//_Data: this+0xB8, Member, Type: class Curve, lutAOA_CD
	//_Data: this+0x138, Member, Type: class Curve, lutGH_CL
	//_Data: this+0x1B8, Member, Type: class Curve, lutGH_CD
	//_Data: this+0x238, Member, Type: float, clGain
	//_Data: this+0x23C, Member, Type: float, cdGain
	//_Data: this+0x240, Member, Type: bool, hasController
	//_Data: this+0x244, Member, Type: float, yawGain
	//_Data: this+0x248, Member, Type: float, area
	//_Data: this+0x24C, Member, Type: bool, isVertical
	//_Func: public void WingData(WingData & __that); @loc=static @len=249 @rva=845872
	//_Func: public void WingData(); @loc=static @len=145 @rva=2826944
	//_Func: public void ~WingData(); @loc=static @len=109 @rva=847680
	//_Func: public WingData & operator=(WingData &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(WingData)==592),"bad size");
		static_assert((offsetof(WingData,name)==0x0),"bad off");
		static_assert((offsetof(WingData,chord)==0x20),"bad off");
		static_assert((offsetof(WingData,span)==0x24),"bad off");
		static_assert((offsetof(WingData,position)==0x28),"bad off");
		static_assert((offsetof(WingData,lutAOA_CL)==0x38),"bad off");
		static_assert((offsetof(WingData,lutAOA_CD)==0xB8),"bad off");
		static_assert((offsetof(WingData,lutGH_CL)==0x138),"bad off");
		static_assert((offsetof(WingData,lutGH_CD)==0x1B8),"bad off");
		static_assert((offsetof(WingData,clGain)==0x238),"bad off");
		static_assert((offsetof(WingData,cdGain)==0x23C),"bad off");
		static_assert((offsetof(WingData,hasController)==0x240),"bad off");
		static_assert((offsetof(WingData,yawGain)==0x244),"bad off");
		static_assert((offsetof(WingData,area)==0x248),"bad off");
		static_assert((offsetof(WingData,isVertical)==0x24C),"bad off");
	};
};

//UDT: class BrushSlipProvider @len=56 @vfcount=1
	//_VTable: 
	//_Func: public void BrushSlipProvider(BrushSlipProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void BrushSlipProvider(float maxAngle, float xu, float flex); @loc=static @len=176 @rva=2830208
	//_Func: public void BrushSlipProvider(); @loc=static @len=52 @rva=2830384
	//_Func: public void ~BrushSlipProvider(); @loc=static @len=20 @rva=2830448
	//_Data: this+0x8, Member, Type: class BrushTyreModel, brushModel
	//_Data: this+0x24, Member, Type: float, asy
	//_Data: this+0x28, Member, Type: int, version
	//_Func: public TyreSlipOutput getSlipForce(TyreSlipInput & input, bool useasy); @intro @virtual vtpo=0 vfid=0 @loc=static @len=157 @rva=2830736
	//_Func: public void setFromMaxSlipAngle(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaximum(); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaxSlip(); @loc=optimized @len=0 @rva=0
	//_Func: public void calcMaximum(float load, float * maximum, float * max_slip); @loc=static @len=253 @rva=2830480
	//_Func: public void recomputeMaximum(); @loc=static @len=21 @rva=2830896
	//_Func: public void init(float  _arg0, float  _arg1, float  _arg2); @loc=optimized @len=0 @rva=0
	//_Data: this+0x2C, Member, Type: float, maximum
	//_Data: this+0x30, Member, Type: float, maxSlip
	//_Func: public BrushSlipProvider & operator=(BrushSlipProvider & __that); @loc=static @len=52 @rva=2610224
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(BrushSlipProvider)==56),"bad size");
		static_assert((offsetof(BrushSlipProvider,brushModel)==0x8),"bad off");
		static_assert((offsetof(BrushSlipProvider,asy)==0x24),"bad off");
		static_assert((offsetof(BrushSlipProvider,version)==0x28),"bad off");
		static_assert((offsetof(BrushSlipProvider,maximum)==0x2C),"bad off");
		static_assert((offsetof(BrushSlipProvider,maxSlip)==0x30),"bad off");
	};
};

//UDT: struct DynamicControllerStage @len=160
	//_Data: this+0x0, Member, Type: enum DynamicControllerInput, inputVar
	//_Data: this+0x4, Member, Type: enum DynamicControllerCombinatorMode, combinatorMode
	//_Data: this+0x8, Member, Type: class Curve, lut
	//_Data: this+0x88, Member, Type: float, filter
	//_Data: this+0x8C, Member, Type: float, upLimit
	//_Data: this+0x90, Member, Type: float, downLimit
	//_Data: this+0x94, Member, Type: float, currentValue
	//_Data: this+0x98, Member, Type: float, constValue
	//_Func: public void DynamicControllerStage(DynamicControllerStage & __that); @loc=static @len=113 @rva=2546288
	//_Func: public void DynamicControllerStage(); @loc=static @len=62 @rva=2819968
	//_Func: public void ~DynamicControllerStage(); @loc=static @len=9 @rva=2549824
	//_Func: public DynamicControllerStage & operator=(DynamicControllerStage &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DynamicControllerStage)==160),"bad size");
		static_assert((offsetof(DynamicControllerStage,inputVar)==0x0),"bad off");
		static_assert((offsetof(DynamicControllerStage,combinatorMode)==0x4),"bad off");
		static_assert((offsetof(DynamicControllerStage,lut)==0x8),"bad off");
		static_assert((offsetof(DynamicControllerStage,filter)==0x88),"bad off");
		static_assert((offsetof(DynamicControllerStage,upLimit)==0x8C),"bad off");
		static_assert((offsetof(DynamicControllerStage,downLimit)==0x90),"bad off");
		static_assert((offsetof(DynamicControllerStage,currentValue)==0x94),"bad off");
		static_assert((offsetof(DynamicControllerStage,constValue)==0x98),"bad off");
	};
};

//UDT: struct TractionControl @len=168
	//_Func: public void ~TractionControl(); @loc=static @len=9 @rva=2685136
	//_Data: this+0x0, Member, Type: bool, isPresent
	//_Data: this+0x1, Member, Type: bool, isActive
	//_Data: this+0x4, Member, Type: float, slipRatioLimit
	//_Data: this+0x8, Member, Type: bool, isInAction
	//_Data: this+0xC, Member, Type: float, frequency
	//_Func: public void init(Car * acar); @loc=static @len=1979 @rva=2685504
	//_Func: public void step(float dt); @loc=static @len=503 @rva=2687488
	//_Func: public void cycleMode(int value); @loc=static @len=202 @rva=2685152
	//_Func: public std::pair<unsigned int,unsigned int> getCurrentMode(); @loc=static @len=129 @rva=2685360
	//_Data: this+0x10, Member, Type: class Car *, car
	//_Data: this+0x18, Member, Type: float, minSpeedMS
	//_Data: this+0x1C, Member, Type: float, timeAccumulator
	//_Data: this+0x20, Member, Type: unsigned int, currentMode
	//_Data: this+0x24, Member, Type: bool, lastValue
	//_Data: this+0x28, Member, Type: class Curve, valueCurve
	//_Func: public void TractionControl(TractionControl &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TractionControl(); @loc=optimized @len=0 @rva=0
	//_Func: public TractionControl & operator=(TractionControl &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TractionControl)==168),"bad size");
		static_assert((offsetof(TractionControl,isPresent)==0x0),"bad off");
		static_assert((offsetof(TractionControl,isActive)==0x1),"bad off");
		static_assert((offsetof(TractionControl,slipRatioLimit)==0x4),"bad off");
		static_assert((offsetof(TractionControl,isInAction)==0x8),"bad off");
		static_assert((offsetof(TractionControl,frequency)==0xC),"bad off");
		static_assert((offsetof(TractionControl,car)==0x10),"bad off");
		static_assert((offsetof(TractionControl,minSpeedMS)==0x18),"bad off");
		static_assert((offsetof(TractionControl,timeAccumulator)==0x1C),"bad off");
		static_assert((offsetof(TractionControl,currentMode)==0x20),"bad off");
		static_assert((offsetof(TractionControl,lastValue)==0x24),"bad off");
		static_assert((offsetof(TractionControl,valueCurve)==0x28),"bad off");
	};
};

//UDT: struct AutoBlip @len=168
	//_Func: public void ~AutoBlip(); @loc=static @len=9 @rva=2547808
	//_Data: this+0x0, Member, Type: bool, isActive
	//_Func: public void init(Car * acar); @loc=static @len=148 @rva=2857232
	//_Func: public void step(float dt); @loc=static @len=304 @rva=2858736
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: class Curve, blipProfile
	//_Data: this+0x90, Member, Type: double, blipStartTime
	//_Data: this+0x98, Member, Type: bool, isElectronic
	//_Data: this+0xA0, Member, Type: double, blipPerformTime
	//_Func: private void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * carModel); @loc=static @len=1332 @rva=2857392
	//_Func: public void AutoBlip(AutoBlip &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void AutoBlip(); @loc=optimized @len=0 @rva=0
	//_Func: public AutoBlip & operator=(AutoBlip &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(AutoBlip)==168),"bad size");
		static_assert((offsetof(AutoBlip,isActive)==0x0),"bad off");
		static_assert((offsetof(AutoBlip,car)==0x8),"bad off");
		static_assert((offsetof(AutoBlip,blipProfile)==0x10),"bad off");
		static_assert((offsetof(AutoBlip,blipStartTime)==0x90),"bad off");
		static_assert((offsetof(AutoBlip,isElectronic)==0x98),"bad off");
		static_assert((offsetof(AutoBlip,blipPerformTime)==0xA0),"bad off");
	};
};

//UDT: struct ABS @len=168
	//_Func: public void ~ABS(); @loc=static @len=9 @rva=2681552
	//_Data: this+0x0, Member, Type: bool, isPresent
	//_Data: this+0x1, Member, Type: bool, isActive
	//_Data: this+0x4, Member, Type: float, slipRatioLimit
	//_Data: this+0x8, Member, Type: float, frequency
	//_Data: this+0xC, Member, Type: int, channels
	//_Func: public void init(Car * acar); @loc=static @len=2432 @rva=2681936
	//_Func: public void step(float td); @loc=static @len=696 @rva=2684432
	//_Func: public void cycleMode(int value); @loc=static @len=217 @rva=2681568
	//_Func: public std::pair<unsigned int,unsigned int> getCurrentMode(); @loc=static @len=132 @rva=2681792
	//_Func: public bool isInAction(); @loc=static @len=54 @rva=2684368
	//_Data: this+0x10, Member, Type: class Car *, car
	//_Data: this+0x18, Member, Type: float, timeAccumulator
	//_Data: this+0x20, Member, Type: class Curve, valueCurve
	//_Data: this+0xA0, Member, Type: unsigned int, currentMode
	//_Data: this+0xA4, Member, Type: float, currentValue
	//_Func: public void ABS(ABS &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ABS(); @loc=optimized @len=0 @rva=0
	//_Func: public ABS & operator=(ABS &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ABS)==168),"bad size");
		static_assert((offsetof(ABS,isPresent)==0x0),"bad off");
		static_assert((offsetof(ABS,isActive)==0x1),"bad off");
		static_assert((offsetof(ABS,slipRatioLimit)==0x4),"bad off");
		static_assert((offsetof(ABS,frequency)==0x8),"bad off");
		static_assert((offsetof(ABS,channels)==0xC),"bad off");
		static_assert((offsetof(ABS,car)==0x10),"bad off");
		static_assert((offsetof(ABS,timeAccumulator)==0x18),"bad off");
		static_assert((offsetof(ABS,valueCurve)==0x20),"bad off");
		static_assert((offsetof(ABS,currentMode)==0xA0),"bad off");
		static_assert((offsetof(ABS,currentValue)==0xA4),"bad off");
	};
};

//UDT: class UDPSocket @len=56
	//_Func: public void UDPSocket(UDPSocket &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void UDPSocket(unsigned short  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void UDPSocket(); @loc=static @len=64 @rva=2477984
	//_Func: public void ~UDPSocket(); @loc=static @len=50 @rva=2478064
	//_Func: public void bindToPort(unsigned short  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setBlockingMode(bool imode); @loc=static @len=13 @rva=2479568
	//_Func: public bool getBlockingMode(); @loc=optimized @len=0 @rva=0
	//_Func: public void addListener(std::function<void __cdecl(UDPMessage const &)> * listener); @loc=static @len=125 @rva=2478736
	//_Func: public int receive(std::function<void __cdecl(UDPMessage const &)>  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public int receive(int maxPackets); @loc=static @len=382 @rva=2479104
	//_Func: public void send(void * data, int length, sockaddr_in target); @loc=static @len=76 @rva=2479488
	//_Func: public unsigned short getPing(); @loc=optimized @len=0 @rva=0
	//_Func: public double getLastPingTime(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: unsigned __int64, soc
	//_Data: this+0x8, Member, Type: bool, isBlocking
	//_Data: this+0x10, Member, Type: class std::vector<std::function<void __cdecl(UDPMessage const &)>,std::allocator<std::function<void __cdecl(UDPMessage const &)> > >, listeners
	//_Data: this+0x28, Member, Type: unsigned short, ping
	//_Data: this+0x2A, Member, Type: bool, shutdownFlag
	//_Data: this+0x30, Member, Type: double, lastPingTime
	//_Func: private void broadcastMessage(UDPMessage &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public UDPSocket & operator=(UDPSocket &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(UDPSocket)==56),"bad size");
		static_assert((offsetof(UDPSocket,soc)==0x0),"bad off");
		static_assert((offsetof(UDPSocket,isBlocking)==0x8),"bad off");
		static_assert((offsetof(UDPSocket,listeners)==0x10),"bad off");
		static_assert((offsetof(UDPSocket,ping)==0x28),"bad off");
		static_assert((offsetof(UDPSocket,shutdownFlag)==0x2A),"bad off");
		static_assert((offsetof(UDPSocket,lastPingTime)==0x30),"bad off");
	};
};

//UDT: struct DebugString @len=80
	//_Func: public void DebugString(DebugString & __that); @loc=static @len=108 @rva=508160
	//_Func: public void DebugString(vec3f &  _arg0, vec4f &  _arg1, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg2, int  _arg3, float  _arg4, float  _arg5); @loc=optimized @len=0 @rva=0
	//_Func: public void DebugString(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x0, Member, Type: class vec3f, p
	//_Data: this+0x10, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, text
	//_Data: this+0x30, Member, Type: class vec4f, color
	//_Data: this+0x40, Member, Type: float, seconds
	//_Data: this+0x44, Member, Type: float, scale
	//_Data: this+0x48, Member, Type: int, stringId
	//_Func: public void ~DebugString(); @loc=static @len=49 @rva=2353488
	//_Func: public DebugString & operator=(DebugString &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DebugString)==80),"bad size");
		static_assert((offsetof(DebugString,p)==0x0),"bad off");
		static_assert((offsetof(DebugString,text)==0x10),"bad off");
		static_assert((offsetof(DebugString,color)==0x30),"bad off");
		static_assert((offsetof(DebugString,seconds)==0x40),"bad off");
		static_assert((offsetof(DebugString,scale)==0x44),"bad off");
		static_assert((offsetof(DebugString,stringId)==0x48),"bad off");
	};
};

//UDT: class ResourceStore @len=32 @vfcount=1
	//_VTable: 
	//_Func: public void ResourceStore(ResourceStore &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ResourceStore(GraphicsManager * rm); @loc=static @len=77 @rva=2097040
	//_Func: public void ~ResourceStore(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=192 @rva=2097120
	//_Func: public Texture getTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename, bool onlyExisting); @loc=static @len=378 @rva=2097360
	//_Func: public Texture getTextureFromBuffer(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, unsigned char * buffer, int size); @loc=static @len=364 @rva=2097744
	//_Func: public bool releaseTexture(Texture &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool removeTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void releaseAll(); @loc=optimized @len=0 @rva=0
	//_Func: public bool hasTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=110 @rva=2098112
	//_Data: this+0x8, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,Texture,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,Texture> > >, store
	//_Data: this+0x18, Member, Type: class GraphicsManager *, graphics
	//_Func: public ResourceStore & operator=(ResourceStore &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ResourceStore)==32),"bad size");
		static_assert((offsetof(ResourceStore,store)==0x8),"bad off");
		static_assert((offsetof(ResourceStore,graphics)==0x18),"bad off");
	};
};

//UDT: class SCTM @len=488 @vfcount=2
	//_Base: class ITyreModel @off=0 @len=8
	//_Func: public void SCTM(SCTM &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SCTM(); @loc=static @len=159 @rva=4503744
	//_Func: public void ~SCTM(); @virtual vtpo=0 vfid=0 @loc=static @len=84 @rva=4503920
	//_Data: this+0x8, Member, Type: float, lsMultY
	//_Data: this+0xC, Member, Type: float, lsExpY
	//_Data: this+0x10, Member, Type: float, lsMultX
	//_Data: this+0x14, Member, Type: float, lsExpX
	//_Data: this+0x18, Member, Type: float, Fz0
	//_Data: this+0x1C, Member, Type: float, maxSlip0
	//_Data: this+0x20, Member, Type: float, maxSlip1
	//_Data: this+0x24, Member, Type: float, asy
	//_Data: this+0x28, Member, Type: float, falloffSpeed
	//_Data: this+0x2C, Member, Type: float, speedSensitivity
	//_Data: this+0x30, Member, Type: float, camberGain
	//_Data: this+0x34, Member, Type: float, dcamber0
	//_Data: this+0x38, Member, Type: float, dcamber1
	//_Data: this+0x3C, Member, Type: float, cfXmult
	//_Data: this+0x40, Member, Type: class Curve, dyLoadCurve
	//_Data: this+0xC0, Member, Type: class Curve, dxLoadCurve
	//_Data: this+0x140, Member, Type: float, pressureCfGain
	//_Data: this+0x144, Member, Type: float, brakeDXMod
	//_Data: this+0x148, Member, Type: class Curve, dCamberCurve
	//_Data: this+0x1C8, Member, Type: bool, useSmoothDCamberCurve
	//_Data: this+0x1CC, Member, Type: float, dCamberBlend
	//_Data: this+0x1D0, Member, Type: float, combinedFactor
	//_Data: this+0x1D4, Member, Type: float, dy0
	//_Data: this+0x1D8, Member, Type: float, dx0
	//_Data: this+0x1DC, Member, Type: float, pacE
	//_Data: this+0x1E0, Member, Type: float, pacCf
	//_Data: this+0x1E4, Member, Type: float, pacFlex
	//_Func: public TyreModelOutput solve(TyreModelInput & in); @virtual vtpo=0 vfid=1 @loc=static @len=1444 @rva=4504608
	//_Func: public float getStaticDY(float load); @loc=static @len=109 @rva=4504496
	//_Func: public float getStaticDX(float load); @loc=static @len=115 @rva=4504368
	//_Func: private float getPureFY(float D, float cf, float load, float slip); @loc=static @len=188 @rva=4504176
	//_Func: private float getPureFX(float  _arg0, float  _arg1, float  _arg2, float  _arg3); @loc=optimized @len=0 @rva=0
	//_Func: public SCTM & operator=(SCTM &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SCTM)==488),"bad size");
		static_assert((offsetof(SCTM,lsMultY)==0x8),"bad off");
		static_assert((offsetof(SCTM,lsExpY)==0xC),"bad off");
		static_assert((offsetof(SCTM,lsMultX)==0x10),"bad off");
		static_assert((offsetof(SCTM,lsExpX)==0x14),"bad off");
		static_assert((offsetof(SCTM,Fz0)==0x18),"bad off");
		static_assert((offsetof(SCTM,maxSlip0)==0x1C),"bad off");
		static_assert((offsetof(SCTM,maxSlip1)==0x20),"bad off");
		static_assert((offsetof(SCTM,asy)==0x24),"bad off");
		static_assert((offsetof(SCTM,falloffSpeed)==0x28),"bad off");
		static_assert((offsetof(SCTM,speedSensitivity)==0x2C),"bad off");
		static_assert((offsetof(SCTM,camberGain)==0x30),"bad off");
		static_assert((offsetof(SCTM,dcamber0)==0x34),"bad off");
		static_assert((offsetof(SCTM,dcamber1)==0x38),"bad off");
		static_assert((offsetof(SCTM,cfXmult)==0x3C),"bad off");
		static_assert((offsetof(SCTM,dyLoadCurve)==0x40),"bad off");
		static_assert((offsetof(SCTM,dxLoadCurve)==0xC0),"bad off");
		static_assert((offsetof(SCTM,pressureCfGain)==0x140),"bad off");
		static_assert((offsetof(SCTM,brakeDXMod)==0x144),"bad off");
		static_assert((offsetof(SCTM,dCamberCurve)==0x148),"bad off");
		static_assert((offsetof(SCTM,useSmoothDCamberCurve)==0x1C8),"bad off");
		static_assert((offsetof(SCTM,dCamberBlend)==0x1CC),"bad off");
		static_assert((offsetof(SCTM,combinedFactor)==0x1D0),"bad off");
		static_assert((offsetof(SCTM,dy0)==0x1D4),"bad off");
		static_assert((offsetof(SCTM,dx0)==0x1D8),"bad off");
		static_assert((offsetof(SCTM,pacE)==0x1DC),"bad off");
		static_assert((offsetof(SCTM,pacCf)==0x1E0),"bad off");
		static_assert((offsetof(SCTM,pacFlex)==0x1E4),"bad off");
	};
};

//UDT: struct TyreThermalModel @len=224
	//_Data: this+0x0, Member, Type: int, elements
	//_Data: this+0x4, Member, Type: int, stripes
	//_Data: this+0x8, Member, Type: class std::vector<TyreThermalPatch,std::allocator<TyreThermalPatch> >, patches
	//_Data: this+0x20, Member, Type: double, phase
	//_Data: this+0x28, Member, Type: struct TyrePatchData, patchData
	//_Data: this+0x3C, Member, Type: float, coreTemp
	//_Data: this+0x40, Member, Type: class Curve, performanceCurve
	//_Data: this+0xC0, Member, Type: bool, isActive
	//_Data: this+0xC4, Member, Type: float, thermalMultD
	//_Data: this+0xC8, Member, Type: float, practicalTemp
	//_Data: this+0xCC, Member, Type: float, camberSpreadK
	//_Func: public void init(int a_elements, int a_stripes, Car * car); @loc=static @len=73 @rva=2810224
	//_Func: public void step(float dt, float angularSpeed, float camberRAD); @loc=static @len=1006 @rva=2810688
	//_Func: public void addThermalInput(float xpos, float pressureRel, float temp); @loc=static @len=621 @rva=2805904
	//_Func: public int getCurrentElementIndex(); @loc=optimized @len=0 @rva=0
	//_Func: public TyreThermalPatch & getPatchAt(int x, int y); @loc=static @len=393 @rva=2809776
	//_Func: public float getCurrentCPTemp(float camber); @loc=static @len=464 @rva=2808800
	//_Func: public float getCorrectedD(float d, float camberRAD); @loc=static @len=21 @rva=2808768
	//_Func: public float getAvgSurfaceTemp(); @loc=static @len=39 @rva=2808720
	//_Func: public void reset(); @loc=static @len=89 @rva=2810304
	//_Func: public void getIMO(float * out); @loc=static @len=512 @rva=2809264
	//_Func: public void setTemperature(float optimumTemp); @loc=static @len=33 @rva=2810640
	//_Func: public float getPracticalTemp(float camberRAD); @loc=static @len=38 @rva=2810176
	//_Func: public void addThermalCoreInput(float temp); @loc=static @len=17 @rva=2805872
	//_Data: this+0xD0, Member, Type: class Car *, car
	//_Data: this+0xD8, Member, Type: float, coreTInput
	//_Func: private void buildTyre(); @loc=static @len=2190 @rva=2806528
	//_Func: public void TyreThermalModel(TyreThermalModel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TyreThermalModel(); @loc=static @len=144 @rva=2547584
	//_Func: public void ~TyreThermalModel(); @loc=static @len=46 @rva=2550256
	//_Func: public TyreThermalModel & operator=(TyreThermalModel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreThermalModel)==224),"bad size");
		static_assert((offsetof(TyreThermalModel,elements)==0x0),"bad off");
		static_assert((offsetof(TyreThermalModel,stripes)==0x4),"bad off");
		static_assert((offsetof(TyreThermalModel,patches)==0x8),"bad off");
		static_assert((offsetof(TyreThermalModel,phase)==0x20),"bad off");
		static_assert((offsetof(TyreThermalModel,patchData)==0x28),"bad off");
		static_assert((offsetof(TyreThermalModel,coreTemp)==0x3C),"bad off");
		static_assert((offsetof(TyreThermalModel,performanceCurve)==0x40),"bad off");
		static_assert((offsetof(TyreThermalModel,isActive)==0xC0),"bad off");
		static_assert((offsetof(TyreThermalModel,thermalMultD)==0xC4),"bad off");
		static_assert((offsetof(TyreThermalModel,practicalTemp)==0xC8),"bad off");
		static_assert((offsetof(TyreThermalModel,camberSpreadK)==0xCC),"bad off");
		static_assert((offsetof(TyreThermalModel,car)==0xD0),"bad off");
		static_assert((offsetof(TyreThermalModel,coreTInput)==0xD8),"bad off");
	};
};

//UDT: class Font @len=32
	//_Func: public void Font(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & fontFamily, eFontType fontType, float size, bool italic, bool bold); @loc=static @len=323 @rva=2099808
	//_Func: public void Font(eFontType fontType, float size, bool italic, bool bold); @loc=static @len=240 @rva=2100144
	//_Func: public void ~Font(); @loc=static @len=3 @rva=96368
	//_Func: public void cleanup(); @pure @loc=static @len=157 @rva=2101040
	//_Data: this+0x0, Member, Type: float, scale
	//_Data: this+0x4, Member, Type: bool, shadowed
	//_Data: this+0x8, Member, Type: float, shadowPixelDistance
	//_Func: public void blitString(float x, float y, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & s, float ascale, eFontAlign align); @loc=static @len=261 @rva=2100768
	//_Func: public void setColor(vec4f & cc); @loc=static @len=113 @rva=2102576
	//_Func: public void setColor(float r, float g, float b, float a); @loc=static @len=101 @rva=2102704
	//_Data: this+0x10, Member, Type: void *, kid
	//_Data: this+0x18, Member, Type: unsigned int, color
	//_Data: this+0x1C, Member, Type: float, currentAlpha
	//_Data: static, [0155A260][0003:00047260], Static Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,void *,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,void *> > >, fontWrappers
	//_Func: private void * getFontWrapper(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, bool italic, bool bold); @pure @loc=static @len=1119 @rva=2101456
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Font)==32),"bad size");
		static_assert((offsetof(Font,scale)==0x0),"bad off");
		static_assert((offsetof(Font,shadowed)==0x4),"bad off");
		static_assert((offsetof(Font,shadowPixelDistance)==0x8),"bad off");
		static_assert((offsetof(Font,kid)==0x10),"bad off");
		static_assert((offsetof(Font,color)==0x18),"bad off");
		static_assert((offsetof(Font,currentAlpha)==0x1C),"bad off");
	};
};

//UDT: struct Telemetry @len=256
	//_Func: public void ~Telemetry(); @loc=static @len=106 @rva=2866608
	//_Data: this+0x0, Member, Type: class std::vector<TelemetryChannel,std::allocator<TelemetryChannel> >, channels
	//_Data: this+0x18, Member, Type: bool, isEnabled
	//_Data: this+0x20, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, driverName
	//_Func: public void init(Car * car); @loc=static @len=11432 @rva=2867216
	//_Func: public void step(float td); @loc=static @len=987 @rva=2880944
	//_Func: public void save(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename); @loc=static @len=1337 @rva=2879600
	//_Data: this+0x40, Member, Type: bool, debugPhysics
	//_Data: this+0x41, Member, Type: bool, debugAI
	//_Data: this+0x48, Member, Type: class Car *, car
	//_Data: this+0x50, Member, Type: float, totLift
	//_Data: this+0x54, Member, Type: float, totDrag
	//_Data: this+0x58, Member, Type: float, lapBeacon
	//_Data: this+0x5C, Member, Type: float, carSpeedMS
	//_Data: this+0x60, Member, Type: float, roty
	//_Data: this+0x64, Member, Type: float, steerAngle
	//_Data: this+0x68, Member, Type: float, gas
	//_Data: this+0x6C, Member, Type: float, brake
	//_Data: this+0x70, Member, Type: float, gear
	//_Data: this+0x74, Member, Type: float, clutch
	//_Data: this+0x78, Member, Type: float, aiTargetSpeed
	//_Data: this+0x7C, Member, Type: float, aiBGTargetSpeed
	//_Data: this+0x80, Member, Type: float, aiOutsideOffset
	//_Data: this+0x84, Member, Type: float, rpms
	//_Data: this+0x88, Member, Type: float[0x4], susTravel
	//_Data: this+0x98, Member, Type: float[0x4], wheelSpeed
	//_Data: this+0xA8, Member, Type: struct TimeTransponder *, timeTransponder
	//_Data: this+0xB0, Member, Type: bool, exportEntireSession
	//_Data: this+0xB4, Member, Type: float, awdFrontShare
	//_Data: this+0xB8, Member, Type: float, awdCenterLock
	//_Data: this+0xBC, Member, Type: float, oversteerFactor
	//_Data: this+0xC0, Member, Type: float, rearSpeedRatio
	//_Data: this+0xC4, Member, Type: float, ebbInstant
	//_Data: this+0xC8, Member, Type: float, clutchOpenState
	//_Data: this+0xCC, Member, Type: float, engineVel
	//_Data: this+0xD0, Member, Type: float, driveVel
	//_Data: this+0xD4, Member, Type: float, rootVel
	//_Data: this+0xD8, Member, Type: float, clutchSlip
	//_Data: this+0xDC, Member, Type: float[0x4], avgSurfaceTemps
	//_Data: this+0xEC, Member, Type: float[0x4], practicalTemps
	//_Data: this+0xFC, Member, Type: float, splinePosition
	//_Func: public void Telemetry(Telemetry &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Telemetry(); @loc=static @len=160 @rva=2546480
	//_Func: public Telemetry & operator=(Telemetry &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Telemetry)==256),"bad size");
		static_assert((offsetof(Telemetry,channels)==0x0),"bad off");
		static_assert((offsetof(Telemetry,isEnabled)==0x18),"bad off");
		static_assert((offsetof(Telemetry,driverName)==0x20),"bad off");
		static_assert((offsetof(Telemetry,debugPhysics)==0x40),"bad off");
		static_assert((offsetof(Telemetry,debugAI)==0x41),"bad off");
		static_assert((offsetof(Telemetry,car)==0x48),"bad off");
		static_assert((offsetof(Telemetry,totLift)==0x50),"bad off");
		static_assert((offsetof(Telemetry,totDrag)==0x54),"bad off");
		static_assert((offsetof(Telemetry,lapBeacon)==0x58),"bad off");
		static_assert((offsetof(Telemetry,carSpeedMS)==0x5C),"bad off");
		static_assert((offsetof(Telemetry,roty)==0x60),"bad off");
		static_assert((offsetof(Telemetry,steerAngle)==0x64),"bad off");
		static_assert((offsetof(Telemetry,gas)==0x68),"bad off");
		static_assert((offsetof(Telemetry,brake)==0x6C),"bad off");
		static_assert((offsetof(Telemetry,gear)==0x70),"bad off");
		static_assert((offsetof(Telemetry,clutch)==0x74),"bad off");
		static_assert((offsetof(Telemetry,aiTargetSpeed)==0x78),"bad off");
		static_assert((offsetof(Telemetry,aiBGTargetSpeed)==0x7C),"bad off");
		static_assert((offsetof(Telemetry,aiOutsideOffset)==0x80),"bad off");
		static_assert((offsetof(Telemetry,rpms)==0x84),"bad off");
		static_assert((offsetof(Telemetry,susTravel)==0x88),"bad off");
		static_assert((offsetof(Telemetry,wheelSpeed)==0x98),"bad off");
		static_assert((offsetof(Telemetry,timeTransponder)==0xA8),"bad off");
		static_assert((offsetof(Telemetry,exportEntireSession)==0xB0),"bad off");
		static_assert((offsetof(Telemetry,awdFrontShare)==0xB4),"bad off");
		static_assert((offsetof(Telemetry,awdCenterLock)==0xB8),"bad off");
		static_assert((offsetof(Telemetry,oversteerFactor)==0xBC),"bad off");
		static_assert((offsetof(Telemetry,rearSpeedRatio)==0xC0),"bad off");
		static_assert((offsetof(Telemetry,ebbInstant)==0xC4),"bad off");
		static_assert((offsetof(Telemetry,clutchOpenState)==0xC8),"bad off");
		static_assert((offsetof(Telemetry,engineVel)==0xCC),"bad off");
		static_assert((offsetof(Telemetry,driveVel)==0xD0),"bad off");
		static_assert((offsetof(Telemetry,rootVel)==0xD4),"bad off");
		static_assert((offsetof(Telemetry,clutchSlip)==0xD8),"bad off");
		static_assert((offsetof(Telemetry,avgSurfaceTemps)==0xDC),"bad off");
		static_assert((offsetof(Telemetry,practicalTemps)==0xEC),"bad off");
		static_assert((offsetof(Telemetry,splinePosition)==0xFC),"bad off");
	};
};

//UDT: struct CarColliderManager @len=128
	//_Func: public void CarColliderManager(CarColliderManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CarColliderManager(); @loc=static @len=184 @rva=2545584
	//_Func: public void ~CarColliderManager(); @loc=static @len=119 @rva=2766048
	//_Data: this+0x0, Member, Type: bool, isLive
	//_Func: public void init(Car * acar); @loc=static @len=68 @rva=2766672
	//_Func: public void addBox(CarCollisionBox &  _arg0, bool  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public int getBoxesCount(); @loc=optimized @len=0 @rva=0
	//_Func: public CarCollisionBox getBox(int index); @loc=static @len=142 @rva=2766528
	//_Func: public void adjustBox(int  _arg0, CarCollisionBox &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void step(float dt); @loc=static @len=61 @rva=2768208
	//_Data: this+0x8, Member, Type: class std::vector<CarCollisionBox,std::allocator<CarCollisionBox> >, boxes
	//_Data: this+0x20, Member, Type: class FileChangeObserver, observer
	//_Data: this+0x50, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, carModel
	//_Data: this+0x70, Member, Type: class IRigidBody *, carBody
	//_Data: this+0x78, Member, Type: class Car *, car
	//_Func: private void loadINI(); @loc=static @len=1449 @rva=2766752
	//_Func: public CarColliderManager & operator=(CarColliderManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CarColliderManager)==128),"bad size");
		static_assert((offsetof(CarColliderManager,isLive)==0x0),"bad off");
		static_assert((offsetof(CarColliderManager,boxes)==0x8),"bad off");
		static_assert((offsetof(CarColliderManager,observer)==0x20),"bad off");
		static_assert((offsetof(CarColliderManager,carModel)==0x50),"bad off");
		static_assert((offsetof(CarColliderManager,carBody)==0x70),"bad off");
		static_assert((offsetof(CarColliderManager,car)==0x78),"bad off");
	};
};

//UDT: struct PenaltyManager @len=72
	//_Func: public void ~PenaltyManager(); @loc=static @len=71 @rva=2513088
	//_Data: this+0x0, Member, Type: class Event<OnPenaltyEvent>, evOnPenalty
	//_Func: public void init(Car * car); @loc=static @len=36 @rva=2514000
	//_Func: public void step(float dt); @loc=static @len=200 @rva=2514448
	//_Func: public void reset(); @loc=static @len=68 @rva=2514240
	//_Func: public bool hasPenalty(); @loc=optimized @len=0 @rva=0
	//_Func: public void addPenalty(PenaltyType  _arg0, unsigned int  _arg1, PenaltyDescription  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public std::vector<PenaltyRecord,std::allocator<PenaltyRecord> > getPenaltyRecords(); @loc=optimized @len=0 @rva=0
	//_Func: public void resetPenalty(); @loc=static @len=120 @rva=2514320
	//_Func: public bool checkBlackFlag(); @loc=static @len=25 @rva=2513936
	//_Func: public void addJumpStartPenalty(); @loc=static @len=375 @rva=2513552
	//_Func: public bool getIsServingPitPenalty(); @loc=optimized @len=0 @rva=0
	//_Func: public void setRearmPitPenalty(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public short getPitPenaltyLaps(); @loc=optimized @len=0 @rva=0
	//_Func: public void decreasePitPenaltyLaps(bool isValidLap); @loc=static @len=27 @rva=2513968
	//_Data: this+0x18, Member, Type: class Car *, car
	//_Data: this+0x20, Member, Type: enum PenaltyType, pendingPenaltyType
	//_Data: this+0x24, Member, Type: bool, isServingPitPenalty
	//_Data: this+0x25, Member, Type: bool, rearmPitPenalty
	//_Data: this+0x26, Member, Type: short, pitPenaltyLaps
	//_Data: this+0x28, Member, Type: class std::vector<PenaltyRecord,std::allocator<PenaltyRecord> >, penaltyRecords
	//_Data: this+0x40, Member, Type: unsigned int, penaltySecs
	//_Data: this+0x44, Member, Type: float, inPitMeterCounter
	//_Func: public void PenaltyManager(PenaltyManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void PenaltyManager(); @loc=optimized @len=0 @rva=0
	//_Func: public PenaltyManager & operator=(PenaltyManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(PenaltyManager)==72),"bad size");
		static_assert((offsetof(PenaltyManager,evOnPenalty)==0x0),"bad off");
		static_assert((offsetof(PenaltyManager,car)==0x18),"bad off");
		static_assert((offsetof(PenaltyManager,pendingPenaltyType)==0x20),"bad off");
		static_assert((offsetof(PenaltyManager,isServingPitPenalty)==0x24),"bad off");
		static_assert((offsetof(PenaltyManager,rearmPitPenalty)==0x25),"bad off");
		static_assert((offsetof(PenaltyManager,pitPenaltyLaps)==0x26),"bad off");
		static_assert((offsetof(PenaltyManager,penaltyRecords)==0x28),"bad off");
		static_assert((offsetof(PenaltyManager,penaltySecs)==0x40),"bad off");
		static_assert((offsetof(PenaltyManager,inPitMeterCounter)==0x44),"bad off");
	};
};

//UDT: class INIReader @len=104 @vfcount=1
	//_VTable: 
	//_Func: public void INIReader(INIReader & __that); @loc=static @len=185 @rva=776000
	//_Func: protected void INIReader(); @loc=static @len=130 @rva=2310160
	//_Func: public void INIReader(std::basic_stringstream<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & stream); @loc=static @len=231 @rva=2310928
	//_Func: public void INIReader(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & ifilename); @loc=static @len=612 @rva=2310304
	//_Func: public void ~INIReader(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=143 @rva=2311168
	//_Data: this+0x8, Member, Type: bool, ready
	//_Data: static, [0155A550][0003:00047550], Static Member, Type: class IErrorHandler *, errorHandler
	//_Data: static, [0155A558][0003:00047558], Static Member, Type: bool, crashAtError
	//_Data: this+0x9, Member, Type: bool, suppressErrorReporting
	//_Data: static, [0155A560][0003:00047560], Static Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, openedFiles
	//_Data: static, [0151D0F9][0003:0000A0F9], Static Member, Type: bool, useCache
	//_Func: public void clearCache(); @pure @loc=static @len=75 @rva=2312464
	//_Func: public bool hasSection(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section); @loc=static @len=110 @rva=2322608
	//_Func: public bool hasKey(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=313 @rva=2322176
	//_Func: public int getHex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=507 @rva=2316912
	//_Func: public int getInt(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=411 @rva=2317424
	//_Func: public float getFloat(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=431 @rva=2316480
	//_Func: public void getSections(std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > & outSections); @loc=static @len=300 @rva=2318272
	//_Func: public void getKeyes(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * section, std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > & keyes); @loc=static @len=419 @rva=2317840
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getSection(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getString(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=527 @rva=2318576
	//_Func: public vec2f getFloat2(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=53 @rva=2316128
	//_Func: public vec3f getFloat3(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=192 @rva=2316192
	//_Func: public vec4f getFloat4(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=96 @rva=2316384
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getFileName(); @loc=static @len=58 @rva=2316064
	//_Func: public void setVerboseMode(bool mode); @loc=static @len=20 @rva=2327152
	//_Func: public bool isEncrypted(); @loc=optimized @len=0 @rva=0
	//_Func: public Curve getCurve(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key); @loc=static @len=1752 @rva=2314304
	//_Func: public void printCode(); @loc=static @len=28 @rva=2327120
	//_Data: this+0x10, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, filename
	//_Data: this+0x30, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,INISection,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,INISection> > >, sections
	//_Data: this+0x40, Member, Type: bool, m_isEncrypted
	//_Data: this+0x41, Member, Type: bool, verbose
	//_Data: static, [0155A578][0003:00047578], Static Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,INISection,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,INISection> > >,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,INISection,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,INISection> > > > > >, cache
	//_Func: protected void load(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=1058 @rva=2322752
	//_Func: protected void loadEncrypt(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataFile); @loc=static @len=271 @rva=2323824
	//_Data: this+0x48, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, code
	//_Func: protected std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getSectionEx(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg1, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: protected void errorReport(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * message); @loc=static @len=300 @rva=2313840
	//_Func: protected void warningReport(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void parse(); @loc=static @len=3021 @rva=2324096
	//_Func: protected void getVector2(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key, float & x, float & y); @loc=static @len=570 @rva=2319104
	//_Func: protected void getVector3(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key, float & x, float & y, float & z); @loc=static @len=1423 @rva=2319680
	//_Func: protected bool getVector4(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & key, float & x, float & y, float & z, float & w); @loc=static @len=1065 @rva=2321104
	//_Func: public INIReader & operator=(INIReader & __that); @loc=static @len=201 @rva=776720
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(INIReader)==104),"bad size");
		static_assert((offsetof(INIReader,ready)==0x8),"bad off");
		static_assert((offsetof(INIReader,suppressErrorReporting)==0x9),"bad off");
		static_assert((offsetof(INIReader,filename)==0x10),"bad off");
		static_assert((offsetof(INIReader,sections)==0x30),"bad off");
		static_assert((offsetof(INIReader,m_isEncrypted)==0x40),"bad off");
		static_assert((offsetof(INIReader,verbose)==0x41),"bad off");
		static_assert((offsetof(INIReader,code)==0x48),"bad off");
	};
};

//UDT: class TCPSocket @len=65656
	//_Func: public void TCPSocket(TCPSocket &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TCPSocket(); @loc=static @len=98 @rva=2481840
	//_Func: public void ~TCPSocket(); @loc=static @len=142 @rva=2482080
	//_Data: this+0x0, Member, Type: class BufferedChannel<unsigned __int64>, chNewConnections
	//_Func: public void setSocket(unsigned __int64  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool connect(IPAddress & target); @loc=static @len=147 @rva=2482592
	//_Func: public void setBlockingMode(bool imode); @loc=static @len=14 @rva=2484144
	//_Func: public bool getBlockingMode(); @loc=optimized @len=0 @rva=0
	//_Func: public void addListener(std::function<void __cdecl(UDPMessage const &)> * listener); @loc=static @len=125 @rva=2482464
	//_Func: public int receive(std::function<void __cdecl(UDPMessage const &)>  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public int receive(int maxPackets); @loc=static @len=285 @rva=2483216
	//_Func: public void send(void * data, int length, sockaddr_in target); @loc=static @len=14 @rva=2484128
	//_Func: public void sendReliable(void *  _arg0, int  _arg1, sockaddr_in  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public std::vector<unsigned char,std::allocator<unsigned char> > receivePacket(); @loc=static @len=416 @rva=2483504
	//_Func: public void listen(unsigned short  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x28, Member, Type: class std::vector<std::function<void __cdecl(UDPMessage const &)>,std::allocator<std::function<void __cdecl(UDPMessage const &)> > >, listeners
	//_Data: this+0x40, Member, Type: class std::thread, listenerThread
	//_Data: this+0x50, Member, Type: bool, isExiting
	//_Data: this+0x58, Member, Type: unsigned char *, recvBuffer
	//_Data: this+0x60, Member, Type: unsigned __int64, soc
	//_Data: this+0x68, Member, Type: unsigned __int64, listenSock
	//_Data: this+0x70, Member, Type: bool, isBlocking
	//_Data: this+0x71, Member, Type: bool, isServer
	//_Data: this+0x74, Member, Type: class TCPQueue, buffer
	//_Func: private void broadcastMessage(UDPMessage &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public TCPSocket & operator=(TCPSocket &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TCPSocket)==65656),"bad size");
		static_assert((offsetof(TCPSocket,chNewConnections)==0x0),"bad off");
		static_assert((offsetof(TCPSocket,listeners)==0x28),"bad off");
		static_assert((offsetof(TCPSocket,listenerThread)==0x40),"bad off");
		static_assert((offsetof(TCPSocket,isExiting)==0x50),"bad off");
		static_assert((offsetof(TCPSocket,recvBuffer)==0x58),"bad off");
		static_assert((offsetof(TCPSocket,soc)==0x60),"bad off");
		static_assert((offsetof(TCPSocket,listenSock)==0x68),"bad off");
		static_assert((offsetof(TCPSocket,isBlocking)==0x70),"bad off");
		static_assert((offsetof(TCPSocket,isServer)==0x71),"bad off");
		static_assert((offsetof(TCPSocket,buffer)==0x74),"bad off");
	};
};

//UDT: class Console @len=296 @multibase=3
	//_Base: class GameObject @off=0 @len=88
	//_Base: class IVarCallback @off=88 @len=8
	//_Base: class IKeyEventListener @off=96 @len=8
	//_Func: public void Console(Console &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Console(Sim * isim); @loc=static @len=879 @rva=1607872
	//_Func: public void ~Console(); @virtual vtpo=0 vfid=0 @loc=static @len=772 @rva=1608752
	//_Data: this+0x68, Member, Type: class Sim *, sim
	//_Data: this+0x70, Member, Type: class std::vector<SVar *,std::allocator<SVar *> >, vars
	//_Func: public Console & singleton(); @pure @loc=static @len=8 @rva=1624096
	//_Func: public void renderHUD(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=672 @rva=1623376
	//_Func: public void onSetVar(SVar * var, float value); @virtual vtpo=88 vfid=0 @loc=static @len=2152 @rva=1619680
	//_Func: public void addVar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, float * var, IVarCallback * callback, bool readOnly, float multiplier); @loc=static @len=358 @rva=1615552
	//_Func: public void addVarLambda(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, std::function<void __cdecl(SVar *,float)> * lambda, bool readyonly, float multiplier); @loc=static @len=414 @rva=1615920
	//_Func: public void show(bool mode); @loc=static @len=39 @rva=1624048
	//_Func: public bool isVisible(); @loc=static @len=5 @rva=1680480
	//_Func: public void addCommand(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name, std::function<bool __cdecl(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >)> * exeFun, std::function<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > __cdecl(void)> * helpFun); @loc=static @len=390 @rva=1615152
	//_Func: public void addCommand(ConsoleCommand *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void onKeyChar(unsigned int key); @virtual vtpo=96 vfid=1 @loc=static @len=416 @rva=1619072
	//_Func: public void onKeyDown(OnKeyEvent & message); @virtual vtpo=96 vfid=0 @loc=static @len=187 @rva=1619488
	//_Func: public void onNewCarLoaded(OnNewCarLoadedEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Console & operator<<(double &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Console & operator<<(float & v); @loc=static @len=409 @rva=1609952
	//_Func: public Console & operator<<(int & v); @loc=static @len=409 @rva=1609536
	//_Func: public Console & operator<<(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * s); @loc=static @len=447 @rva=1610928
	//_Func: public Console & operator<<(std::basic_string<char,std::char_traits<char>,std::allocator<char> > * s); @loc=static @len=560 @rva=1610368
	//_Func: public Console & operator<<(unsigned int &  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x88, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, lines
	//_Data: this+0xA0, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, history
	//_Data: this+0xB8, Member, Type: unsigned int, historyPos
	//_Data: this+0xC0, Member, Type: class Font *, font
	//_Data: this+0xC8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, commandLine
	//_Data: this+0xE8, Member, Type: class std::vector<ConsoleCommand *,std::allocator<ConsoleCommand *> >, commands
	//_Data: this+0x100, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, accumulator
	//_Data: static, [01559C08][0003:00046C08], Static Member, Type: class Console *, _singleton
	//_Data: this+0x120, Member, Type: bool, disableAutoShow
	//_Func: protected void parse(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * c); @loc=static @len=1246 @rva=1621840
	//_Func: protected void initCommands(); @loc=static @len=2553 @rva=1616512
	//_Func: protected void autoShow(); @loc=optimized @len=0 @rva=0
	//_Func: public Console & operator=(Console &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Console)==296),"bad size");
		static_assert((offsetof(Console,sim)==0x68),"bad off");
		static_assert((offsetof(Console,vars)==0x70),"bad off");
		static_assert((offsetof(Console,lines)==0x88),"bad off");
		static_assert((offsetof(Console,history)==0xA0),"bad off");
		static_assert((offsetof(Console,historyPos)==0xB8),"bad off");
		static_assert((offsetof(Console,font)==0xC0),"bad off");
		static_assert((offsetof(Console,commandLine)==0xC8),"bad off");
		static_assert((offsetof(Console,commands)==0xE8),"bad off");
		static_assert((offsetof(Console,accumulator)==0x100),"bad off");
		static_assert((offsetof(Console,disableAutoShow)==0x120),"bad off");
	};
};

//UDT: struct CameraCarDefinition @len=76
	//_Func: public void CameraCarDefinition(CameraCarDefinition &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CameraCarDefinition(); @loc=static @len=160 @rva=839936
	//_Data: this+0x0, Member, Type: class mat44f, matrix
	//_Data: this+0x40, Member, Type: float, fov
	//_Data: this+0x44, Member, Type: float, exposure
	//_Data: this+0x48, Member, Type: bool, externalSound
//UDT;

struct CameraCarDefinition {
public:
	mat44f matrix;
	float fov;
	float exposure;
	bool externalSound;
	inline CameraCarDefinition()  { }
	inline void ctor() { typedef void (*_fpt)(CameraCarDefinition *pthis); _fpt _f=(_fpt)_drva(839936); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(CameraCarDefinition)==76),"bad size");
		static_assert((offsetof(CameraCarDefinition,matrix)==0x0),"bad off");
		static_assert((offsetof(CameraCarDefinition,fov)==0x40),"bad off");
		static_assert((offsetof(CameraCarDefinition,exposure)==0x44),"bad off");
		static_assert((offsetof(CameraCarDefinition,externalSound)==0x48),"bad off");
	};
};

//UDT: class INIReaderDocuments @len=104 @vfcount=1
	//_Base: class INIReader @off=0 @len=104
	//_Func: public void INIReaderDocuments(INIReaderDocuments &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void INIReaderDocuments(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iniName, bool createFile); @loc=static @len=1637 @rva=2327376
	//_Func: public void ~INIReaderDocuments(); @virtual vtpo=0 vfid=0 @loc=static @len=15 @rva=2329024
	//_Data: static, [0155A588][0003:00047588], Static Member, Type: bool, initialized
	//_Data: static, [0151D100][0003:0000A100], Static Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, basePath
	//_Data: static, [0151D120][0003:0000A120], Static Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, programName
	//_Func: public INIReaderDocuments & operator=(INIReaderDocuments &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class INIReaderDocuments : public INIReader {
public:
	inline INIReaderDocuments()  { }
	inline void ctor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iniName, bool createFile) { typedef void (*_fpt)(INIReaderDocuments *pthis, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > *, bool); _fpt _f=(_fpt)_drva(2327376); _f(this, iniName, createFile); }
	virtual ~INIReaderDocuments();
	inline void dtor() { typedef void (*_fpt)(INIReaderDocuments *pthis); _fpt _f=(_fpt)_drva(2329024); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(INIReaderDocuments)==104),"bad size");
	};
};

//UDT: struct SlipStream @len=104
	//_Func: public void ~SlipStream(); @loc=static @len=5 @rva=2796272
	//_Data: this+0x0, Member, Type: class Triangle, triangle
	//_Data: this+0x40, Member, Type: float, speedFactorMult
	//_Data: this+0x44, Member, Type: float, effectGainMult
	//_Data: this+0x48, Member, Type: class vec3f, dir
	//_Func: public void init(PhysicsEngine * pe); @loc=static @len=179 @rva=2796992
	//_Func: public float getSlipEffect(vec3f & p); @loc=static @len=349 @rva=2796640
	//_Func: public void setPosition(vec3f & pos, vec3f & vel); @loc=static @len=677 @rva=2797184
	//_Func: public void setSpeedFactor(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getSpeedFactor(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x58, Member, Type: class PhysicsEngine *, physicsEngine
	//_Data: this+0x60, Member, Type: float, length
	//_Data: this+0x64, Member, Type: float, speedFactor
	//_Func: public void SlipStream(SlipStream &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SlipStream(); @loc=optimized @len=0 @rva=0
	//_Func: public SlipStream & operator=(SlipStream &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SlipStream)==104),"bad size");
		static_assert((offsetof(SlipStream,triangle)==0x0),"bad off");
		static_assert((offsetof(SlipStream,speedFactorMult)==0x40),"bad off");
		static_assert((offsetof(SlipStream,effectGainMult)==0x44),"bad off");
		static_assert((offsetof(SlipStream,dir)==0x48),"bad off");
		static_assert((offsetof(SlipStream,physicsEngine)==0x58),"bad off");
		static_assert((offsetof(SlipStream,length)==0x60),"bad off");
		static_assert((offsetof(SlipStream,speedFactor)==0x64),"bad off");
	};
};

//UDT: class IRigidBody @len=8 @vfcount=42
	//_VTable: 
	//_Func: public void setMassExplicitInertia(float  _arg0, float  _arg1, float  _arg2, float  _arg3); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public bool isEnabled(); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void setEnabled(bool  _arg0); @intro @pure @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Func: public void setAutoDisable(bool  _arg0); @intro @pure @virtual vtpo=0 vfid=3 @loc=optimized @len=0 @rva=0
	//_Func: public void removeCollisionObjects(); @intro @pure @virtual vtpo=0 vfid=4 @loc=optimized @len=0 @rva=0
	//_Func: public float getMass(); @intro @pure @virtual vtpo=0 vfid=5 @loc=optimized @len=0 @rva=0
	//_Func: public void release(); @intro @pure @virtual vtpo=0 vfid=6 @loc=optimized @len=0 @rva=0
	//_Func: public void setMassBox(float  _arg0, float  _arg1, float  _arg2, float  _arg3); @intro @pure @virtual vtpo=0 vfid=7 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getLocalInertia(); @intro @pure @virtual vtpo=0 vfid=8 @loc=optimized @len=0 @rva=0
	//_Func: public mat44f getWorldMatrix(float  _arg0); @intro @pure @virtual vtpo=0 vfid=9 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f localToWorld(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=10 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f worldToLocal(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=11 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f localToWorldNormal(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=12 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f worldToLocalNormal(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=13 @loc=optimized @len=0 @rva=0
	//_Func: public void stop(float  _arg0); @intro @pure @virtual vtpo=0 vfid=14 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getVelocity(); @intro @pure @virtual vtpo=0 vfid=15 @loc=optimized @len=0 @rva=0
	//_Func: public void setVelocity(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=16 @loc=optimized @len=0 @rva=0
	//_Func: public void setAngularVelocity(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=17 @loc=optimized @len=0 @rva=0
	//_Func: public void setPosition(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=18 @loc=optimized @len=0 @rva=0
	//_Func: public void setRotation(mat44f &  _arg0); @intro @pure @virtual vtpo=0 vfid=19 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getPosition(float  _arg0); @intro @pure @virtual vtpo=0 vfid=20 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getAngularVelocity(); @intro @pure @virtual vtpo=0 vfid=21 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getLocalAngularVelocity(); @intro @pure @virtual vtpo=0 vfid=22 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getLocalVelocity(); @intro @pure @virtual vtpo=0 vfid=23 @loc=optimized @len=0 @rva=0
	//_Func: public void setBoxColliderMask(unsigned __int64  _arg0, unsigned long  _arg1); @intro @pure @virtual vtpo=0 vfid=24 @loc=optimized @len=0 @rva=0
	//_Func: public unsigned __int64 addBoxCollider(vec3f &  _arg0, vec3f &  _arg1, unsigned int  _arg2, unsigned long  _arg3, unsigned int  _arg4); @intro @pure @virtual vtpo=0 vfid=25 @loc=optimized @len=0 @rva=0
	//_Func: public void addSphereCollider(vec3f &  _arg0, float  _arg1, unsigned int  _arg2, ISphereCollisionCallback *  _arg3); @intro @pure @virtual vtpo=0 vfid=26 @loc=optimized @len=0 @rva=0
	//_Func: public void addLocalForce(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=27 @loc=optimized @len=0 @rva=0
	//_Func: public void addLocalTorque(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=28 @loc=optimized @len=0 @rva=0
	//_Func: public void addLocalForceAtPos(vec3f &  _arg0, vec3f &  _arg1); @intro @pure @virtual vtpo=0 vfid=29 @loc=optimized @len=0 @rva=0
	//_Func: public void addLocalForceAtLocalPos(vec3f &  _arg0, vec3f &  _arg1); @intro @pure @virtual vtpo=0 vfid=30 @loc=optimized @len=0 @rva=0
	//_Func: public void addForceAtLocalPos(vec3f &  _arg0, vec3f &  _arg1); @intro @pure @virtual vtpo=0 vfid=31 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getLocalPointVelocity(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=32 @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getPointVelocity(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=33 @loc=optimized @len=0 @rva=0
	//_Func: public void addForceAtPos(vec3f &  _arg0, vec3f &  _arg1); @intro @pure @virtual vtpo=0 vfid=34 @loc=optimized @len=0 @rva=0
	//_Func: public void addTorque(vec3f &  _arg0); @intro @pure @virtual vtpo=0 vfid=35 @loc=optimized @len=0 @rva=0
	//_Func: public void addMeshCollider(float *  _arg0, unsigned int  _arg1, unsigned short *  _arg2, unsigned int  _arg3, mat44f  _arg4, unsigned long  _arg5, unsigned long  _arg6, unsigned int  _arg7); @intro @pure @virtual vtpo=0 vfid=36 @loc=optimized @len=0 @rva=0
	//_Func: public void setMeshCollideCategory(unsigned int  _arg0, unsigned long  _arg1); @intro @pure @virtual vtpo=0 vfid=37 @loc=optimized @len=0 @rva=0
	//_Func: public void setMeshCollideMask(unsigned int  _arg0, unsigned long  _arg1); @intro @pure @virtual vtpo=0 vfid=38 @loc=optimized @len=0 @rva=0
	//_Func: public unsigned long getMeshCollideCategory(unsigned int  _arg0); @intro @pure @virtual vtpo=0 vfid=39 @loc=optimized @len=0 @rva=0
	//_Func: public unsigned long getMeshCollideMask(unsigned int  _arg0); @intro @pure @virtual vtpo=0 vfid=40 @loc=optimized @len=0 @rva=0
	//_Func: protected void ~IRigidBody(); @intro @virtual vtpo=0 vfid=41 @loc=optimized @len=0 @rva=0
	//_Func: public void IRigidBody(IRigidBody &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IRigidBody(); @loc=optimized @len=0 @rva=0
	//_Func: public IRigidBody & operator=(IRigidBody &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: protected void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=41 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(IRigidBody)==8),"bad size");
	};
};

//UDT: class DynamicController @len=40
	//_Func: public void DynamicController(DynamicController &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void DynamicController(); @loc=static @len=24 @rva=2819936
	//_Func: public void DynamicController(Car * car, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=5164 @rva=2814768
	//_Func: public void ~DynamicController(); @loc=static @len=9 @rva=2820032
	//_Func: public float eval(); @loc=static @len=364 @rva=2821120
	//_Func: public float getOversteerFactor(Car * car); @pure @loc=static @len=92 @rva=2822608
	//_Func: public float getRearSpeedRatio(Car * car); @pure @loc=static @len=61 @rva=2822704
	//_Func: public float getInput(DynamicControllerInput input); @loc=static @len=1116 @rva=2821488
	//_Func: public bool isReady(); @loc=static @len=5 @rva=2864688
	//_Data: this+0x0, Member, Type: class Car *, car
	//_Data: this+0x8, Member, Type: class std::vector<DynamicControllerStage,std::allocator<DynamicControllerStage> >, stages
	//_Data: this+0x20, Member, Type: bool, ready
	//_Func: public DynamicController & operator=(DynamicController & __that); @loc=static @len=56 @rva=2550784
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DynamicController)==40),"bad size");
		static_assert((offsetof(DynamicController,car)==0x0),"bad off");
		static_assert((offsetof(DynamicController,stages)==0x8),"bad off");
		static_assert((offsetof(DynamicController,ready)==0x20),"bad off");
	};
};

//UDT: struct CarPhysicsInfo @len=320
	//_Data: this+0x0, Member, Type: float, steerLock
	//_Data: this+0x4, Member, Type: int, maxGear
	//_Data: this+0x8, Member, Type: float, caster
	//_Data: this+0xC, Member, Type: float[0x4], tyreRadius
	//_Data: this+0x1C, Member, Type: float[0x4], tyreWidth
	//_Data: this+0x2C, Member, Type: float, totalMass
	//_Data: this+0x30, Member, Type: float, bodyMass
	//_Data: this+0x34, Member, Type: float[0x4], unsprungWeights
	//_Data: this+0x48, Member, Type: class std::vector<WingData,std::allocator<WingData> >, wingData
	//_Data: this+0x60, Member, Type: float, maxTorqueNM
	//_Data: this+0x64, Member, Type: float, maxPowerW
	//_Data: this+0x68, Member, Type: float[0x4], bumpStopsUp
	//_Data: this+0x78, Member, Type: float[0x4], bumpStopsDn
	//_Data: this+0x88, Member, Type: double, maxFuel
	//_Data: this+0x90, Member, Type: class vec3f[0x2], ridePickupPoint
	//_Data: this+0xA8, Member, Type: class vec3f[0x4], susBasePos
	//_Data: this+0xD8, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, tyreCompounds
	//_Data: this+0xF0, Member, Type: float, minHeightM
	//_Data: this+0xF4, Member, Type: float[0x4], wheelAngularInertia
	//_Data: this+0x104, Member, Type: struct KPI, kpi
	//_Data: this+0x10C, Member, Type: float, maxTurboBoost
	//_Data: this+0x110, Member, Type: int, turboCount
	//_Data: this+0x114, Member, Type: float, kersMaxJ
	//_Data: this+0x118, Member, Type: float, ersMaxJ
	//_Data: this+0x11C, Member, Type: float, powerClassIndex
	//_Data: this+0x120, Member, Type: bool, hasKERS
	//_Data: this+0x121, Member, Type: bool, hasERS
	//_Data: this+0x122, Member, Type: bool, hasCockpitERSRecovery
	//_Data: this+0x123, Member, Type: bool, hasCockpitERSDeliveryProfile
	//_Data: this+0x124, Member, Type: bool, hasCockpitEngineBrake
	//_Data: this+0x125, Member, Type: bool, hasCockpitERSMguHMode
	//_Data: this+0x126, Member, Type: bool, hasDRS
	//_Data: this+0x128, Member, Type: int, engineBrakeSettingsCount
	//_Data: this+0x12C, Member, Type: int, ersPowerControllerCount
	//_Data: this+0x130, Member, Type: int, engineDamageRPM
	//_Data: this+0x134, Member, Type: bool, hasAdjustableTurbo
	//_Data: this+0x138, Member, Type: int, maxRpm
	//_Func: public void CarPhysicsInfo(CarPhysicsInfo &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CarPhysicsInfo(); @loc=static @len=192 @rva=844768
	//_Func: public void ~CarPhysicsInfo(); @loc=static @len=84 @rva=847488
	//_Func: public CarPhysicsInfo & operator=(CarPhysicsInfo & __that); @loc=static @len=702 @rva=1596080
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CarPhysicsInfo)==320),"bad size");
		static_assert((offsetof(CarPhysicsInfo,steerLock)==0x0),"bad off");
		static_assert((offsetof(CarPhysicsInfo,maxGear)==0x4),"bad off");
		static_assert((offsetof(CarPhysicsInfo,caster)==0x8),"bad off");
		static_assert((offsetof(CarPhysicsInfo,tyreRadius)==0xC),"bad off");
		static_assert((offsetof(CarPhysicsInfo,tyreWidth)==0x1C),"bad off");
		static_assert((offsetof(CarPhysicsInfo,totalMass)==0x2C),"bad off");
		static_assert((offsetof(CarPhysicsInfo,bodyMass)==0x30),"bad off");
		static_assert((offsetof(CarPhysicsInfo,unsprungWeights)==0x34),"bad off");
		static_assert((offsetof(CarPhysicsInfo,wingData)==0x48),"bad off");
		static_assert((offsetof(CarPhysicsInfo,maxTorqueNM)==0x60),"bad off");
		static_assert((offsetof(CarPhysicsInfo,maxPowerW)==0x64),"bad off");
		static_assert((offsetof(CarPhysicsInfo,bumpStopsUp)==0x68),"bad off");
		static_assert((offsetof(CarPhysicsInfo,bumpStopsDn)==0x78),"bad off");
		static_assert((offsetof(CarPhysicsInfo,maxFuel)==0x88),"bad off");
		static_assert((offsetof(CarPhysicsInfo,ridePickupPoint)==0x90),"bad off");
		static_assert((offsetof(CarPhysicsInfo,susBasePos)==0xA8),"bad off");
		static_assert((offsetof(CarPhysicsInfo,tyreCompounds)==0xD8),"bad off");
		static_assert((offsetof(CarPhysicsInfo,minHeightM)==0xF0),"bad off");
		static_assert((offsetof(CarPhysicsInfo,wheelAngularInertia)==0xF4),"bad off");
		static_assert((offsetof(CarPhysicsInfo,kpi)==0x104),"bad off");
		static_assert((offsetof(CarPhysicsInfo,maxTurboBoost)==0x10C),"bad off");
		static_assert((offsetof(CarPhysicsInfo,turboCount)==0x110),"bad off");
		static_assert((offsetof(CarPhysicsInfo,kersMaxJ)==0x114),"bad off");
		static_assert((offsetof(CarPhysicsInfo,ersMaxJ)==0x118),"bad off");
		static_assert((offsetof(CarPhysicsInfo,powerClassIndex)==0x11C),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasKERS)==0x120),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasERS)==0x121),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasCockpitERSRecovery)==0x122),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasCockpitERSDeliveryProfile)==0x123),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasCockpitEngineBrake)==0x124),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasCockpitERSMguHMode)==0x125),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasDRS)==0x126),"bad off");
		static_assert((offsetof(CarPhysicsInfo,engineBrakeSettingsCount)==0x128),"bad off");
		static_assert((offsetof(CarPhysicsInfo,ersPowerControllerCount)==0x12C),"bad off");
		static_assert((offsetof(CarPhysicsInfo,engineDamageRPM)==0x130),"bad off");
		static_assert((offsetof(CarPhysicsInfo,hasAdjustableTurbo)==0x134),"bad off");
		static_assert((offsetof(CarPhysicsInfo,maxRpm)==0x138),"bad off");
	};
};

//UDT: struct Autoclutch @len=424
	//_Func: public void ~Autoclutch(); @loc=static @len=62 @rva=2851776
	//_Data: this+0x0, Member, Type: float, rpmMin
	//_Data: this+0x4, Member, Type: float, rpmMax
	//_Data: this+0x8, Member, Type: float, clutchSpeed
	//_Data: this+0xC, Member, Type: bool, useAutoOnStart
	//_Data: this+0xD, Member, Type: bool, useAutoOnChange
	//_Data: this+0xE, Member, Type: bool, isForced
	//_Func: public void init(Car * car); @loc=static @len=179 @rva=2852816
	//_Func: public void step(float dt); @loc=static @len=519 @rva=2856336
	//_Func: public void onGearRequest(OnGearRequestEvent & ev); @loc=static @len=267 @rva=2855760
	//_Func: public bool isInChangeSequence(); @loc=optimized @len=0 @rva=0
	//_Func: public void setDownshiftProfile(Curve & dp); @loc=static @len=122 @rva=2750528
	//_Func: public float getDownshiftSequenceDuration(); @loc=static @len=63 @rva=2741152
	//_Data: this+0x10, Member, Type: class Car *, car
	//_Data: this+0x18, Member, Type: struct ClutchSequence, clutchSequence
	//_Data: this+0xA0, Member, Type: class Curve, upshiftProfile
	//_Data: this+0x120, Member, Type: class Curve, downshiftProfile
	//_Data: this+0x1A0, Member, Type: float, clutchValueSignal
	//_Func: protected void stepSequence(float dt); @loc=static @len=168 @rva=2856864
	//_Func: protected void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * carModel); @loc=static @len=2746 @rva=2853008
	//_Func: public void Autoclutch(Autoclutch &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Autoclutch(); @loc=static @len=126 @rva=2538848
	//_Func: public Autoclutch & operator=(Autoclutch &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Autoclutch)==424),"bad size");
		static_assert((offsetof(Autoclutch,rpmMin)==0x0),"bad off");
		static_assert((offsetof(Autoclutch,rpmMax)==0x4),"bad off");
		static_assert((offsetof(Autoclutch,clutchSpeed)==0x8),"bad off");
		static_assert((offsetof(Autoclutch,useAutoOnStart)==0xC),"bad off");
		static_assert((offsetof(Autoclutch,useAutoOnChange)==0xD),"bad off");
		static_assert((offsetof(Autoclutch,isForced)==0xE),"bad off");
		static_assert((offsetof(Autoclutch,car)==0x10),"bad off");
		static_assert((offsetof(Autoclutch,clutchSequence)==0x18),"bad off");
		static_assert((offsetof(Autoclutch,upshiftProfile)==0xA0),"bad off");
		static_assert((offsetof(Autoclutch,downshiftProfile)==0x120),"bad off");
		static_assert((offsetof(Autoclutch,clutchValueSignal)==0x1A0),"bad off");
	};
};

//UDT: struct NetCarStateProviderDef @len=224
	//_Data: this+0x0, Member, Type: class ACClient *, client
	//_Data: this+0x8, Member, Type: unsigned char, sessionID
	//_Data: this+0x9, Member, Type: unsigned char, guid
	//_Data: this+0x10, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, driverName
	//_Data: this+0x30, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, team
	//_Data: this+0x50, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, nationCode
	//_Data: this+0x70, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, model
	//_Data: this+0x90, Member, Type: class IRayTrackCollisionProvider *, rayCastProvider
	//_Data: this+0x98, Member, Type: class PhysicsAvatar *, physicsAvatar
	//_Data: this+0xA0, Member, Type: class mat44f, pitPosition
	//_Func: public void NetCarStateProviderDef(NetCarStateProviderDef &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void NetCarStateProviderDef(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~NetCarStateProviderDef(); @loc=static @len=163 @rva=1660448
	//_Func: public NetCarStateProviderDef & operator=(NetCarStateProviderDef &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(NetCarStateProviderDef)==224),"bad size");
		static_assert((offsetof(NetCarStateProviderDef,client)==0x0),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,sessionID)==0x8),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,guid)==0x9),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,driverName)==0x10),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,team)==0x30),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,nationCode)==0x50),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,model)==0x70),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,rayCastProvider)==0x90),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,physicsAvatar)==0x98),"bad off");
		static_assert((offsetof(NetCarStateProviderDef,pitPosition)==0xA0),"bad off");
	};
};

//UDT: class Spline @len=40 @vfcount=2
	//_VTable: 
	//_Func: public void Spline(Spline &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Spline(); @loc=static @len=34 @rva=2019712
	//_Func: public void ~Spline(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=60 @rva=2019760
	//_Func: public vec3f pointAt(unsigned int index); @loc=static @len=92 @rva=2025504
	//_Func: public void setPointAt(unsigned int index, vec3f point); @loc=static @len=75 @rva=2025904
	//_Func: public SplinePoint splinePointAt(unsigned int index); @loc=static @len=130 @rva=2025984
	//_Func: public float distanceAt(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void addPoint(vec3f & p, int tag); @loc=static @len=188 @rva=2020960
	//_Func: public void addSplinePoint(SplinePoint & p); @loc=static @len=9 @rva=2021152
	//_Func: public void reSampleSpline(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void clear(); @intro @virtual vtpo=0 vfid=1 @loc=static @len=16 @rva=2021616
	//_Func: public float length(); @loc=static @len=20 @rva=2024352
	//_Func: public unsigned int pointsCount(); @loc=static @len=39 @rva=2025600
	//_Func: public unsigned int closestPointIndex(vec3f & point, float * distance); @loc=static @len=261 @rva=2021632
	//_Func: public unsigned int closestPointIndexWithBounds(vec3f & point, std::vector<SplineIndexBound,std::allocator<SplineIndexBound> > & bounds, float * distance); @loc=static @len=238 @rva=2021904
	//_Func: public std::vector<unsigned int,std::allocator<unsigned int> > closestPointIndicesFlat(vec3f & point, unsigned int numberOfNeigbours, float * distance); @loc=static @len=1102 @rva=2022144
	//_Func: public vec3f getSplineDirection(vec3f & point); @loc=static @len=313 @rva=2023888
	//_Func: public unsigned int wrapIndex(int index); @loc=static @len=197 @rva=2026192
	//_Func: public void loadFromCSV(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename); @loc=static @len=1111 @rva=2024384
	//_Func: public vec3f getPointSmooth(float v); @loc=static @len=371 @rva=2023504
	//_Func: public void applyMatrix(mat44f & m); @loc=static @len=297 @rva=2021168
	//_Func: public int tagAt(unsigned int index); @loc=static @len=62 @rva=2026128
	//_Func: public bool isClosed(); @loc=static @len=134 @rva=2024208
	//_Func: public void reverse(); @loc=static @len=64 @rva=2025840
	//_Data: this+0x8, Member, Type: class std::vector<SplinePoint,std::allocator<SplinePoint> >, points
	//_Data: this+0x20, Member, Type: float, m_length
	//_Data: this+0x24, Member, Type: bool, m_closed
	//_Func: protected void smoothPoint(int  _arg0, int  _arg1, float  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: protected void updateSplineLength(); @loc=optimized @len=0 @rva=0
	//_Func: protected float boundInsideSpline(float value); @loc=static @len=129 @rva=2021472
	//_Func: public Spline & operator=(Spline &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Spline)==40),"bad size");
		static_assert((offsetof(Spline,points)==0x8),"bad off");
		static_assert((offsetof(Spline,m_length)==0x20),"bad off");
		static_assert((offsetof(Spline,m_closed)==0x24),"bad off");
	};
};

//UDT: struct RenderState @len=504
	//_Func: public void RenderState(RenderState &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void RenderState(); @loc=static @len=284 @rva=2105216
	//_Data: , Static Member, Type: const unsigned int, maxTextureNumber
	//_Data: this+0x0, Member, Type: void *[0x20], textures
	//_Data: this+0x100, Member, Type: enum CullMode, cullMode
	//_Data: this+0x104, Member, Type: enum BlendMode, blendMode
	//_Data: this+0x108, Member, Type: enum DepthMode, depthState
	//_Data: this+0x110, Member, Type: class Material *, material
	//_Data: this+0x118, Member, Type: class mat44f, projectionMatrix
	//_Data: this+0x158, Member, Type: class mat44f, viewMatrix
	//_Data: this+0x198, Member, Type: class mat44f, worldMatrix
	//_Data: this+0x1D8, Member, Type: class Shader *, shader
	//_Data: this+0x1E0, Member, Type: void *, currentRenderTarget
	//_Data: this+0x1E8, Member, Type: void *, currentDepth
	//_Data: this+0x1F0, Member, Type: bool, overrideNoMS
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(RenderState)==504),"bad size");
		static_assert((offsetof(RenderState,textures)==0x0),"bad off");
		static_assert((offsetof(RenderState,cullMode)==0x100),"bad off");
		static_assert((offsetof(RenderState,blendMode)==0x104),"bad off");
		static_assert((offsetof(RenderState,depthState)==0x108),"bad off");
		static_assert((offsetof(RenderState,material)==0x110),"bad off");
		static_assert((offsetof(RenderState,projectionMatrix)==0x118),"bad off");
		static_assert((offsetof(RenderState,viewMatrix)==0x158),"bad off");
		static_assert((offsetof(RenderState,worldMatrix)==0x198),"bad off");
		static_assert((offsetof(RenderState,shader)==0x1D8),"bad off");
		static_assert((offsetof(RenderState,currentRenderTarget)==0x1E0),"bad off");
		static_assert((offsetof(RenderState,currentDepth)==0x1E8),"bad off");
		static_assert((offsetof(RenderState,overrideNoMS)==0x1F0),"bad off");
	};
};

//UDT: struct TyreCompoundDef @len=1008
	//_Func: public void TyreCompoundDef(TyreCompoundDef & __that); @loc=static @len=380 @rva=2608864
	//_Func: public void TyreCompoundDef(); @loc=static @len=296 @rva=2609248
	//_Data: this+0x0, Member, Type: unsigned int, index
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x28, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, shortName
	//_Data: this+0x48, Member, Type: struct TyreModelData, modelData
	//_Data: this+0x2D8, Member, Type: struct TyreData, data
	//_Data: this+0x320, Member, Type: class BrushSlipProvider, slipProvider
	//_Data: this+0x358, Member, Type: float, pressureStatic
	//_Data: this+0x35C, Member, Type: struct TyrePatchData, thermalPatchData
	//_Data: this+0x370, Member, Type: class Curve, thermalPerformanceCurve
	//_Func: public void ~TyreCompoundDef(); @loc=static @len=188 @rva=2549984
	//_Func: public TyreCompoundDef & operator=(TyreCompoundDef &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(TyreCompoundDef)==1008),"bad size");
		static_assert((offsetof(TyreCompoundDef,index)==0x0),"bad off");
		static_assert((offsetof(TyreCompoundDef,name)==0x8),"bad off");
		static_assert((offsetof(TyreCompoundDef,shortName)==0x28),"bad off");
		static_assert((offsetof(TyreCompoundDef,modelData)==0x48),"bad off");
		static_assert((offsetof(TyreCompoundDef,data)==0x2D8),"bad off");
		static_assert((offsetof(TyreCompoundDef,slipProvider)==0x320),"bad off");
		static_assert((offsetof(TyreCompoundDef,pressureStatic)==0x358),"bad off");
		static_assert((offsetof(TyreCompoundDef,thermalPatchData)==0x35C),"bad off");
		static_assert((offsetof(TyreCompoundDef,thermalPerformanceCurve)==0x370),"bad off");
	};
};

//UDT: struct DynamicWingController @len=176
	//_Func: public void DynamicWingController(DynamicWingController & __that); @loc=static @len=111 @rva=2839024
	//_Func: public void DynamicWingController(CarPhysicsState * state, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & carUnixName, INIReader & ini, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section); @loc=static @len=118 @rva=2793072
	//_Func: public void DynamicWingController(Car * car, INIReader & ini, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section); @loc=static @len=120 @rva=2792944
	//_Func: public void ~DynamicWingController(); @loc=static @len=9 @rva=1147056
	//_Tag 12
	//_Tag 12
	//_Data: this+0x0, Member, Type: float, outputAngle
	//_Data: this+0x4, Member, Type: float, upLimit
	//_Data: this+0x8, Member, Type: float, downLimit
	//_Data: this+0xC, Member, Type: enum DynamicWingController::eCombinatorMode, combinatorMode
	//_Func: public void step(); @loc=static @len=115 @rva=2796144
	//_Data: this+0x10, Member, Type: enum DynamicWingController::eInputVar, inputVar
	//_Data: this+0x18, Member, Type: class Curve, lut
	//_Data: this+0x98, Member, Type: float, filter
	//_Data: this+0xA0, Member, Type: const struct CarPhysicsState *, state
	//_Data: this+0xA8, Member, Type: class Car *, car
	//_Func: protected float getInput(); @loc=static @len=480 @rva=2793200
	//_Func: protected void initCommon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & carUnixName, INIReader & ini, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section); @loc=static @len=2450 @rva=2793680
	//_Func: public DynamicWingController & operator=(DynamicWingController &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(DynamicWingController)==176),"bad size");
		static_assert((offsetof(DynamicWingController,outputAngle)==0x0),"bad off");
		static_assert((offsetof(DynamicWingController,upLimit)==0x4),"bad off");
		static_assert((offsetof(DynamicWingController,downLimit)==0x8),"bad off");
		static_assert((offsetof(DynamicWingController,combinatorMode)==0xC),"bad off");
		static_assert((offsetof(DynamicWingController,inputVar)==0x10),"bad off");
		static_assert((offsetof(DynamicWingController,lut)==0x18),"bad off");
		static_assert((offsetof(DynamicWingController,filter)==0x98),"bad off");
		static_assert((offsetof(DynamicWingController,state)==0xA0),"bad off");
		static_assert((offsetof(DynamicWingController,car)==0xA8),"bad off");
	};
};

//UDT: class UDPPacket @len=32
	//_Func: public void UDPPacket(UDPPacket & r); @loc=static @len=104 @rva=2479904
	//_Func: public void UDPPacket(UDPMessage & msg); @loc=static @len=158 @rva=2479744
	//_Func: public void UDPPacket(std::vector<unsigned char,std::allocator<unsigned char> > & idata); @loc=static @len=107 @rva=2480016
	//_Func: public void UDPPacket(); @loc=static @len=61 @rva=2480128
	//_Func: public void ~UDPPacket(); @loc=static @len=36 @rva=2480192
	//_Data: this+0x0, Member, Type: class IPAddress, targetIP
	//_Func: public UDPPacket & operator=(UDPPacket & r); @loc=static @len=76 @rva=2480240
	//_Func: public void writeString(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & st); @loc=static @len=210 @rva=2481520
	//_Func: public void writeString(std::basic_string<char,std::char_traits<char>,std::allocator<char> > & st); @loc=static @len=201 @rva=2481312
	//_Func: public void writeStringANSI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & st); @loc=static @len=93 @rva=2481744
	//_Func: public std::basic_string<char,std::char_traits<char>,std::allocator<char> > readString(); @loc=static @len=181 @rva=2480592
	//_Func: public std::basic_string<char,std::char_traits<char>,std::allocator<char> > readBigString(); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > readStringW(); @loc=static @len=209 @rva=2480784
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > readBigStringW(); @loc=static @len=209 @rva=2480368
	//_Func: public void write(void *  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void read(void *  _arg0, unsigned int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void seek(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getSize(); @loc=static @len=4 @rva=2480320
	//_Func: public void send(TCPSocket & sok); @loc=static @len=254 @rva=2481008
	//_Func: public void send(UDPSocket & sok); @loc=static @len=41 @rva=2481264
	//_Func: public bool isEOF(); @loc=static @len=18 @rva=2480336
	//_Func: public unsigned int getCurrentPos(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x10, Member, Type: unsigned char *, data
	//_Data: this+0x18, Member, Type: unsigned int, currentDataPos
	//_Data: this+0x1C, Member, Type: unsigned int, size
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(UDPPacket)==32),"bad size");
		static_assert((offsetof(UDPPacket,targetIP)==0x0),"bad off");
		static_assert((offsetof(UDPPacket,data)==0x10),"bad off");
		static_assert((offsetof(UDPPacket,currentDataPos)==0x18),"bad off");
		static_assert((offsetof(UDPPacket,size)==0x1C),"bad off");
	};
};

//UDT: struct CarPhysicsState @len=2928
	//_Func: public void CarPhysicsState(CarPhysicsState & __that); @loc=static @len=2617 @rva=1179376
	//_Func: public void CarPhysicsState(); @loc=static @len=1570 @rva=781920
	//_Data: this+0x0, Member, Type: unsigned char, physicsGUID
	//_Data: this+0x4, Member, Type: class mat44f, worldMatrix
	//_Data: this+0x44, Member, Type: class mat44f[0x4], suspensionMatrix
	//_Data: this+0x144, Member, Type: class mat44f[0x4], tyreMatrix
	//_Data: this+0x244, Member, Type: float, engineRPM
	//_Data: this+0x248, Member, Type: bool, isEngineLimiterOn
	//_Data: this+0x24C, Member, Type: float[0x4], wheelAngularSpeed
	//_Data: this+0x25C, Member, Type: float, steer
	//_Data: this+0x260, Member, Type: float, gas
	//_Data: this+0x264, Member, Type: float, brake
	//_Data: this+0x268, Member, Type: float, clutch
	//_Data: this+0x26C, Member, Type: int, gear
	//_Data: this+0x270, Member, Type: class Speed, speed
	//_Data: this+0x274, Member, Type: class vec3f, velocity
	//_Data: this+0x280, Member, Type: class vec3f, localVelocity
	//_Data: this+0x28C, Member, Type: class vec3f, localAngularVelocity
	//_Data: this+0x298, Member, Type: class vec3f, angularVelocity
	//_Data: this+0x2A4, Member, Type: float[0x4], slipAngle
	//_Data: this+0x2B4, Member, Type: float[0x4], slipRatio
	//_Data: this+0x2C4, Member, Type: float[0x4], tyreSlip
	//_Data: this+0x2D4, Member, Type: float[0x4], ndSlip
	//_Data: this+0x2E4, Member, Type: float[0x4], load
	//_Data: this+0x2F4, Member, Type: float[0x4], Dy
	//_Data: this+0x304, Member, Type: float[0x4], Mz
	//_Data: this+0x314, Member, Type: float[0x4], tyreDirtyLevel
	//_Data: this+0x328, Member, Type: struct SurfaceDef[0x4], tyreSurfaceDef
	//_Data: this+0x648, Member, Type: float, cgHeight
	//_Data: this+0x64C, Member, Type: class vec3f, accG
	//_Data: this+0x658, Member, Type: unsigned int, lapTime
	//_Data: this+0x65C, Member, Type: unsigned int, lastLap
	//_Data: this+0x660, Member, Type: unsigned int, bestLap
	//_Data: this+0x664, Member, Type: unsigned int, lapCount
	//_Data: this+0x668, Member, Type: float, lastFF_Pure
	//_Data: this+0x66C, Member, Type: float, lastFF_Final
	//_Data: this+0x670, Member, Type: struct SCarStateAero, aero
	//_Data: this+0x67C, Member, Type: class vec3f[0x4], tyreContactPoint
	//_Data: this+0x6AC, Member, Type: class vec3f[0x4], tyreContactNormal
	//_Data: this+0x6DC, Member, Type: float[0x4], camberRAD
	//_Data: this+0x6EC, Member, Type: float[0x4], tyreRadius
	//_Data: this+0x6FC, Member, Type: float[0x4], tyreLoadedRadius
	//_Data: this+0x70C, Member, Type: float[0x4], suspensionTravel
	//_Data: this+0x71C, Member, Type: float, normalizedSplinePosition
	//_Data: this+0x720, Member, Type: float, driftPoints
	//_Data: this+0x724, Member, Type: float, instantDrift
	//_Data: this+0x728, Member, Type: bool, isDriftValid
	//_Data: this+0x72C, Member, Type: int, driftComboCounter
	//_Data: this+0x730, Member, Type: bool, driftBonusOn
	//_Data: this+0x734, Member, Type: float, drivetrainSpeed
	//_Data: this+0x738, Member, Type: float, turboBoost
	//_Data: this+0x73C, Member, Type: float, performanceMeter
	//_Data: this+0x740, Member, Type: float, performanceMeterSpeedDiffMS
	//_Data: this+0x744, Member, Type: bool, isGearGrinding
	//_Data: this+0x748, Member, Type: float, bodyWorkVolume
	//_Data: this+0x74C, Member, Type: float[0x4], tyreVirtualKM
	//_Data: this+0x75C, Member, Type: float[0x5], damageZoneLevel
	//_Data: this+0x770, Member, Type: int, limiterRPM
	//_Data: this+0x774, Member, Type: class plane4f, groundPlane
	//_Data: this+0x788, Member, Type: double, timeStamp
	//_Data: this+0x790, Member, Type: float, airDensity
	//_Data: this+0x794, Member, Type: float, fuel
	//_Data: this+0x798, Member, Type: float, fuelLaps
	//_Data: this+0x79C, Member, Type: float[0x2], rideHeight
	//_Data: this+0x7A4, Member, Type: bool, isRetired
	//_Data: this+0x7A8, Member, Type: float, engineLifeLeft
	//_Data: this+0x7AC, Member, Type: float, turboBov
	//_Data: this+0x7B0, Member, Type: float, turboBoostLevel
	//_Data: this+0x7B4, Member, Type: float[0x4], tyreGrain
	//_Data: this+0x7C4, Member, Type: float[0x4], tyreBlister
	//_Data: this+0x7D4, Member, Type: struct DriverActionsState, actionsState
	//_Data: this+0x7D8, Member, Type: enum CarSetupState, setupState
	//_Data: this+0x7DC, Member, Type: float[0x4], tyreInflation
	//_Data: this+0x7EC, Member, Type: float, kersCharge
	//_Data: this+0x7F0, Member, Type: float, kersInput
	//_Data: this+0x7F4, Member, Type: float, gearRpmWindow
	//_Data: this+0x7F8, Member, Type: float[0x4], susDamage
	//_Data: this+0x808, Member, Type: float[0x4], tyreFlatSpot
	//_Data: this+0x818, Member, Type: float, water
	//_Data: this+0x81C, Member, Type: struct TyreThermalState[0x4], tyreThermalStates
	//_Data: this+0xB1C, Member, Type: float[0x4], discTemps
	//_Data: this+0xB2C, Member, Type: float[0x4], wear
	//_Data: this+0xB3C, Member, Type: float[0x4], wearMult
	//_Data: this+0xB4C, Member, Type: float, lockControlsTime
	//_Data: this+0xB50, Member, Type: float, kersCurrentKJ
	//_Data: this+0xB54, Member, Type: bool, kersIsCharging
	//_Data: this+0xB58, Member, Type: unsigned int, statusBytes
	//_Data: this+0xB5C, Member, Type: unsigned char, p2pStatus
	//_Data: this+0xB5D, Member, Type: unsigned char, p2pActivations
	//_Data: this+0xB60, Member, Type: float, antiSquat
	//_Data: this+0xB64, Member, Type: float[0x2], caster
	//_Func: public void ~CarPhysicsState(); @loc=static @len=12 @rva=783504
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CarPhysicsState)==2928),"bad size");
		static_assert((offsetof(CarPhysicsState,physicsGUID)==0x0),"bad off");
		static_assert((offsetof(CarPhysicsState,worldMatrix)==0x4),"bad off");
		static_assert((offsetof(CarPhysicsState,suspensionMatrix)==0x44),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreMatrix)==0x144),"bad off");
		static_assert((offsetof(CarPhysicsState,engineRPM)==0x244),"bad off");
		static_assert((offsetof(CarPhysicsState,isEngineLimiterOn)==0x248),"bad off");
		static_assert((offsetof(CarPhysicsState,wheelAngularSpeed)==0x24C),"bad off");
		static_assert((offsetof(CarPhysicsState,steer)==0x25C),"bad off");
		static_assert((offsetof(CarPhysicsState,gas)==0x260),"bad off");
		static_assert((offsetof(CarPhysicsState,brake)==0x264),"bad off");
		static_assert((offsetof(CarPhysicsState,clutch)==0x268),"bad off");
		static_assert((offsetof(CarPhysicsState,gear)==0x26C),"bad off");
		static_assert((offsetof(CarPhysicsState,speed)==0x270),"bad off");
		static_assert((offsetof(CarPhysicsState,velocity)==0x274),"bad off");
		static_assert((offsetof(CarPhysicsState,localVelocity)==0x280),"bad off");
		static_assert((offsetof(CarPhysicsState,localAngularVelocity)==0x28C),"bad off");
		static_assert((offsetof(CarPhysicsState,angularVelocity)==0x298),"bad off");
		static_assert((offsetof(CarPhysicsState,slipAngle)==0x2A4),"bad off");
		static_assert((offsetof(CarPhysicsState,slipRatio)==0x2B4),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreSlip)==0x2C4),"bad off");
		static_assert((offsetof(CarPhysicsState,ndSlip)==0x2D4),"bad off");
		static_assert((offsetof(CarPhysicsState,load)==0x2E4),"bad off");
		static_assert((offsetof(CarPhysicsState,Dy)==0x2F4),"bad off");
		static_assert((offsetof(CarPhysicsState,Mz)==0x304),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreDirtyLevel)==0x314),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreSurfaceDef)==0x328),"bad off");
		static_assert((offsetof(CarPhysicsState,cgHeight)==0x648),"bad off");
		static_assert((offsetof(CarPhysicsState,accG)==0x64C),"bad off");
		static_assert((offsetof(CarPhysicsState,lapTime)==0x658),"bad off");
		static_assert((offsetof(CarPhysicsState,lastLap)==0x65C),"bad off");
		static_assert((offsetof(CarPhysicsState,bestLap)==0x660),"bad off");
		static_assert((offsetof(CarPhysicsState,lapCount)==0x664),"bad off");
		static_assert((offsetof(CarPhysicsState,lastFF_Pure)==0x668),"bad off");
		static_assert((offsetof(CarPhysicsState,lastFF_Final)==0x66C),"bad off");
		static_assert((offsetof(CarPhysicsState,aero)==0x670),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreContactPoint)==0x67C),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreContactNormal)==0x6AC),"bad off");
		static_assert((offsetof(CarPhysicsState,camberRAD)==0x6DC),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreRadius)==0x6EC),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreLoadedRadius)==0x6FC),"bad off");
		static_assert((offsetof(CarPhysicsState,suspensionTravel)==0x70C),"bad off");
		static_assert((offsetof(CarPhysicsState,normalizedSplinePosition)==0x71C),"bad off");
		static_assert((offsetof(CarPhysicsState,driftPoints)==0x720),"bad off");
		static_assert((offsetof(CarPhysicsState,instantDrift)==0x724),"bad off");
		static_assert((offsetof(CarPhysicsState,isDriftValid)==0x728),"bad off");
		static_assert((offsetof(CarPhysicsState,driftComboCounter)==0x72C),"bad off");
		static_assert((offsetof(CarPhysicsState,driftBonusOn)==0x730),"bad off");
		static_assert((offsetof(CarPhysicsState,drivetrainSpeed)==0x734),"bad off");
		static_assert((offsetof(CarPhysicsState,turboBoost)==0x738),"bad off");
		static_assert((offsetof(CarPhysicsState,performanceMeter)==0x73C),"bad off");
		static_assert((offsetof(CarPhysicsState,performanceMeterSpeedDiffMS)==0x740),"bad off");
		static_assert((offsetof(CarPhysicsState,isGearGrinding)==0x744),"bad off");
		static_assert((offsetof(CarPhysicsState,bodyWorkVolume)==0x748),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreVirtualKM)==0x74C),"bad off");
		static_assert((offsetof(CarPhysicsState,damageZoneLevel)==0x75C),"bad off");
		static_assert((offsetof(CarPhysicsState,limiterRPM)==0x770),"bad off");
		static_assert((offsetof(CarPhysicsState,groundPlane)==0x774),"bad off");
		static_assert((offsetof(CarPhysicsState,timeStamp)==0x788),"bad off");
		static_assert((offsetof(CarPhysicsState,airDensity)==0x790),"bad off");
		static_assert((offsetof(CarPhysicsState,fuel)==0x794),"bad off");
		static_assert((offsetof(CarPhysicsState,fuelLaps)==0x798),"bad off");
		static_assert((offsetof(CarPhysicsState,rideHeight)==0x79C),"bad off");
		static_assert((offsetof(CarPhysicsState,isRetired)==0x7A4),"bad off");
		static_assert((offsetof(CarPhysicsState,engineLifeLeft)==0x7A8),"bad off");
		static_assert((offsetof(CarPhysicsState,turboBov)==0x7AC),"bad off");
		static_assert((offsetof(CarPhysicsState,turboBoostLevel)==0x7B0),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreGrain)==0x7B4),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreBlister)==0x7C4),"bad off");
		static_assert((offsetof(CarPhysicsState,actionsState)==0x7D4),"bad off");
		static_assert((offsetof(CarPhysicsState,setupState)==0x7D8),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreInflation)==0x7DC),"bad off");
		static_assert((offsetof(CarPhysicsState,kersCharge)==0x7EC),"bad off");
		static_assert((offsetof(CarPhysicsState,kersInput)==0x7F0),"bad off");
		static_assert((offsetof(CarPhysicsState,gearRpmWindow)==0x7F4),"bad off");
		static_assert((offsetof(CarPhysicsState,susDamage)==0x7F8),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreFlatSpot)==0x808),"bad off");
		static_assert((offsetof(CarPhysicsState,water)==0x818),"bad off");
		static_assert((offsetof(CarPhysicsState,tyreThermalStates)==0x81C),"bad off");
		static_assert((offsetof(CarPhysicsState,discTemps)==0xB1C),"bad off");
		static_assert((offsetof(CarPhysicsState,wear)==0xB2C),"bad off");
		static_assert((offsetof(CarPhysicsState,wearMult)==0xB3C),"bad off");
		static_assert((offsetof(CarPhysicsState,lockControlsTime)==0xB4C),"bad off");
		static_assert((offsetof(CarPhysicsState,kersCurrentKJ)==0xB50),"bad off");
		static_assert((offsetof(CarPhysicsState,kersIsCharging)==0xB54),"bad off");
		static_assert((offsetof(CarPhysicsState,statusBytes)==0xB58),"bad off");
		static_assert((offsetof(CarPhysicsState,p2pStatus)==0xB5C),"bad off");
		static_assert((offsetof(CarPhysicsState,p2pActivations)==0xB5D),"bad off");
		static_assert((offsetof(CarPhysicsState,antiSquat)==0xB60),"bad off");
		static_assert((offsetof(CarPhysicsState,caster)==0xB64),"bad off");
	};
};

//UDT: class IPhysicsCore @len=8 @vfcount=21
	//_VTable: 
	//_Func: public void resetCollisions(); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void initMultithreading(); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void step(float  _arg0); @intro @pure @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Func: public IRigidBody * createRigidBody(); @intro @pure @virtual vtpo=0 vfid=3 @loc=optimized @len=0 @rva=0
	//_Func: public void release(); @intro @pure @virtual vtpo=0 vfid=4 @loc=optimized @len=0 @rva=0
	//_Func: public IJoint * createDistanceJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2, vec3f &  _arg3); @intro @pure @virtual vtpo=0 vfid=5 @loc=optimized @len=0 @rva=0
	//_Func: public IJoint * createBumpJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2, float  _arg3, float  _arg4); @intro @pure @virtual vtpo=0 vfid=6 @loc=optimized @len=0 @rva=0
	//_Func: public ICollisionObject * createCollisionMesh(float *  _arg0, unsigned int  _arg1, unsigned short *  _arg2, int  _arg3, mat44f &  _arg4, IRigidBody *  _arg5, unsigned long  _arg6, unsigned long  _arg7, unsigned int  _arg8); @intro @pure @virtual vtpo=0 vfid=7 @loc=optimized @len=0 @rva=0
	//_Func: public RayCastHit rayCast(vec3f &  _arg0, vec3f &  _arg1, float  _arg2); @intro @pure @virtual vtpo=0 vfid=8 @loc=optimized @len=0 @rva=0
	//_Func: public void setCollisionCallback(ICollisionCallback *  _arg0); @intro @pure @virtual vtpo=0 vfid=9 @loc=optimized @len=0 @rva=0
	//_Func: public void reseatDistanceJointLocal(IJoint *  _arg0, vec3f &  _arg1, vec3f &  _arg2); @intro @pure @virtual vtpo=0 vfid=10 @loc=optimized @len=0 @rva=0
	//_Func: public void reseatDistanceJointLength(IJoint *  _arg0, float  _arg1); @intro @pure @virtual vtpo=0 vfid=11 @loc=optimized @len=0 @rva=0
	//_Func: public void setRigidBodyIterations(int  _arg0); @intro @pure @virtual vtpo=0 vfid=12 @loc=optimized @len=0 @rva=0
	//_Func: public IJoint * createSliderJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2); @intro @pure @virtual vtpo=0 vfid=13 @loc=optimized @len=0 @rva=0
	//_Func: public void setSliderAxis(IJoint *  _arg0, vec3f &  _arg1, vec3f &  _arg2); @intro @pure @virtual vtpo=0 vfid=14 @loc=optimized @len=0 @rva=0
	//_Func: public IJoint * createBallJoint(IRigidBody *  _arg0, IRigidBody *  _arg1, vec3f &  _arg2); @intro @pure @virtual vtpo=0 vfid=15 @loc=optimized @len=0 @rva=0
	//_Func: public IJoint * createFixedJoint(IRigidBody *  _arg0, IRigidBody *  _arg1); @intro @pure @virtual vtpo=0 vfid=16 @loc=optimized @len=0 @rva=0
	//_Func: public IRayCaster * createRayCaster(float  _arg0); @intro @pure @virtual vtpo=0 vfid=17 @loc=optimized @len=0 @rva=0
	//_Func: public CoreCPUTimes getCoreCPUTimes(); @intro @pure @virtual vtpo=0 vfid=18 @loc=optimized @len=0 @rva=0
	//_Func: public void setNoCollisionSteps(int  _arg0); @intro @pure @virtual vtpo=0 vfid=19 @loc=optimized @len=0 @rva=0
	//_Func: protected void ~IPhysicsCore(); @intro @virtual vtpo=0 vfid=20 @loc=static @len=11 @rva=2931728
	//_Func: public void IPhysicsCore(IPhysicsCore &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IPhysicsCore(); @loc=optimized @len=0 @rva=0
	//_Func: public IPhysicsCore & operator=(IPhysicsCore &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: protected void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=20 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(IPhysicsCore)==8),"bad size");
	};
};

//UDT: class Node @len=224 @vfcount=11
	//_VTable: 
	//_Tag 17
	//_Tag 17
	//_Tag 17
	//_Tag 17
	//_Func: public void Node(Node & node); @loc=static @len=329 @rva=2152896
	//_Func: public void Node(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & n); @loc=static @len=326 @rva=2153232
	//_Func: public void ~Node(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=237 @rva=2153568
	//_Data: this+0x8, Member, Type: class mat44f, matrix
	//_Data: this+0x48, Member, Type: class mat44f, matrixWS
	//_Data: this+0x88, Member, Type: bool, isWSIdentity
	//_Data: this+0x90, Member, Type: class std::vector<Node *,std::allocator<Node *> >, nodes
	//_Data: this+0xA8, Member, Type: class Node *, parent
	//_Data: this+0xB0, Member, Type: int, priority
	//_Data: this+0xB4, Member, Type: bool, needsMatrixWS
	//_Data: this+0xB8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Func: public Node * getRootNode(); @loc=optimized @len=0 @rva=0
	//_Func: public void collapse(); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f worldToLocal(vec3f & pos); @loc=static @len=407 @rva=2156336
	//_Func: public vec3f worldToLocalNormal(vec3f & pos); @loc=static @len=355 @rva=2156752
	//_Func: public vec3f localToWorld(vec3f & pos); @loc=static @len=58 @rva=2155056
	//_Func: public vec3f localToWorldNormal(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Node * getParent(); @loc=static @len=8 @rva=2154880
	//_Func: public void addChild(Node * n); @intro @virtual vtpo=0 vfid=1 @loc=static @len=66 @rva=2153856
	//_Func: public void compile(CompileContext * cc); @intro @virtual vtpo=0 vfid=2 @loc=static @len=110 @rva=2153936
	//_Func: public void render(RenderContext * rc); @intro @virtual vtpo=0 vfid=3 @loc=static @len=135 @rva=2155792
	//_Func: public void removeChild(Node * n); @intro @virtual vtpo=0 vfid=4 @loc=static @len=99 @rva=2155680
	//_Func: public Node * findChildByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & sname, bool recursive); @intro @virtual vtpo=0 vfid=5 @loc=static @len=268 @rva=2154048
	//_Func: public void findChildrenByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & nname, std::vector<Node *,std::allocator<Node *> > & result); @intro @virtual vtpo=0 vfid=6 @loc=static @len=254 @rva=2154320
	//_Func: public void findChildrenByPrefix(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & prefix, std::vector<Node *,std::allocator<Node *> > & result); @intro @virtual vtpo=0 vfid=7 @loc=static @len=302 @rva=2154576
	//_Func: public mat44f getWorldMatrix(); @intro @virtual vtpo=0 vfid=8 @loc=static @len=147 @rva=2154896
	//_Func: public void printTree(int ident); @intro @virtual vtpo=0 vfid=9 @loc=static @len=548 @rva=2155120
	//_Func: public void visit(std::function<bool __cdecl(Node *)> * onVisit, std::function<void __cdecl(Node *)> * onPostVisit); @loc=static @len=281 @rva=2156048
	//_Func: public void sortNodes(std::function<bool __cdecl(Node *,Node *)>  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public std::vector<Node *,std::allocator<Node *> > getNodesCopy(); @loc=optimized @len=0 @rva=0
	//_Func: public void renderAudio(); @intro @virtual vtpo=0 vfid=10 @loc=static @len=101 @rva=2155936
	//_Func: public void setActive(bool a); @loc=static @len=7 @rva=673008
	//_Func: public bool getActive(); @loc=static @len=8 @rva=418752
	//_Data: this+0xD8, Member, Type: bool, isActive
	//_Func: protected void fillHierarchy(std::vector<Node *,std::allocator<Node *> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Node & operator=(Node &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Node)==224),"bad size");
		static_assert((offsetof(Node,matrix)==0x8),"bad off");
		static_assert((offsetof(Node,matrixWS)==0x48),"bad off");
		static_assert((offsetof(Node,isWSIdentity)==0x88),"bad off");
		static_assert((offsetof(Node,nodes)==0x90),"bad off");
		static_assert((offsetof(Node,parent)==0xA8),"bad off");
		static_assert((offsetof(Node,priority)==0xB0),"bad off");
		static_assert((offsetof(Node,needsMatrixWS)==0xB4),"bad off");
		static_assert((offsetof(Node,name)==0xB8),"bad off");
		static_assert((offsetof(Node,isActive)==0xD8),"bad off");
	};
};

//UDT: class ksgui::Control @len=376 @vfcount=21
	//_VTable: 
	//_Func: public void Control(ksgui_Control &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Control(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui); @loc=static @len=942 @rva=2376016
	//_Func: public void ~Control(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=338 @rva=2376960
	//_Data: this+0x8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x28, Member, Type: struct ksgui::ksRect, rect
	//_Data: this+0x38, Member, Type: class vec4f, backColor
	//_Data: this+0x48, Member, Type: class vec4f, borderColor
	//_Data: this+0x58, Member, Type: class vec4f, foreColor
	//_Data: this+0x68, Member, Type: class vec4f, highlightTextColor
	//_Data: this+0x78, Member, Type: bool, highlight
	//_Data: this+0x7C, Member, Type: class vec4f, backTextureColor
	//_Data: this+0x90, Member, Type: class std::shared_ptr<Font>, font
	//_Data: this+0xA0, Member, Type: class std::vector<ksgui::Control *,std::allocator<ksgui::Control *> >, controls
	//_Data: this+0xB8, Member, Type: class ksgui::Control *, parent
	//_Data: this+0xC0, Member, Type: enum eFontAlign, fontAlign
	//_Data: this+0xC8, Member, Type: class Event<ksgui::OnControlClicked>, evClicked
	//_Data: this+0xE0, Member, Type: void *, tag
	//_Data: this+0xE8, Member, Type: class Texture, backTexture
	//_Data: this+0x110, Member, Type: bool, drawBorder
	//_Data: this+0x111, Member, Type: bool, drawBackground
	//_Data: this+0x114, Member, Type: float, repeatInterval
	//_Data: this+0x118, Member, Type: float, fontScale
	//_Func: public void setVisible(bool value); @loc=static @len=22 @rva=2380192
	//_Func: public bool isVisible(); @loc=static @len=8 @rva=349312
	//_Func: public bool hitTest(int x, int y); @intro @virtual vtpo=0 vfid=1 @loc=static @len=60 @rva=2377520
	//_Func: public void addControl(ksgui_Control * c); @intro @virtual vtpo=0 vfid=2 @loc=static @len=48 @rva=2377360
	//_Func: public void render(float dt); @intro @virtual vtpo=0 vfid=3 @loc=static @len=835 @rva=2378624
	//_Func: public vec2f localToWorld(vec2f & pos); @intro @virtual vtpo=0 vfid=4 @loc=static @len=95 @rva=2377584
	//_Func: public vec2f worldToLocal(vec2f & pos); @intro @virtual vtpo=0 vfid=5 @loc=static @len=95 @rva=2380464
	//_Func: public void getWorldRect(ksgui_ksRect & irect); @intro @virtual vtpo=0 vfid=6 @loc=static @len=106 @rva=2377408
	//_Func: public float getWidth(); @loc=static @len=11 @rva=220128
	//_Func: public float getHeight(); @loc=static @len=11 @rva=220080
	//_Func: public void setPosition(float x, float y); @loc=static @len=102 @rva=2379936
	//_Func: public void setSize(float w, float h); @intro @virtual vtpo=0 vfid=7 @loc=static @len=118 @rva=2380064
	//_Func: public void onVisibleChanged(bool newValue); @intro @virtual vtpo=0 vfid=8 @loc=static @len=86 @rva=221168
	//_Func: public bool isMousePressing(); @intro @virtual vtpo=0 vfid=9 @loc=static @len=8 @rva=220144
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @intro @virtual vtpo=0 vfid=10 @loc=static @len=279 @rva=2377872
	//_Func: public void onMouseUp(OnMouseUpEvent & message); @intro @virtual vtpo=0 vfid=11 @loc=static @len=125 @rva=2378384
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @intro @virtual vtpo=0 vfid=12 @loc=static @len=210 @rva=2378160
	//_Func: public void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message); @intro @virtual vtpo=0 vfid=13 @loc=static @len=110 @rva=2378512
	//_Func: public void onKeyChar(unsigned int key); @intro @virtual vtpo=0 vfid=14 @loc=static @len=84 @rva=2377680
	//_Func: public void onKeyDown(OnKeyEvent & message); @intro @virtual vtpo=0 vfid=15 @loc=static @len=85 @rva=2377776
	//_Func: public void setRepeatInterval(float i); @intro @virtual vtpo=0 vfid=16 @loc=static @len=9 @rva=2380048
	//_Func: public void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText); @intro @virtual vtpo=0 vfid=17 @loc=static @len=25 @rva=2011536
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & getText(); @intro @virtual vtpo=0 vfid=18 @loc=static @len=8 @rva=220112
	//_Func: public void scaleByMult(float value); @intro @virtual vtpo=0 vfid=19 @loc=static @len=332 @rva=2379600
	//_Func: public void scaleByMult(); @intro @virtual vtpo=0 vfid=20 @loc=static @len=18 @rva=2386448
	//_Func: public void resetBaseRect(); @loc=static @len=123 @rva=2379472
	//_Data: this+0x120, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, text
	//_Data: this+0x140, Member, Type: bool, isHighlight
	//_Data: this+0x148, Member, Type: class ksgui::GUI *, gui
	//_Data: this+0x150, Member, Type: bool, visible
	//_Data: this+0x151, Member, Type: bool, isMouseDown
	//_Data: this+0x154, Member, Type: float, intervalCounter
	//_Data: this+0x158, Member, Type: bool, isRepeating
	//_Data: this+0x15C, Member, Type: float, scaleMult
	//_Data: this+0x160, Member, Type: class GLRenderer *, controlGLR
	//_Func: protected void stepRepeatInterval(float dt); @loc=static @len=236 @rva=2380224
	//_Data: this+0x168, Member, Type: struct ksgui::ksRect, rectBase
	//_Func: private float getWidthBase(); @loc=optimized @len=0 @rva=0
	//_Func: private float getHeightBase(); @loc=optimized @len=0 @rva=0
	//_Func: public ksgui_Control & operator=(ksgui_Control &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Control)==376),"bad size");
		static_assert((offsetof(ksgui_Control,name)==0x8),"bad off");
		static_assert((offsetof(ksgui_Control,rect)==0x28),"bad off");
		static_assert((offsetof(ksgui_Control,backColor)==0x38),"bad off");
		static_assert((offsetof(ksgui_Control,borderColor)==0x48),"bad off");
		static_assert((offsetof(ksgui_Control,foreColor)==0x58),"bad off");
		static_assert((offsetof(ksgui_Control,highlightTextColor)==0x68),"bad off");
		static_assert((offsetof(ksgui_Control,highlight)==0x78),"bad off");
		static_assert((offsetof(ksgui_Control,backTextureColor)==0x7C),"bad off");
		static_assert((offsetof(ksgui_Control,font)==0x90),"bad off");
		static_assert((offsetof(ksgui_Control,controls)==0xA0),"bad off");
		static_assert((offsetof(ksgui_Control,parent)==0xB8),"bad off");
		static_assert((offsetof(ksgui_Control,fontAlign)==0xC0),"bad off");
		static_assert((offsetof(ksgui_Control,evClicked)==0xC8),"bad off");
		static_assert((offsetof(ksgui_Control,tag)==0xE0),"bad off");
		static_assert((offsetof(ksgui_Control,backTexture)==0xE8),"bad off");
		static_assert((offsetof(ksgui_Control,drawBorder)==0x110),"bad off");
		static_assert((offsetof(ksgui_Control,drawBackground)==0x111),"bad off");
		static_assert((offsetof(ksgui_Control,repeatInterval)==0x114),"bad off");
		static_assert((offsetof(ksgui_Control,fontScale)==0x118),"bad off");
		static_assert((offsetof(ksgui_Control,text)==0x120),"bad off");
		static_assert((offsetof(ksgui_Control,isHighlight)==0x140),"bad off");
		static_assert((offsetof(ksgui_Control,gui)==0x148),"bad off");
		static_assert((offsetof(ksgui_Control,visible)==0x150),"bad off");
		static_assert((offsetof(ksgui_Control,isMouseDown)==0x151),"bad off");
		static_assert((offsetof(ksgui_Control,intervalCounter)==0x154),"bad off");
		static_assert((offsetof(ksgui_Control,isRepeating)==0x158),"bad off");
		static_assert((offsetof(ksgui_Control,scaleMult)==0x15C),"bad off");
		static_assert((offsetof(ksgui_Control,controlGLR)==0x160),"bad off");
		static_assert((offsetof(ksgui_Control,rectBase)==0x168),"bad off");
	};
};

//UDT: class RenderWindow @len=240 @vfcount=1
	//_VTable: 
	//_Func: public void RenderWindow(RenderWindow &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void RenderWindow(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, VideoSettings & videoSettings); @loc=static @len=403 @rva=2086464
	//_Func: public void ~RenderWindow(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=121 @rva=2086880
	//_Data: this+0x8, Member, Type: void *, hWnd
	//_Data: this+0x10, Member, Type: class Event<OnMouseDownEvent>, evOnMouseDown
	//_Data: this+0x28, Member, Type: class Event<OnMouseWheelMovedEvent>, evOnMouseWheelMoved
	//_Data: this+0x40, Member, Type: class Event<OnMouseUpEvent>, evOnMouseUp
	//_Data: this+0x58, Member, Type: class Event<OnMouseMoveEvent>, evOnMouseMove
	//_Data: this+0x70, Member, Type: class Event<OnWindowResizeEvent>, evOnWindowResize
	//_Data: this+0x88, Member, Type: class Event<OnKeyCharEvent>, evOnKeyChar
	//_Data: this+0xA0, Member, Type: class Event<OnKeyEvent>, evOnKeyDown
	//_Data: this+0xB8, Member, Type: class Event<OnWindowClosedEvent>, evOnWindowClosed
	//_Data: this+0xD0, Member, Type: class Event<OnKeyEvent>, evOnKeyUp
	//_Data: this+0xE8, Member, Type: bool, disableEventsWithMouseHidden
	//_Func: public bool step(); @loc=static @len=23 @rva=2088352
	//_Func: public bool hasFocus(); @loc=static @len=23 @rva=2087216
	//_Func: public void setFocus(); @loc=static @len=9 @rva=2088336
	//_Func: public RenderWindow & operator=(RenderWindow &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(RenderWindow)==240),"bad size");
		static_assert((offsetof(RenderWindow,hWnd)==0x8),"bad off");
		static_assert((offsetof(RenderWindow,evOnMouseDown)==0x10),"bad off");
		static_assert((offsetof(RenderWindow,evOnMouseWheelMoved)==0x28),"bad off");
		static_assert((offsetof(RenderWindow,evOnMouseUp)==0x40),"bad off");
		static_assert((offsetof(RenderWindow,evOnMouseMove)==0x58),"bad off");
		static_assert((offsetof(RenderWindow,evOnWindowResize)==0x70),"bad off");
		static_assert((offsetof(RenderWindow,evOnKeyChar)==0x88),"bad off");
		static_assert((offsetof(RenderWindow,evOnKeyDown)==0xA0),"bad off");
		static_assert((offsetof(RenderWindow,evOnWindowClosed)==0xB8),"bad off");
		static_assert((offsetof(RenderWindow,evOnKeyUp)==0xD0),"bad off");
		static_assert((offsetof(RenderWindow,disableEventsWithMouseHidden)==0xE8),"bad off");
	};
};

//UDT: struct DrivetrainControllers @len=32
	//_Data: this+0x0, Member, Type: class std::unique_ptr<DynamicController,std::default_delete<DynamicController> >, awdFrontShare
	//_Data: this+0x8, Member, Type: class std::unique_ptr<DynamicController,std::default_delete<DynamicController> >, awdCenterLock
	//_Data: this+0x10, Member, Type: class std::unique_ptr<DynamicController,std::default_delete<DynamicController> >, singleDiffLock
	//_Data: this+0x18, Member, Type: class std::unique_ptr<DynamicController,std::default_delete<DynamicController> >, awd2
	//_Func: public void DrivetrainControllers(DrivetrainControllers &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void DrivetrainControllers(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~DrivetrainControllers(); @loc=static @len=123 @rva=2515184
	//_Func: public DrivetrainControllers & operator=(DrivetrainControllers &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct DrivetrainControllers {
public:
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > awdFrontShare;
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > awdCenterLock;
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > singleDiffLock;
	std::unique_ptr<DynamicController,std::default_delete<DynamicController> > awd2;
	inline DrivetrainControllers()  { }
	inline void dtor() { typedef void (*_fpt)(DrivetrainControllers *pthis); _fpt _f=(_fpt)_drva(2515184); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(DrivetrainControllers)==32),"bad size");
		static_assert((offsetof(DrivetrainControllers,awdFrontShare)==0x0),"bad off");
		static_assert((offsetof(DrivetrainControllers,awdCenterLock)==0x8),"bad off");
		static_assert((offsetof(DrivetrainControllers,singleDiffLock)==0x10),"bad off");
		static_assert((offsetof(DrivetrainControllers,awd2)==0x18),"bad off");
	};
};

//UDT: struct SteerBrake @len=48
	//_Data: this+0x0, Member, Type: bool, isActive
	//_Data: this+0x8, Member, Type: class DynamicController, controller
	//_Func: public void SteerBrake(SteerBrake &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SteerBrake(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~SteerBrake(); @loc=static @len=9 @rva=2549856
	//_Func: public SteerBrake & operator=(SteerBrake &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct SteerBrake {
public:
	bool isActive;
	DynamicController controller;
	inline SteerBrake()  { }
	inline void dtor() { typedef void (*_fpt)(SteerBrake *pthis); _fpt _f=(_fpt)_drva(2549856); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(SteerBrake)==48),"bad size");
		static_assert((offsetof(SteerBrake,isActive)==0x0),"bad off");
		static_assert((offsetof(SteerBrake,controller)==0x8),"bad off");
	};
};

//UDT: struct ACClient::ClientSessionTransition @len=216
	//_Data: this+0x0, Member, Type: bool, isTransitioning
	//_Data: this+0x8, Member, Type: class UDPPacket, sessionPacket
	//_Data: this+0x28, Member, Type: struct RemoteSessionResume, sessionResume
	//_Func: public void ClientSessionTransition(ACClient_ClientSessionTransition &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ClientSessionTransition(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~ClientSessionTransition(); @loc=static @len=66 @rva=246384
	//_Func: public ACClient_ClientSessionTransition & operator=(ACClient_ClientSessionTransition &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct ACClient_ClientSessionTransition {
public:
	bool isTransitioning;
	UDPPacket sessionPacket;
	RemoteSessionResume sessionResume;
	inline ACClient_ClientSessionTransition()  { }
	inline void dtor() { typedef void (*_fpt)(ACClient_ClientSessionTransition *pthis); _fpt _f=(_fpt)_drva(246384); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(ACClient_ClientSessionTransition)==216),"bad size");
		static_assert((offsetof(ACClient_ClientSessionTransition,isTransitioning)==0x0),"bad off");
		static_assert((offsetof(ACClient_ClientSessionTransition,sessionPacket)==0x8),"bad off");
		static_assert((offsetof(ACClient_ClientSessionTransition,sessionResume)==0x28),"bad off");
	};
};

//UDT: struct SteeringSystem @len=64
	//_Func: public void ~SteeringSystem(); @loc=static @len=9 @rva=2864640
	//_Data: this+0x0, Member, Type: float, linearRatio
	//_Func: public void init(Car * car); @loc=static @len=221 @rva=2851024
	//_Func: public void step(float dt); @loc=static @len=164 @rva=2851248
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: bool, has4ws
	//_Data: this+0x18, Member, Type: class DynamicController, ctrl4ws
	//_Func: public void SteeringSystem(SteeringSystem &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void SteeringSystem(); @loc=optimized @len=0 @rva=0
	//_Func: public SteeringSystem & operator=(SteeringSystem &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(SteeringSystem)==64),"bad size");
		static_assert((offsetof(SteeringSystem,linearRatio)==0x0),"bad off");
		static_assert((offsetof(SteeringSystem,car)==0x8),"bad off");
		static_assert((offsetof(SteeringSystem,has4ws)==0x10),"bad off");
		static_assert((offsetof(SteeringSystem,ctrl4ws)==0x18),"bad off");
	};
};

//UDT: struct ERSPowerController @len=72
	//_Data: this+0x0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, name
	//_Data: this+0x20, Member, Type: class DynamicController, ctrl
	//_Func: public void ERSPowerController(ERSPowerController & __that); @loc=static @len=102 @rva=2692640
	//_Func: public void ERSPowerController(); @loc=optimized @len=0 @rva=0
	//_Func: public void ~ERSPowerController(); @loc=static @len=70 @rva=2693104
	//_Func: public ERSPowerController & operator=(ERSPowerController &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct ERSPowerController {
public:
	std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > name;
	DynamicController ctrl;
	inline ERSPowerController()  { }
	inline void ctor(ERSPowerController & __that) { typedef void (*_fpt)(ERSPowerController *pthis, ERSPowerController &); _fpt _f=(_fpt)_drva(2692640); _f(this, __that); }
	inline void dtor() { typedef void (*_fpt)(ERSPowerController *pthis); _fpt _f=(_fpt)_drva(2693104); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(ERSPowerController)==72),"bad size");
		static_assert((offsetof(ERSPowerController,name)==0x0),"bad off");
		static_assert((offsetof(ERSPowerController,ctrl)==0x20),"bad off");
	};
};

//UDT: struct TurboDynamicController @len=56
	//_Data: this+0x0, Member, Type: class Turbo *, turbo
	//_Data: this+0x8, Member, Type: class DynamicController, controller
	//_Data: this+0x30, Member, Type: bool, isWastegate
	//_Func: public void TurboDynamicController(TurboDynamicController &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TurboDynamicController(); @loc=static @len=38 @rva=2643056
	//_Func: public void ~TurboDynamicController(); @loc=static @len=9 @rva=2549856
	//_Func: public TurboDynamicController & operator=(TurboDynamicController &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

struct TurboDynamicController {
public:
	Turbo * turbo;
	DynamicController controller;
	bool isWastegate;
	inline TurboDynamicController()  { }
	inline void ctor() { typedef void (*_fpt)(TurboDynamicController *pthis); _fpt _f=(_fpt)_drva(2643056); _f(this); }
	inline void dtor() { typedef void (*_fpt)(TurboDynamicController *pthis); _fpt _f=(_fpt)_drva(2549856); _f(this); }
	inline void _guard_obj() {
		static_assert((sizeof(TurboDynamicController)==56),"bad size");
		static_assert((offsetof(TurboDynamicController,turbo)==0x0),"bad off");
		static_assert((offsetof(TurboDynamicController,controller)==0x8),"bad off");
		static_assert((offsetof(TurboDynamicController,isWastegate)==0x30),"bad off");
	};
};

//UDT: class ICarPhysicsStateProvider @len=8 @vfcount=3
	//_VTable: 
	//_Func: public void ~ICarPhysicsStateProvider(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=783520
	//_Func: public void getPhysicsState(CarPhysicsState &  _arg0); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: public void getWingState(std::vector<WingState,std::allocator<WingState> > &  _arg0); @intro @pure @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Func: public void ICarPhysicsStateProvider(ICarPhysicsStateProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ICarPhysicsStateProvider(); @loc=optimized @len=0 @rva=0
	//_Func: public ICarPhysicsStateProvider & operator=(ICarPhysicsStateProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

class ICarPhysicsStateProvider {
public:
	inline ICarPhysicsStateProvider()  { }
	virtual ~ICarPhysicsStateProvider();
	inline void dtor() { typedef void (*_fpt)(ICarPhysicsStateProvider *pthis); _fpt _f=(_fpt)_drva(783520); _f(this); }
	virtual void getPhysicsState_vf1(CarPhysicsState &  _arg0) = 0;
	inline void getPhysicsState(CarPhysicsState &  _arg0) { return getPhysicsState_vf1( _arg0); }
	virtual void getWingState_vf2(std::vector<WingState,std::allocator<WingState> > &  _arg0) = 0;
	inline void getWingState(std::vector<WingState,std::allocator<WingState> > &  _arg0) { return getWingState_vf2( _arg0); }
	inline void _guard_obj() {
		static_assert((sizeof(ICarPhysicsStateProvider)==8),"bad size");
	};
};

//UDT: struct AntirollBar @len=72
	//_Func: public void ~AntirollBar(); @loc=static @len=9 @rva=2864640
	//_Data: this+0x0, Member, Type: class IRigidBody *, carBody
	//_Data: this+0x8, Member, Type: class ISuspension *[0x2], hubs
	//_Data: this+0x18, Member, Type: class DynamicController, ctrl
	//_Data: this+0x40, Member, Type: float, k
	//_Func: public void init(IRigidBody * cb, ISuspension * * sus); @loc=static @len=26 @rva=2864656
	//_Func: public void step(float dt); @loc=static @len=699 @rva=2864704
	//_Func: public void AntirollBar(AntirollBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void AntirollBar(); @loc=static @len=41 @rva=2538800
	//_Func: public AntirollBar & operator=(AntirollBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(AntirollBar)==72),"bad size");
		static_assert((offsetof(AntirollBar,carBody)==0x0),"bad off");
		static_assert((offsetof(AntirollBar,hubs)==0x8),"bad off");
		static_assert((offsetof(AntirollBar,ctrl)==0x18),"bad off");
		static_assert((offsetof(AntirollBar,k)==0x40),"bad off");
	};
};

//UDT: class ksgui::Label @len=384 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void Label(ksgui_Label &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Label(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui); @loc=static @len=521 @rva=2403696
	//_Func: public void ~Label(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: unsigned int, maxNumberOfCharDisplayed
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=394 @rva=2404288
	//_Func: protected void scaleByMult(float value); @virtual vtpo=0 vfid=19 @loc=static @len=5 @rva=2404688
	//_Func: public ksgui_Label & operator=(ksgui_Label &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Label)==384),"bad size");
		static_assert((offsetof(ksgui_Label,maxNumberOfCharDisplayed)==0x178),"bad off");
	};
};

//UDT: class ksgui::ProgressBar @len=392 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void ProgressBar(ksgui_ProgressBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ProgressBar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui); @loc=static @len=184 @rva=2416368
	//_Func: public void ~ProgressBar(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: float, maxValue
	//_Data: this+0x17C, Member, Type: float, minValue
	//_Data: this+0x180, Member, Type: float, value
	//_Data: this+0x184, Member, Type: bool, isVertical
	//_Data: this+0x185, Member, Type: bool, isInverted
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=59 @rva=2416624
	//_Func: protected void renderVertical(float dt); @loc=static @len=329 @rva=2417120
	//_Func: protected void renderHorizontal(float dt); @loc=static @len=420 @rva=2416688
	//_Func: public ksgui_ProgressBar & operator=(ksgui_ProgressBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ProgressBar)==392),"bad size");
		static_assert((offsetof(ksgui_ProgressBar,maxValue)==0x178),"bad off");
		static_assert((offsetof(ksgui_ProgressBar,minValue)==0x17C),"bad off");
		static_assert((offsetof(ksgui_ProgressBar,value)==0x180),"bad off");
		static_assert((offsetof(ksgui_ProgressBar,isVertical)==0x184),"bad off");
		static_assert((offsetof(ksgui_ProgressBar,isInverted)==0x185),"bad off");
	};
};

//UDT: class ksgui::ConnectedLabel @len=472 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void ConnectedLabel(ksgui_ConnectedLabel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ConnectedLabel(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui, int * var); @loc=static @len=155 @rva=4515840
	//_Func: public void ConnectedLabel(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui, float * var); @loc=static @len=151 @rva=4516000
	//_Func: public void ~ConnectedLabel(); @virtual vtpo=0 vfid=0 @loc=static @len=137 @rva=4516160
	//_Data: this+0x178, Member, Type: enum ksgui::VariableConnection, variableConnection
	//_Data: this+0x180, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, label
	//_Data: this+0x1A0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, suffix
	//_Data: this+0x1C0, Member, Type: int, precision
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=592 @rva=4516784
	//_Data: this+0x1C8, Member, Type: float *, fValue
	//_Data: this+0x1D0, Member, Type: int *, iValue
	//_Func: protected void init(); @loc=static @len=429 @rva=4516352
	//_Func: public ksgui_ConnectedLabel & operator=(ksgui_ConnectedLabel &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ConnectedLabel)==472),"bad size");
		static_assert((offsetof(ksgui_ConnectedLabel,variableConnection)==0x178),"bad off");
		static_assert((offsetof(ksgui_ConnectedLabel,label)==0x180),"bad off");
		static_assert((offsetof(ksgui_ConnectedLabel,suffix)==0x1A0),"bad off");
		static_assert((offsetof(ksgui_ConnectedLabel,precision)==0x1C0),"bad off");
		static_assert((offsetof(ksgui_ConnectedLabel,fValue)==0x1C8),"bad off");
		static_assert((offsetof(ksgui_ConnectedLabel,iValue)==0x1D0),"bad off");
	};
};

//UDT: class KeyboardManager @len=48 @vfcount=1
	//_VTable: 
	//_Func: public void KeyboardManager(KeyboardManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void KeyboardManager(RenderWindow & arenderWindow); @loc=static @len=69 @rva=2372096
	//_Func: public void ~KeyboardManager(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=62 @rva=2372176
	//_Func: public void registerEventHandlers(); @loc=static @len=127 @rva=2375440
	//_Func: public void onKeyDownEvent(OnKeyEvent & message); @loc=static @len=124 @rva=2374432
	//_Func: public void onKeyPressEvent(OnKeyCharEvent & message); @loc=static @len=126 @rva=2374560
	//_Func: public void addGodListener(IKeyEventListener * l); @loc=static @len=28 @rva=2373984
	//_Func: public void removeListener(IKeyEventListener *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void clearListeners(); @loc=optimized @len=0 @rva=0
	//_Func: public void getFocus(IKeyEventListener * l); @loc=static @len=5 @rva=2944432
	//_Func: public void releaseFocus(IKeyEventListener * l); @loc=static @len=15 @rva=2375568
	//_Data: this+0x8, Member, Type: class RenderWindow &, renderWindow
	//_Data: this+0x10, Member, Type: class std::vector<IKeyEventListener *,std::allocator<IKeyEventListener *> >, listeners
	//_Data: this+0x28, Member, Type: class IKeyEventListener *, focusListener
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(KeyboardManager)==48),"bad size");
		static_assert((offsetof(KeyboardManager,listeners)==0x10),"bad off");
		static_assert((offsetof(KeyboardManager,focusListener)==0x28),"bad off");
	};
};

//UDT: class Kers @len=248 @vfcount=2
	//_Base: class ITorqueGenerator @off=0 @len=8
	//_Func: public void ~Kers(); @virtual vtpo=0 vfid=0 @loc=static @len=71 @rva=2847200
	//_Data: this+0x8, Member, Type: enum KersAttachment, attachment
	//_Func: public void init(Car * car); @loc=static @len=2718 @rva=2847584
	//_Func: public void step(float dt); @loc=static @len=694 @rva=2850320
	//_Func: public float getOutputTorque(); @virtual vtpo=0 vfid=1 @loc=static @len=166 @rva=2847408
	//_Func: public float getInput(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isPresent(); @loc=static @len=5 @rva=888160
	//_Func: public float getCharge(); @loc=optimized @len=0 @rva=0
	//_Func: public void reset(); @loc=static @len=15 @rva=2850304
	//_Func: public float getCurrentJ(); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaxJ(); @loc=optimized @len=0 @rva=0
	//_Func: public float getDischargeTimeS(); @loc=static @len=22 @rva=2847376
	//_Data: this+0x10, Member, Type: class Car *, car
	//_Data: this+0x18, Member, Type: bool, present
	//_Data: this+0x1C, Member, Type: float, input
	//_Data: this+0x20, Member, Type: float, brakeForMaxCharge
	//_Data: this+0x24, Member, Type: float, charge
	//_Data: this+0x28, Member, Type: float, chargeK
	//_Data: this+0x2C, Member, Type: float, dischargeK
	//_Data: this+0x30, Member, Type: float, angularVelocity
	//_Data: this+0x34, Member, Type: float, negativeInputChargeK
	//_Data: this+0x38, Member, Type: bool, hasButtonOverride
	//_Data: this+0x3C, Member, Type: float, currentJ
	//_Data: this+0x40, Member, Type: float, maxJ
	//_Data: this+0x48, Member, Type: class Curve, torqueLUT
	//_Data: this+0xC8, Member, Type: class DynamicController, controller
	//_Data: this+0xF0, Member, Type: bool, hasController
	//_Func: public void Kers(Kers &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Kers(); @loc=optimized @len=0 @rva=0
	//_Func: public Kers & operator=(Kers &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Kers)==248),"bad size");
		static_assert((offsetof(Kers,attachment)==0x8),"bad off");
		static_assert((offsetof(Kers,car)==0x10),"bad off");
		static_assert((offsetof(Kers,present)==0x18),"bad off");
		static_assert((offsetof(Kers,input)==0x1C),"bad off");
		static_assert((offsetof(Kers,brakeForMaxCharge)==0x20),"bad off");
		static_assert((offsetof(Kers,charge)==0x24),"bad off");
		static_assert((offsetof(Kers,chargeK)==0x28),"bad off");
		static_assert((offsetof(Kers,dischargeK)==0x2C),"bad off");
		static_assert((offsetof(Kers,angularVelocity)==0x30),"bad off");
		static_assert((offsetof(Kers,negativeInputChargeK)==0x34),"bad off");
		static_assert((offsetof(Kers,hasButtonOverride)==0x38),"bad off");
		static_assert((offsetof(Kers,currentJ)==0x3C),"bad off");
		static_assert((offsetof(Kers,maxJ)==0x40),"bad off");
		static_assert((offsetof(Kers,torqueLUT)==0x48),"bad off");
		static_assert((offsetof(Kers,controller)==0xC8),"bad off");
		static_assert((offsetof(Kers,hasController)==0xF0),"bad off");
	};
};

//UDT: class ksgui::TextBox @len=400 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void TextBox(ksgui_TextBox &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TextBox(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui, eFontType fontType); @loc=static @len=309 @rva=2417456
	//_Func: public void ~TextBox(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setFormattedText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aText); @loc=static @len=1004 @rva=2418400
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=305 @rva=2418080
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=16 @rva=2418064
	//_Data: this+0x178, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, textLines
	//_Func: public ksgui_TextBox & operator=(ksgui_TextBox &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_TextBox)==400),"bad size");
		static_assert((offsetof(ksgui_TextBox,textLines)==0x178),"bad off");
	};
};

//UDT: class ksgui::PopOver @len=392 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void PopOver(ksgui_PopOver &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void PopOver(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * aGui); @loc=static @len=733 @rva=2447104
	//_Func: public void ~PopOver(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=79 @rva=2447904
	//_Func: public void renderPopOver(float dt); @loc=static @len=572 @rva=2447984
	//_Func: public void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aText); @loc=static @len=163 @rva=2448672
	//_Func: public void setLabelTitle(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aTitle); @loc=static @len=106 @rva=2448560
	//_Data: this+0x178, Member, Type: class ksgui::TextBox *, textBox
	//_Data: this+0x180, Member, Type: class ksgui::Label *, title
	//_Func: public ksgui_PopOver & operator=(ksgui_PopOver &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_PopOver)==392),"bad size");
		static_assert((offsetof(ksgui_PopOver,textBox)==0x178),"bad off");
		static_assert((offsetof(ksgui_PopOver,title)==0x180),"bad off");
	};
};

//UDT: class ksgui::CheckBox @len=424 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void CheckBox(ksgui_CheckBox &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CheckBox(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui); @loc=static @len=551 @rva=2402416
	//_Func: public void ~CheckBox(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: class Event<ksgui::OnCheckBoxChanged>, evOnCheckBoxChanged
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=326 @rva=2403312
	//_Func: public void setCheck(bool aCheck); @loc=static @len=41 @rva=2403648
	//_Func: public bool isChecked(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x190, Member, Type: bool, boxValue
	//_Data: this+0x198, Member, Type: class ksgui::Control *, box
	//_Data: this+0x1A0, Member, Type: class ksgui::Control *, label
	//_Func: public ksgui_CheckBox & operator=(ksgui_CheckBox &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_CheckBox)==424),"bad size");
		static_assert((offsetof(ksgui_CheckBox,evOnCheckBoxChanged)==0x178),"bad off");
		static_assert((offsetof(ksgui_CheckBox,boxValue)==0x190),"bad off");
		static_assert((offsetof(ksgui_CheckBox,box)==0x198),"bad off");
		static_assert((offsetof(ksgui_CheckBox,label)==0x1A0),"bad off");
	};
};

//UDT: class ksgui::ListBoxRow @len=424 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void ListBoxRow(ksgui_ListBoxRow &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ListBoxRow(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, unsigned int numberOfColumns, ksgui_GUI * aGui); @loc=static @len=1007 @rva=2448848
	//_Func: public void ~ListBoxRow(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: class vec4f, rowBackground
	//_Data: this+0x188, Member, Type: int, id
	//_Data: this+0x18C, Member, Type: bool, drawRowBackground
	//_Func: public unsigned int getColumnCount(); @loc=optimized @len=0 @rva=0
	//_Func: public void setSize(float width, float height); @virtual vtpo=0 vfid=7 @loc=static @len=216 @rva=2451392
	//_Func: public void setPosition(float x, float y); @loc=static @len=202 @rva=2450864
	//_Func: public void setRow(ksgui_ListBoxRowData * data); @loc=static @len=308 @rva=2451072
	//_Func: public void setColor(vec4f  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setColor(vec4f  _arg0, unsigned int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void setFontSize(float size); @loc=static @len=75 @rva=2450784
	//_Func: public void clear(); @loc=static @len=225 @rva=2449984
	//_Func: public void setFontAlignment(unsigned int  _arg0, eFontAlign  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void setFontAlignment(eFontAlign alignment); @loc=static @len=70 @rva=2450704
	//_Func: public ksgui_Label * getColumn(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=332 @rva=2450368
	//_Data: this+0x190, Member, Type: class std::vector<ksgui::Label *,std::allocator<ksgui::Label *> >, columns
	//_Func: public ksgui_ListBoxRow & operator=(ksgui_ListBoxRow &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ListBoxRow)==424),"bad size");
		static_assert((offsetof(ksgui_ListBoxRow,rowBackground)==0x178),"bad off");
		static_assert((offsetof(ksgui_ListBoxRow,id)==0x188),"bad off");
		static_assert((offsetof(ksgui_ListBoxRow,drawRowBackground)==0x18C),"bad off");
		static_assert((offsetof(ksgui_ListBoxRow,columns)==0x190),"bad off");
	};
};

//UDT: class ksgui::CustomSpinner @len=472 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void CustomSpinner(ksgui_CustomSpinner &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CustomSpinner(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * aGui, CustomSpinnerMode aMode); @loc=static @len=1417 @rva=2419408
	//_Func: public void ~CustomSpinner(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: bool, pressToIncrement
	//_Data: this+0x180, Member, Type: class ksgui::Control *, leftButton
	//_Data: this+0x188, Member, Type: class ksgui::Control *, rightButton
	//_Data: this+0x190, Member, Type: class ksgui::Control *, upButton
	//_Data: this+0x198, Member, Type: class ksgui::Control *, downButton
	//_Data: this+0x1A0, Member, Type: class ksgui::Label *, title
	//_Data: this+0x1A8, Member, Type: class Texture, bar
	//_Data: this+0x1D0, Member, Type: float, barPositionNormalized
	//_Func: public void setSize(float w, float h); @virtual vtpo=0 vfid=7 @loc=static @len=755 @rva=2422512
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=760 @rva=2420928
	//_Func: protected void scaleByMult(float value); @virtual vtpo=0 vfid=19 @loc=static @len=43 @rva=2422464
	//_Data: this+0x1D4, Member, Type: enum CustomSpinnerMode, mode
	//_Func: private void resize(float value); @loc=static @len=765 @rva=2421696
	//_Func: public ksgui_CustomSpinner & operator=(ksgui_CustomSpinner &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_CustomSpinner)==472),"bad size");
		static_assert((offsetof(ksgui_CustomSpinner,pressToIncrement)==0x178),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,leftButton)==0x180),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,rightButton)==0x188),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,upButton)==0x190),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,downButton)==0x198),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,title)==0x1A0),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,bar)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,barPositionNormalized)==0x1D0),"bad off");
		static_assert((offsetof(ksgui_CustomSpinner,mode)==0x1D4),"bad off");
	};
};

//UDT: class ksgui::TextInput @len=464 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void TextInput(ksgui_TextInput &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TextInput(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui); @loc=static @len=579 @rva=2380560
	//_Func: public void ~TextInput(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: class vec4f, backColorFocus
	//_Data: this+0x188, Member, Type: class vec4f, backColorUnfocus
	//_Data: this+0x198, Member, Type: class Event<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >, evValidate
	//_Data: this+0x1B0, Member, Type: class Event<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >, evKeyDown
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=26 @rva=2381536
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=284 @rva=2381568
	//_Func: public void onKeyChar(unsigned int key); @virtual vtpo=0 vfid=14 @loc=static @len=266 @rva=2381264
	//_Func: public void onKeyDown(OnKeyEvent & message); @virtual vtpo=0 vfid=15 @loc=static @len=3 @rva=96368
	//_Func: public void setFocus(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void validateText(); @loc=static @len=109 @rva=2381856
	//_Data: this+0x1C8, Member, Type: bool, hasFocus
	//_Data: this+0x1CC, Member, Type: unsigned int, maxNumberOfChar
	//_Func: private bool hasSpaceToWrite(); @loc=optimized @len=0 @rva=0
	//_Func: public ksgui_TextInput & operator=(ksgui_TextInput &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_TextInput)==464),"bad size");
		static_assert((offsetof(ksgui_TextInput,backColorFocus)==0x178),"bad off");
		static_assert((offsetof(ksgui_TextInput,backColorUnfocus)==0x188),"bad off");
		static_assert((offsetof(ksgui_TextInput,evValidate)==0x198),"bad off");
		static_assert((offsetof(ksgui_TextInput,evKeyDown)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_TextInput,hasFocus)==0x1C8),"bad off");
		static_assert((offsetof(ksgui_TextInput,maxNumberOfChar)==0x1CC),"bad off");
	};
};

//UDT: class ksgui::MovingBar @len=512 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void MovingBar(ksgui_MovingBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void MovingBar(ksgui_ScrollBar * scrollBar, ksgui_GUI * aGui); @loc=static @len=755 @rva=2468704
	//_Func: public void ~MovingBar(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=160 @rva=2471936
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=383 @rva=2472112
	//_Func: public void onMouseUp(OnMouseUpEvent & message); @virtual vtpo=0 vfid=11 @loc=static @len=12 @rva=2472640
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=391 @rva=2472912
	//_Data: this+0x178, Member, Type: class ksgui::ScrollBar *, sb
	//_Data: this+0x180, Member, Type: class Texture, upperTexture
	//_Data: this+0x1A8, Member, Type: class Texture, lowerTexture
	//_Data: this+0x1D0, Member, Type: class Texture, middleTexture
	//_Data: this+0x1F8, Member, Type: class vec2f, pressingPoint
	//_Func: private vec2f getCenter(); @loc=optimized @len=0 @rva=0
	//_Func: public ksgui_MovingBar & operator=(ksgui_MovingBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_MovingBar)==512),"bad size");
		static_assert((offsetof(ksgui_MovingBar,sb)==0x178),"bad off");
		static_assert((offsetof(ksgui_MovingBar,upperTexture)==0x180),"bad off");
		static_assert((offsetof(ksgui_MovingBar,lowerTexture)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_MovingBar,middleTexture)==0x1D0),"bad off");
		static_assert((offsetof(ksgui_MovingBar,pressingPoint)==0x1F8),"bad off");
	};
};

//UDT: class ksgui::ActiveButton @len=576 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void ActiveButton(ksgui_ActiveButton &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ActiveButton(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * gui); @loc=static @len=594 @rva=2404704
	//_Func: public void ~ActiveButton(); @virtual vtpo=0 vfid=0 @loc=static @len=120 @rva=2405312
	//_Data: this+0x178, Member, Type: class vec2f, textCoord
	//_Data: this+0x180, Member, Type: class vec4f, unselectedColor
	//_Data: this+0x190, Member, Type: class vec4f, selectedColor
	//_Data: this+0x1A0, Member, Type: class vec4f, rollOnColor
	//_Data: this+0x1B0, Member, Type: class vec4f, inactiveColor
	//_Data: this+0x1C0, Member, Type: enum ksgui::eActiveButtonStates, status
	//_Data: this+0x1C4, Member, Type: bool, blanked
	//_Data: this+0x1C5, Member, Type: bool, highlightOnCursorOver
	//_Data: this+0x1C6, Member, Type: bool, isActive
	//_Func: public void setMultipleTexture(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename); @loc=static @len=894 @rva=2406144
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=489 @rva=2405648
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=130 @rva=2405488
	//_Func: public void select(); @loc=optimized @len=0 @rva=0
	//_Func: public void deselect(); @loc=optimized @len=0 @rva=0
	//_Func: public void setSelected(bool value); @loc=static @len=17 @rva=2407040
	//_Func: public bool isSelected(); @loc=optimized @len=0 @rva=0
	//_Func: public void onVisibleChanged(bool newValue); @virtual vtpo=0 vfid=8 @loc=static @len=11 @rva=2405632
	//_Func: public void setText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aText); @virtual vtpo=0 vfid=17 @loc=static @len=25 @rva=2407072
	//_Func: public void setUsingMultipleTexture(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x1C8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, activeButtonText
	//_Data: this+0x1E8, Member, Type: bool, selected
	//_Data: this+0x1E9, Member, Type: bool, usingMultipleTextures
	//_Data: this+0x1EC, Member, Type: enum ksgui::eActiveButtonStates, oldStatus
	//_Data: this+0x1F0, Member, Type: class Texture, textureOn
	//_Data: this+0x218, Member, Type: class Texture, textureOff
	//_Func: public ksgui_ActiveButton & operator=(ksgui_ActiveButton &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ActiveButton)==576),"bad size");
		static_assert((offsetof(ksgui_ActiveButton,textCoord)==0x178),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,unselectedColor)==0x180),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,selectedColor)==0x190),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,rollOnColor)==0x1A0),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,inactiveColor)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,status)==0x1C0),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,blanked)==0x1C4),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,highlightOnCursorOver)==0x1C5),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,isActive)==0x1C6),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,activeButtonText)==0x1C8),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,selected)==0x1E8),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,usingMultipleTextures)==0x1E9),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,oldStatus)==0x1EC),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,textureOn)==0x1F0),"bad off");
		static_assert((offsetof(ksgui_ActiveButton,textureOff)==0x218),"bad off");
	};
};

//UDT: class ksgui::Form @len=440 @vfcount=23
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void Form(ksgui_Form &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Form(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui, bool icanBeScaled); @loc=static @len=2859 @rva=2381968
	//_Func: public void ~Form(); @virtual vtpo=0 vfid=0 @loc=static @len=15 @rva=2384832
	//_Func: public void setAutoHideMode(bool mode); @loc=static @len=176 @rva=2386480
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=5 @rva=2386144
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=262 @rva=2385696
	//_Func: public void onTitleClicked(ksgui_OnControlClicked & message); @intro @virtual vtpo=0 vfid=21 @loc=static @len=38 @rva=2385968
	//_Func: public void onKeyChar(unsigned int key); @virtual vtpo=0 vfid=14 @loc=static @len=14 @rva=2385664
	//_Func: public void onKeyDown(OnKeyEvent & message); @virtual vtpo=0 vfid=15 @loc=static @len=14 @rva=2385680
	//_Func: public void onIconClicked(ksgui_OnControlClicked &  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: class ksgui::Control *, formTitle
	//_Func: public void setSize(float w, float h); @virtual vtpo=0 vfid=7 @loc=static @len=137 @rva=2387296
	//_Func: public void setIcon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * fileName); @loc=static @len=390 @rva=2386704
	//_Func: public void setPosition(float left, float top); @loc=static @len=5 @rva=2387280
	//_Func: public Texture getIcon(); @loc=optimized @len=0 @rva=0
	//_Func: public void setIconVisible(bool visible); @loc=static @len=133 @rva=2387136
	//_Func: public void setIconPosition(float x, float y); @loc=static @len=12 @rva=2387104
	//_Func: public void setIconSize(float width, float height); @loc=static @len=14 @rva=2387120
	//_Func: public void onVisibleChanged(bool newValue); @virtual vtpo=0 vfid=8 @loc=static @len=119 @rva=2386016
	//_Func: public void shutdown(); @intro @virtual vtpo=0 vfid=22 @loc=static @len=3 @rva=96368
	//_Func: public bool isBLocked(); @loc=optimized @len=0 @rva=0
	//_Func: public void setBlocked(bool value); @loc=static @len=46 @rva=2386656
	//_Func: public bool isDevApp(); @loc=optimized @len=0 @rva=0
	//_Func: public void setScaleMult(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setScaleToBeSaved(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getScaleToBeSaved(); @loc=optimized @len=0 @rva=0
	//_Func: public bool hitTest(int x, int y); @virtual vtpo=0 vfid=1 @loc=static @len=84 @rva=2385568
	//_Func: public void updateScaleByMult(); @loc=static @len=36 @rva=2387440
	//_Func: public float getHeaderHeight(); @loc=static @len=9 @rva=3194240
	//_Data: this+0x180, Member, Type: bool, devApp
	//_Data: this+0x181, Member, Type: bool, autohide
	//_Data: this+0x182, Member, Type: bool, blocked
	//_Data: this+0x184, Member, Type: float, scaleToBeSaved
	//_Data: this+0x188, Member, Type: bool, canBeScaled
	//_Data: this+0x190, Member, Type: class ksgui::Control *, icon
	//_Data: this+0x198, Member, Type: class ksgui::ActiveButton *, pinIcon
	//_Data: this+0x1A0, Member, Type: class ksgui::Control *, zoomInIcon
	//_Data: this+0x1A8, Member, Type: class ksgui::Control *, zoomOutIcon
	//_Func: protected void scaleByMult(float value); @virtual vtpo=0 vfid=19 @loc=static @len=280 @rva=2386160
	//_Func: protected void scaleByMult(); @virtual vtpo=0 vfid=20 @loc=static @len=18 @rva=2386448
	//_Data: this+0x1B0, Member, Type: float, HEADER_HEIGHT
	//_Data: this+0x1B4, Member, Type: float, scaleStep
	//_Func: public ksgui_Form & operator=(ksgui_Form &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Form)==440),"bad size");
		static_assert((offsetof(ksgui_Form,formTitle)==0x178),"bad off");
		static_assert((offsetof(ksgui_Form,devApp)==0x180),"bad off");
		static_assert((offsetof(ksgui_Form,autohide)==0x181),"bad off");
		static_assert((offsetof(ksgui_Form,blocked)==0x182),"bad off");
		static_assert((offsetof(ksgui_Form,scaleToBeSaved)==0x184),"bad off");
		static_assert((offsetof(ksgui_Form,canBeScaled)==0x188),"bad off");
		static_assert((offsetof(ksgui_Form,icon)==0x190),"bad off");
		static_assert((offsetof(ksgui_Form,pinIcon)==0x198),"bad off");
		static_assert((offsetof(ksgui_Form,zoomInIcon)==0x1A0),"bad off");
		static_assert((offsetof(ksgui_Form,zoomOutIcon)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_Form,HEADER_HEIGHT)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_Form,scaleStep)==0x1B4),"bad off");
	};
};

//UDT: class ksgui::TaskBarIcon @len=536 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void TaskBarIcon(ksgui_TaskBarIcon &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void TaskBarIcon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, std::shared_ptr<Font> * appFont, ksgui_GUI * igui); @loc=static @len=1211 @rva=2474464
	//_Func: public void ~TaskBarIcon(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: enum eTaskBarStatus, status
	//_Data: this+0x17C, Member, Type: bool, isOver
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=264 @rva=2475968
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=363 @rva=2476240
	//_Func: public void setDescriptionText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * text); @loc=static @len=280 @rva=2476608
	//_Func: public void hide(bool value); @loc=static @len=159 @rva=2475808
	//_Data: this+0x180, Member, Type: class Texture, texture
	//_Data: this+0x1A8, Member, Type: class Texture, textureOn
	//_Data: this+0x1D0, Member, Type: class vec4f, unselectedColor
	//_Data: this+0x1E0, Member, Type: class vec4f, selectedColor
	//_Data: this+0x1F0, Member, Type: class vec4f, rollOverColor
	//_Data: this+0x200, Member, Type: class ksgui::Control *, descriptionRect
	//_Data: this+0x208, Member, Type: class ksgui::Label *, descriptionLabel
	//_Data: this+0x210, Member, Type: float, targetX
	//_Func: private void setIcon(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename); @loc=static @len=847 @rva=2476896
	//_Func: public ksgui_TaskBarIcon & operator=(ksgui_TaskBarIcon &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_TaskBarIcon)==536),"bad size");
		static_assert((offsetof(ksgui_TaskBarIcon,status)==0x178),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,isOver)==0x17C),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,texture)==0x180),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,textureOn)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,unselectedColor)==0x1D0),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,selectedColor)==0x1E0),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,rollOverColor)==0x1F0),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,descriptionRect)==0x200),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,descriptionLabel)==0x208),"bad off");
		static_assert((offsetof(ksgui_TaskBarIcon,targetX)==0x210),"bad off");
	};
};

//UDT: class ksgui::GameScreen @len=432 @vfcount=22
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void GameScreen(ksgui_GameScreen &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void GameScreen(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui); @loc=static @len=163 @rva=2427856
	//_Func: public void ~GameScreen(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: bool, appInteractionEnabled
	//_Data: this+0x180, Member, Type: class Event<bool>, evDesktopShown
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=138 @rva=2428384
	//_Func: public void addControl(ksgui_Control * c, bool addToTaskBar); @intro @virtual vtpo=0 vfid=21 @loc=static @len=110 @rva=2428176
	//_Func: public void showDesktop(); @loc=static @len=585 @rva=2428528
	//_Func: public int getOldAppsCount(); @loc=static @len=19 @rva=2428288
	//_Func: public void shutdown(); @loc=static @len=240 @rva=2429120
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=16 @rva=2428320
	//_Func: public void onMouseUp(OnMouseUpEvent & message); @virtual vtpo=0 vfid=11 @loc=static @len=14 @rva=2428352
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=14 @rva=2428336
	//_Func: public void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message); @virtual vtpo=0 vfid=13 @loc=static @len=14 @rva=2428368
	//_Data: this+0x198, Member, Type: class std::vector<int,std::allocator<int> >, oldAppsOpened
	//_Func: public ksgui_GameScreen & operator=(ksgui_GameScreen &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_GameScreen)==432),"bad size");
		static_assert((offsetof(ksgui_GameScreen,appInteractionEnabled)==0x178),"bad off");
		static_assert((offsetof(ksgui_GameScreen,evDesktopShown)==0x180),"bad off");
		static_assert((offsetof(ksgui_GameScreen,oldAppsOpened)==0x198),"bad off");
	};
};

//UDT: class ksgui::Slider @len=456 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void Slider(ksgui_Slider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Slider(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, ksgui_GUI * myGui, bool dressed); @loc=static @len=1705 @rva=2423280
	//_Func: public void ~Slider(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: class Event<ksgui::OnSliderInteraction>, evOnSliderInteraction
	//_Data: this+0x190, Member, Type: class Event<ksgui::OnCutExtremesChanged>, evOnCutExtremesChanged
	//_Func: public void setValue(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=652 @rva=2426240
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=335 @rva=2425280
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=392 @rva=2425616
	//_Func: public void onMouseUp(OnMouseUpEvent & message); @virtual vtpo=0 vfid=11 @loc=static @len=221 @rva=2426016
	//_Func: public void didSelectACutMarker(ksgui_OnControlClicked &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setSize(float w, float h); @virtual vtpo=0 vfid=7 @loc=static @len=27 @rva=2427504
	//_Func: public void updateMarkers(); @loc=static @len=307 @rva=2427536
	//_Func: public void resetMarkers(); @loc=static @len=293 @rva=2426896
	//_Func: public void setMarkerLeftPosition(float perc); @loc=static @len=158 @rva=2427200
	//_Func: public void setMarkerRightPosition(float perc); @loc=static @len=142 @rva=2427360
	//_Data: this+0x1A8, Member, Type: float, value
	//_Data: this+0x1B0, Member, Type: class ksgui::Control *, mPos
	//_Data: this+0x1B8, Member, Type: class ksgui::Control *, markerLeft
	//_Data: this+0x1C0, Member, Type: class ksgui::Control *, markerRight
	//_Func: public ksgui_Slider & operator=(ksgui_Slider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Slider)==456),"bad size");
		static_assert((offsetof(ksgui_Slider,evOnSliderInteraction)==0x178),"bad off");
		static_assert((offsetof(ksgui_Slider,evOnCutExtremesChanged)==0x190),"bad off");
		static_assert((offsetof(ksgui_Slider,value)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_Slider,mPos)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_Slider,markerLeft)==0x1B8),"bad off");
		static_assert((offsetof(ksgui_Slider,markerRight)==0x1C0),"bad off");
	};
};

//UDT: class ksgui::ScrollBar @len=448 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void ScrollBar(ksgui_ScrollBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ScrollBar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui, bool isDressed); @loc=static @len=1446 @rva=2469472
	//_Func: public void ~ScrollBar(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Data: this+0x178, Member, Type: class Event<ksgui::OnScrollBarValueChanged>, evOnValueChanged
	//_Data: this+0x190, Member, Type: class ksgui::Control *, butPlus
	//_Data: this+0x198, Member, Type: class ksgui::Control *, butMinus
	//_Func: public void setItemsPerPage(unsigned int v); @loc=static @len=24 @rva=2474160
	//_Func: public void setSize(float w, float h); @virtual vtpo=0 vfid=7 @loc=static @len=91 @rva=2474224
	//_Func: public unsigned int getItemsPerPage(); @loc=static @len=7 @rva=2471920
	//_Func: public void setItemsNumber(unsigned int v); @loc=static @len=7 @rva=2474144
	//_Func: public int getItemsNumber(); @loc=optimized @len=0 @rva=0
	//_Func: public float getLastItem(); @loc=optimized @len=0 @rva=0
	//_Func: public void setValue(unsigned int v); @loc=static @len=139 @rva=2474320
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=822 @rva=2473312
	//_Func: public int getValue(); @loc=optimized @len=0 @rva=0
	//_Func: public void setRepeatInterval(float i); @virtual vtpo=0 vfid=16 @loc=static @len=31 @rva=2474192
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=5 @rva=2472096
	//_Func: public void onMouseUp(OnMouseUpEvent & message); @virtual vtpo=0 vfid=11 @loc=static @len=5 @rva=2472656
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=134 @rva=2472496
	//_Func: public int getStep(); @loc=optimized @len=0 @rva=0
	//_Func: public int getMinValue(); @loc=optimized @len=0 @rva=0
	//_Func: public int getMaxValue(); @loc=optimized @len=0 @rva=0
	//_Func: public float getPercentage(); @loc=optimized @len=0 @rva=0
	//_Func: public void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message); @virtual vtpo=0 vfid=13 @loc=static @len=225 @rva=2472672
	//_Data: this+0x1A0, Member, Type: bool, dressed
	//_Data: this+0x1A4, Member, Type: unsigned int, minValue
	//_Data: this+0x1A8, Member, Type: unsigned int, maxValue
	//_Data: this+0x1AC, Member, Type: unsigned int, step
	//_Data: this+0x1B0, Member, Type: unsigned int, value
	//_Data: this+0x1B8, Member, Type: class ksgui::MovingBar *, bar
	//_Func: private void drawArrow(ksgui_Control * button, ksgui_eArrowsDirection direction); @loc=static @len=535 @rva=2471376
	//_Func: public ksgui_ScrollBar & operator=(ksgui_ScrollBar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ScrollBar)==448),"bad size");
		static_assert((offsetof(ksgui_ScrollBar,evOnValueChanged)==0x178),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,butPlus)==0x190),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,butMinus)==0x198),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,dressed)==0x1A0),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,minValue)==0x1A4),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,maxValue)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,step)==0x1AC),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,value)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_ScrollBar,bar)==0x1B8),"bad off");
	};
};

//UDT: class RaceEngineer @len=24 @vfcount=1
	//_VTable: 
	//_Func: public void RaceEngineer(RaceEngineer &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void RaceEngineer(Car * icar); @loc=static @len=25 @rva=2595824
	//_Func: public void ~RaceEngineer(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=11 @rva=2595856
	//_Func: public float getCompoundDY(TyreCompoundDef & def, float load); @pure @loc=static @len=45 @rva=2604016
	//_Func: public float projectCarForwardForceAtGas(float gas); @loc=static @len=196 @rva=2608160
	//_Func: public AccelerationProfile getAccelerationProfile(); @loc=static @len=445 @rva=2602320
	//_Func: public Car * getCar(); @loc=static @len=5 @rva=100256
	//_Func: public float getBetaRAD(); @loc=static @len=132 @rva=2603664
	//_Func: public float getUndersteerFactor(); @loc=optimized @len=0 @rva=0
	//_Func: public float getFrontCasterRAD(); @loc=static @len=19 @rva=2604352
	//_Func: public float getCasterRAD(ISuspension * isus); @loc=static @len=204 @rva=2603808
	//_Func: public void getToeInRAD(float *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getFrontNaturalFrequencyHZ(); @loc=static @len=229 @rva=2604832
	//_Func: public float getRearNaturalFrequencyHZ(); @loc=static @len=231 @rva=2607344
	//_Func: public float getNaturalFrequencyHZ(float  _arg0, float  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public float getWeightDistribution(); @loc=static @len=136 @rva=2608016
	//_Func: public float getDampingRatio(float  _arg0, float  _arg1, float  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: public std::tuple<float,float> getFrontDampingRatio(); @loc=static @len=443 @rva=2604384
	//_Func: public std::tuple<float,float> getRearDampingRatio(); @loc=static @len=539 @rva=2606800
	//_Func: public float evalLateralGrip(float speed, float mass, float aeroModifier, float * in_loads, float current_lift_kg); @loc=static @len=823 @rva=2598160
	//_Func: public float getPointGroundHeight(vec3f & p); @loc=static @len=371 @rva=2606416
	//_Func: public float getPointFrontShare(vec3f & p); @loc=static @len=127 @rva=2606288
	//_Func: public float projectWingLift(Wing *  _arg0, float  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public float projectWingDrag(Wing *  _arg0, float  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public float projectWingsLift(float speed); @loc=static @len=180 @rva=2608576
	//_Func: public float projectWingsDrag(float speed); @loc=static @len=200 @rva=2608368
	//_Func: public int getIdealGear(float  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public float getLookAheadBraking(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getTorqueToGripRatio(); @loc=static @len=277 @rva=2607712
	//_Func: public Tyre * getLeftDrivenTyre(); @loc=static @len=33 @rva=2605504
	//_Func: public Tyre * getRightDrivenTyre(); @loc=static @len=33 @rva=2607664
	//_Func: public float getFrontTrack(); @loc=static @len=66 @rva=2605072
	//_Func: public float getRearTrack(); @loc=static @len=67 @rva=2607584
	//_Func: public float getDynamicIndex(); @loc=static @len=216 @rva=2604128
	//_Func: public float getTractionMaxSlipRatio(); @loc=optimized @len=0 @rva=0
	//_Func: public float getOptimalBrake(); @loc=static @len=427 @rva=2605856
	//_Func: public float getDrivingTyresSlip(); @loc=static @len=56 @rva=2604064
	//_Func: public float getMaxBrakingForce(); @loc=static @len=78 @rva=2605552
	//_Func: public float getBaseCarHeight(); @loc=static @len=136 @rva=2603520
	//_Func: public Speed getMaxSpeedFromGear(unsigned int relativeCarIndex); @loc=static @len=221 @rva=2605632
	//_Func: public float evalAvailableBrake(float speed, float latg, float weight); @loc=static @len=415 @rva=2596688
	//_Func: public float evalFrontRideHeight(); @loc=static @len=365 @rva=2597104
	//_Func: public float evalRearRideHeight(); @loc=static @len=372 @rva=2598992
	//_Func: public float evalRideHeight(int index); @loc=static @len=13 @rva=2599376
	//_Func: public float projectFrontWeight(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getFrontWheelbase(); @loc=optimized @len=0 @rva=0
	//_Func: public float evaluateFuelPerLapFromTrackSpline(); @loc=static @len=1846 @rva=2600032
	//_Func: public int findTyreCompound(FindTyreCompoundLogic logic); @loc=static @len=421 @rva=2601888
	//_Func: public WheelValues evalTyreLoad(float speedMS, float latg, float longg); @loc=static @len=625 @rva=2599392
	//_Func: public float getWheelbase(); @loc=optimized @len=0 @rva=0
	//_Func: public KPI getKPI_RAD(int index); @loc=static @len=347 @rva=2605152
	//_Func: public float getCurrentLateralGrip(float  _arg0, float *  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public bool shouldChangeUp(); @loc=optimized @len=0 @rva=0
	//_Func: public float getAntiSquat(); @loc=static @len=745 @rva=2602768
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Data: this+0x10, Member, Type: float, fuelPerLapEvaluated
	//_Data: static, [0155A888][0003:00047888], Static Member, Type: class std::unique_ptr<AISpline,std::default_delete<AISpline> >, trackSpline
	//_Func: protected float evalLateralGFromLoads(float speed, float aeroRatio, float mass, float * loads); @loc=static @len=680 @rva=2597472
	//_Func: public RaceEngineer & operator=(RaceEngineer &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(RaceEngineer)==24),"bad size");
		static_assert((offsetof(RaceEngineer,car)==0x8),"bad off");
		static_assert((offsetof(RaceEngineer,fuelPerLapEvaluated)==0x10),"bad off");
	};
};

//UDT: class ksgui::Taskbar @len=616 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void Taskbar(ksgui_Taskbar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Taskbar(ksgui_GUI * igui); @loc=static @len=2106 @rva=2460192
	//_Func: public void ~Taskbar(); @virtual vtpo=0 vfid=0 @loc=static @len=260 @rva=2462304
	//_Data: this+0x178, Member, Type: int, targetX
	//_Data: this+0x17C, Member, Type: bool, taskBarFullShowed
	//_Data: this+0x180, Member, Type: class Event<ksgui::OnControlClicked>, evAppOpened
	//_Data: this+0x198, Member, Type: class Event<ksgui::OnControlClicked>, evAppDismiss
	//_Data: this+0x1B0, Member, Type: class std::vector<ksgui::Form *,std::allocator<ksgui::Form *> >, forms
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=599 @rva=2465872
	//_Func: public void addForm(ksgui_Form * form); @loc=static @len=467 @rva=2464784
	//_Func: public void shutdown(); @loc=optimized @len=0 @rva=0
	//_Func: public void onAppClicked(ksgui_OnControlClicked &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=70 @rva=2465648
	//_Func: public void mouseWheelMoved(OnMouseWheelMovedEvent & message); @loc=static @len=186 @rva=2465456
	//_Func: public void selectVirtualDesktop(int id); @loc=static @len=170 @rva=2466480
	//_Func: public int getNumDekstops(); @loc=optimized @len=0 @rva=0
	//_Func: public short cycleDekstop(); @loc=static @len=177 @rva=2465264
	//_Func: public float getTargetFullShow(); @loc=optimized @len=0 @rva=0
	//_Func: public float getTargetOutOfSight(); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getFormCount(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x1C8, Member, Type: class Event<int>, evOnDesktopChange
	//_Data: this+0x1E0, Member, Type: class vec4f, selectedColor
	//_Data: this+0x1F0, Member, Type: class vec4f, unselectedColor
	//_Data: this+0x200, Member, Type: class ksgui::Control *, topScroller
	//_Data: this+0x208, Member, Type: class ksgui::Control *, bottomScroller
	//_Data: this+0x210, Member, Type: class std::vector<ksgui::Control *,std::allocator<ksgui::Control *> >, desktops
	//_Data: this+0x228, Member, Type: float, targetOutOfSight
	//_Data: this+0x22C, Member, Type: float, targetFullShow
	//_Data: this+0x230, Member, Type: int, topAppIndex
	//_Data: this+0x238, Member, Type: class std::shared_ptr<Font>, appFont
	//_Data: this+0x248, Member, Type: class std::vector<ksgui::TaskBarIcon *,std::allocator<ksgui::TaskBarIcon *> >, icons
	//_Data: this+0x260, Member, Type: short, virtualDesktop
	//_Func: protected void setTopAppIndex(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void updateAppIconsPosition(); @loc=static @len=309 @rva=2466656
	//_Func: protected ksgui_Control * addDesktop(int id, int x, int y); @loc=static @len=543 @rva=2464240
	//_Func: public ksgui_Taskbar & operator=(ksgui_Taskbar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Taskbar)==616),"bad size");
		static_assert((offsetof(ksgui_Taskbar,targetX)==0x178),"bad off");
		static_assert((offsetof(ksgui_Taskbar,taskBarFullShowed)==0x17C),"bad off");
		static_assert((offsetof(ksgui_Taskbar,evAppOpened)==0x180),"bad off");
		static_assert((offsetof(ksgui_Taskbar,evAppDismiss)==0x198),"bad off");
		static_assert((offsetof(ksgui_Taskbar,forms)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_Taskbar,evOnDesktopChange)==0x1C8),"bad off");
		static_assert((offsetof(ksgui_Taskbar,selectedColor)==0x1E0),"bad off");
		static_assert((offsetof(ksgui_Taskbar,unselectedColor)==0x1F0),"bad off");
		static_assert((offsetof(ksgui_Taskbar,topScroller)==0x200),"bad off");
		static_assert((offsetof(ksgui_Taskbar,bottomScroller)==0x208),"bad off");
		static_assert((offsetof(ksgui_Taskbar,desktops)==0x210),"bad off");
		static_assert((offsetof(ksgui_Taskbar,targetOutOfSight)==0x228),"bad off");
		static_assert((offsetof(ksgui_Taskbar,targetFullShow)==0x22C),"bad off");
		static_assert((offsetof(ksgui_Taskbar,topAppIndex)==0x230),"bad off");
		static_assert((offsetof(ksgui_Taskbar,appFont)==0x238),"bad off");
		static_assert((offsetof(ksgui_Taskbar,icons)==0x248),"bad off");
		static_assert((offsetof(ksgui_Taskbar,virtualDesktop)==0x260),"bad off");
	};
};

//UDT: class ksgui::Spinner @len=624 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void Spinner(ksgui_Spinner &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Spinner(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iname, ksgui_GUI * igui, bool isDressed, bool mirrored); @loc=static @len=3253 @rva=2407104
	//_Func: public void ~Spinner(); @virtual vtpo=0 vfid=0 @loc=static @len=132 @rva=2410368
	//_Data: this+0x178, Member, Type: bool, homogeneousPressingDelay
	//_Data: this+0x17C, Member, Type: float, displayValueMult
	//_Data: this+0x180, Member, Type: int, minValue
	//_Data: this+0x184, Member, Type: int, maxValue
	//_Data: this+0x188, Member, Type: int, step
	//_Data: this+0x190, Member, Type: class Event<ksgui::OnSpinnerValueChanged>, evOnValueChanged
	//_Data: this+0x1A8, Member, Type: class ksgui::PopOver *, popOver
	//_Data: this+0x1B0, Member, Type: bool, displayPopover
	//_Data: this+0x1B1, Member, Type: bool, drawUnderline
	//_Data: this+0x1B2, Member, Type: bool, displayCentralValue
	//_Data: this+0x1B3, Member, Type: bool, displayLabel
	//_Data: this+0x1B8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, units
	//_Data: this+0x1D8, Member, Type: float, textSize
	//_Data: this+0x1DC, Member, Type: bool, isReadOnly
	//_Func: public void setValue(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & itemName); @loc=static @len=292 @rva=2415920
	//_Func: public void setValue(int v); @loc=static @len=139 @rva=2416224
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=2271 @rva=2411984
	//_Func: public int getValue(); @loc=static @len=7 @rva=2411712
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getItemAt(unsigned int index); @loc=static @len=68 @rva=3316640
	//_Func: public void setRepeatInterval(float i); @virtual vtpo=0 vfid=16 @loc=static @len=31 @rva=2415552
	//_Func: public void addItem(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * item); @loc=static @len=150 @rva=2411392
	//_Func: public void setFlatStyle(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setLabelText(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * text); @loc=static @len=106 @rva=2415296
	//_Func: public void setSize(float w, float h); @virtual vtpo=0 vfid=7 @loc=static @len=329 @rva=2415584
	//_Func: public void setPosition(float x, float y); @loc=static @len=142 @rva=2415408
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getCurrentItem(); @loc=static @len=147 @rva=2411552
	//_Func: public void setLabelMaxChar(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void timedIncrement(); @loc=optimized @len=0 @rva=0
	//_Func: public void timedDecrement(); @loc=optimized @len=0 @rva=0
	//_Func: public bool onMouseDown(OnMouseDownEvent & message); @virtual vtpo=0 vfid=10 @loc=static @len=64 @rva=2411728
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @virtual vtpo=0 vfid=12 @loc=static @len=181 @rva=2411792
	//_Func: public void setDrawArrows(bool draw); @loc=static @len=641 @rva=2414640
	//_Func: protected void scaleByMult(float value); @virtual vtpo=0 vfid=19 @loc=static @len=43 @rva=2414592
	//_Data: this+0x1E0, Member, Type: float, counter
	//_Data: this+0x1E4, Member, Type: float, pressionCounter
	//_Data: this+0x1E8, Member, Type: class ksgui::Control *, butPlus
	//_Data: this+0x1F0, Member, Type: class ksgui::Control *, butMinus
	//_Data: this+0x1F8, Member, Type: class ksgui::Control *, selectionSlider
	//_Data: this+0x200, Member, Type: class ksgui::Label *, label
	//_Data: this+0x208, Member, Type: class Texture, valueBack
	//_Data: this+0x230, Member, Type: bool, dressed
	//_Data: this+0x238, Member, Type: struct ksgui::OnControlClicked, lastOnControlClicked
	//_Data: this+0x248, Member, Type: int, value
	//_Data: this+0x250, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, items
	//_Data: this+0x268, Member, Type: bool, mirrored
	//_Data: this+0x26C, Member, Type: float, triangleWidth
	//_Func: private void drawArrow(ksgui_Control *  _arg0, ksgui_eArrowsDirection  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: private void resize(float value); @loc=static @len=325 @rva=2414256
	//_Func: public ksgui_Spinner & operator=(ksgui_Spinner &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Spinner)==624),"bad size");
		static_assert((offsetof(ksgui_Spinner,homogeneousPressingDelay)==0x178),"bad off");
		static_assert((offsetof(ksgui_Spinner,displayValueMult)==0x17C),"bad off");
		static_assert((offsetof(ksgui_Spinner,minValue)==0x180),"bad off");
		static_assert((offsetof(ksgui_Spinner,maxValue)==0x184),"bad off");
		static_assert((offsetof(ksgui_Spinner,step)==0x188),"bad off");
		static_assert((offsetof(ksgui_Spinner,evOnValueChanged)==0x190),"bad off");
		static_assert((offsetof(ksgui_Spinner,popOver)==0x1A8),"bad off");
		static_assert((offsetof(ksgui_Spinner,displayPopover)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_Spinner,drawUnderline)==0x1B1),"bad off");
		static_assert((offsetof(ksgui_Spinner,displayCentralValue)==0x1B2),"bad off");
		static_assert((offsetof(ksgui_Spinner,displayLabel)==0x1B3),"bad off");
		static_assert((offsetof(ksgui_Spinner,units)==0x1B8),"bad off");
		static_assert((offsetof(ksgui_Spinner,textSize)==0x1D8),"bad off");
		static_assert((offsetof(ksgui_Spinner,isReadOnly)==0x1DC),"bad off");
		static_assert((offsetof(ksgui_Spinner,counter)==0x1E0),"bad off");
		static_assert((offsetof(ksgui_Spinner,pressionCounter)==0x1E4),"bad off");
		static_assert((offsetof(ksgui_Spinner,butPlus)==0x1E8),"bad off");
		static_assert((offsetof(ksgui_Spinner,butMinus)==0x1F0),"bad off");
		static_assert((offsetof(ksgui_Spinner,selectionSlider)==0x1F8),"bad off");
		static_assert((offsetof(ksgui_Spinner,label)==0x200),"bad off");
		static_assert((offsetof(ksgui_Spinner,valueBack)==0x208),"bad off");
		static_assert((offsetof(ksgui_Spinner,dressed)==0x230),"bad off");
		static_assert((offsetof(ksgui_Spinner,lastOnControlClicked)==0x238),"bad off");
		static_assert((offsetof(ksgui_Spinner,value)==0x248),"bad off");
		static_assert((offsetof(ksgui_Spinner,items)==0x250),"bad off");
		static_assert((offsetof(ksgui_Spinner,mirrored)==0x268),"bad off");
		static_assert((offsetof(ksgui_Spinner,triangleWidth)==0x26C),"bad off");
	};
};

//UDT: class GLRenderer @len=272 @vfcount=1
	//_VTable: 
	//_Func: public void ~GLRenderer(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=202 @rva=2089920
	//_Func: public void init(); @loc=optimized @len=0 @rva=0
	//_Func: public void begin(eGLPrimitiveType type, Shader * ishader); @loc=static @len=19 @rva=2091008
	//_Func: public void vertex3f(vec3f & v); @loc=static @len=19 @rva=2096128
	//_Func: public void vertex3f(float x, float y, float z); @loc=static @len=878 @rva=2096160
	//_Func: public void vertex3fv(float * v); @loc=static @len=19 @rva=2096128
	//_Func: public void end(); @loc=static @len=547 @rva=2091120
	//_Func: public void color3f(float r, float g, float b); @loc=static @len=23 @rva=2091040
	//_Func: public void color4fv(float *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void color4f(vec4f & v); @loc=static @len=8 @rva=2091072
	//_Func: public void color4f(float r, float g, float b, float a); @loc=static @len=27 @rva=2091088
	//_Func: public void texCoord2f(float u, float v); @loc=static @len=15 @rva=2096112
	//_Func: public void fullScreenQuad(Shader * ishader); @loc=static @len=129 @rva=2091680
	//_Func: public void quad(float x, float y, float width, float height, bool textured, Shader * shader); @loc=static @len=286 @rva=2094112
	//_Func: public void quadCentred(float x, float y, float width, float height, bool textured, Shader * shader); @loc=static @len=355 @rva=2094400
	//_Func: public void quadCentredRotated(float x, float y, float width, float height, float rotationRAD, bool textured, Shader * shader); @loc=static @len=650 @rva=2094768
	//_Func: public void spline(Spline & spline); @loc=static @len=686 @rva=2095424
	//_Func: public unsigned int getMaxVertices(); @loc=optimized @len=0 @rva=0
	//_Func: public void GLRenderer(GLRenderer &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void GLRenderer(GraphicsManager * r, unsigned int maxVertices, bool withFullScreenQuad); @loc=static @len=977 @rva=2088928
	//_Data: this+0x8, Member, Type: class std::vector<std::pair<int,void *>,std::allocator<std::pair<int,void *> > >, buffers
	//_Data: this+0x20, Member, Type: int, currentIndex
	//_Data: this+0x28, Member, Type: class GraphicsManager *, graphics
	//_Data: this+0x30, Member, Type: enum eGLPrimitiveType, primitive
	//_Data: this+0x34, Member, Type: class vec4f, color
	//_Data: this+0x44, Member, Type: bool, useTexture
	//_Data: this+0x48, Member, Type: class vec2f, texCoord
	//_Data: this+0x50, Member, Type: class Shader *, shader
	//_Data: this+0x58, Member, Type: class IndexBuffer *, fullQuadIB
	//_Data: this+0x60, Member, Type: class VertexBuffer<MeshVertex> *, fullQuadVB
	//_Data: this+0x68, Member, Type: class Shader *, glShader
	//_Data: this+0x70, Member, Type: class Shader *, glShaderTex
	//_Data: this+0x78, Member, Type: unsigned int, tempCounter
	//_Data: this+0x7C, Member, Type: struct MeshVertex[0x3], tempVertices
	//_Data: this+0x100, Member, Type: struct MeshVertex *, tempBuffer
	//_Data: this+0x108, Member, Type: unsigned int, maxVertices
	//_Func: protected void initFullScreenQuad(); @loc=static @len=2281 @rva=2091824
	//_Func: public GLRenderer & operator=(GLRenderer &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(GLRenderer)==272),"bad size");
		static_assert((offsetof(GLRenderer,buffers)==0x8),"bad off");
		static_assert((offsetof(GLRenderer,currentIndex)==0x20),"bad off");
		static_assert((offsetof(GLRenderer,graphics)==0x28),"bad off");
		static_assert((offsetof(GLRenderer,primitive)==0x30),"bad off");
		static_assert((offsetof(GLRenderer,color)==0x34),"bad off");
		static_assert((offsetof(GLRenderer,useTexture)==0x44),"bad off");
		static_assert((offsetof(GLRenderer,texCoord)==0x48),"bad off");
		static_assert((offsetof(GLRenderer,shader)==0x50),"bad off");
		static_assert((offsetof(GLRenderer,fullQuadIB)==0x58),"bad off");
		static_assert((offsetof(GLRenderer,fullQuadVB)==0x60),"bad off");
		static_assert((offsetof(GLRenderer,glShader)==0x68),"bad off");
		static_assert((offsetof(GLRenderer,glShaderTex)==0x70),"bad off");
		static_assert((offsetof(GLRenderer,tempCounter)==0x78),"bad off");
		static_assert((offsetof(GLRenderer,tempVertices)==0x7C),"bad off");
		static_assert((offsetof(GLRenderer,tempBuffer)==0x100),"bad off");
		static_assert((offsetof(GLRenderer,maxVertices)==0x108),"bad off");
	};
};

//UDT: class ksgui::ListBox @len=536 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void ListBox(ksgui_ListBox &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ListBox(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, unsigned int rowNumber, unsigned int colNumber, ksgui_GUI * aGui, bool isDressed); @loc=static @len=2176 @rva=2392896
	//_Func: public void ~ListBox(); @virtual vtpo=0 vfid=0 @loc=static @len=222 @rva=2395296
	//_Data: this+0x178, Member, Type: bool, allowDeselection
	//_Data: this+0x179, Member, Type: bool, allowSelectionOnClick
	//_Data: this+0x17A, Member, Type: bool, drawUnderline
	//_Data: this+0x17B, Member, Type: bool, drawScrollbarPlaceholder
	//_Data: this+0x17C, Member, Type: enum ksgui::eListBoxSelectionMode, selectionMode
	//_Data: this+0x180, Member, Type: class Event<ksgui::OnListBoxItemClickedEvent>, evOnListBoxItemClicked
	//_Data: this+0x198, Member, Type: class Event<ksgui::OnListBoxItemClickedEvent>, evOnListBoxItemDeselected
	//_Func: public ksgui_ListBoxRowData & getItem(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void addItem(ksgui_ListBoxRowData & item); @loc=static @len=255 @rva=2397680
	//_Func: public void clear(); @loc=static @len=182 @rva=2397936
	//_Func: public ksgui_ListBoxRowData * getSelectedItem(); @loc=static @len=42 @rva=2398592
	//_Data: this+0x1B0, Member, Type: class ksgui::ListBoxRow *, titleRow
	//_Data: this+0x1B8, Member, Type: class ksgui::ListBoxRow *, bottomRow
	//_Func: public void scrollToTop(); @loc=static @len=62 @rva=2400848
	//_Func: public void scrollToBottom(); @loc=optimized @len=0 @rva=0
	//_Func: public void scrollToCenterOnSelected(); @loc=static @len=131 @rva=2400704
	//_Func: public void scrollToIndex(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setItemsFontSize(float size); @loc=static @len=110 @rva=2401696
	//_Func: public int getSelectedItemIndex(); @loc=optimized @len=0 @rva=0
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=1016 @rva=2399280
	//_Func: public void setSize(float w, float h); @virtual vtpo=0 vfid=7 @loc=static @len=597 @rva=2401808
	//_Func: public void onMouseWheelMovedEvent(OnMouseWheelMovedEvent & message); @virtual vtpo=0 vfid=13 @loc=static @len=138 @rva=2398640
	//_Func: public void selectItem(int itemIndex); @loc=static @len=291 @rva=2400912
	//_Func: public void selectItemByName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & itemText); @loc=static @len=468 @rva=2401216
	//_Func: public int getItemIdFromName(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & aName); @loc=static @len=301 @rva=2398288
	//_Func: public ksgui_ListBoxRow * getCell(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getCellCount(); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getItemsCount(); @loc=optimized @len=0 @rva=0
	//_Func: public void drawCellsBackgrounds(bool value); @loc=static @len=79 @rva=2398128
	//_Data: this+0x1C0, Member, Type: class std::vector<ksgui::ListBoxRowData,std::allocator<ksgui::ListBoxRowData> >, items
	//_Data: this+0x1D8, Member, Type: class std::vector<ksgui::ListBoxRow *,std::allocator<ksgui::ListBoxRow *> >, rows
	//_Data: this+0x1F0, Member, Type: unsigned int, columnNumber
	//_Data: this+0x1F8, Member, Type: class std::map<unsigned int,vec4f,std::less<unsigned int>,std::allocator<std::pair<unsigned int const ,vec4f> > >, category
	//_Data: this+0x208, Member, Type: int, selectedItemIndex
	//_Data: this+0x20C, Member, Type: int, pivot
	//_Data: this+0x210, Member, Type: class ksgui::ScrollBar *, scrollBar
	//_Func: private void scrollListBoxToItem(int itemId); @loc=static @len=398 @rva=2400304
	//_Func: private void onScrollBarValueChanged(ksgui_OnScrollBarValueChanged &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private int getItemIdFromRow(ksgui_ListBoxRow *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public ksgui_ListBox & operator=(ksgui_ListBox &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_ListBox)==536),"bad size");
		static_assert((offsetof(ksgui_ListBox,allowDeselection)==0x178),"bad off");
		static_assert((offsetof(ksgui_ListBox,allowSelectionOnClick)==0x179),"bad off");
		static_assert((offsetof(ksgui_ListBox,drawUnderline)==0x17A),"bad off");
		static_assert((offsetof(ksgui_ListBox,drawScrollbarPlaceholder)==0x17B),"bad off");
		static_assert((offsetof(ksgui_ListBox,selectionMode)==0x17C),"bad off");
		static_assert((offsetof(ksgui_ListBox,evOnListBoxItemClicked)==0x180),"bad off");
		static_assert((offsetof(ksgui_ListBox,evOnListBoxItemDeselected)==0x198),"bad off");
		static_assert((offsetof(ksgui_ListBox,titleRow)==0x1B0),"bad off");
		static_assert((offsetof(ksgui_ListBox,bottomRow)==0x1B8),"bad off");
		static_assert((offsetof(ksgui_ListBox,items)==0x1C0),"bad off");
		static_assert((offsetof(ksgui_ListBox,rows)==0x1D8),"bad off");
		static_assert((offsetof(ksgui_ListBox,columnNumber)==0x1F0),"bad off");
		static_assert((offsetof(ksgui_ListBox,category)==0x1F8),"bad off");
		static_assert((offsetof(ksgui_ListBox,selectedItemIndex)==0x208),"bad off");
		static_assert((offsetof(ksgui_ListBox,pivot)==0x20C),"bad off");
		static_assert((offsetof(ksgui_ListBox,scrollBar)==0x210),"bad off");
	};
};

//UDT: struct Tyre @len=2136
	//_Func: public void ~Tyre(); @loc=static @len=262 @rva=2609952
	//_Data: this+0x0, Member, Type: struct TyreInputs, inputs
	//_Data: this+0xC, Member, Type: struct TyreData, data
	//_Data: this+0x58, Member, Type: struct TyreModelData, modelData
	//_Data: this+0x2E8, Member, Type: struct TyreStatus, status
	//_Data: this+0x3A0, Member, Type: class ISuspension *, hub
	//_Data: this+0x3A8, Member, Type: class mat44f, worldRotation
	//_Data: this+0x3E8, Member, Type: class vec3f, unmodifiedContactPoint
	//_Data: this+0x3F4, Member, Type: class vec3f, contactPoint
	//_Data: this+0x400, Member, Type: class vec3f, contactNormal
	//_Data: this+0x410, Member, Type: struct SurfaceDef *, surfaceDef
	//_Data: this+0x418, Member, Type: bool, debugOutput
	//_Data: this+0x41C, Member, Type: float, absOverride
	//_Data: this+0x420, Member, Type: struct TyreThermalModel, thermalModel
	//_Data: this+0x500, Member, Type: class std::vector<TyreCompoundDef,std::allocator<TyreCompoundDef> >, compoundDefs
	//_Data: this+0x518, Member, Type: class std::function<void __cdecl(void)>, onStepCompleted
	//_Data: this+0x538, Member, Type: float, aiMult
	//_Data: this+0x540, Member, Type: class BrushSlipProvider, slipProvider
	//_Data: this+0x578, Member, Type: struct TyreExternalInputs, externalInputs
	//_Data: this+0x588, Member, Type: class vec3f, roadRight
	//_Data: this+0x594, Member, Type: class vec3f, roadHeading
	//_Data: this+0x5A0, Member, Type: bool, useLoadForVKM
	//_Data: this+0x5A1, Member, Type: bool, driven
	//_Func: public float loadSensLinearD(float d0, float d1, float load); @pure @loc=static @len=20 @rva=2634368
	//_Func: public float loadSensExpD(float exp, float mult, float load); @pure @loc=static @len=62 @rva=2634304
	//_Func: public void init(ISuspension * ihub, IRayTrackCollisionProvider * rcp, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * dataPath, int index, int carID, Car * car); @loc=static @len=422 @rva=2623056
	//_Func: public void step(float dt); @loc=static @len=3011 @rva=2635776
	//_Func: public mat44f getFinalTyreRotation(); @loc=static @len=152 @rva=2622864
	//_Func: public void stepRotationMatrix(float dt); @loc=static @len=284 @rva=2640768
	//_Func: public void reset(); @loc=static @len=350 @rva=2634624
	//_Func: public vec3f getRoadRight(); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getRoadHeading(); @loc=optimized @len=0 @rva=0
	//_Func: public vec3f getWorldPosition(); @loc=static @len=25 @rva=2623024
	//_Func: public float getDYWithExp(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getDXWithExp(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getDY(float load); @loc=static @len=385 @rva=2622416
	//_Func: public float getDX(float load); @loc=static @len=393 @rva=2622016
	//_Func: public bool setCompound(int cindex); @loc=static @len=786 @rva=2634976
	//_Func: public int getCompound(); @loc=optimized @len=0 @rva=0
	//_Func: public float getDynamicK(); @loc=static @len=41 @rva=2622816
	//_Func: public float getFxFyRatio(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getCorrectedD(float d, float * wear_mult); @loc=static @len=169 @rva=2621840
	//_Func: public float getLiveRadius(); @loc=optimized @len=0 @rva=0
	//_Func: public float getCamberedDy(float camberRAD, float dy); @loc=static @len=174 @rva=2621568
	//_Data: this+0x5A8, Member, Type: class IRayTrackCollisionProvider *, rayCollisionProvider
	//_Data: this+0x5B0, Member, Type: float, oldAngularVelocity
	//_Data: this+0x5B4, Member, Type: float, totalSlideVelocity
	//_Data: this+0x5B8, Member, Type: class mat44f, localWheelRotation
	//_Data: this+0x5F8, Member, Type: class vec3f, worldPosition
	//_Data: this+0x604, Member, Type: float, slidingVelocityY
	//_Data: this+0x608, Member, Type: float, slidingVelocityX
	//_Data: this+0x60C, Member, Type: float, roadVelocityX
	//_Data: this+0x610, Member, Type: float, roadVelocityY
	//_Data: this+0x614, Member, Type: float, totalHubVelocity
	//_Data: this+0x618, Member, Type: float, rSlidingVelocityX
	//_Data: this+0x61C, Member, Type: float, rSlidingVelocityY
	//_Data: this+0x620, Member, Type: class IRayCaster *, rayCaster
	//_Data: this+0x628, Member, Type: int, index
	//_Data: this+0x630, Member, Type: class SinSignalGenerator, shakeGenerator
	//_Data: this+0x640, Member, Type: class Car *, car
	//_Data: this+0x648, Member, Type: int, currentCompoundIndex
	//_Data: this+0x64C, Member, Type: bool, tyreBlanketsOn
	//_Data: this+0x650, Member, Type: const float, flatSpotK
	//_Data: this+0x658, Member, Type: class ITyreModel *, tyreModel
	//_Data: this+0x660, Member, Type: class SCTM, scTM
	//_Data: this+0x848, Member, Type: float, explosionTemperature
	//_Data: this+0x84C, Member, Type: float, blanketTemperature
	//_Data: this+0x850, Member, Type: float, pressureTemperatureGain
	//_Data: this+0x854, Member, Type: float, localMX
	//_Func: protected void addGroundContact(vec3f & pos, vec3f & normal); @loc=static @len=634 @rva=2611584
	//_Func: protected void addTyreForces(vec3f & pos, vec3f & normal, SurfaceDef * surfaceDef, float dt); @loc=static @len=3000 @rva=2613664
	//_Func: protected void addTyreForcesV10(vec3f & pos, vec3f & normal, SurfaceDef * surfaceDef, float dt); @loc=static @len=2554 @rva=2616672
	//_Func: protected void updateAngularSpeed(float dt); @loc=static @len=206 @rva=2641824
	//_Func: protected void updateLockedState(float dt); @loc=static @len=119 @rva=2642032
	//_Func: protected void stepRelaxationLength(float svx, float svy, float hubVelocity, float dt); @loc=static @len=320 @rva=2640448
	//_Func: protected void stepDirtyLevel(float dt, float hubSpeed); @loc=static @len=293 @rva=2638800
	//_Func: protected void stepThermalModel(float dt); @loc=static @len=609 @rva=2641056
	//_Func: protected float similarityF(float  _arg0, float  _arg1, float  _arg2, float  _arg3, float  _arg4); @loc=optimized @len=0 @rva=0
	//_Func: protected void initCompounds(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataPath, int index); @loc=static @len=10809 @rva=2623488
	//_Func: protected void stepGrainBlister(float dt, float hubVelocity); @loc=static @len=937 @rva=2639360
	//_Func: protected void stepPuncture(float dt, float hubSpeed); @loc=static @len=142 @rva=2640304
	//_Func: protected void stepTyreBlankets(float dt); @loc=static @len=132 @rva=2641680
	//_Func: protected void stepFlatSpot(float dt, float hubVelocity); @loc=static @len=243 @rva=2639104
	//_Func: protected bool generateCompoundNames(); @loc=static @len=2260 @rva=2619296
	//_Func: protected void rayCast(vec3f &  _arg0, vec3f &  _arg1, RayCastResult *  _arg2); @loc=optimized @len=0 @rva=0
	//_Func: protected void addTyreForceToHub(vec3f & pos, vec3f & force); @loc=static @len=1439 @rva=2612224
	//_Func: public void Tyre(Tyre &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Tyre(); @loc=static @len=639 @rva=2546640
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Tyre)==2136),"bad size");
		static_assert((offsetof(Tyre,inputs)==0x0),"bad off");
		static_assert((offsetof(Tyre,data)==0xC),"bad off");
		static_assert((offsetof(Tyre,modelData)==0x58),"bad off");
		static_assert((offsetof(Tyre,status)==0x2E8),"bad off");
		static_assert((offsetof(Tyre,hub)==0x3A0),"bad off");
		static_assert((offsetof(Tyre,worldRotation)==0x3A8),"bad off");
		static_assert((offsetof(Tyre,unmodifiedContactPoint)==0x3E8),"bad off");
		static_assert((offsetof(Tyre,contactPoint)==0x3F4),"bad off");
		static_assert((offsetof(Tyre,contactNormal)==0x400),"bad off");
		static_assert((offsetof(Tyre,surfaceDef)==0x410),"bad off");
		static_assert((offsetof(Tyre,debugOutput)==0x418),"bad off");
		static_assert((offsetof(Tyre,absOverride)==0x41C),"bad off");
		static_assert((offsetof(Tyre,thermalModel)==0x420),"bad off");
		static_assert((offsetof(Tyre,compoundDefs)==0x500),"bad off");
		static_assert((offsetof(Tyre,onStepCompleted)==0x518),"bad off");
		static_assert((offsetof(Tyre,aiMult)==0x538),"bad off");
		static_assert((offsetof(Tyre,slipProvider)==0x540),"bad off");
		static_assert((offsetof(Tyre,externalInputs)==0x578),"bad off");
		static_assert((offsetof(Tyre,roadRight)==0x588),"bad off");
		static_assert((offsetof(Tyre,roadHeading)==0x594),"bad off");
		static_assert((offsetof(Tyre,useLoadForVKM)==0x5A0),"bad off");
		static_assert((offsetof(Tyre,driven)==0x5A1),"bad off");
		static_assert((offsetof(Tyre,rayCollisionProvider)==0x5A8),"bad off");
		static_assert((offsetof(Tyre,oldAngularVelocity)==0x5B0),"bad off");
		static_assert((offsetof(Tyre,totalSlideVelocity)==0x5B4),"bad off");
		static_assert((offsetof(Tyre,localWheelRotation)==0x5B8),"bad off");
		static_assert((offsetof(Tyre,worldPosition)==0x5F8),"bad off");
		static_assert((offsetof(Tyre,slidingVelocityY)==0x604),"bad off");
		static_assert((offsetof(Tyre,slidingVelocityX)==0x608),"bad off");
		static_assert((offsetof(Tyre,roadVelocityX)==0x60C),"bad off");
		static_assert((offsetof(Tyre,roadVelocityY)==0x610),"bad off");
		static_assert((offsetof(Tyre,totalHubVelocity)==0x614),"bad off");
		static_assert((offsetof(Tyre,rSlidingVelocityX)==0x618),"bad off");
		static_assert((offsetof(Tyre,rSlidingVelocityY)==0x61C),"bad off");
		static_assert((offsetof(Tyre,rayCaster)==0x620),"bad off");
		static_assert((offsetof(Tyre,index)==0x628),"bad off");
		static_assert((offsetof(Tyre,shakeGenerator)==0x630),"bad off");
		static_assert((offsetof(Tyre,car)==0x640),"bad off");
		static_assert((offsetof(Tyre,currentCompoundIndex)==0x648),"bad off");
		static_assert((offsetof(Tyre,tyreBlanketsOn)==0x64C),"bad off");
		static_assert((offsetof(Tyre,flatSpotK)==0x650),"bad off");
		static_assert((offsetof(Tyre,tyreModel)==0x658),"bad off");
		static_assert((offsetof(Tyre,scTM)==0x660),"bad off");
		static_assert((offsetof(Tyre,explosionTemperature)==0x848),"bad off");
		static_assert((offsetof(Tyre,blanketTemperature)==0x84C),"bad off");
		static_assert((offsetof(Tyre,pressureTemperatureGain)==0x850),"bad off");
		static_assert((offsetof(Tyre,localMX)==0x854),"bad off");
	};
};

//UDT: class ksgui::GUI @len=400 @vfcount=3
	//_Base: class IKeyEventListener @off=0 @len=8
	//_Tag 11
	//_Func: public void GUI(ksgui_GUI &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void GUI(GraphicsManager * graphics, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * aapplicationName, KeyboardManager * keyManager); @loc=static @len=2038 @rva=2431680
	//_Func: public void ~GUI(); @intro @virtual vtpo=0 vfid=2 @loc=static @len=424 @rva=2433760
	//_Data: this+0x8, Member, Type: bool, isActive
	//_Data: this+0x10, Member, Type: class GraphicsManager *, graphics
	//_Data: this+0x18, Member, Type: class std::shared_ptr<Font>, font
	//_Data: this+0x28, Member, Type: class INIReader, skinReader
	//_Data: this+0x90, Member, Type: int, version
	//_Data: this+0x98, Member, Type: class ksgui::Taskbar *, taskbar
	//_Data: this+0xA0, Member, Type: class std::vector<ksgui::Control *,std::allocator<ksgui::Control *> >, controls
	//_Data: this+0xB8, Member, Type: bool, isPointerVisible
	//_Func: public void preRender3D(float dt); @loc=static @len=305 @rva=2443200
	//_Func: public void render3D(float dt); @loc=static @len=777 @rva=2443520
	//_Func: public void render(float dt); @loc=static @len=21 @rva=2444304
	//_Func: public void addControl(ksgui_Control * c); @loc=static @len=95 @rva=2436688
	//_Func: public vec4f getSkinColor(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * key, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * value); @loc=static @len=171 @rva=2440800
	//_Func: public void setCapturedForm(ksgui_Form * form, int wx, int wy); @loc=static @len=58 @rva=2447040
	//_Func: public void serializeForms(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * filename); @loc=static @len=1651 @rva=2444640
	//_Func: public void deserializeForm(ksgui_Form * form); @loc=static @len=1027 @rva=2437008
	//_Func: public void attachToRenderWindowEvents(RenderWindow & window); @loc=static @len=214 @rva=2436784
	//_Func: public void setAudioEngine(AudioEngine * engine); @loc=static @len=8 @rva=2447024
	//_Func: public void playClick(); @loc=static @len=3 @rva=96368
	//_Func: public void setAllowFormSerialization(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void init3DMode(unsigned int width, unsigned int height, Node * referenceFrame, Camera * camera, vec3f posOffset); @loc=static @len=351 @rva=2440976
	//_Func: public void onMouseUp(OnMouseUpEvent & message); @loc=static @len=339 @rva=2442688
	//_Func: public void onMouseMove(OnMouseMoveEvent & message); @loc=static @len=662 @rva=2442016
	//_Func: public void onMouseDown(OnMouseDownEvent & message); @loc=static @len=141 @rva=2441872
	//_Func: public void onMouseWheelMoved(OnMouseWheelMovedEvent & message); @loc=static @len=145 @rva=2443040
	//_Func: public void onKeyChar(unsigned int key); @virtual vtpo=0 vfid=1 @loc=static @len=72 @rva=2441696
	//_Func: public void onKeyDown(OnKeyEvent & message); @virtual vtpo=0 vfid=0 @loc=static @len=85 @rva=2441776
	//_Func: public void renderVPointer(); @loc=static @len=296 @rva=2444336
	//_Data: this+0xB9, Member, Type: bool, allowFormSerialization
	//_Data: this+0xC0, Member, Type: class AudioEngine *, audioEngine
	//_Data: this+0xC8, Member, Type: class ksgui::Form *, capturedForm
	//_Data: this+0xD0, Member, Type: class vec2f, capturedPos
	//_Data: this+0xD8, Member, Type: class vec2f, oldGoodPos
	//_Data: this+0xE0, Member, Type: bool, moveApp
	//_Data: this+0xE1, Member, Type: bool, renderDrawingArea
	//_Data: this+0xE2, Member, Type: bool, allowOverlappingForms
	//_Data: this+0xE8, Member, Type: class ksgui::EventReplicator<OnMouseDownEvent>, onMouseDownReplicator
	//_Data: this+0xF8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, applicationName
	//_Data: this+0x118, Member, Type: class RenderTarget *, renderTarget
	//_Data: this+0x120, Member, Type: class Camera *, camera
	//_Data: this+0x128, Member, Type: class Node *, referenceFrameForOrientation
	//_Data: this+0x130, Member, Type: class Texture, txPointer
	//_Data: this+0x158, Member, Type: class vec2f, mousePos
	//_Data: this+0x160, Member, Type: float, mouseSpeed
	//_Data: this+0x164, Member, Type: class vec2f, oldMousePos
	//_Data: this+0x16C, Member, Type: class vec3f, positionOffset3D
	//_Data: this+0x178, Member, Type: int, virtualDesktop
	//_Data: this+0x180, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,ksgui::GUI::FormData,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,ksgui::GUI::FormData> > >, formDataMap
	//_Func: protected void doRender2D(float dt); @loc=static @len=325 @rva=2439632
	//_Func: protected void deserializeOldForm(ksgui_Form * form); @loc=static @len=306 @rva=2439312
	//_Func: protected void deserializeFormDesktop(ksgui_Form * form, int idDesktop); @loc=static @len=1257 @rva=2438048
	//_Func: protected void serializeMemoryForm(ksgui_Form * form); @loc=static @len=719 @rva=2446304
	//_Func: protected void fromForm(ksgui_GUI_FormData &  _arg0, ksgui_Form *  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: protected void fromIni(ksgui_GUI_FormData & data, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section); @loc=static @len=826 @rva=2439968
	//_Func: protected void fromMemory(ksgui_GUI_FormData &  _arg0, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public ksgui_GUI & operator=(ksgui_GUI &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_GUI)==400),"bad size");
		static_assert((offsetof(ksgui_GUI,isActive)==0x8),"bad off");
		static_assert((offsetof(ksgui_GUI,graphics)==0x10),"bad off");
		static_assert((offsetof(ksgui_GUI,font)==0x18),"bad off");
		static_assert((offsetof(ksgui_GUI,skinReader)==0x28),"bad off");
		static_assert((offsetof(ksgui_GUI,version)==0x90),"bad off");
		static_assert((offsetof(ksgui_GUI,taskbar)==0x98),"bad off");
		static_assert((offsetof(ksgui_GUI,controls)==0xA0),"bad off");
		static_assert((offsetof(ksgui_GUI,isPointerVisible)==0xB8),"bad off");
		static_assert((offsetof(ksgui_GUI,allowFormSerialization)==0xB9),"bad off");
		static_assert((offsetof(ksgui_GUI,audioEngine)==0xC0),"bad off");
		static_assert((offsetof(ksgui_GUI,capturedForm)==0xC8),"bad off");
		static_assert((offsetof(ksgui_GUI,capturedPos)==0xD0),"bad off");
		static_assert((offsetof(ksgui_GUI,oldGoodPos)==0xD8),"bad off");
		static_assert((offsetof(ksgui_GUI,moveApp)==0xE0),"bad off");
		static_assert((offsetof(ksgui_GUI,renderDrawingArea)==0xE1),"bad off");
		static_assert((offsetof(ksgui_GUI,allowOverlappingForms)==0xE2),"bad off");
		static_assert((offsetof(ksgui_GUI,onMouseDownReplicator)==0xE8),"bad off");
		static_assert((offsetof(ksgui_GUI,applicationName)==0xF8),"bad off");
		static_assert((offsetof(ksgui_GUI,renderTarget)==0x118),"bad off");
		static_assert((offsetof(ksgui_GUI,camera)==0x120),"bad off");
		static_assert((offsetof(ksgui_GUI,referenceFrameForOrientation)==0x128),"bad off");
		static_assert((offsetof(ksgui_GUI,txPointer)==0x130),"bad off");
		static_assert((offsetof(ksgui_GUI,mousePos)==0x158),"bad off");
		static_assert((offsetof(ksgui_GUI,mouseSpeed)==0x160),"bad off");
		static_assert((offsetof(ksgui_GUI,oldMousePos)==0x164),"bad off");
		static_assert((offsetof(ksgui_GUI,positionOffset3D)==0x16C),"bad off");
		static_assert((offsetof(ksgui_GUI,virtualDesktop)==0x178),"bad off");
		static_assert((offsetof(ksgui_GUI,formDataMap)==0x180),"bad off");
	};
};

//UDT: class PhysicsEngine @len=632 @vfcount=2
	//_Base: class ICollisionCallback @off=0 @len=8
	//_Func: public void PhysicsEngine(PhysicsEngine &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void PhysicsEngine(); @loc=static @len=2107 @rva=2499632
	//_Func: public void ~PhysicsEngine(); @virtual vtpo=0 vfid=0 @loc=static @len=334 @rva=2501872
	//_Data: this+0x8, Member, Type: class std::vector<Car *,std::allocator<Car *> >, cars
	//_Data: this+0x20, Member, Type: double, physicsTime
	//_Data: this+0x28, Member, Type: bool, validated
	//_Data: this+0x30, Member, Type: class Concurrency::concurrent_queue<ACPhysicsEvent,std::allocator<ACPhysicsEvent> >, eventQueue
	//_Data: this+0x58, Member, Type: class Event<OnFlagEvent>, evOnFlagEvent
	//_Data: this+0x70, Member, Type: class Event<double>, evOnStepCompleted
	//_Data: this+0x88, Member, Type: class Event<double>, evOnPreStep
	//_Data: this+0xA0, Member, Type: class Event<SessionInfo>, evOnNewSessionPhysics
	//_Data: this+0xB8, Member, Type: bool, allowTyreBlankets
	//_Data: this+0xC0, Member, Type: double, lockGearboxAtStartTimeMS
	//_Data: this+0xC8, Member, Type: float, fuelConsumptionRate
	//_Data: this+0xCC, Member, Type: float, tyreConsumptionRate
	//_Data: this+0xD0, Member, Type: int, allowedTyresOut
	//_Data: this+0xD4, Member, Type: enum PenaltyMode, penaltyMode
	//_Data: this+0xD8, Member, Type: struct PenaltyRules, penaltyRules
	//_Data: this+0xE0, Member, Type: class std::vector<SlipStream *,std::allocator<SlipStream *> >, slipStreams
	//_Data: this+0xF8, Member, Type: double, gameTime
	//_Data: this+0x100, Member, Type: float, ambientTemperature
	//_Data: this+0x104, Member, Type: float, roadTemperature
	//_Data: this+0x108, Member, Type: float, mechanicalDamageRate
	//_Data: this+0x110, Member, Type: struct PhysicsCPUTimes, physicsCPUTimes
	//_Data: this+0x138, Member, Type: const float, flatSpotFFGain
	//_Data: this+0x13C, Member, Type: struct SteerMzLowSpeedReduction, mzLowSpeedReduction
	//_Data: this+0x144, Member, Type: bool, isEngineStallEnabled
	//_Data: this+0x148, Member, Type: float, gyroWheelGain
	//_Data: this+0x14C, Member, Type: float, spinTorqueGain
	//_Data: static, [0155A770][0003:00047770], Static Member, Type: bool, isTestMode
	//_Data: static, [0155A771][0003:00047771], Static Member, Type: bool, autoDrive
	//_Data: static, [0155A774][0003:00047774], Static Member, Type: enum TestMode, testMode
	//_Data: this+0x150, Member, Type: float, damperMinValue
	//_Data: this+0x154, Member, Type: float, damperGain
	//_Data: this+0x158, Member, Type: struct Wind, wind
	//_Func: public void setWind(Speed speed, float directionDEG); @loc=static @len=433 @rva=2508192
	//_Func: public void setCurrentSessionStartTime(double st); @loc=static @len=9 @rva=2508176
	//_Func: public SessionInfo getCurrentSession(); @loc=optimized @len=0 @rva=0
	//_Func: public Track * getTrack(); @loc=optimized @len=0 @rva=0
	//_Func: public void setTrack(Track *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void removeCar(Car * c); @loc=static @len=73 @rva=2507328
	//_Func: public void step(float dt, double currentTime, double gt); @loc=static @len=2996 @rva=2508640
	//_Func: public void stepPaused(); @loc=static @len=87 @rva=2511648
	//_Func: public IPhysicsCore * getCore(); @loc=static @len=8 @rva=2505344
	//_Func: public void onCollisionCallBack(void * userData0, void * shape0, void * userData1, void * shape1, vec3f normal, vec3f pos, float depth); @virtual vtpo=0 vfid=1 @loc=static @len=398 @rva=2506784
	//_Func: public bool hasSessionStarted(double lag); @loc=static @len=17 @rva=2505840
	//_Func: public void addDebugLine(DebugLine &  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void addDebugString(DebugString &  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void setDebugVisualizer(IDebugVisualizer *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public double getOddTimeOffset(); @loc=static @len=35 @rva=2505360
	//_Func: public void addAdditionalPhysicsProvider(ICarPhysicsStateProvider * p); @loc=static @len=31 @rva=2505216
	//_Func: public void setSessionInfo(SessionInfo & info); @loc=static @len=34 @rva=2508128
	//_Func: public void setSessionStartTimeMS(double new_start_time); @loc=static @len=9 @rva=2508176
	//_Func: public void addLegalTyre(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & short_name); @loc=static @len=64 @rva=2505248
	//_Func: public bool isTyreLegal(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & short_name); @loc=static @len=234 @rva=2506544
	//_Func: public float getAirDensity(); @loc=static @len=29 @rva=2505312
	//_Func: public void writeTestResult(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & result); @loc=static @len=837 @rva=2512176
	//_Func: public void getPhysicsStates(std::vector<CarPhysicsState,std::allocator<CarPhysicsState> > & states, std::vector<std::vector<WingState,std::allocator<WingState> >,std::allocator<std::vector<WingState,std::allocator<WingState> > > > & wingStates); @loc=static @len=419 @rva=2505408
	//_Func: public void setDynamicTempData(DynamicTempData & data); @loc=static @len=96 @rva=1315344
	//_Data: this+0x170, Member, Type: struct SessionInfo, sessionInfo
	//_Data: this+0x190, Member, Type: class IPhysicsCore *, core
	//_Data: this+0x198, Member, Type: class Track *, track
	//_Data: this+0x1A0, Member, Type: class IDebugVisualizer *, debugVisualizer
	//_Data: this+0x1A8, Member, Type: unsigned int, stepCounter
	//_Data: this+0x1B0, Member, Type: class std::vector<ICarPhysicsStateProvider *,std::allocator<ICarPhysicsStateProvider *> >, additionalPhysicsProviders
	//_Data: this+0x1C8, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, legalTyreList
	//_Data: this+0x1E0, Member, Type: class std::unique_ptr<ThreadPool,std::default_delete<ThreadPool> >, pool
	//_Data: this+0x1E8, Member, Type: struct DynamicTempData, dynamicTemp
	//_Func: protected void validate(); @loc=optimized @len=0 @rva=0
	//_Func: protected void loadConfiguration(); @loc=optimized @len=0 @rva=0
	//_Func: protected void initLowSpeedFF(); @loc=static @len=670 @rva=2505872
	//_Func: protected void stepTemperatureCurve(); @loc=optimized @len=0 @rva=0
	//_Func: protected void stepWind(float dt); @loc=static @len=391 @rva=2511744
	//_Func: public PhysicsEngine & operator=(PhysicsEngine &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(PhysicsEngine)==632),"bad size");
		static_assert((offsetof(PhysicsEngine,cars)==0x8),"bad off");
		static_assert((offsetof(PhysicsEngine,physicsTime)==0x20),"bad off");
		static_assert((offsetof(PhysicsEngine,validated)==0x28),"bad off");
		static_assert((offsetof(PhysicsEngine,eventQueue)==0x30),"bad off");
		static_assert((offsetof(PhysicsEngine,evOnFlagEvent)==0x58),"bad off");
		static_assert((offsetof(PhysicsEngine,evOnStepCompleted)==0x70),"bad off");
		static_assert((offsetof(PhysicsEngine,evOnPreStep)==0x88),"bad off");
		static_assert((offsetof(PhysicsEngine,evOnNewSessionPhysics)==0xA0),"bad off");
		static_assert((offsetof(PhysicsEngine,allowTyreBlankets)==0xB8),"bad off");
		static_assert((offsetof(PhysicsEngine,lockGearboxAtStartTimeMS)==0xC0),"bad off");
		static_assert((offsetof(PhysicsEngine,fuelConsumptionRate)==0xC8),"bad off");
		static_assert((offsetof(PhysicsEngine,tyreConsumptionRate)==0xCC),"bad off");
		static_assert((offsetof(PhysicsEngine,allowedTyresOut)==0xD0),"bad off");
		static_assert((offsetof(PhysicsEngine,penaltyMode)==0xD4),"bad off");
		static_assert((offsetof(PhysicsEngine,penaltyRules)==0xD8),"bad off");
		static_assert((offsetof(PhysicsEngine,slipStreams)==0xE0),"bad off");
		static_assert((offsetof(PhysicsEngine,gameTime)==0xF8),"bad off");
		static_assert((offsetof(PhysicsEngine,ambientTemperature)==0x100),"bad off");
		static_assert((offsetof(PhysicsEngine,roadTemperature)==0x104),"bad off");
		static_assert((offsetof(PhysicsEngine,mechanicalDamageRate)==0x108),"bad off");
		static_assert((offsetof(PhysicsEngine,physicsCPUTimes)==0x110),"bad off");
		static_assert((offsetof(PhysicsEngine,flatSpotFFGain)==0x138),"bad off");
		static_assert((offsetof(PhysicsEngine,mzLowSpeedReduction)==0x13C),"bad off");
		static_assert((offsetof(PhysicsEngine,isEngineStallEnabled)==0x144),"bad off");
		static_assert((offsetof(PhysicsEngine,gyroWheelGain)==0x148),"bad off");
		static_assert((offsetof(PhysicsEngine,spinTorqueGain)==0x14C),"bad off");
		static_assert((offsetof(PhysicsEngine,damperMinValue)==0x150),"bad off");
		static_assert((offsetof(PhysicsEngine,damperGain)==0x154),"bad off");
		static_assert((offsetof(PhysicsEngine,wind)==0x158),"bad off");
		static_assert((offsetof(PhysicsEngine,sessionInfo)==0x170),"bad off");
		static_assert((offsetof(PhysicsEngine,core)==0x190),"bad off");
		static_assert((offsetof(PhysicsEngine,track)==0x198),"bad off");
		static_assert((offsetof(PhysicsEngine,debugVisualizer)==0x1A0),"bad off");
		static_assert((offsetof(PhysicsEngine,stepCounter)==0x1A8),"bad off");
		static_assert((offsetof(PhysicsEngine,additionalPhysicsProviders)==0x1B0),"bad off");
		static_assert((offsetof(PhysicsEngine,legalTyreList)==0x1C8),"bad off");
		static_assert((offsetof(PhysicsEngine,pool)==0x1E0),"bad off");
		static_assert((offsetof(PhysicsEngine,dynamicTemp)==0x1E8),"bad off");
	};
};

//UDT: class Sim @len=720 @multibase=2
	//_Base: class GameObject @off=0 @len=88
	//_Base: class IKeyEventListener @off=88 @len=8
	//_Func: public void Sim(Sim &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Sim(Game * igame); @loc=static @len=13670 @rva=1646704
	//_Func: public void ~Sim(); @virtual vtpo=0 vfid=0 @loc=static @len=810 @rva=1660624
	//_Data: this+0x60, Member, Type: class Event<OnNewCarLoadedEvent>, evNewCarLoaded
	//_Data: this+0x78, Member, Type: class Event<OnReplayStatusChanged>, evOnReplayStatusChanged
	//_Data: this+0x90, Member, Type: class Event<bool>, evOnPauseModeChanged
	//_Data: this+0xA8, Member, Type: class Event<CollisionEvent>, evOnCollisionEvent
	//_Data: this+0xC0, Member, Type: class Event<OnNewSessionEvent>, evOnNewSession
	//_Data: this+0xD8, Member, Type: struct HDRLevels, hdrLevels
	//_Data: this+0xE0, Member, Type: class CommandManager, commandManager
	//_Data: this+0xF0, Member, Type: class OptionsManager, optionsManager
	//_Data: this+0x108, Member, Type: class ksgui::GameScreen *, gameScreen
	//_Data: this+0x110, Member, Type: class PauseMenu *, pauseMenu
	//_Data: this+0x118, Member, Type: class ESCMenu *, escMenu
	//_Data: this+0x120, Member, Type: class Node *, rootNode
	//_Data: this+0x128, Member, Type: class Node *, trackNode
	//_Data: this+0x130, Member, Type: class CarNodeSorter *, carsNode
	//_Data: this+0x138, Member, Type: class Node *, skidMarkNode
	//_Data: this+0x140, Member, Type: class Node *, particlesNode
	//_Data: this+0x148, Member, Type: class NodeEvent *, carFakeShadowsNode
	//_Data: this+0x150, Member, Type: class TrackAvatar *, track
	//_Data: this+0x158, Member, Type: class Node *, blurredNode
	//_Data: this+0x160, Member, Type: class Node *, unblurredNode
	//_Data: this+0x168, Member, Type: class DrivingAssistManager *, drivingAidsManager
	//_Data: this+0x170, Member, Type: class NodeEvent *, renderFinishedNodeEvent
	//_Data: this+0x178, Member, Type: class NodeEvent *, beforeCarsNode
	//_Data: this+0x180, Member, Type: class MirrorTextureRenderer *, mirrorTextureRenderer
	//_Data: this+0x188, Member, Type: class ACCameraManager *, cameraManager
	//_Data: this+0x190, Member, Type: class NodeDirtCamera *, cameraDirtNode
	//_Data: this+0x198, Member, Type: class SystemMessage *, systemMessage
	//_Data: this+0x1A0, Member, Type: class SystemNotification *, systemNotification
	//_Data: this+0x1A8, Member, Type: class RaceManager *, raceManager
	//_Data: this+0x1B0, Member, Type: class ReplayManager *, replayManager
	//_Data: this+0x1B8, Member, Type: class PhysicsAvatar *, physicsAvatar
	//_Data: this+0x1C0, Member, Type: class ScreenCapturer *, screenCapturer
	//_Data: this+0x1C8, Member, Type: class ACClient *, client
	//_Data: this+0x1D0, Member, Type: class PitStop *, pitStop
	//_Data: this+0x1D8, Member, Type: class TimeLimitedTest *, timeLimitedTest
	//_Data: this+0x1E0, Member, Type: bool, useProView
	//_Data: this+0x1E1, Member, Type: bool, isVrConnected
	//_Data: this+0x1E2, Member, Type: bool, isRoomVR
	//_Data: this+0x1E3, Member, Type: bool, isVirtualMirrorForced
	//_Data: static, [01559C12][0003:00046C12], Static Member, Type: bool, benchmarkMode
	//_Data: static, [0151AA50][0003:00007A50], Static Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, acVersionString
	//_Data: this+0x1E4, Member, Type: bool, useMousePitstop
	//_Data: this+0x1E8, Member, Type: class QuickMenu *, quickMenu
	//_Data: this+0x1F0, Member, Type: class MicroSectors *, microSectors
	//_Func: public void applyCustomWeather(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=648 @rva=1671184
	//_Func: public void startGame(); @loc=static @len=203 @rva=1698560
	//_Func: public Camera * getSceneCamera(); @loc=static @len=8 @rva=1676848
	//_Func: public void update(float deltaT); @virtual vtpo=0 vfid=1 @loc=static @len=2483 @rva=1699728
	//_Func: public void render(float deltaT); @virtual vtpo=0 vfid=2 @loc=static @len=58 @rva=1696944
	//_Func: public void renderHUD(float deltaT); @virtual vtpo=0 vfid=3 @loc=static @len=120 @rva=1697008
	//_Func: public CarAvatar * addCar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & model, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin); @loc=static @len=565 @rva=1669280
	//_Func: public CarAvatar * addRemoteSlaveCar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public CarAvatar * addNetCar(ClientRemoteCarDef & desc, ACClient * client); @loc=static @len=986 @rva=1670192
	//_Func: public void renderScene(float deltaT); @loc=static @len=319 @rva=1697136
	//_Func: public void loadTrack(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & tname, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, int playerPitPosition); @loc=static @len=637 @rva=1680576
	//_Func: public CarAvatar * getCar(unsigned int index); @loc=static @len=41 @rva=1676000
	//_Func: public unsigned int getCarsCount(); @loc=static @len=19 @rva=210640
	//_Func: public int getConnectedCarsCount(); @loc=optimized @len=0 @rva=0
	//_Func: public void onPostLoad(); @loc=static @len=152 @rva=1695776
	//_Func: public CameraForward * createCamera(INIReaderDocuments & videoIni, float useBlur); @loc=static @len=2642 @rva=1671904
	//_Func: public void setSplashMessage(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & msg); @loc=static @len=125 @rva=1697888
	//_Func: public void setSplashLoadingCar(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & carUnixName); @loc=static @len=125 @rva=1697760
	//_Func: public void shutdown(); @virtual vtpo=0 vfid=5 @loc=static @len=286 @rva=1698272
	//_Func: public void onESCMenuTriggeredEvent(OnESCMenuTriggered  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool isInTripleScreenMode(); @loc=static @len=8 @rva=1680448
	//_Func: public unsigned int nextCar(unsigned int anIndex); @loc=static @len=455 @rva=1681216
	//_Func: public unsigned int previousCar(unsigned int anIndex); @loc=static @len=454 @rva=1696128
	//_Data: static, [01559C13][0003:00046C13], Static Member, Type: bool, forceOnline
	//_Func: public void writeOutputJson(); @loc=static @len=115 @rva=1702224
	//_Func: public bool isDisplayingResults(); @loc=static @len=18 @rva=503088
	//_Func: public void displayResults(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void displayLeaderboard(bool value); @loc=static @len=54 @rva=280848
	//_Func: public void onKeyDown(OnKeyEvent & message); @virtual vtpo=88 vfid=0 @loc=static @len=14045 @rva=1681728
	//_Func: public void onKeyChar(unsigned int key); @virtual vtpo=88 vfid=1 @loc=static @len=3 @rva=96368
	//_Func: public void onReplayModeChanged(OnReplayStatusChanged & message); @loc=static @len=182 @rva=1695936
	//_Func: public Console & getConsole(); @loc=static @len=8 @rva=1676048
	//_Func: public TrackData getTrackData(); @loc=static @len=339 @rva=1676864
	//_Func: public void setPauseMode(bool mode); @loc=static @len=212 @rva=1697488
	//_Func: public void activateGameGui(); @loc=static @len=81 @rva=1669088
	//_Func: public void activatePauseMenu(); @loc=static @len=81 @rva=1669184
	//_Func: public void activateEscMenu(); @loc=static @len=81 @rva=1668992
	//_Func: public void setFocusedCarIndex(unsigned int aCarIndex); @loc=static @len=32 @rva=1697456
	//_Func: public unsigned int getFocusedCarIndex(); @loc=static @len=7 @rva=1676080
	//_Func: public void executeOnMainThread(std::function<void __cdecl(void)> * f); @loc=static @len=129 @rva=1675840
	//_Func: public void unloadMeshResources(Node * node); @loc=static @len=444 @rva=1699280
	//_Func: public Texture * getNationFlag(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & nationCode); @loc=static @len=731 @rva=1676112
	//_Func: public bool isVrRoom(); @loc=static @len=8 @rva=888336
	//_Data: this+0x1F8, Member, Type: bool, serializeForms
	//_Data: this+0x1FC, Member, Type: int, connectedCarsCount
	//_Data: this+0x200, Member, Type: bool, allowFreeCamera
	//_Data: this+0x201, Member, Type: bool, tripleScreenMode
	//_Data: this+0x202, Member, Type: bool, audioPerformanceSpew
	//_Data: this+0x208, Member, Type: class std::vector<CarAvatar *,std::allocator<CarAvatar *> >, cars
	//_Data: this+0x220, Member, Type: unsigned int, focusedCarIndex
	//_Data: this+0x224, Member, Type: float, cameraFadeTime
	//_Data: this+0x228, Member, Type: class Console *, console
	//_Data: this+0x230, Member, Type: class SimScreen *, simScreen
	//_Data: this+0x238, Member, Type: class CameraForward *, sceneCamera
	//_Data: this+0x240, Member, Type: class SkyBox *, skyBox
	//_Data: this+0x248, Member, Type: class ACErrorHandler *, errorHandler
	//_Data: this+0x250, Member, Type: class DebugVisualizer *, debugVisualizer
	//_Data: this+0x258, Member, Type: class ksgui::FormRenderStats *, formRenderStats
	//_Data: this+0x260, Member, Type: class EndSessionDisplayer *, endSessionDisplayer
	//_Data: this+0x268, Member, Type: class SessionLeaderboard *, leaderboard
	//_Data: this+0x270, Member, Type: class Trigger, changingCameraTrigger
	//_Data: this+0x280, Member, Type: class VirtualMirrorRenderer *, virtualMirrorRenderer
	//_Data: this+0x288, Member, Type: class BufferedChannel<std::function<void __cdecl(void)> >, chFunctions
	//_Data: this+0x2B0, Member, Type: float, lastDT
	//_Data: this+0x2B8, Member, Type: class NodeEvent *, node3DGUI
	//_Func: protected void initSceneGraph(); @loc=static @len=1321 @rva=1678704
	//_Func: protected void stepPhysicsEvent(); @loc=static @len=512 @rva=1698768
	//_Func: protected void initCubemaps(); @loc=static @len=860 @rva=1677216
	//_Func: protected void renderVirtualMirror(); @loc=optimized @len=0 @rva=0
	//_Func: protected void initHDRLevels(); @loc=static @len=616 @rva=1678080
	//_Func: protected void addForm(ksgui_Form * aForm, bool isInDevMode); @loc=static @len=82 @rva=1669856
	//_Func: protected void initStaticCubemap(); @loc=static @len=413 @rva=1680032
	//_Data: this+0x2C0, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,Texture,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,Texture> > >, flagMapCache
	//_Tag 11
	//_Func: public Sim & operator=(Sim &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Sim)==720),"bad size");
		static_assert((offsetof(Sim,evNewCarLoaded)==0x60),"bad off");
		static_assert((offsetof(Sim,evOnReplayStatusChanged)==0x78),"bad off");
		static_assert((offsetof(Sim,evOnPauseModeChanged)==0x90),"bad off");
		static_assert((offsetof(Sim,evOnCollisionEvent)==0xA8),"bad off");
		static_assert((offsetof(Sim,evOnNewSession)==0xC0),"bad off");
		static_assert((offsetof(Sim,hdrLevels)==0xD8),"bad off");
		static_assert((offsetof(Sim,commandManager)==0xE0),"bad off");
		static_assert((offsetof(Sim,optionsManager)==0xF0),"bad off");
		static_assert((offsetof(Sim,gameScreen)==0x108),"bad off");
		static_assert((offsetof(Sim,pauseMenu)==0x110),"bad off");
		static_assert((offsetof(Sim,escMenu)==0x118),"bad off");
		static_assert((offsetof(Sim,rootNode)==0x120),"bad off");
		static_assert((offsetof(Sim,trackNode)==0x128),"bad off");
		static_assert((offsetof(Sim,carsNode)==0x130),"bad off");
		static_assert((offsetof(Sim,skidMarkNode)==0x138),"bad off");
		static_assert((offsetof(Sim,particlesNode)==0x140),"bad off");
		static_assert((offsetof(Sim,carFakeShadowsNode)==0x148),"bad off");
		static_assert((offsetof(Sim,track)==0x150),"bad off");
		static_assert((offsetof(Sim,blurredNode)==0x158),"bad off");
		static_assert((offsetof(Sim,unblurredNode)==0x160),"bad off");
		static_assert((offsetof(Sim,drivingAidsManager)==0x168),"bad off");
		static_assert((offsetof(Sim,renderFinishedNodeEvent)==0x170),"bad off");
		static_assert((offsetof(Sim,beforeCarsNode)==0x178),"bad off");
		static_assert((offsetof(Sim,mirrorTextureRenderer)==0x180),"bad off");
		static_assert((offsetof(Sim,cameraManager)==0x188),"bad off");
		static_assert((offsetof(Sim,cameraDirtNode)==0x190),"bad off");
		static_assert((offsetof(Sim,systemMessage)==0x198),"bad off");
		static_assert((offsetof(Sim,systemNotification)==0x1A0),"bad off");
		static_assert((offsetof(Sim,raceManager)==0x1A8),"bad off");
		static_assert((offsetof(Sim,replayManager)==0x1B0),"bad off");
		static_assert((offsetof(Sim,physicsAvatar)==0x1B8),"bad off");
		static_assert((offsetof(Sim,screenCapturer)==0x1C0),"bad off");
		static_assert((offsetof(Sim,client)==0x1C8),"bad off");
		static_assert((offsetof(Sim,pitStop)==0x1D0),"bad off");
		static_assert((offsetof(Sim,timeLimitedTest)==0x1D8),"bad off");
		static_assert((offsetof(Sim,useProView)==0x1E0),"bad off");
		static_assert((offsetof(Sim,isVrConnected)==0x1E1),"bad off");
		static_assert((offsetof(Sim,isRoomVR)==0x1E2),"bad off");
		static_assert((offsetof(Sim,isVirtualMirrorForced)==0x1E3),"bad off");
		static_assert((offsetof(Sim,useMousePitstop)==0x1E4),"bad off");
		static_assert((offsetof(Sim,quickMenu)==0x1E8),"bad off");
		static_assert((offsetof(Sim,microSectors)==0x1F0),"bad off");
		static_assert((offsetof(Sim,serializeForms)==0x1F8),"bad off");
		static_assert((offsetof(Sim,connectedCarsCount)==0x1FC),"bad off");
		static_assert((offsetof(Sim,allowFreeCamera)==0x200),"bad off");
		static_assert((offsetof(Sim,tripleScreenMode)==0x201),"bad off");
		static_assert((offsetof(Sim,audioPerformanceSpew)==0x202),"bad off");
		static_assert((offsetof(Sim,cars)==0x208),"bad off");
		static_assert((offsetof(Sim,focusedCarIndex)==0x220),"bad off");
		static_assert((offsetof(Sim,cameraFadeTime)==0x224),"bad off");
		static_assert((offsetof(Sim,console)==0x228),"bad off");
		static_assert((offsetof(Sim,simScreen)==0x230),"bad off");
		static_assert((offsetof(Sim,sceneCamera)==0x238),"bad off");
		static_assert((offsetof(Sim,skyBox)==0x240),"bad off");
		static_assert((offsetof(Sim,errorHandler)==0x248),"bad off");
		static_assert((offsetof(Sim,debugVisualizer)==0x250),"bad off");
		static_assert((offsetof(Sim,formRenderStats)==0x258),"bad off");
		static_assert((offsetof(Sim,endSessionDisplayer)==0x260),"bad off");
		static_assert((offsetof(Sim,leaderboard)==0x268),"bad off");
		static_assert((offsetof(Sim,changingCameraTrigger)==0x270),"bad off");
		static_assert((offsetof(Sim,virtualMirrorRenderer)==0x280),"bad off");
		static_assert((offsetof(Sim,chFunctions)==0x288),"bad off");
		static_assert((offsetof(Sim,lastDT)==0x2B0),"bad off");
		static_assert((offsetof(Sim,node3DGUI)==0x2B8),"bad off");
		static_assert((offsetof(Sim,flagMapCache)==0x2C0),"bad off");
	};
};

//UDT: class PhysicsCarStateProvider @len=16 @vfcount=3
	//_Base: class ICarPhysicsStateProvider @off=0 @len=8
	//_Func: public void PhysicsCarStateProvider(PhysicsCarStateProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void PhysicsCarStateProvider(Car * car); @loc=static @len=18 @rva=1190384
	//_Func: public void ~PhysicsCarStateProvider(); @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void getPhysicsState(CarPhysicsState & physicsState); @virtual vtpo=0 vfid=1 @loc=static @len=9 @rva=1190464
	//_Func: public void getWingState(std::vector<WingState,std::allocator<WingState> > & wingStatus); @virtual vtpo=0 vfid=2 @loc=static @len=9 @rva=1190480
	//_Func: public mat44f getBodyMatrix(); @loc=optimized @len=0 @rva=0
	//_Func: public mat44f getSuspensionMatrix(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public mat44f getTyreMatrix(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x8, Member, Type: class Car *, car
	//_Func: public PhysicsCarStateProvider & operator=(PhysicsCarStateProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(PhysicsCarStateProvider)==16),"bad size");
		static_assert((offsetof(PhysicsCarStateProvider,car)==0x8),"bad off");
	};
};

//UDT: class ksgui::Graph @len=440 @vfcount=21
	//_Base: class ksgui::Control @off=0 @len=376
	//_Func: public void Graph(ksgui_Graph &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Graph(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * iname, ksgui_GUI * igui); @loc=static @len=193 @rva=2388336
	//_Func: public void ~Graph(); @virtual vtpo=0 vfid=0 @loc=static @len=163 @rva=2388544
	//_Data: this+0x178, Member, Type: float, minY
	//_Data: this+0x17C, Member, Type: float, maxY
	//_Data: this+0x180, Member, Type: unsigned int, maxValuesCount
	//_Data: this+0x184, Member, Type: bool, autoAdjustMaxValue
	//_Func: public void render(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=690 @rva=2390240
	//_Func: public void addSerie(vec3f color); @loc=static @len=221 @rva=2389536
	//_Func: public void addValue(unsigned int serieIndex, float v); @loc=static @len=92 @rva=2389760
	//_Func: public void clearSerie(unsigned int serieIndex); @loc=static @len=66 @rva=2389856
	//_Func: public void addReferenceAxis(ksgui_GraphReferenceAxis & axis); @loc=static @len=12 @rva=2389520
	//_Func: public int getSeriesCount(); @loc=static @len=19 @rva=2389936
	//_Func: public int getSeriesValuesCount(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getValuesAtPercentage(int serieIndex, float perc); @loc=static @len=68 @rva=2389968
	//_Func: public float getValuesAt(int  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Data: this+0x188, Member, Type: class std::vector<ksgui::ValueSerie *,std::allocator<ksgui::ValueSerie *> >, series
	//_Data: this+0x1A0, Member, Type: class std::vector<ksgui::GraphReferenceAxis,std::allocator<ksgui::GraphReferenceAxis> >, axes
	//_Data: static, [0155A6E0][0003:000476E0], Static Member, Type: class GLRenderer *, localGL
	//_Func: protected void checkMaxValuesCount(); @loc=optimized @len=0 @rva=0
	//_Func: protected void renderAxes(GLRenderer & gl); @loc=static @len=111 @rva=2390944
	//_Func: public ksgui_Graph & operator=(ksgui_Graph &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ksgui_Graph)==440),"bad size");
		static_assert((offsetof(ksgui_Graph,minY)==0x178),"bad off");
		static_assert((offsetof(ksgui_Graph,maxY)==0x17C),"bad off");
		static_assert((offsetof(ksgui_Graph,maxValuesCount)==0x180),"bad off");
		static_assert((offsetof(ksgui_Graph,autoAdjustMaxValue)==0x184),"bad off");
		static_assert((offsetof(ksgui_Graph,series)==0x188),"bad off");
		static_assert((offsetof(ksgui_Graph,axes)==0x1A0),"bad off");
	};
};

//UDT: struct BrakeSystem @len=1008
	//_Data: this+0x0, Member, Type: float, frontBias
	//_Data: this+0x4, Member, Type: float, brakePowerMultiplier
	//_Data: this+0x8, Member, Type: float, electronicOverride
	//_Data: this+0xC, Member, Type: float, handBrakeTorque
	//_Data: this+0x10, Member, Type: float, ebbInstant
	//_Data: this+0x18, Member, Type: struct BrakeDisc[0x4], discs
	//_Data: this+0x258, Member, Type: float, limitDown
	//_Data: this+0x25C, Member, Type: float, limitUp
	//_Data: this+0x260, Member, Type: float, rearCorrectionTorque
	//_Func: public void init(Car * car); @loc=static @len=466 @rva=2676368
	//_Func: public void step(float dt); @loc=static @len=731 @rva=2680384
	//_Func: public float getBrakePower(); @loc=static @len=9 @rva=2676256
	//_Func: public void setFrontBias(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setManualFrontBias(int value); @loc=static @len=107 @rva=2680272
	//_Func: public float getFrontBias(); @loc=static @len=22 @rva=2676272
	//_Func: public void reset(); @loc=static @len=71 @rva=2679952
	//_Func: public bool isUsingEBB(); @loc=static @len=12 @rva=2879392
	//_Func: public void activateTempRunFile(bool mode); @loc=static @len=543 @rva=2675712
	//_Data: this+0x268, Member, Type: class Car *, car
	//_Data: this+0x270, Member, Type: float, brakePower
	//_Data: this+0x274, Member, Type: float, biasOverride
	//_Data: this+0x278, Member, Type: bool, hasCockpitBias
	//_Data: this+0x27C, Member, Type: float, biasStep
	//_Data: this+0x280, Member, Type: enum EBBMode, ebbMode
	//_Data: this+0x284, Member, Type: float, ebbFrontMultiplier
	//_Data: this+0x288, Member, Type: struct SteerBrake, steerBrake
	//_Data: this+0x2B8, Member, Type: bool, hasBrakeTempsData
	//_Data: this+0x2C0, Member, Type: class std::basic_ofstream<char,std::char_traits<char> >, tempRunFile
	//_Data: this+0x3C8, Member, Type: class DynamicController, ebbController
	//_Func: protected void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * dataPath); @loc=static @len=3091 @rva=2676848
	//_Func: protected void stepTemps(float dt); @loc=static @len=417 @rva=2681120
	//_Func: protected void saveTempsRunFame(); @loc=static @len=227 @rva=2680032
	//_Func: public void BrakeSystem(BrakeSystem &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void BrakeSystem(); @loc=static @len=220 @rva=2539040
	//_Func: public void ~BrakeSystem(); @loc=static @len=93 @rva=2547824
	//_Func: public BrakeSystem & operator=(BrakeSystem &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(BrakeSystem)==1008),"bad size");
		static_assert((offsetof(BrakeSystem,frontBias)==0x0),"bad off");
		static_assert((offsetof(BrakeSystem,brakePowerMultiplier)==0x4),"bad off");
		static_assert((offsetof(BrakeSystem,electronicOverride)==0x8),"bad off");
		static_assert((offsetof(BrakeSystem,handBrakeTorque)==0xC),"bad off");
		static_assert((offsetof(BrakeSystem,ebbInstant)==0x10),"bad off");
		static_assert((offsetof(BrakeSystem,discs)==0x18),"bad off");
		static_assert((offsetof(BrakeSystem,limitDown)==0x258),"bad off");
		static_assert((offsetof(BrakeSystem,limitUp)==0x25C),"bad off");
		static_assert((offsetof(BrakeSystem,rearCorrectionTorque)==0x260),"bad off");
		static_assert((offsetof(BrakeSystem,car)==0x268),"bad off");
		static_assert((offsetof(BrakeSystem,brakePower)==0x270),"bad off");
		static_assert((offsetof(BrakeSystem,biasOverride)==0x274),"bad off");
		static_assert((offsetof(BrakeSystem,hasCockpitBias)==0x278),"bad off");
		static_assert((offsetof(BrakeSystem,biasStep)==0x27C),"bad off");
		static_assert((offsetof(BrakeSystem,ebbMode)==0x280),"bad off");
		static_assert((offsetof(BrakeSystem,ebbFrontMultiplier)==0x284),"bad off");
		static_assert((offsetof(BrakeSystem,steerBrake)==0x288),"bad off");
		static_assert((offsetof(BrakeSystem,hasBrakeTempsData)==0x2B8),"bad off");
		static_assert((offsetof(BrakeSystem,tempRunFile)==0x2C0),"bad off");
		static_assert((offsetof(BrakeSystem,ebbController)==0x3C8),"bad off");
	};
};

//UDT: class ERS @len=688 @multibase=2
	//_Base: class ITorqueGenerator @off=0 @len=8
	//_Base: class ICoastGenerator @off=8 @len=8
	//_Func: public void ERS(ERS &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ERS(); @loc=static @len=275 @rva=2692352
	//_Func: public void ~ERS(); @virtual vtpo=0 vfid=0 @loc=static @len=279 @rva=2692816
	//_Data: this+0x10, Member, Type: bool, present
	//_Data: this+0x14, Member, Type: float, kineticRecovery
	//_Data: this+0x18, Member, Type: struct ERSStatus, status
	//_Data: this+0x20, Member, Type: bool, isHeatCharginBattery
	//_Data: this+0x28, Member, Type: class std::vector<ERSPowerController,std::allocator<ERSPowerController> >, ersPowerControllers
	//_Data: this+0x40, Member, Type: class std::vector<ERSPowerController,std::allocator<ERSPowerController> >, ersPowerControllersFront
	//_Data: this+0x58, Member, Type: int, defaultPowerControllerIndex
	//_Data: this+0x5C, Member, Type: bool, isCharging
	//_Data: this+0x5D, Member, Type: struct ERSCockpitControls, cockpitControls
	//_Func: public void init(Car * car); @loc=static @len=4756 @rva=2694192
	//_Func: public void step(float dt); @loc=static @len=1505 @rva=2699488
	//_Func: public float getOutputTorque(); @virtual vtpo=0 vfid=1 @loc=static @len=82 @rva=2694096
	//_Func: public float getCoastTorque(); @virtual vtpo=8 vfid=1 @loc=static @len=139 @rva=2693952
	//_Func: public void reset(); @loc=static @len=28 @rva=2699168
	//_Func: public float getCharge(); @loc=optimized @len=0 @rva=0
	//_Func: public float getInput(); @loc=optimized @len=0 @rva=0
	//_Func: public float getCurrentJ(); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaxJ(); @loc=optimized @len=0 @rva=0
	//_Func: public void setPowerController(int index); @loc=static @len=287 @rva=2699200
	//_Data: this+0x60, Member, Type: class Car *, car
	//_Data: this+0x68, Member, Type: double, chargeK
	//_Data: this+0x70, Member, Type: double, dischargeK
	//_Data: this+0x78, Member, Type: double, dischargeKFront
	//_Data: this+0x80, Member, Type: bool, hasButtonOverride
	//_Data: this+0x88, Member, Type: class Curve, torqueLUT
	//_Data: this+0x108, Member, Type: class Curve, coastLUT
	//_Data: this+0x188, Member, Type: class DynamicController, controller
	//_Data: this+0x1B0, Member, Type: class DynamicController, controllerFront
	//_Data: this+0x1D8, Member, Type: double, charge
	//_Data: this+0x1E0, Member, Type: float, maxJ
	//_Data: this+0x1E4, Member, Type: float, currentJ
	//_Data: this+0x1E8, Member, Type: float, input
	//_Data: this+0x1F0, Member, Type: double, heatChargeK
	//_Data: this+0x1F8, Member, Type: float, heatTorque
	//_Data: this+0x1FC, Member, Type: float, rearCorrectionTorque
	//_Data: this+0x200, Member, Type: class DynamicController, frontController
	//_Data: this+0x228, Member, Type: class Curve, frontTorqueLUT
	//_Data: this+0x2A8, Member, Type: float, frontTorqueVectoringBias
	//_Func: public ERS & operator=(ERS &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ERS)==688),"bad size");
		static_assert((offsetof(ERS,present)==0x10),"bad off");
		static_assert((offsetof(ERS,kineticRecovery)==0x14),"bad off");
		static_assert((offsetof(ERS,status)==0x18),"bad off");
		static_assert((offsetof(ERS,isHeatCharginBattery)==0x20),"bad off");
		static_assert((offsetof(ERS,ersPowerControllers)==0x28),"bad off");
		static_assert((offsetof(ERS,ersPowerControllersFront)==0x40),"bad off");
		static_assert((offsetof(ERS,defaultPowerControllerIndex)==0x58),"bad off");
		static_assert((offsetof(ERS,isCharging)==0x5C),"bad off");
		static_assert((offsetof(ERS,cockpitControls)==0x5D),"bad off");
		static_assert((offsetof(ERS,car)==0x60),"bad off");
		static_assert((offsetof(ERS,chargeK)==0x68),"bad off");
		static_assert((offsetof(ERS,dischargeK)==0x70),"bad off");
		static_assert((offsetof(ERS,dischargeKFront)==0x78),"bad off");
		static_assert((offsetof(ERS,hasButtonOverride)==0x80),"bad off");
		static_assert((offsetof(ERS,torqueLUT)==0x88),"bad off");
		static_assert((offsetof(ERS,coastLUT)==0x108),"bad off");
		static_assert((offsetof(ERS,controller)==0x188),"bad off");
		static_assert((offsetof(ERS,controllerFront)==0x1B0),"bad off");
		static_assert((offsetof(ERS,charge)==0x1D8),"bad off");
		static_assert((offsetof(ERS,maxJ)==0x1E0),"bad off");
		static_assert((offsetof(ERS,currentJ)==0x1E4),"bad off");
		static_assert((offsetof(ERS,input)==0x1E8),"bad off");
		static_assert((offsetof(ERS,heatChargeK)==0x1F0),"bad off");
		static_assert((offsetof(ERS,heatTorque)==0x1F8),"bad off");
		static_assert((offsetof(ERS,rearCorrectionTorque)==0x1FC),"bad off");
		static_assert((offsetof(ERS,frontController)==0x200),"bad off");
		static_assert((offsetof(ERS,frontTorqueLUT)==0x228),"bad off");
		static_assert((offsetof(ERS,frontTorqueVectoringBias)==0x2A8),"bad off");
	};
};

//UDT: class Wing @len=784
	//_Data: this+0x0, Member, Type: struct WingData, data
	//_Data: this+0x250, Member, Type: struct WingState, status
	//_Data: this+0x298, Member, Type: class std::vector<DynamicWingController,std::allocator<DynamicWingController> >, dynamicControllers
	//_Data: this+0x2B0, Member, Type: class Car *, car
	//_Data: this+0x2B8, Member, Type: class RaceEngineer, engineer
	//_Data: this+0x2D0, Member, Type: float[0x5], damageCL
	//_Data: this+0x2E4, Member, Type: float[0x5], damageCD
	//_Data: this+0x2F8, Member, Type: bool, hasDamage
	//_Data: this+0x2FC, Member, Type: struct WingOverrideDef, overrideStatus
	//_Data: this+0x304, Member, Type: const float, SPEED_DAMAGE_COEFF
	//_Data: this+0x308, Member, Type: const float, SURFACE_DAMAGE_COEFF
	//_Func: public void Wing(Wing & __that); @loc=static @len=294 @rva=2839136
	//_Func: public void Wing(Car * a_car, INIReader & ini, int index, bool isVertical); @loc=static @len=3958 @rva=2822976
	//_Func: public void ~Wing(); @loc=static @len=61 @rva=2827120
	//_Func: public void step(float dt); @loc=static @len=515 @rva=2829248
	//_Func: public void setOverrideAngle(float iangle); @loc=static @len=16 @rva=2829232
	//_Func: public void clearOverrides(); @loc=static @len=8 @rva=2829168
	//_Func: public float getCurrentModifiedAngle(); @loc=static @len=35 @rva=2829184
	//_Func: private void stepDynamicControllers(float dt); @loc=static @len=165 @rva=2829776
	//_Func: private void addDrag(vec3f & lv); @loc=static @len=776 @rva=2827296
	//_Func: private void addLift(vec3f & lv); @loc=static @len=1088 @rva=2828080
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Wing)==784),"bad size");
		static_assert((offsetof(Wing,data)==0x0),"bad off");
		static_assert((offsetof(Wing,status)==0x250),"bad off");
		static_assert((offsetof(Wing,dynamicControllers)==0x298),"bad off");
		static_assert((offsetof(Wing,car)==0x2B0),"bad off");
		static_assert((offsetof(Wing,engineer)==0x2B8),"bad off");
		static_assert((offsetof(Wing,damageCL)==0x2D0),"bad off");
		static_assert((offsetof(Wing,damageCD)==0x2E4),"bad off");
		static_assert((offsetof(Wing,hasDamage)==0x2F8),"bad off");
		static_assert((offsetof(Wing,overrideStatus)==0x2FC),"bad off");
		static_assert((offsetof(Wing,SPEED_DAMAGE_COEFF)==0x304),"bad off");
		static_assert((offsetof(Wing,SURFACE_DAMAGE_COEFF)==0x308),"bad off");
	};
};

//UDT: struct Engine @len=1000 @vfcount=3
	//_VTable: 
	//_Func: public void Engine(Engine &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Engine(); @loc=static @len=435 @rva=2642608
	//_Func: public void ~Engine(); @loc=static @len=235 @rva=2643264
	//_Data: this+0x8, Member, Type: struct acEngineData, data
	//_Data: this+0x130, Member, Type: struct EngineStatus, status
	//_Data: this+0x148, Member, Type: float, coastTorqueMultiplier
	//_Data: this+0x14C, Member, Type: float, limiterMultiplier
	//_Data: this+0x150, Member, Type: float, fuelPressure
	//_Data: this+0x154, Member, Type: float, bov
	//_Data: this+0x158, Member, Type: class std::vector<Turbo,std::allocator<Turbo> >, turbos
	//_Data: this+0x170, Member, Type: bool, isEngineStallEnabled
	//_Data: this+0x174, Member, Type: float, starterTorque
	//_Data: this+0x178, Member, Type: float, rpmDamageThreshold
	//_Data: this+0x17C, Member, Type: float, restrictor
	//_Data: this+0x180, Member, Type: struct PushToPass, p2p
	//_Func: public bool init(char * carModel); @intro @virtual vtpo=0 vfid=0 @loc=static @len=3 @rva=706688
	//_Func: public void init(Car * car); @loc=static @len=114 @rva=2645520
	//_Func: public void addTorqueGenerator(ITorqueGenerator * generator); @loc=static @len=31 @rva=2644496
	//_Func: public void addCoastGenerator(ICoastGenerator * generator); @loc=static @len=31 @rva=2644464
	//_Func: public void setTurboBoostLevel(float value); @loc=static @len=73 @rva=2654352
	//_Func: public float getTurboBoostLevel(); @loc=static @len=58 @rva=2645440
	//_Func: public void step(SACEngineInput & input, float dt); @loc=static @len=1640 @rva=2654432
	//_Func: public int getLimiterRPM(); @intro @virtual vtpo=0 vfid=1 @loc=static @len=24 @rva=2644560
	//_Func: public bool isLimiterOn(); @intro @virtual vtpo=0 vfid=2 @loc=static @len=12 @rva=2645648
	//_Func: public float getAngularInertia(); @loc=optimized @len=0 @rva=0
	//_Func: public void setElectronicOverride(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getElectronicOverride(); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaxPowerW(); @loc=static @len=79 @rva=2644608
	//_Func: public float getMaxTorqueNM(); @loc=static @len=9 @rva=2644688
	//_Func: public float getMaxPowerRPM(); @loc=static @len=9 @rva=2644592
	//_Func: public float getMaxTorqueRPM(); @loc=static @len=9 @rva=2644704
	//_Func: public float getTorqueAtRPM(float rpm, float gas); @loc=static @len=169 @rva=2645264
	//_Func: public int getDefaultEngineLimiter(); @loc=optimized @len=0 @rva=0
	//_Func: public void reset(); @loc=static @len=120 @rva=2654096
	//_Func: public float getGasUsage(); @loc=optimized @len=0 @rva=0
	//_Func: public float getLifeLeft(); @loc=optimized @len=0 @rva=0
	//_Func: public void setLifeLeft(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool isTurboAdjustableFromCockpit(); @loc=optimized @len=0 @rva=0
	//_Func: public float getMaxTurboBoost(bool with_wastegate); @loc=static @len=83 @rva=2644720
	//_Func: public float getSafeTurboLevel(); @loc=static @len=59 @rva=2644816
	//_Func: public void blowUp(); @loc=static @len=18 @rva=2644528
	//_Func: public float getThrottleResponseGas(float gas, float rpm); @loc=static @len=379 @rva=2644880
	//_Func: public int getCoastSettingsCount(); @loc=optimized @len=0 @rva=0
	//_Func: public void setCoastSettings(int s); @loc=static @len=124 @rva=2654224
	//_Func: public int getCoastSettingsIndex(); @loc=optimized @len=0 @rva=0
	//_Func: public bool hasTurboControllers(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x1A8, Member, Type: class std::vector<ITorqueGenerator *,std::allocator<ITorqueGenerator *> >, torqueGenerators
	//_Data: this+0x1C0, Member, Type: class std::vector<ICoastGenerator *,std::allocator<ICoastGenerator *> >, coastGenerators
	//_Data: this+0x1D8, Member, Type: bool, turboAdjustableFromCockpit
	//_Data: this+0x1DC, Member, Type: struct SACEngineInput, lastInput
	//_Data: this+0x1EC, Member, Type: int, defaultEngineLimiter
	//_Data: this+0x1F0, Member, Type: float, inertia
	//_Data: this+0x1F4, Member, Type: int, limiterOn
	//_Data: this+0x1F8, Member, Type: float, electronicOverride
	//_Data: this+0x1FC, Member, Type: float, maxPowerW_Dynamic
	//_Data: this+0x200, Member, Type: float, maxPowerW
	//_Data: this+0x204, Member, Type: float, maxTorqueNM
	//_Data: this+0x208, Member, Type: float, maxPowerRPM
	//_Data: this+0x20C, Member, Type: float, maxTorqueRPM
	//_Data: this+0x210, Member, Type: class PhysicsEngine *, physicsEngine
	//_Data: this+0x218, Member, Type: class Curve, throttleResponseCurve
	//_Data: this+0x298, Member, Type: class Curve, throttleResponseCurveMax
	//_Data: this+0x318, Member, Type: float, throttleResponseCurveMaxRef
	//_Data: this+0x31C, Member, Type: float, gasUsage
	//_Data: this+0x320, Member, Type: double, lifeLeft
	//_Data: this+0x328, Member, Type: float, turboBoostDamageThreshold
	//_Data: this+0x32C, Member, Type: float, turboBoostDamageK
	//_Data: this+0x330, Member, Type: float, rpmDamageK
	//_Data: this+0x334, Member, Type: float, bovThreshold
	//_Data: this+0x338, Member, Type: class Car *, car
	//_Data: this+0x340, Member, Type: class std::vector<TurboDynamicController,std::allocator<TurboDynamicController> >, turboControllers
	//_Data: this+0x358, Member, Type: float, gasCoastOffset
	//_Data: this+0x360, Member, Type: class Curve, gasCoastOffsetCurve
	//_Data: this+0x3E0, Member, Type: int, coastSettingsDefaultIndex
	//_Data: this+0x3E4, Member, Type: int, coastEntryRpm
	//_Func: private void stepP2P(float dt); @loc=static @len=418 @rva=2656080
	//_Func: private void loadINI(); @loc=static @len=7172 @rva=2646272
	//_Func: private void precalculatePowerAndTorque(); @loc=static @len=203 @rva=2653456
	//_Func: private void stepTurbos(); @loc=static @len=267 @rva=2656512
	//_Func: private CoastSettings loadCoastSettings(INIReader & r, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & section); @loc=static @len=600 @rva=2645664
	//_Func: public Engine & operator=(Engine &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Engine)==1000),"bad size");
		static_assert((offsetof(Engine,data)==0x8),"bad off");
		static_assert((offsetof(Engine,status)==0x130),"bad off");
		static_assert((offsetof(Engine,coastTorqueMultiplier)==0x148),"bad off");
		static_assert((offsetof(Engine,limiterMultiplier)==0x14C),"bad off");
		static_assert((offsetof(Engine,fuelPressure)==0x150),"bad off");
		static_assert((offsetof(Engine,bov)==0x154),"bad off");
		static_assert((offsetof(Engine,turbos)==0x158),"bad off");
		static_assert((offsetof(Engine,isEngineStallEnabled)==0x170),"bad off");
		static_assert((offsetof(Engine,starterTorque)==0x174),"bad off");
		static_assert((offsetof(Engine,rpmDamageThreshold)==0x178),"bad off");
		static_assert((offsetof(Engine,restrictor)==0x17C),"bad off");
		static_assert((offsetof(Engine,p2p)==0x180),"bad off");
		static_assert((offsetof(Engine,torqueGenerators)==0x1A8),"bad off");
		static_assert((offsetof(Engine,coastGenerators)==0x1C0),"bad off");
		static_assert((offsetof(Engine,turboAdjustableFromCockpit)==0x1D8),"bad off");
		static_assert((offsetof(Engine,lastInput)==0x1DC),"bad off");
		static_assert((offsetof(Engine,defaultEngineLimiter)==0x1EC),"bad off");
		static_assert((offsetof(Engine,inertia)==0x1F0),"bad off");
		static_assert((offsetof(Engine,limiterOn)==0x1F4),"bad off");
		static_assert((offsetof(Engine,electronicOverride)==0x1F8),"bad off");
		static_assert((offsetof(Engine,maxPowerW_Dynamic)==0x1FC),"bad off");
		static_assert((offsetof(Engine,maxPowerW)==0x200),"bad off");
		static_assert((offsetof(Engine,maxTorqueNM)==0x204),"bad off");
		static_assert((offsetof(Engine,maxPowerRPM)==0x208),"bad off");
		static_assert((offsetof(Engine,maxTorqueRPM)==0x20C),"bad off");
		static_assert((offsetof(Engine,physicsEngine)==0x210),"bad off");
		static_assert((offsetof(Engine,throttleResponseCurve)==0x218),"bad off");
		static_assert((offsetof(Engine,throttleResponseCurveMax)==0x298),"bad off");
		static_assert((offsetof(Engine,throttleResponseCurveMaxRef)==0x318),"bad off");
		static_assert((offsetof(Engine,gasUsage)==0x31C),"bad off");
		static_assert((offsetof(Engine,lifeLeft)==0x320),"bad off");
		static_assert((offsetof(Engine,turboBoostDamageThreshold)==0x328),"bad off");
		static_assert((offsetof(Engine,turboBoostDamageK)==0x32C),"bad off");
		static_assert((offsetof(Engine,rpmDamageK)==0x330),"bad off");
		static_assert((offsetof(Engine,bovThreshold)==0x334),"bad off");
		static_assert((offsetof(Engine,car)==0x338),"bad off");
		static_assert((offsetof(Engine,turboControllers)==0x340),"bad off");
		static_assert((offsetof(Engine,gasCoastOffset)==0x358),"bad off");
		static_assert((offsetof(Engine,gasCoastOffsetCurve)==0x360),"bad off");
		static_assert((offsetof(Engine,coastSettingsDefaultIndex)==0x3E0),"bad off");
		static_assert((offsetof(Engine,coastEntryRpm)==0x3E4),"bad off");
	};
};

//UDT: class Game @len=632 @vfcount=2
	//_VTable: 
	//_Func: public void Game(Game &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Game(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, VideoSettings & videoSettings); @loc=static @len=1554 @rva=2365568
	//_Func: public void ~Game(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=279 @rva=2367152
	//_Data: this+0x8, Member, Type: bool, isRenderingGui
	//_Data: this+0x9, Member, Type: bool, isSwappingBuffer
	//_Data: this+0xA, Member, Type: bool, isSuppressingAudioUpdate
	//_Data: this+0x10, Member, Type: class GameTime, gameTime
	//_Data: this+0x48, Member, Type: class RenderWindow, window
	//_Data: this+0x138, Member, Type: class GraphicsManager *, graphics
	//_Data: this+0x140, Member, Type: float, masterVolume
	//_Data: this+0x148, Member, Type: class AudioEngine *, audioEngine
	//_Data: this+0x150, Member, Type: struct GameStats, stats
	//_Data: this+0x178, Member, Type: class GameObject *, root
	//_Data: this+0x180, Member, Type: class ksgui::GUI *, gui
	//_Data: this+0x188, Member, Type: class KeyboardManager, keyboardManager
	//_Data: this+0x1B8, Member, Type: class JoypadManager, joypadManager
	//_Data: this+0x1C0, Member, Type: int, sleepTime
	//_Data: this+0x1C8, Member, Type: class Event<float>, evOnPreGUI
	//_Data: this+0x1E0, Member, Type: class Event<float>, evOnPostGui
	//_Data: this+0x1F8, Member, Type: class Event<float>, evOnRenderFinished
	//_Data: this+0x210, Member, Type: class Event<float>, evOnPostUpdate
	//_Data: this+0x228, Member, Type: class Event<double>, evOnBeginFrame
	//_Data: this+0x240, Member, Type: class Event<double>, evOnEndFrame
	//_Data: this+0x258, Member, Type: class Event<int>, evOnShutdownRequested
	//_Func: public void onIdle(); @loc=static @len=953 @rva=2369328
	//_Func: public void run(); @loc=static @len=248 @rva=2371248
	//_Func: public void shutdown(); @intro @virtual vtpo=0 vfid=1 @loc=static @len=8 @rva=2371504
	//_Func: public bool isGameClosing(); @loc=optimized @len=0 @rva=0
	//_Func: public void overrideGameClosing(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void shutdownObject(GameObject * o); @loc=static @len=78 @rva=2371520
	//_Func: public void renderHUDOnDemand(float dt); @loc=static @len=15 @rva=2371232
	//_Func: public void onWindowResize(OnWindowResizeEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void onWindowClosed(OnWindowClosedEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Data: this+0x270, Member, Type: bool, isClosing
	//_Func: protected void update(GameObject * o, float dt); @loc=static @len=109 @rva=2371600
	//_Func: protected void render(GameObject * o, float dt); @loc=static @len=109 @rva=2370896
	//_Func: protected void renderHUD(GameObject * o, float dt); @loc=static @len=109 @rva=2371120
	//_Func: protected void renderAudio(GameObject * o, float dt); @loc=static @len=109 @rva=2371008
	//_Func: protected void deleteGameObjectRec(GameObject * go); @loc=static @len=123 @rva=2369200
	//_Func: public Game & operator=(Game &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Game)==632),"bad size");
		static_assert((offsetof(Game,isRenderingGui)==0x8),"bad off");
		static_assert((offsetof(Game,isSwappingBuffer)==0x9),"bad off");
		static_assert((offsetof(Game,isSuppressingAudioUpdate)==0xA),"bad off");
		static_assert((offsetof(Game,gameTime)==0x10),"bad off");
		static_assert((offsetof(Game,window)==0x48),"bad off");
		static_assert((offsetof(Game,graphics)==0x138),"bad off");
		static_assert((offsetof(Game,masterVolume)==0x140),"bad off");
		static_assert((offsetof(Game,audioEngine)==0x148),"bad off");
		static_assert((offsetof(Game,stats)==0x150),"bad off");
		static_assert((offsetof(Game,root)==0x178),"bad off");
		static_assert((offsetof(Game,gui)==0x180),"bad off");
		static_assert((offsetof(Game,keyboardManager)==0x188),"bad off");
		static_assert((offsetof(Game,joypadManager)==0x1B8),"bad off");
		static_assert((offsetof(Game,sleepTime)==0x1C0),"bad off");
		static_assert((offsetof(Game,evOnPreGUI)==0x1C8),"bad off");
		static_assert((offsetof(Game,evOnPostGui)==0x1E0),"bad off");
		static_assert((offsetof(Game,evOnRenderFinished)==0x1F8),"bad off");
		static_assert((offsetof(Game,evOnPostUpdate)==0x210),"bad off");
		static_assert((offsetof(Game,evOnBeginFrame)==0x228),"bad off");
		static_assert((offsetof(Game,evOnEndFrame)==0x240),"bad off");
		static_assert((offsetof(Game,evOnShutdownRequested)==0x258),"bad off");
		static_assert((offsetof(Game,isClosing)==0x270),"bad off");
	};
};

//UDT: class NetCarStateProvider @len=5560 @multibase=2
	//_Base: class GameObject @off=0 @len=88
	//_Base: class ICarPhysicsStateProvider @off=88 @len=8
	//_Func: public void NetCarStateProvider(NetCarStateProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void NetCarStateProvider(NetCarStateProviderDef & def); @loc=static @len=4691 @rva=1141872
	//_Func: public void ~NetCarStateProvider(); @virtual vtpo=0 vfid=0 @loc=static @len=521 @rva=1147072
	//_Data: this+0x60, Member, Type: class Event<OnSectorSplitEvent>, evOnSectorSplit
	//_Data: this+0x78, Member, Type: unsigned char, sessionID
	//_Data: this+0x80, Member, Type: struct DriverInfo, driverInfo
	//_Data: this+0x100, Member, Type: class vec3f, errorVector
	//_Data: this+0x10C, Member, Type: int, guid
	//_Data: this+0x110, Member, Type: int, ping
	//_Data: this+0x114, Member, Type: unsigned int, lastLap
	//_Data: this+0x118, Member, Type: unsigned int, bestLap
	//_Data: this+0x120, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, currentSplits
	//_Data: this+0x138, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, personalBestSplits
	//_Data: this+0x150, Member, Type: double, lastUpdateTime
	//_Data: this+0x158, Member, Type: double, lastTime
	//_Data: this+0x160, Member, Type: class std::vector<Lap,std::allocator<Lap> >, laps
	//_Data: this+0x178, Member, Type: struct NetCarPushToPass, p2p
	//_Data: this+0x18C, Member, Type: bool, hasEverReceivedAPacket
	//_Data: this+0x190, Member, Type: float, ballastKG
	//_Data: this+0x194, Member, Type: float, restrictor
	//_Data: this+0x198, Member, Type: float, hasCollisionInThisStep
	//_Func: public void update(float dt); @virtual vtpo=0 vfid=1 @loc=static @len=82 @rva=1169744
	//_Func: public void getPhysicsState(CarPhysicsState & physicsState); @virtual vtpo=88 vfid=1 @loc=static @len=239 @rva=1151520
	//_Func: public void getWingState(std::vector<WingState,std::allocator<WingState> > & ws); @virtual vtpo=88 vfid=2 @loc=static @len=18 @rva=1153600
	//_Func: public void onRemoteStateReceived(NetCarState & s); @loc=static @len=1814 @rva=1159648
	//_Func: public void setCarAvatar(CarAvatar * av); @loc=static @len=91 @rva=1162368
	//_Func: public CarAvatar * getCarAvatar(); @loc=optimized @len=0 @rva=0
	//_Func: public NetCarQoS getQos(); @loc=optimized @len=0 @rva=0
	//_Func: public void resetQos(); @loc=optimized @len=0 @rva=0
	//_Func: public void setDisconnected(); @loc=optimized @len=0 @rva=0
	//_Func: public IRigidBody * getBody(); @loc=optimized @len=0 @rva=0
	//_Func: public void resetDamage(); @loc=static @len=64 @rva=1162048
	//_Func: public void setDamageZoneLevel(float * zones); @loc=static @len=20 @rva=1162464
	//_Func: public void activateP2P(); @loc=static @len=24 @rva=1150672
	//_Tag 17
	//_Data: this+0x19C, Member, Type: struct NetCarQoS, qos
	//_Data: this+0x1A8, Member, Type: class ACClient *, client
	//_Data: this+0x1B0, Member, Type: class mat44f, bodyMatrix
	//_Data: this+0x1F0, Member, Type: class mat44f[0x4], wheelMatrix
	//_Data: this+0x2F0, Member, Type: class mat44f[0x4], suspMatrix
	//_Data: this+0x3F0, Member, Type: struct NetCarState[0x3], netStates
	//_Data: this+0x588, Member, Type: class vec3f, lastScreenPos
	//_Data: this+0x594, Member, Type: class mat44f[0x4], wheelBasePosLS
	//_Data: this+0x698, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, unixName
	//_Data: this+0x6B8, Member, Type: class IRayTrackCollisionProvider *, rayCastProvider
	//_Data: this+0x6C0, Member, Type: class vec3f, lastVelocity
	//_Data: this+0x6CC, Member, Type: class vec3f, instantVelocity
	//_Data: this+0x6D8, Member, Type: class vec3f, currentVelocity
	//_Data: this+0x6E8, Member, Type: struct CarPhysicsState, state
	//_Data: this+0x1258, Member, Type: class std::vector<WingState,std::allocator<WingState> >, wingStates
	//_Data: this+0x1270, Member, Type: class std::vector<std::vector<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> >,std::allocator<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> > > >,std::allocator<std::vector<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> >,std::allocator<std::unique_ptr<DynamicWingController,std::default_delete<DynamicWingController> > > > > >, wingControllerLists
	//_Data: this+0x1288, Member, Type: class std::unique_ptr<Node,std::default_delete<Node> >, colliderModel
	//_Data: this+0x1290, Member, Type: class mat44f, pitPosition
	//_Data: this+0x12D0, Member, Type: struct SlipStream, slipStream
	//_Data: this+0x1338, Member, Type: struct DriverInfo, oldDriverInfo
	//_Data: this+0x13B8, Member, Type: bool, isDisconnected
	//_Data: this+0x13BC, Member, Type: float, lastStepSpeedkmh
	//_Data: this+0x13C0, Member, Type: float, lastStepGas
	//_Data: this+0x13C8, Member, Type: class std::vector<DRSWingSetting,std::allocator<DRSWingSetting> >, drsWingSettings
	//_Data: this+0x13E0, Member, Type: class vec3f, smoothVelocity
	//_Data: this+0x13F0, Member, Type: class Concurrency::concurrent_queue<NetCarState,std::allocator<NetCarState> >, incomingStateQueue
	//_Data: this+0x1418, Member, Type: class std::vector<CarPhysicsState,std::allocator<CarPhysicsState> >, stateCache
	//_Data: this+0x1430, Member, Type: double, lastIntegrationTime
	//_Data: this+0x1438, Member, Type: float[0x4], tyreRadius
	//_Data: this+0x1448, Member, Type: class vec3f, graphicsOffset
	//_Data: this+0x1454, Member, Type: float, graphicsPitchRotation
	//_Data: this+0x1458, Member, Type: class mat44f[0x4], tyreLocalRotation
	//_Data: this+0x1558, Member, Type: class CarAvatar *, avatar
	//_Data: this+0x1560, Member, Type: class IRigidBody *, body
	//_Data: this+0x1568, Member, Type: class vec3f, lastDistanceV
	//_Data: this+0x1574, Member, Type: class vec3f, lastAxis
	//_Data: this+0x1580, Member, Type: struct SplineLocationData, splineLocationData
	//_Data: this+0x1588, Member, Type: double, lastSliceTimeStamp
	//_Data: this+0x1590, Member, Type: float, currentAOA
	//_Data: this+0x1594, Member, Type: float, carVerticalOffset
	//_Data: this+0x1598, Member, Type: bool, useLog
	//_Tag 11
	//_Data: this+0x15A0, Member, Type: struct NetCarStateProvider::LagDebug, lagDebug
	//_Func: private bool isLagging(double  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private mat44f getTyreMatrix(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private mat44f getSmoothBodyMatrix(double physics_time, CarPhysicsState & state); @loc=static @len=1825 @rva=1151760
	//_Func: private vec3f getNetStateLocalVel(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private vec3f getNetStateVelocity(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private vec3f getNetStateAcceleration(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private vec3f projectNetStatePos(int ti, double t); @loc=static @len=334 @rva=1161472
	//_Func: private double getCorrectionDeltaTime(double  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private void getNetStateAngularVelocity(int  _arg0, mat44f &  _arg1, mat44f &  _arg2, vec3f &  _arg3, float &  _arg4); @loc=optimized @len=0 @rva=0
	//_Func: private void initPhysicsValues(); @loc=static @len=2715 @rva=1155040
	//_Func: private void step(double physics_time); @loc=static @len=4695 @rva=1162496
	//_Func: private void stepLagging(double physics_time); @loc=static @len=2030 @rva=1167200
	//_Func: private void debugLag(bool isLagging, double physics_time); @loc=static @len=207 @rva=1151312
	//_Func: private void initWings(); @loc=static @len=1884 @rva=1157760
	//_Func: private void stepWings(); @loc=static @len=389 @rva=1169344
	//_Func: private void initP2P(); @loc=static @len=675 @rva=1154352
	//_Func: private void stepP2P(); @loc=static @len=99 @rva=1169232
	//_Func: private void initDRS(); @loc=static @len=706 @rva=1153632
	//_Func: public NetCarStateProvider & operator=(NetCarStateProvider &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(NetCarStateProvider)==5560),"bad size");
		static_assert((offsetof(NetCarStateProvider,evOnSectorSplit)==0x60),"bad off");
		static_assert((offsetof(NetCarStateProvider,sessionID)==0x78),"bad off");
		static_assert((offsetof(NetCarStateProvider,driverInfo)==0x80),"bad off");
		static_assert((offsetof(NetCarStateProvider,errorVector)==0x100),"bad off");
		static_assert((offsetof(NetCarStateProvider,guid)==0x10C),"bad off");
		static_assert((offsetof(NetCarStateProvider,ping)==0x110),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastLap)==0x114),"bad off");
		static_assert((offsetof(NetCarStateProvider,bestLap)==0x118),"bad off");
		static_assert((offsetof(NetCarStateProvider,currentSplits)==0x120),"bad off");
		static_assert((offsetof(NetCarStateProvider,personalBestSplits)==0x138),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastUpdateTime)==0x150),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastTime)==0x158),"bad off");
		static_assert((offsetof(NetCarStateProvider,laps)==0x160),"bad off");
		static_assert((offsetof(NetCarStateProvider,p2p)==0x178),"bad off");
		static_assert((offsetof(NetCarStateProvider,hasEverReceivedAPacket)==0x18C),"bad off");
		static_assert((offsetof(NetCarStateProvider,ballastKG)==0x190),"bad off");
		static_assert((offsetof(NetCarStateProvider,restrictor)==0x194),"bad off");
		static_assert((offsetof(NetCarStateProvider,hasCollisionInThisStep)==0x198),"bad off");
		static_assert((offsetof(NetCarStateProvider,qos)==0x19C),"bad off");
		static_assert((offsetof(NetCarStateProvider,client)==0x1A8),"bad off");
		static_assert((offsetof(NetCarStateProvider,bodyMatrix)==0x1B0),"bad off");
		static_assert((offsetof(NetCarStateProvider,wheelMatrix)==0x1F0),"bad off");
		static_assert((offsetof(NetCarStateProvider,suspMatrix)==0x2F0),"bad off");
		static_assert((offsetof(NetCarStateProvider,netStates)==0x3F0),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastScreenPos)==0x588),"bad off");
		static_assert((offsetof(NetCarStateProvider,wheelBasePosLS)==0x594),"bad off");
		static_assert((offsetof(NetCarStateProvider,unixName)==0x698),"bad off");
		static_assert((offsetof(NetCarStateProvider,rayCastProvider)==0x6B8),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastVelocity)==0x6C0),"bad off");
		static_assert((offsetof(NetCarStateProvider,instantVelocity)==0x6CC),"bad off");
		static_assert((offsetof(NetCarStateProvider,currentVelocity)==0x6D8),"bad off");
		static_assert((offsetof(NetCarStateProvider,state)==0x6E8),"bad off");
		static_assert((offsetof(NetCarStateProvider,wingStates)==0x1258),"bad off");
		static_assert((offsetof(NetCarStateProvider,wingControllerLists)==0x1270),"bad off");
		static_assert((offsetof(NetCarStateProvider,colliderModel)==0x1288),"bad off");
		static_assert((offsetof(NetCarStateProvider,pitPosition)==0x1290),"bad off");
		static_assert((offsetof(NetCarStateProvider,slipStream)==0x12D0),"bad off");
		static_assert((offsetof(NetCarStateProvider,oldDriverInfo)==0x1338),"bad off");
		static_assert((offsetof(NetCarStateProvider,isDisconnected)==0x13B8),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastStepSpeedkmh)==0x13BC),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastStepGas)==0x13C0),"bad off");
		static_assert((offsetof(NetCarStateProvider,drsWingSettings)==0x13C8),"bad off");
		static_assert((offsetof(NetCarStateProvider,smoothVelocity)==0x13E0),"bad off");
		static_assert((offsetof(NetCarStateProvider,incomingStateQueue)==0x13F0),"bad off");
		static_assert((offsetof(NetCarStateProvider,stateCache)==0x1418),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastIntegrationTime)==0x1430),"bad off");
		static_assert((offsetof(NetCarStateProvider,tyreRadius)==0x1438),"bad off");
		static_assert((offsetof(NetCarStateProvider,graphicsOffset)==0x1448),"bad off");
		static_assert((offsetof(NetCarStateProvider,graphicsPitchRotation)==0x1454),"bad off");
		static_assert((offsetof(NetCarStateProvider,tyreLocalRotation)==0x1458),"bad off");
		static_assert((offsetof(NetCarStateProvider,avatar)==0x1558),"bad off");
		static_assert((offsetof(NetCarStateProvider,body)==0x1560),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastDistanceV)==0x1568),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastAxis)==0x1574),"bad off");
		static_assert((offsetof(NetCarStateProvider,splineLocationData)==0x1580),"bad off");
		static_assert((offsetof(NetCarStateProvider,lastSliceTimeStamp)==0x1588),"bad off");
		static_assert((offsetof(NetCarStateProvider,currentAOA)==0x1590),"bad off");
		static_assert((offsetof(NetCarStateProvider,carVerticalOffset)==0x1594),"bad off");
		static_assert((offsetof(NetCarStateProvider,useLog)==0x1598),"bad off");
		static_assert((offsetof(NetCarStateProvider,lagDebug)==0x15A0),"bad off");
	};
};

//UDT: class GraphicsManager @len=1248 @vfcount=1
	//_VTable: 
	//_Func: public void GraphicsManager(GraphicsManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void GraphicsManager(VideoSettings & ivideoSettings); @loc=static @len=1912 @rva=2102816
	//_Func: public void ~GraphicsManager(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=610 @rva=2105552
	//_Data: this+0x8, Member, Type: bool, useCustomSunDirection
	//_Data: this+0xC, Member, Type: float, exposureMultiplier
	//_Data: this+0x10, Member, Type: struct VideoSettings, videoSettings
	//_Data: this+0x60, Member, Type: struct RenderStats, stats
	//_Data: this+0x74, Member, Type: struct LightingSettings, lightingSettings
	//_Data: this+0x128, Member, Type: struct RendererFlags, renderFlags
	//_Data: this+0x130, Member, Type: bool, suspendViewportUpdateOnSetRenderTarget
	//_Data: this+0x138, Member, Type: class GPUProfiler *, gpuProfiler
	//_Data: this+0x140, Member, Type: class PvsProcessor *, pvsProcessor
	//_Data: this+0x148, Member, Type: class CubeMap *, currentCubeMap
	//_Data: this+0x150, Member, Type: class Event<OnWindowResize>, evWindowResize
	//_Data: this+0x168, Member, Type: class Event<OnWindowResize>, evWindowPreResize
	//_Data: this+0x180, Member, Type: class std::unique_ptr<ResourceStore,std::default_delete<ResourceStore> >, resourceStore
	//_Func: public SamplerStates & getSampleStates(); @loc=optimized @len=0 @rva=0
	//_Func: public std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > getErrorStrings(); @loc=static @len=41 @rva=2107968
	//_Func: public void addErrorString(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void beginMainRenderPass(); @loc=static @len=5 @rva=2106736
	//_Func: public void endMainRenderPass(); @loc=static @len=5 @rva=2107456
	//_Func: public GLRenderer & getGL(); @loc=static @len=8 @rva=148688
	//_Func: public GLRenderer * createGLRenderer(unsigned int maxVertices); @loc=static @len=220 @rva=2107152
	//_Func: public void clearRenderTarget(vec4f & color); @loc=static @len=8 @rva=2106944
	//_Func: public void clearRenderTargetDepth(float value); @loc=static @len=8 @rva=2106960
	//_Func: public void beginScene(); @loc=static @len=177 @rva=2106752
	//_Func: public void endScene(); @loc=static @len=148 @rva=2107472
	//_Func: public void setViewport(int left, int top, int width, int height); @loc=static @len=170 @rva=2117616
	//_Func: public void drawPrimitive(unsigned int indicesCount, unsigned int baseIndex, unsigned int baseVertex, unsigned int numVertices); @loc=static @len=68 @rva=2107376
	//_Func: public void render(Node *  _arg0, RenderContext *  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void compile(Node * root); @loc=static @len=36 @rva=2107104
	//_Func: public void setScreenSpaceMode(); @loc=static @len=275 @rva=2115808
	//_Func: public void onResize(int width, int height); @loc=static @len=215 @rva=2114352
	//_Func: public void updateLightingSetttings(); @loc=static @len=1752 @rva=2118032
	//_Func: public void loadLightingSettings(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=4184 @rva=2110032
	//_Func: public void loadLightingSettings(); @loc=static @len=127 @rva=2114224
	//_Func: public void setOverrideNoMS(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void resetRenderStates(); @loc=static @len=147 @rva=2114624
	//_Func: public bool setMaximumFrameLatency(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void overrideSamplerState(SamplerState sampler); @loc=static @len=43 @rva=2114576
	//_Func: public Shader * getShader(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=46 @rva=2108512
	//_Func: public void setShader(Shader * sh); @loc=static @len=51 @rva=2116096
	//_Func: public std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > > getShaders(); @loc=optimized @len=0 @rva=0
	//_Func: public void initShaders(); @loc=optimized @len=0 @rva=0
	//_Func: public void setBlendMode(BlendMode mode); @loc=static @len=47 @rva=2114784
	//_Func: public void commitShaderChanges(); @loc=static @len=62 @rva=2107040
	//_Func: public void setDepthMode(DepthMode mode); @loc=static @len=164 @rva=2114944
	//_Func: public void setCullMode(CullMode mode); @loc=static @len=66 @rva=2114832
	//_Func: public void setShadowMapBias(float b0, float b1, float b2); @loc=static @len=169 @rva=2116160
	//_Func: public vec3f getShadowMapBias(); @loc=static @len=48 @rva=2108560
	//_Func: public void setProjectionMatrix(mat44f & matrix); @loc=static @len=256 @rva=2115168
	//_Func: public void setViewMatrix(mat44f & matrix, Camera * camera); @loc=static @len=747 @rva=2116864
	//_Func: public void setWorldMatrix(mat44f & matrix); @loc=static @len=239 @rva=2117792
	//_Func: public mat44f getWorldMatrix(); @loc=static @len=47 @rva=2108656
	//_Func: public mat44f getProjectionMatrix(); @loc=static @len=47 @rva=2108464
	//_Func: public mat44f getViewMatrix(); @loc=static @len=47 @rva=2108608
	//_Func: public void setVB(IVertexBuffer * vb); @loc=static @len=8 @rva=2116848
	//_Func: public void setIB(IndexBuffer * ib); @loc=static @len=9 @rva=2115152
	//_Func: public CBuffer * getCBuffer(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=332 @rva=2107632
	//_Func: public void setRenderTarget(RenderTarget * rt); @loc=static @len=148 @rva=2115424
	//_Func: public void setTextureRT(unsigned int slot, RenderTarget * rt); @loc=static @len=19 @rva=2116816
	//_Func: public void setDepthTextureRT(unsigned int slot, RenderTarget * rt); @loc=static @len=19 @rva=2115120
	//_Func: public void setScreenRenderTargets(); @loc=static @len=88 @rva=2115712
	//_Func: public void setTexture(int slot, Texture & tex); @loc=static @len=96 @rva=2116720
	//_Func: public void setShadowMapTexture(int level, RenderTarget * rt); @loc=static @len=116 @rva=2116592
	//_Func: public void setShadowMapMatrix(int level, mat44f & matrix); @loc=static @len=244 @rva=2116336
	//_Func: public void setCustomSunDirection(vec3f sunDirection); @loc=static @len=22 @rva=2114912
	//_Func: public void setSamplerState(); @loc=static @len=113 @rva=2115584
	//_Func: public void clearTextureSlot(int slot); @loc=static @len=50 @rva=2106976
	//_Func: public vec3f getLDRColor(vec3f & c); @loc=static @len=308 @rva=2108016
	//_Func: public float getLDRScale(vec3f & c); @loc=static @len=119 @rva=2108336
	//_Data: this+0x188, Member, Type: struct RenderState, state
	//_Data: this+0x380, Member, Type: class GLRenderer *, gl
	//_Data: this+0x388, Member, Type: class vec3f, customSunDirection
	//_Data: this+0x398, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, errorStrings
	//_Data: this+0x3B0, Member, Type: class std::vector<GLRenderer *,std::allocator<GLRenderer *> >, glRenderers
	//_Data: this+0x3C8, Member, Type: int, multiSampleQuality
	//_Data: this+0x3D0, Member, Type: struct SamplerStates, samplerStates
	//_Data: this+0x408, Member, Type: struct SystemCBuffers, sysBuffers
	//_Data: this+0x4A8, Member, Type: class std::map<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,CBuffer *,std::less<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > >,std::allocator<std::pair<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > const ,CBuffer *> > >, cBuffersMap
	//_Data: this+0x4B8, Member, Type: class ShaderManager, shaderManager
	//_Func: protected void initRenderFlags(); @loc=static @len=529 @rva=2109280
	//_Func: protected void initSamplerStates(); @loc=static @len=194 @rva=2109824
	//_Func: protected void initCBuffers(); @loc=static @len=563 @rva=2108704
	//_Func: public GraphicsManager & operator=(GraphicsManager &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(GraphicsManager)==1248),"bad size");
		static_assert((offsetof(GraphicsManager,useCustomSunDirection)==0x8),"bad off");
		static_assert((offsetof(GraphicsManager,exposureMultiplier)==0xC),"bad off");
		static_assert((offsetof(GraphicsManager,videoSettings)==0x10),"bad off");
		static_assert((offsetof(GraphicsManager,stats)==0x60),"bad off");
		static_assert((offsetof(GraphicsManager,lightingSettings)==0x74),"bad off");
		static_assert((offsetof(GraphicsManager,renderFlags)==0x128),"bad off");
		static_assert((offsetof(GraphicsManager,suspendViewportUpdateOnSetRenderTarget)==0x130),"bad off");
		static_assert((offsetof(GraphicsManager,gpuProfiler)==0x138),"bad off");
		static_assert((offsetof(GraphicsManager,pvsProcessor)==0x140),"bad off");
		static_assert((offsetof(GraphicsManager,currentCubeMap)==0x148),"bad off");
		static_assert((offsetof(GraphicsManager,evWindowResize)==0x150),"bad off");
		static_assert((offsetof(GraphicsManager,evWindowPreResize)==0x168),"bad off");
		static_assert((offsetof(GraphicsManager,resourceStore)==0x180),"bad off");
		static_assert((offsetof(GraphicsManager,state)==0x188),"bad off");
		static_assert((offsetof(GraphicsManager,gl)==0x380),"bad off");
		static_assert((offsetof(GraphicsManager,customSunDirection)==0x388),"bad off");
		static_assert((offsetof(GraphicsManager,errorStrings)==0x398),"bad off");
		static_assert((offsetof(GraphicsManager,glRenderers)==0x3B0),"bad off");
		static_assert((offsetof(GraphicsManager,multiSampleQuality)==0x3C8),"bad off");
		static_assert((offsetof(GraphicsManager,samplerStates)==0x3D0),"bad off");
		static_assert((offsetof(GraphicsManager,sysBuffers)==0x408),"bad off");
		static_assert((offsetof(GraphicsManager,cBuffersMap)==0x4A8),"bad off");
		static_assert((offsetof(GraphicsManager,shaderManager)==0x4B8),"bad off");
	};
};

//UDT: class ACClient @len=67608 @vfcount=6
	//_Base: class GameObject @off=0 @len=88
	//_Func: public void ACClient(ACClient &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void ACClient(Sim * isim); @loc=static @len=3257 @rva=238272
	//_Func: public void ~ACClient(); @virtual vtpo=0 vfid=0 @loc=static @len=1088 @rva=244880
	//_Data: this+0x58, Member, Type: class Event<OnChatMessageEvent>, evOnChatMessage
	//_Data: this+0x70, Member, Type: class Event<RemoteSession>, evOnOnlineNewSession
	//_Data: this+0x88, Member, Type: class Event<RemoteSessionResume>, evOnOnlineEndSession
	//_Data: this+0xA0, Member, Type: class Event<OnLapCompletedEvent>, evOnLapCompleted
	//_Data: this+0xB8, Member, Type: class Event<ReceivedVoteDef>, evOnVoteReceived
	//_Data: this+0xD0, Member, Type: class Event<bool>, evOnVoteNotPassed
	//_Data: this+0xE8, Member, Type: class Event<bool>, evOnMandatoryPitDone
	//_Data: this+0x100, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, serverName
	//_Data: this+0x120, Member, Type: float, remoteSpring
	//_Data: this+0x124, Member, Type: float, remoteDamper
	//_Data: this+0x128, Member, Type: float, remoteFactor
	//_Data: this+0x130, Member, Type: struct DriverInfo, driverInfo
	//_Data: this+0x1B0, Member, Type: struct ServerInfo, serverInfo
	//_Data: this+0x1F8, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, guid
	//_Data: this+0x218, Member, Type: struct ClientRules, rules
	//_Data: this+0x21C, Member, Type: struct ServerDrivingAssists, serverDrivingAssists
	//_Data: this+0x228, Member, Type: bool, isTVMode
	//_Data: this+0x230, Member, Type: double, handshakeServerTimeS
	//_Data: this+0x238, Member, Type: class std::vector<unsigned char,std::allocator<unsigned char> >, playerCarMD5
	//_Func: public void renderHUD(float dt); @virtual vtpo=0 vfid=3 @loc=static @len=723 @rva=333392
	//_Func: public ClientHandshakeResult handshakeTCP(); @loc=static @len=8613 @rva=297808
	//_Func: public bool getRemoteCarList(std::vector<ClientRemoteCarDef,std::allocator<ClientRemoteCarDef> > & remoteCars); @loc=static @len=4972 @rva=290688
	//_Func: public void beginUpdateMode(); @loc=static @len=284 @rva=280288
	//_Func: public void sendChat(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & message); @loc=static @len=263 @rva=338576
	//_Func: public void sendVote(VoteType aVote, bool aVoteValue, int targetAvatarGUID); @loc=static @len=253 @rva=338848
	//_Func: public bool hasVoted(); @loc=static @len=12 @rva=306432
	//_Func: public CarAvatar * getCurrentVotingTarget(); @loc=static @len=12 @rva=287824
	//_Func: public float getVotingTimeLeft(); @loc=static @len=13 @rva=296512
	//_Func: public VoteType getCurrentVoteType(); @loc=static @len=11 @rva=287808
	//_Func: public float getVotingMaxTime(); @loc=static @len=13 @rva=296496
	//_Func: public void addNetCar(NetCarStateProvider * nc); @loc=static @len=31 @rva=279232
	//_Func: public NetCarStateProvider * getNetCarFromSessionID(unsigned char id); @loc=static @len=44 @rva=290528
	//_Func: public void update(float deltaT); @virtual vtpo=0 vfid=1 @loc=static @len=1323 @rva=342832
	//_Func: public double getPhysicsTime(); @loc=static @len=20 @rva=290576
	//_Func: public bool isAllowedToSendChatMessage(); @loc=static @len=29 @rva=307616
	//_Func: public int getPing(CarAvatar * anAvatar); @loc=static @len=75 @rva=290608
	//_Func: public int getQOS(); @loc=optimized @len=0 @rva=0
	//_Func: public RemoteSession getCurrentSession(); @loc=static @len=41 @rva=287648
	//_Func: public RemoteSessionResult & getCurrentSessionResults(); @loc=static @len=8 @rva=287792
	//_Func: public CarAvatar * getCarAvatarFromSessionID(unsigned char sesid); @loc=static @len=107 @rva=286720
	//_Func: public int getSessionIDFromCarAvatar(CarAvatar * anAvatar); @loc=static @len=78 @rva=295696
	//_Func: public int getSessionCount(); @loc=static @len=19 @rva=295664
	//_Func: public int getCurrentSessionIndex(); @loc=static @len=88 @rva=287696
	//_Func: public RemoteSession getSessionInfo(int index); @loc=static @len=152 @rva=295936
	//_Func: public void shutdown(); @virtual vtpo=0 vfid=5 @loc=static @len=120 @rva=339248
	//_Func: public double getSessionTimeLeft(); @loc=static @len=57 @rva=296096
	//_Func: public double getSessionOverTime(); @loc=optimized @len=0 @rva=0
	//_Func: public int getFinishPosition(); @loc=optimized @len=0 @rva=0
	//_Func: public int getHandshakeCarPosition(); @loc=static @len=7 @rva=1278848
	//_Func: public unsigned int getCarPosition(CarAvatar * car); @loc=static @len=189 @rva=287168
	//_Func: public unsigned int getBestLap(); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getBestLap(CarAvatar * car); @loc=static @len=108 @rva=286272
	//_Func: public unsigned int getInstanceBestLap(CarAvatar * car); @loc=static @len=108 @rva=287856
	//_Func: public Lap getLastLap(CarAvatar * car); @loc=static @len=325 @rva=290000
	//_Func: public unsigned int getLapCount(CarAvatar * car); @loc=static @len=177 @rva=287968
	//_Func: public void getLaps(CarAvatar * car, std::vector<Lap,std::allocator<Lap> > & laps); @loc=static @len=1834 @rva=288160
	//_Func: public bool isBestSplit(int & sector, int & t, bool & isGlobal, CarAvatar * car); @loc=static @len=202 @rva=307648
	//_Func: public unsigned int getBestSplit(int & sector, bool & isGlobal, CarAvatar * car); @loc=static @len=322 @rva=286384
	//_Func: public unsigned int getLastSplit(CarAvatar * car, int & sector); @loc=static @len=178 @rva=290336
	//_Func: public unsigned int getSplit(CarAvatar * car, int & sector); @loc=static @len=274 @rva=296160
	//_Func: public void addSplitToBest(std::vector<unsigned int,std::allocator<unsigned int> > & splits, std::vector<unsigned int,std::allocator<unsigned int> > & personalSplits, unsigned int & sector, unsigned int & time, unsigned int & cuts); @loc=static @len=397 @rva=279264
	//_Func: public void storeSplitToLap(std::vector<unsigned int,std::allocator<unsigned int> > & splits, std::vector<unsigned int,std::allocator<unsigned int> > & personalSplits, std::vector<Lap,std::allocator<Lap> > & laps, unsigned int & laptime, unsigned int & cuts); @loc=static @len=407 @rva=339488
	//_Func: public Lap getCurrentLap(CarAvatar * car); @loc=static @len=273 @rva=287360
	//_Func: public void resetCurrentLaps(); @loc=static @len=769 @rva=334128
	//_Func: public double getSendInterval(); @loc=optimized @len=0 @rva=0
	//_Func: public NetCarStateProvider * getNetCarFromBody(IRigidBody *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public std::vector<unsigned char,std::allocator<unsigned char> > getCarMD5(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & unix_name); @loc=static @len=322 @rva=286832
	//_Func: public double getTimetoWait(); @loc=static @len=9 @rva=296448
	//_Func: public bool getHasExtraLap(); @loc=optimized @len=0 @rva=0
	//_Func: public bool getIsLastLap(); @loc=optimized @len=0 @rva=0
	//_Func: public bool getHasLeaderFinished(); @loc=optimized @len=0 @rva=0
	//_Func: public bool getHasPlayerFinished(); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getPitWindowStart(); @loc=optimized @len=0 @rva=0
	//_Func: public unsigned int getPitWindowEnd(); @loc=optimized @len=0 @rva=0
	//_Func: public void setInvertedGridPositions(int pos); @loc=static @len=7 @rva=1315584
	//_Func: public int getInvertedGridPositions(); @loc=static @len=7 @rva=1279296
	//_Func: public void askForP2Pvalue(); @loc=static @len=169 @rva=279664
	//_Data: this+0x250, Member, Type: bool, debugStartingLights
	//_Data: this+0x251, Member, Type: bool, isAssociated
	//_Tag 11
	//_Data: this+0x258, Member, Type: struct ACClient::ClientSessionTransition, transitionInfo
	//_Data: this+0x330, Member, Type: struct RemoteSessionResult, sessionResultsGT
	//_Data: this+0x3A0, Member, Type: class UDPSocket, sok
	//_Tag 17
	//_Data: this+0x3D8, Member, Type: class std::vector<std::vector<unsigned char,std::allocator<unsigned char> >,std::allocator<std::vector<unsigned char,std::allocator<unsigned char> > > >, checksumResults
	//_Data: this+0x3F0, Member, Type: class Sim *, sim
	//_Data: this+0x3F8, Member, Type: class Car *, car
	//_Data: this+0x400, Member, Type: class ACClientVotingManager *, votingManager
	//_Data: this+0x408, Member, Type: class CarAvatar *, avatar
	//_Data: this+0x410, Member, Type: double, lastSendTime
	//_Data: this+0x418, Member, Type: double, sendInterval
	//_Data: this+0x420, Member, Type: double, lastChatMessage
	//_Data: this+0x428, Member, Type: double, lastSpectatorPulse
	//_Data: this+0x430, Member, Type: class IPAddress, serverIP
	//_Data: this+0x440, Member, Type: unsigned char, sessionID
	//_Data: this+0x448, Member, Type: class std::vector<NetCarStateProvider *,std::allocator<NetCarStateProvider *> >, netCars
	//_Data: this+0x460, Member, Type: unsigned char, pakSequenceIndex
	//_Data: this+0x464, Member, Type: int, handshakeCarPosition
	//_Data: this+0x468, Member, Type: class WrongWayIndicator *, wrongWayIndicator
	//_Data: this+0x470, Member, Type: int, numWrongWayInfractions
	//_Data: this+0x478, Member, Type: class TCPSocket, tcpSock
	//_Data: this+0x104F0, Member, Type: struct RemoteSession, currentSession
	//_Data: this+0x10530, Member, Type: int, finishPosition
	//_Data: this+0x10538, Member, Type: class Font *, font
	//_Data: this+0x10540, Member, Type: class std::vector<RemoteSession,std::allocator<RemoteSession> >, sessions
	//_Data: this+0x10558, Member, Type: struct DisconnectCountdown, dcCountdown
	//_Data: this+0x10580, Member, Type: double, lastSessionCheckTime
	//_Data: this+0x10588, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, welcomeMessage
	//_Data: this+0x105A8, Member, Type: struct DamageReportDef, damageReport
	//_Data: this+0x105C8, Member, Type: bool, isFlashingCache
	//_Data: this+0x105CC, Member, Type: struct WreckerProtection, wreckerProtection
	//_Data: this+0x105DC, Member, Type: unsigned int, resultScreenTime
	//_Data: this+0x105E0, Member, Type: unsigned int, raceOverTime
	//_Data: this+0x105E4, Member, Type: bool, isGasPenaltyDisabled
	//_Data: this+0x105E8, Member, Type: unsigned int, pitWindowStart
	//_Data: this+0x105EC, Member, Type: unsigned int, pitWindowEnd
	//_Data: this+0x105F0, Member, Type: bool, hasLeaderFinished
	//_Data: this+0x105F1, Member, Type: bool, hasPlayerFinished
	//_Data: this+0x105F8, Member, Type: double, raceClosingTime
	//_Data: this+0x10600, Member, Type: bool, isResultScreenOn
	//_Data: this+0x10601, Member, Type: bool, hasExtraLap
	//_Data: this+0x10602, Member, Type: bool, isLastLap
	//_Data: this+0x10603, Member, Type: bool, ignoreResultTeleport
	//_Data: this+0x10604, Member, Type: int, invertedGridPositions
	//_Data: this+0x10608, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, localCarCurrentSplits
	//_Data: this+0x10620, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, localCarBestSplits
	//_Data: this+0x10638, Member, Type: class std::vector<unsigned int,std::allocator<unsigned int> >, bestSplits
	//_Data: this+0x10650, Member, Type: unsigned int, globalBestlap
	//_Data: this+0x10658, Member, Type: class std::vector<Lap,std::allocator<Lap> >, localCarLaps
	//_Data: this+0x10670, Member, Type: unsigned int, localCarBestLap
	//_Data: this+0x10674, Member, Type: unsigned int, instanceLocalCarBestLap
	//_Data: this+0x10678, Member, Type: struct Lap, localCarLastLap
	//_Data: this+0x106C0, Member, Type: bool, useLog
	//_Data: this+0x106C1, Member, Type: bool, playerIsSpectator
	//_Data: this+0x106C8, Member, Type: class std::basic_ofstream<wchar_t,std::char_traits<wchar_t> >, log
	//_Data: this+0x107D0, Member, Type: struct ClientQOSData, qos
	//_Tag 11
	//_Data: this+0x107E8, Member, Type: struct ACClient::ClientEndSession, endSession
	//_Data: this+0x107F8, Member, Type: class std::vector<ClientCollisionEvent,std::allocator<ClientCollisionEvent> >, clientColissionEvents
	//_Data: this+0x10810, Member, Type: double, lastClientEventSendTime
	//_Func: private void sendCarPosition(); @loc=static @len=2142 @rva=336432
	//_Func: private void onMessage(UDPMessage & msg); @loc=static @len=3695 @rva=309712
	//_Func: private void onLapCompleted(OnLapCompletedEvent & ev); @loc=static @len=410 @rva=309296
	//_Func: private void onNewSession(UDPPacket & pak, bool isHandShake); @loc=static @len=3369 @rva=320672
	//_Func: private void onRemoteLapCompleted(UDPPacket & pak); @loc=static @len=2377 @rva=324720
	//_Func: private void onRemoteSectorSplit(UDPPacket & pak); @loc=static @len=300 @rva=327104
	//_Func: private void onPhysicsStep(double pt); @loc=static @len=667 @rva=324048
	//_Func: private void receiveFromSocket(); @loc=optimized @len=0 @rva=0
	//_Func: private void logMessage(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & message); @loc=static @len=203 @rva=307872
	//_Func: private int getNumberOfActiveCars(); @loc=optimized @len=0 @rva=0
	//_Func: private unsigned char getSessionIdFromCarAvatarID(int carAvatarID); @loc=static @len=149 @rva=295776
	//_Func: private void updateQOS(double pt); @loc=static @len=324 @rva=344528
	//_Func: private void associate(); @loc=static @len=147 @rva=280128
	//_Func: private void onMessageTCP(UDPMessage & msg); @loc=static @len=7258 @rva=313408
	//_Func: private bool handleLocalAdminMessages(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * msg); @loc=static @len=1241 @rva=296560
	//_Func: private void onTyreCompoundChanged(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & shortName); @loc=static @len=136 @rva=328656
	//_Func: private void onCollisionEvent(OnCollisionEvent &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: private void addCollisionEvent(NetCarStateProvider * netCar, float speed, vec3f & worldPos, vec3f & relPos); @loc=static @len=388 @rva=277152
	//_Func: private void onWelcomeMessageReceived(UDPPacket & pak); @loc=static @len=144 @rva=328800
	//_Func: private void onSetupReceived(UDPPacket & pak); @loc=static @len=820 @rva=327824
	//_Func: private void onDRSZoneReceived(UDPPacket & pak); @loc=static @len=771 @rva=308512
	//_Func: private void onRemoteSunAngleReceived(float angle); @loc=static @len=117 @rva=327408
	//_Func: private void updateDamageReport(); @loc=static @len=360 @rva=344160
	//_Func: private void onSectorSplit(OnSectorSplitEvent & ev); @loc=static @len=282 @rva=327536
	//_Func: private void onCollisionWithCar(); @loc=static @len=428 @rva=308080
	//_Func: public ACClient & operator=(ACClient &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(ACClient)==67608),"bad size");
		static_assert((offsetof(ACClient,evOnChatMessage)==0x58),"bad off");
		static_assert((offsetof(ACClient,evOnOnlineNewSession)==0x70),"bad off");
		static_assert((offsetof(ACClient,evOnOnlineEndSession)==0x88),"bad off");
		static_assert((offsetof(ACClient,evOnLapCompleted)==0xA0),"bad off");
		static_assert((offsetof(ACClient,evOnVoteReceived)==0xB8),"bad off");
		static_assert((offsetof(ACClient,evOnVoteNotPassed)==0xD0),"bad off");
		static_assert((offsetof(ACClient,evOnMandatoryPitDone)==0xE8),"bad off");
		static_assert((offsetof(ACClient,serverName)==0x100),"bad off");
		static_assert((offsetof(ACClient,remoteSpring)==0x120),"bad off");
		static_assert((offsetof(ACClient,remoteDamper)==0x124),"bad off");
		static_assert((offsetof(ACClient,remoteFactor)==0x128),"bad off");
		static_assert((offsetof(ACClient,driverInfo)==0x130),"bad off");
		static_assert((offsetof(ACClient,serverInfo)==0x1B0),"bad off");
		static_assert((offsetof(ACClient,guid)==0x1F8),"bad off");
		static_assert((offsetof(ACClient,rules)==0x218),"bad off");
		static_assert((offsetof(ACClient,serverDrivingAssists)==0x21C),"bad off");
		static_assert((offsetof(ACClient,isTVMode)==0x228),"bad off");
		static_assert((offsetof(ACClient,handshakeServerTimeS)==0x230),"bad off");
		static_assert((offsetof(ACClient,playerCarMD5)==0x238),"bad off");
		static_assert((offsetof(ACClient,debugStartingLights)==0x250),"bad off");
		static_assert((offsetof(ACClient,isAssociated)==0x251),"bad off");
		static_assert((offsetof(ACClient,transitionInfo)==0x258),"bad off");
		static_assert((offsetof(ACClient,sessionResultsGT)==0x330),"bad off");
		static_assert((offsetof(ACClient,sok)==0x3A0),"bad off");
		static_assert((offsetof(ACClient,checksumResults)==0x3D8),"bad off");
		static_assert((offsetof(ACClient,sim)==0x3F0),"bad off");
		static_assert((offsetof(ACClient,car)==0x3F8),"bad off");
		static_assert((offsetof(ACClient,votingManager)==0x400),"bad off");
		static_assert((offsetof(ACClient,avatar)==0x408),"bad off");
		static_assert((offsetof(ACClient,lastSendTime)==0x410),"bad off");
		static_assert((offsetof(ACClient,sendInterval)==0x418),"bad off");
		static_assert((offsetof(ACClient,lastChatMessage)==0x420),"bad off");
		static_assert((offsetof(ACClient,lastSpectatorPulse)==0x428),"bad off");
		static_assert((offsetof(ACClient,serverIP)==0x430),"bad off");
		static_assert((offsetof(ACClient,sessionID)==0x440),"bad off");
		static_assert((offsetof(ACClient,netCars)==0x448),"bad off");
		static_assert((offsetof(ACClient,pakSequenceIndex)==0x460),"bad off");
		static_assert((offsetof(ACClient,handshakeCarPosition)==0x464),"bad off");
		static_assert((offsetof(ACClient,wrongWayIndicator)==0x468),"bad off");
		static_assert((offsetof(ACClient,numWrongWayInfractions)==0x470),"bad off");
		static_assert((offsetof(ACClient,tcpSock)==0x478),"bad off");
		static_assert((offsetof(ACClient,currentSession)==0x104F0),"bad off");
		static_assert((offsetof(ACClient,finishPosition)==0x10530),"bad off");
		static_assert((offsetof(ACClient,font)==0x10538),"bad off");
		static_assert((offsetof(ACClient,sessions)==0x10540),"bad off");
		static_assert((offsetof(ACClient,dcCountdown)==0x10558),"bad off");
		static_assert((offsetof(ACClient,lastSessionCheckTime)==0x10580),"bad off");
		static_assert((offsetof(ACClient,welcomeMessage)==0x10588),"bad off");
		static_assert((offsetof(ACClient,damageReport)==0x105A8),"bad off");
		static_assert((offsetof(ACClient,isFlashingCache)==0x105C8),"bad off");
		static_assert((offsetof(ACClient,wreckerProtection)==0x105CC),"bad off");
		static_assert((offsetof(ACClient,resultScreenTime)==0x105DC),"bad off");
		static_assert((offsetof(ACClient,raceOverTime)==0x105E0),"bad off");
		static_assert((offsetof(ACClient,isGasPenaltyDisabled)==0x105E4),"bad off");
		static_assert((offsetof(ACClient,pitWindowStart)==0x105E8),"bad off");
		static_assert((offsetof(ACClient,pitWindowEnd)==0x105EC),"bad off");
		static_assert((offsetof(ACClient,hasLeaderFinished)==0x105F0),"bad off");
		static_assert((offsetof(ACClient,hasPlayerFinished)==0x105F1),"bad off");
		static_assert((offsetof(ACClient,raceClosingTime)==0x105F8),"bad off");
		static_assert((offsetof(ACClient,isResultScreenOn)==0x10600),"bad off");
		static_assert((offsetof(ACClient,hasExtraLap)==0x10601),"bad off");
		static_assert((offsetof(ACClient,isLastLap)==0x10602),"bad off");
		static_assert((offsetof(ACClient,ignoreResultTeleport)==0x10603),"bad off");
		static_assert((offsetof(ACClient,invertedGridPositions)==0x10604),"bad off");
		static_assert((offsetof(ACClient,localCarCurrentSplits)==0x10608),"bad off");
		static_assert((offsetof(ACClient,localCarBestSplits)==0x10620),"bad off");
		static_assert((offsetof(ACClient,bestSplits)==0x10638),"bad off");
		static_assert((offsetof(ACClient,globalBestlap)==0x10650),"bad off");
		static_assert((offsetof(ACClient,localCarLaps)==0x10658),"bad off");
		static_assert((offsetof(ACClient,localCarBestLap)==0x10670),"bad off");
		static_assert((offsetof(ACClient,instanceLocalCarBestLap)==0x10674),"bad off");
		static_assert((offsetof(ACClient,localCarLastLap)==0x10678),"bad off");
		static_assert((offsetof(ACClient,useLog)==0x106C0),"bad off");
		static_assert((offsetof(ACClient,playerIsSpectator)==0x106C1),"bad off");
		static_assert((offsetof(ACClient,log)==0x106C8),"bad off");
		static_assert((offsetof(ACClient,qos)==0x107D0),"bad off");
		static_assert((offsetof(ACClient,endSession)==0x107E8),"bad off");
		static_assert((offsetof(ACClient,clientColissionEvents)==0x107F8),"bad off");
		static_assert((offsetof(ACClient,lastClientEventSendTime)==0x10810),"bad off");
	};
};

//UDT: struct AeroMap @len=112
	//_Func: public void ~AeroMap(); @loc=static @len=9 @rva=2839440
	//_Data: this+0x0, Member, Type: float, referenceArea
	//_Data: this+0x4, Member, Type: float, CD
	//_Data: this+0x8, Member, Type: float, CL
	//_Data: this+0xC, Member, Type: float, frontShare
	//_Data: this+0x10, Member, Type: float, CDX
	//_Data: this+0x14, Member, Type: float, CDY
	//_Data: this+0x18, Member, Type: float, CDA
	//_Data: this+0x20, Member, Type: class IRigidBody *, carBody
	//_Data: this+0x28, Member, Type: float, dynamicCD
	//_Data: this+0x2C, Member, Type: float, dynamicCL
	//_Data: this+0x30, Member, Type: float, airDensity
	//_Data: this+0x38, Member, Type: class std::vector<Wing,std::allocator<Wing> >, wings
	//_Func: public void init(Car * a_car, vec3f & frontAP, vec3f & rearAP, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * dataPath); @loc=static @len=164 @rva=2841760
	//_Func: public void step(float dt); @loc=static @len=135 @rva=2847056
	//_Func: public std::vector<WingState,std::allocator<WingState> > getWingStatus(); @loc=static @len=213 @rva=2841536
	//_Func: public float getCurrentLiftKG(); @loc=static @len=36 @rva=2841488
	//_Func: public float getCurrentDragKG(); @loc=static @len=36 @rva=2841440
	//_Data: this+0x50, Member, Type: class vec3f, frontApplicationPoint
	//_Data: this+0x5C, Member, Type: class vec3f, rearApplicationPoint
	//_Data: this+0x68, Member, Type: class Car *, car
	//_Func: protected void addDrag(vec3f & lv); @loc=static @len=551 @rva=2840672
	//_Func: protected void addLift(vec3f & localVelocity); @loc=static @len=194 @rva=2841232
	//_Func: protected void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataPath); @loc=static @len=4692 @rva=2841936
	//_Func: public void AeroMap(AeroMap &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void AeroMap(); @loc=optimized @len=0 @rva=0
	//_Func: public AeroMap & operator=(AeroMap &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(AeroMap)==112),"bad size");
		static_assert((offsetof(AeroMap,referenceArea)==0x0),"bad off");
		static_assert((offsetof(AeroMap,CD)==0x4),"bad off");
		static_assert((offsetof(AeroMap,CL)==0x8),"bad off");
		static_assert((offsetof(AeroMap,frontShare)==0xC),"bad off");
		static_assert((offsetof(AeroMap,CDX)==0x10),"bad off");
		static_assert((offsetof(AeroMap,CDY)==0x14),"bad off");
		static_assert((offsetof(AeroMap,CDA)==0x18),"bad off");
		static_assert((offsetof(AeroMap,carBody)==0x20),"bad off");
		static_assert((offsetof(AeroMap,dynamicCD)==0x28),"bad off");
		static_assert((offsetof(AeroMap,dynamicCL)==0x2C),"bad off");
		static_assert((offsetof(AeroMap,airDensity)==0x30),"bad off");
		static_assert((offsetof(AeroMap,wings)==0x38),"bad off");
		static_assert((offsetof(AeroMap,frontApplicationPoint)==0x50),"bad off");
		static_assert((offsetof(AeroMap,rearApplicationPoint)==0x5C),"bad off");
		static_assert((offsetof(AeroMap,car)==0x68),"bad off");
	};
};

//UDT: struct Drivetrain @len=1608
	//_Func: public void ~Drivetrain(); @loc=static @len=195 @rva=2514976
	//_Data: this+0x0, Member, Type: bool, isGearGrinding
	//_Data: this+0x4, Member, Type: float, finalRatio
	//_Data: this+0x8, Member, Type: struct GearElement, engine
	//_Data: this+0x20, Member, Type: struct GearElement, drive
	//_Data: this+0x38, Member, Type: struct GearElement, outShaftL
	//_Data: this+0x50, Member, Type: struct GearElement, outShaftR
	//_Data: this+0x68, Member, Type: struct GearElement, outShaftLF
	//_Data: this+0x80, Member, Type: struct GearElement, outShaftRF
	//_Data: this+0x98, Member, Type: class std::vector<SGearRatio,std::allocator<SGearRatio> >, gears
	//_Data: this+0xB0, Member, Type: double, rootVelocity
	//_Data: this+0xB8, Member, Type: bool, clutchOpenState
	//_Data: this+0xC0, Member, Type: double, ratio
	//_Data: this+0xC8, Member, Type: float, diffPowerRamp
	//_Data: this+0xCC, Member, Type: float, diffCoastRamp
	//_Data: this+0xD0, Member, Type: float, diffPreLoad
	//_Data: this+0xD4, Member, Type: enum DifferentialType, diffType
	//_Data: this+0xD8, Member, Type: double, cutOff
	//_Data: this+0xE0, Member, Type: struct Engine, acEngine
	//_Data: this+0x4C8, Member, Type: bool, isShifterSupported
	//_Data: this+0x4D0, Member, Type: double, clutchMaxTorque
	//_Data: this+0x4D8, Member, Type: float, totalTorque
	//_Data: this+0x4DC, Member, Type: float, awdFrontShare
	//_Data: this+0x4E0, Member, Type: struct DifferentialSetting, awdFrontDiff
	//_Data: this+0x4F0, Member, Type: struct DifferentialSetting, awdRearDiff
	//_Data: this+0x500, Member, Type: struct DifferentialSetting, awdCenterDiff
	//_Data: this+0x510, Member, Type: struct DownshiftProtection, downshiftProtection
	//_Data: this+0x520, Member, Type: struct AWD2Data, awd2
	//_Data: this+0x538, Member, Type: float, currentClutchTorque
	//_Data: this+0x540, Member, Type: class std::function<void __cdecl(void)>, downshiftProtectionFunction
	//_Data: this+0x560, Member, Type: class Event<OnGearRequestEvent>, evOnGearRequest
	//_Func: public void init(Car * car); @loc=static @len=682 @rva=2518464
	//_Func: public void reset(); @loc=static @len=81 @rva=2527872
	//_Func: public void addGear(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * name, double ratio); @loc=static @len=237 @rva=2516336
	//_Func: public void step(float dt); @loc=static @len=201 @rva=2535728
	//_Func: public bool hasDynamicControllers(); @loc=static @len=46 @rva=2518416
	//_Func: public SGearRatio getCurrentGear(); @loc=static @len=88 @rva=2517776
	//_Func: public void setCurrentGear(int index, bool force); @loc=static @len=400 @rva=2527968
	//_Func: public bool gearUp(); @loc=static @len=288 @rva=2517488
	//_Func: public bool gearDown(); @loc=static @len=905 @rva=2516576
	//_Func: public int getCurrentGearIndex(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isChangingGear(); @loc=static @len=12 @rva=2519984
	//_Func: public TractionType getTractionType(); @loc=static @len=7 @rva=2608000
	//_Func: public float getEngineRPM(); @loc=static @len=26 @rva=2517888
	//_Func: public float getDrivetrainSpeed(); @loc=static @len=10 @rva=2517872
	//_Func: public void setGearRatio(int index, float value); @loc=static @len=104 @rva=2528368
	//_Func: public bool hasAutoCutoff(); @loc=optimized @len=0 @rva=0
	//_Func: public void forceAutoCutoffTime(double value); @loc=static @len=9 @rva=864032
	//_Func: public void addWheelTorqueGenerator(ITorqueGenerator *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getRpmWindowStatus(); @loc=static @len=21 @rva=2518384
	//_Func: public double getGearUpTime(); @loc=optimized @len=0 @rva=0
	//_Func: public double getGearDnTime(); @loc=static @len=9 @rva=2741824
	//_Func: public float projectRPMAtDownshift(); @loc=static @len=112 @rva=2526944
	//_Data: this+0x578, Member, Type: class Car *, car
	//_Data: this+0x580, Member, Type: enum TractionType, tractionType
	//_Data: this+0x584, Member, Type: int, currentGear
	//_Data: this+0x588, Member, Type: double, lastRatio
	//_Data: this+0x590, Member, Type: struct Tyre *, tyreLeft
	//_Data: this+0x598, Member, Type: struct Tyre *, tyreRight
	//_Data: this+0x5A0, Member, Type: struct GearRequestStatus, gearRequest
	//_Data: this+0x5C0, Member, Type: double, gearUpTime
	//_Data: this+0x5C8, Member, Type: double, gearDnTime
	//_Data: this+0x5D0, Member, Type: double, autoCutOffTime
	//_Data: this+0x5D8, Member, Type: double, validShiftRPMWindow
	//_Data: this+0x5E0, Member, Type: double, controlsWindowGain
	//_Data: this+0x5E8, Member, Type: class std::vector<ITorqueGenerator *,std::allocator<ITorqueGenerator *> >, wheelTorqueGenerators
	//_Data: this+0x600, Member, Type: double, damageRpmWindow
	//_Data: this+0x608, Member, Type: double, orgRpmWindow
	//_Data: this+0x610, Member, Type: int[0x4], lockCounter
	//_Data: this+0x620, Member, Type: struct DrivetrainControllers, controllers
	//_Data: this+0x640, Member, Type: float, clutchInertia
	//_Data: this+0x644, Member, Type: float, locClutch
	//_Func: protected double getInertiaFromEngine(); @loc=static @len=119 @rva=2517920
	//_Func: protected double getInertiaFromWheels(); @loc=static @len=328 @rva=2518048
	//_Func: protected void updateElementsStatus(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void stepEngine(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void reallignSpeeds(float dt); @loc=static @len=333 @rva=2527488
	//_Func: protected void stepClutch(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void stepGearRequests(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void loadINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & dataPath); @loc=static @len=6813 @rva=2520128
	//_Func: protected bool isGearboxLocked(); @loc=static @len=125 @rva=2520000
	//_Func: protected void accelerateDrivetrainBlock(double acc, bool fromEngine); @loc=static @len=174 @rva=2516160
	//_Func: protected void accelerateFromWheels(double  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void setDrivenTyres(Tyre *  _arg0, Tyre *  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: protected void step2WD(float dt); @loc=static @len=3381 @rva=2528480
	//_Func: protected void step4WD(float dt); @loc=static @len=2907 @rva=2531872
	//_Func: protected void step4WD_new(float dt); @loc=static @len=930 @rva=2534784
	//_Func: protected void initControllers(); @loc=static @len=827 @rva=2519152
	//_Func: protected void stepControllers(float dt); @loc=static @len=239 @rva=2535936
	//_Func: protected double getClutchTorque(); @loc=optimized @len=0 @rva=0
	//_Func: public void Drivetrain(Drivetrain &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Drivetrain(); @loc=static @len=505 @rva=2545776
	//_Func: public Drivetrain & operator=(Drivetrain &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Drivetrain)==1608),"bad size");
		static_assert((offsetof(Drivetrain,isGearGrinding)==0x0),"bad off");
		static_assert((offsetof(Drivetrain,finalRatio)==0x4),"bad off");
		static_assert((offsetof(Drivetrain,engine)==0x8),"bad off");
		static_assert((offsetof(Drivetrain,drive)==0x20),"bad off");
		static_assert((offsetof(Drivetrain,outShaftL)==0x38),"bad off");
		static_assert((offsetof(Drivetrain,outShaftR)==0x50),"bad off");
		static_assert((offsetof(Drivetrain,outShaftLF)==0x68),"bad off");
		static_assert((offsetof(Drivetrain,outShaftRF)==0x80),"bad off");
		static_assert((offsetof(Drivetrain,gears)==0x98),"bad off");
		static_assert((offsetof(Drivetrain,rootVelocity)==0xB0),"bad off");
		static_assert((offsetof(Drivetrain,clutchOpenState)==0xB8),"bad off");
		static_assert((offsetof(Drivetrain,ratio)==0xC0),"bad off");
		static_assert((offsetof(Drivetrain,diffPowerRamp)==0xC8),"bad off");
		static_assert((offsetof(Drivetrain,diffCoastRamp)==0xCC),"bad off");
		static_assert((offsetof(Drivetrain,diffPreLoad)==0xD0),"bad off");
		static_assert((offsetof(Drivetrain,diffType)==0xD4),"bad off");
		static_assert((offsetof(Drivetrain,cutOff)==0xD8),"bad off");
		static_assert((offsetof(Drivetrain,acEngine)==0xE0),"bad off");
		static_assert((offsetof(Drivetrain,isShifterSupported)==0x4C8),"bad off");
		static_assert((offsetof(Drivetrain,clutchMaxTorque)==0x4D0),"bad off");
		static_assert((offsetof(Drivetrain,totalTorque)==0x4D8),"bad off");
		static_assert((offsetof(Drivetrain,awdFrontShare)==0x4DC),"bad off");
		static_assert((offsetof(Drivetrain,awdFrontDiff)==0x4E0),"bad off");
		static_assert((offsetof(Drivetrain,awdRearDiff)==0x4F0),"bad off");
		static_assert((offsetof(Drivetrain,awdCenterDiff)==0x500),"bad off");
		static_assert((offsetof(Drivetrain,downshiftProtection)==0x510),"bad off");
		static_assert((offsetof(Drivetrain,awd2)==0x520),"bad off");
		static_assert((offsetof(Drivetrain,currentClutchTorque)==0x538),"bad off");
		static_assert((offsetof(Drivetrain,downshiftProtectionFunction)==0x540),"bad off");
		static_assert((offsetof(Drivetrain,evOnGearRequest)==0x560),"bad off");
		static_assert((offsetof(Drivetrain,car)==0x578),"bad off");
		static_assert((offsetof(Drivetrain,tractionType)==0x580),"bad off");
		static_assert((offsetof(Drivetrain,currentGear)==0x584),"bad off");
		static_assert((offsetof(Drivetrain,lastRatio)==0x588),"bad off");
		static_assert((offsetof(Drivetrain,tyreLeft)==0x590),"bad off");
		static_assert((offsetof(Drivetrain,tyreRight)==0x598),"bad off");
		static_assert((offsetof(Drivetrain,gearRequest)==0x5A0),"bad off");
		static_assert((offsetof(Drivetrain,gearUpTime)==0x5C0),"bad off");
		static_assert((offsetof(Drivetrain,gearDnTime)==0x5C8),"bad off");
		static_assert((offsetof(Drivetrain,autoCutOffTime)==0x5D0),"bad off");
		static_assert((offsetof(Drivetrain,validShiftRPMWindow)==0x5D8),"bad off");
		static_assert((offsetof(Drivetrain,controlsWindowGain)==0x5E0),"bad off");
		static_assert((offsetof(Drivetrain,wheelTorqueGenerators)==0x5E8),"bad off");
		static_assert((offsetof(Drivetrain,damageRpmWindow)==0x600),"bad off");
		static_assert((offsetof(Drivetrain,orgRpmWindow)==0x608),"bad off");
		static_assert((offsetof(Drivetrain,lockCounter)==0x610),"bad off");
		static_assert((offsetof(Drivetrain,controllers)==0x620),"bad off");
		static_assert((offsetof(Drivetrain,clutchInertia)==0x640),"bad off");
		static_assert((offsetof(Drivetrain,locClutch)==0x644),"bad off");
	};
};

//UDT: class CarAvatar @len=4776 @vfcount=6
	//_Base: class GameObject @off=0 @len=88
	//_Func: public void CarAvatar(CarAvatar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void CarAvatar(Sim * isim, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & unixName, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin); @loc=static @len=3097 @rva=840096
	//_Func: public void CarAvatar(Sim * isim, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & unixName, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & skin, ICarPhysicsStateProvider * physicsStateProvider); @loc=static @len=1567 @rva=843200
	//_Func: public void ~CarAvatar(); @virtual vtpo=0 vfid=0 @loc=static @len=865 @rva=846608
	//_Data: this+0x58, Member, Type: class EventTriggerOnChange<int>, evOnGearChanged
	//_Data: this+0x88, Member, Type: class Event<OnLapCompletedEvent>, evOnLapCompleted
	//_Data: this+0xA0, Member, Type: class Event<OnSectorSplitEvent>, evOnSectorSplit
	//_Data: this+0xB8, Member, Type: class Event<OnTyreCompoundChanged>, evOnTyreCompoundChanged
	//_Data: this+0xD0, Member, Type: class Event<bool>, evOnBackfireTriggered
	//_Data: this+0xE8, Member, Type: class Event<bool>, evOnDownshiftProtection
	//_Data: this+0x100, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, externalTyreCompoundShortName
	//_Data: this+0x120, Member, Type: class vec3f, mirrorPosition
	//_Data: this+0x130, Member, Type: class Sim *, sim
	//_Data: this+0x138, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, unixName
	//_Data: this+0x158, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, configName
	//_Data: this+0x178, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, guiName
	//_Data: this+0x198, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, guiShortName
	//_Data: this+0x1B8, Member, Type: class Node *, bodyTransform
	//_Data: this+0x1C0, Member, Type: class Node *, steerTransformHR
	//_Data: this+0x1C8, Member, Type: class Node *, steerTransformLR
	//_Data: this+0x1D0, Member, Type: class mat44f, orgSteerMatrix
	//_Data: this+0x210, Member, Type: class NodeBoundingSphere *, carNode
	//_Data: this+0x218, Member, Type: class vec3f, driverEyesPosition
	//_Data: this+0x224, Member, Type: class mat44f, bodyMatrix
	//_Data: this+0x268, Member, Type: struct CarPhysicsState, physicsState
	//_Data: this+0xDD8, Member, Type: struct AIState, aiState
	//_Data: this+0xE18, Member, Type: class CarAudioFMOD *, carAudioFMOD
	//_Data: this+0xE20, Member, Type: class SkidMarkBuffer *[0x4], skidMarkBuffers
	//_Data: this+0xE40, Member, Type: struct CarPhysicsInfo, physicsInfo
	//_Data: this+0xF80, Member, Type: class std::unique_ptr<RaceEngineer,std::default_delete<RaceEngineer> >, raceEngineer
	//_Data: this+0xF88, Member, Type: struct ModelBoundariesCoordinates, modelBoundaries
	//_Data: this+0xFA0, Member, Type: class std::vector<WingState,std::allocator<WingState> >, wingsStatus
	//_Data: this+0xFB8, Member, Type: float, onBoardExposure
	//_Data: this+0xFBC, Member, Type: float, outBoardExposure
	//_Data: this+0xFC0, Member, Type: float, dashBoardExposure
	//_Data: this+0xFC8, Member, Type: class std::vector<CameraCarDefinition,std::allocator<CameraCarDefinition> >, cameras
	//_Data: this+0xFE0, Member, Type: bool, isDriverHR
	//_Data: this+0xFE8, Member, Type: class DriverModel *, driverModel_HR
	//_Data: this+0xFF0, Member, Type: class DriverModel *, driverModel_LR
	//_Data: this+0xFF8, Member, Type: class ISuspensionAvatar *, suspensionAvatar
	//_Data: this+0x1000, Member, Type: bool, isBlackFlagged
	//_Data: this+0x1008, Member, Type: class NetCarStateProvider *, netCarStateProvider
	//_Data: this+0x1010, Member, Type: class CarLodManager *, lodManager
	//_Data: this+0x1018, Member, Type: class ConstrainedObjectsManager *, constrainedObjectManager
	//_Data: this+0x1020, Member, Type: class BufferedChannel<std::pair<int,int> >, chTyreCompound
	//_Func: public void setP2PActivations(int activations); @loc=static @len=132 @rva=894352
	//_Func: public void setP2PStartingActivations(int pos); @loc=static @len=142 @rva=894496
	//_Func: public void setRestrictor(float v); @loc=static @len=134 @rva=894640
	//_Func: public float getRestrictor(); @loc=static @len=45 @rva=865792
	//_Func: public void makeBodyMatrix(mat44f & bm, mat44f & res); @loc=static @len=343 @rva=888512
	//_Func: public void setNewPhysicsState(CarPhysicsState & ps, float dt); @loc=static @len=361 @rva=893984
	//_Func: public DriverModel * getActiveDriverModel(); @loc=static @len=25 @rva=864432
	//_Func: public int getAILapsToComplete(); @loc=static @len=23 @rva=864400
	//_Func: public PitStopTime getPitstopTime(float fuel_requested, bool changeTyres, bool repairBody, bool repairEngine, bool repairSus, bool useRandomizer); @loc=static @len=133 @rva=865648
	//_Func: public bool isRequestingPitStop(); @loc=static @len=21 @rva=888176
	//_Func: public float modifyUserFFGain(float offset); @loc=static @len=200 @rva=889072
	//_Func: public void setUserFFGain(float gain); @loc=static @len=142 @rva=896336
	//_Func: public float getUserFFGain(); @loc=static @len=9 @rva=866960
	//_Func: public bool isCurrentLapValid(); @loc=static @len=75 @rva=887632
	//_Func: public void setSkin(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setFFMult(float mult); @loc=static @len=134 @rva=893280
	//_Func: public float getFFMult(); @loc=static @len=25 @rva=865424
	//_Func: public void setTurboBoost(float value); @loc=static @len=490 @rva=895456
	//_Func: public std::pair<unsigned int,unsigned int> getTCMode(); @loc=static @len=56 @rva=866336
	//_Func: public std::pair<unsigned int,unsigned int> getABSMode(); @loc=static @len=56 @rva=864336
	//_Func: public bool isConnected(); @loc=static @len=15 @rva=887616
	//_Func: public bool isInPit(); @loc=static @len=94 @rva=887760
	//_Func: public void armFirstLap(); @loc=static @len=79 @rva=860928
	//_Func: public void setTyreCompound(unsigned int  _arg0, int  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public void setTyreCompound(unsigned int index, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & name); @loc=static @len=369 @rva=895952
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getTyreCompound(unsigned int index, bool fullname); @loc=static @len=249 @rva=866688
	//_Func: public unsigned int getTyreCompoundIndex(unsigned int tyre_index); @loc=static @len=10 @rva=866944
	//_Func: public float getWingAngle(int wingIndex); @loc=static @len=63 @rva=866976
	//_Func: public ICarPhysicsStateProvider * getPhysicsStateProvider(); @loc=optimized @len=0 @rva=0
	//_Func: public CarColliderManager * getColliderManager(); @loc=static @len=20 @rva=864576
	//_Func: public void setDamageLevel(float lvl); @loc=static @len=134 @rva=892960
	//_Func: public int getGuid(); @loc=static @len=7 @rva=865536
	//_Func: public void update(float deltaT); @virtual vtpo=0 vfid=1 @loc=static @len=1453 @rva=899120
	//_Func: public void goToSpawnPosition(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName); @loc=static @len=134 @rva=867040
	//_Func: public bool isInSpawnPosition(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName); @loc=static @len=248 @rva=887872
	//_Func: public void setSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & setName, int index); @loc=static @len=352 @rva=894816
	//_Func: public void setControlsProvider(ICarControlsProvider * controls); @loc=static @len=133 @rva=892816
	//_Func: public vec3f getGraphicsOffset(); @loc=static @len=25 @rva=865504
	//_Func: public float getGraphicsPitchRotation(); @loc=optimized @len=0 @rva=0
	//_Func: public void setGraphicsPitchRotation(float v); @loc=static @len=9 @rva=893744
	//_Func: public void setGraphicsOffset(vec3f np); @loc=static @len=22 @rva=893712
	//_Func: public ICarControlsProvider * getControlsProvider(); @loc=static @len=19 @rva=864608
	//_Func: public std::vector<DebugLine,std::allocator<DebugLine> > getSuspensionDebugLines(int index); @loc=static @len=77 @rva=866256
	//_Func: public vec3f getRoadDirection(); @loc=static @len=110 @rva=865840
	//_Func: public SetupManager * getSetupManager(); @loc=static @len=20 @rva=866064
	//_Func: public void onPostLoad(); @loc=static @len=322 @rva=889520
	//_Func: public void onStartReplay(bool & mode); @loc=static @len=283 @rva=889856
	//_Func: public void onStopReplay(bool & mode); @loc=static @len=264 @rva=890144
	//_Func: public void onReplayRewind(int &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool isAbsInAction(); @loc=static @len=83 @rva=887520
	//_Func: public void setAbsEnabled(bool mode, bool force); @loc=static @len=137 @rva=891840
	//_Func: public void cycleAbsMode(int value); @loc=static @len=394 @rva=862288
	//_Func: public bool isAbsAvailable(); @loc=static @len=21 @rva=887440
	//_Func: public bool isAbsEnabled(); @loc=static @len=40 @rva=887472
	//_Func: public bool isTcInAction(); @loc=static @len=21 @rva=888288
	//_Func: public void setTcEnabled(bool mode, bool force); @loc=static @len=137 @rva=895312
	//_Func: public void cycleTcMode(int value); @loc=static @len=394 @rva=863584
	//_Func: public bool isTcEnabled(); @loc=static @len=40 @rva=888240
	//_Func: public bool isTcAvailable(); @loc=static @len=21 @rva=888208
	//_Func: public bool isEdlEnabled(); @loc=static @len=40 @rva=887712
	//_Func: public void setEdlEnabled(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getEdlOutLevel(); @loc=static @len=25 @rva=865392
	//_Func: public void setAutoBlip(bool mode); @loc=static @len=132 @rva=891984
	//_Func: public bool getAutoBlip(); @loc=static @len=21 @rva=864464
	//_Func: public float getStabilityControl(); @loc=static @len=25 @rva=866224
	//_Func: public void setStabilityControl(float gain); @loc=static @len=134 @rva=895168
	//_Func: public float getFrontBias(); @loc=optimized @len=0 @rva=0
	//_Func: public void setFrontBias(int value); @loc=static @len=132 @rva=893424
	//_Func: public void setAutoShifter(bool mode); @loc=static @len=132 @rva=892240
	//_Func: public bool getAutoShifter(); @loc=static @len=21 @rva=864496
	//_Func: public void setVisible(bool vis); @loc=static @len=115 @rva=896480
	//_Func: public bool isVisible(); @loc=static @len=15 @rva=888320
	//_Func: public void updateERSCharge(); @loc=static @len=88 @rva=906032
	//_Func: public float getERSNormalizedRecharge(); @loc=optimized @len=0 @rva=0
	//_Func: public void setHeadlights(bool value); @loc=static @len=132 @rva=893760
	//_Func: public void resetPenalty(bool isTotalReset); @loc=static @len=132 @rva=891616
	//_Func: public TimeTransponder * getTimeTransponder(); @loc=static @len=20 @rva=866656
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getScreenName(); @loc=static @len=100 @rva=865952
	//_Func: public void setAutoClutchEnabled(bool mode); @loc=static @len=112 @rva=892128
	//_Func: public float getPackerRange(int index); @loc=static @len=37 @rva=865600
	//_Data: this+0x1048, Member, Type: float, fuelInExhaust
	//_Func: public void initGhostCar(bool isRecording, bool isPlaying); @loc=static @len=212 @rva=881936
	//_Func: public void setControlsLock(bool value); @loc=static @len=132 @rva=892672
	//_Func: public void lockGearBox(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setGentleStop(bool mode); @loc=static @len=132 @rva=893568
	//_Func: public double hasPenalty(); @loc=static @len=20 @rva=867184
	//_Func: public bool isMinSpeedPenaltyClearDisabled(); @loc=static @len=19 @rva=888128
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getCurrentSkin(); @loc=static @len=61 @rva=618160
	//_Func: public void resetTimeTransponder(); @loc=static @len=79 @rva=891760
	//_Func: public void resetFlames(); @loc=static @len=115 @rva=891440
	//_Func: public void setDriverInfo(DriverInfo & info); @loc=static @len=172 @rva=893104
	//_Func: public DriverInfo & getDriverInfo(); @loc=static @len=8 @rva=287840
	//_Func: public float getGraphicSteerDeg(); @loc=static @len=37 @rva=865456
	//_Func: public int getSpawnPositionIndex(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * setName); @loc=static @len=115 @rva=866096
	//_Func: public void setSlipStreamEffects(float receive, float generationSpeedFactor); @loc=static @len=17 @rva=894784
	//_Func: public double getKmPerLiter(); @loc=static @len=48 @rva=865552
	//_Func: public void setBlackFlag(PenaltyDescription descr); @loc=static @len=132 @rva=892528
	//_Func: public void lockControlsUntilTime(double time, double start, bool forceToPits); @loc=static @len=150 @rva=888352
	//_Func: public bool isInPitlane(); @loc=static @len=8 @rva=887856
	//_Func: public void getPitlaneTimes(float &  _arg0, float &  _arg1); @loc=optimized @len=0 @rva=0
	//_Func: public Car * getPhysicsCar(); @loc=optimized @len=0 @rva=0
	//_Func: public void resetGuidCounter(); @pure @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getCurrentCompoundShortName(); @loc=static @len=162 @rva=864768
	//_Func: public int getCurrentCompoundIdealPressure(unsigned short tyreIndex); @loc=static @len=125 @rva=864640
	//_Func: public int getCurrentCompoundStaticPressure(unsigned short tyreIndex); @loc=static @len=125 @rva=864944
	//_Func: public int getCurrentCompoundIndex(); @loc=optimized @len=0 @rva=0
	//_Func: public void getDistancesOnSpline(CarAvatar & car, float * frontDistance, float * backDistance); @loc=static @len=165 @rva=865072
	//_Func: public double getTimeDifferenceOnSpline(CarAvatar & car); @loc=static @len=255 @rva=866400
	//_Func: public float getBallastKG(); @loc=static @len=46 @rva=864528
	//_Func: public void setBallastKG(float ballast); @loc=static @len=134 @rva=892384
	//_Func: public std::vector<PenaltyRecord,std::allocator<PenaltyRecord> > getPenaltyRecords(); @loc=optimized @len=0 @rva=0
	//_Func: public float getFilteredSpeed(); @loc=optimized @len=0 @rva=0
	//_Func: public void setMultVolume(int value, bool active); @loc=static @len=65 @rva=893904
	//_Func: public void resetMultVolume(); @loc=static @len=35 @rva=891568
	//_Func: public INIReader openINI(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=509 @rva=890416
	//_Func: public int cycleEngineBrake(int value); @loc=static @len=202 @rva=863376
	//_Func: public bool cycleERSHeatCharging(); @loc=static @len=155 @rva=862688
	//_Func: public std::pair<int,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > cycleERSPower(int value); @loc=static @len=316 @rva=862848
	//_Func: public int cycleERSRecovery(int value); @loc=static @len=198 @rva=863168
	//_Func: public int getERSRecovery(); @loc=optimized @len=0 @rva=0
	//_Func: public std::pair<int,std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > getERSPower(); @loc=static @len=133 @rva=865248
	//_Func: public int getEngineBrakeSetting(); @loc=optimized @len=0 @rva=0
	//_Func: public bool getERSHeatCharging(); @loc=static @len=8 @rva=1243840
	//_Func: public bool getMandatoryPitstop(); @loc=optimized @len=0 @rva=0
	//_Func: public void setMandatoryPitstop(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public float getLastPitstopTime(); @loc=optimized @len=0 @rva=0
	//_Func: public void setLastPitstopTime(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void resetPitTimes(); @loc=optimized @len=0 @rva=0
	//_Data: this+0x104C, Member, Type: class vec3f, graphicsOffset
	//_Data: this+0x1058, Member, Type: float, graphicsPitchRotation
	//_Data: this+0x105C, Member, Type: float, userFFGain
	//_Data: this+0x1060, Member, Type: float, graphicSteerLockDegrees
	//_Data: this+0x1068, Member, Type: struct BackfireParams *, backfireParams
	//_Data: this+0x1070, Member, Type: class Node *, meshCollider
	//_Data: this+0x1078, Member, Type: class mat44f, pitPosition
	//_Data: this+0x10B8, Member, Type: struct DriverInfo, driverInfo
	//_Data: this+0x1138, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, currentSkin
	//_Data: static, [01559A94][0003:00046A94], Static Member, Type: int, guidCounter
	//_Data: this+0x1158, Member, Type: int, guid
	//_Data: this+0x1160, Member, Type: class Node *, modelLink
	//_Data: this+0x1168, Member, Type: class Car *, physics
	//_Data: this+0x1170, Member, Type: class vec3f[0x4], lastSkidPosition
	//_Data: this+0x11A0, Member, Type: class std::vector<IEventTrigger *,std::allocator<IEventTrigger *> >, eventTriggers
	//_Data: this+0x11B8, Member, Type: class ICarPhysicsStateProvider *, physicsStateProvider
	//_Data: this+0x11C0, Member, Type: bool, lockVirtualSteer
	//_Data: this+0x11C1, Member, Type: bool, hideArmsInCockpit
	//_Data: this+0x11C2, Member, Type: bool, hideSteer
	//_Data: this+0x11C8, Member, Type: class CarRaceInfo, raceInfo
	//_Data: this+0x11F0, Member, Type: class std::unique_ptr<PhysicsCarStateProvider,std::default_delete<PhysicsCarStateProvider> >, physicsCarStateProvider
	//_Data: this+0x11F8, Member, Type: class BufferedChannel<OnLapCompletedEvent>, lapQueue
	//_Data: this+0x1220, Member, Type: class BufferedChannel<OnSectorSplitEvent>, splitQueue
	//_Data: this+0x1248, Member, Type: class ICarPhysicsStateProvider *, nonReplayPhysicsStateProvider
	//_Data: this+0x1250, Member, Type: class Node *, beltOffNode
	//_Data: this+0x1258, Member, Type: class Node *, beltOnNode
	//_Data: this+0x1260, Member, Type: int[0x4], currentTyreCompoundIndex
	//_Data: this+0x1270, Member, Type: class CarAnimations *, carAnimations
	//_Data: this+0x1278, Member, Type: float, filteredSpeed
	//_Data: this+0x127C, Member, Type: int, currentEngineBrakeSetting
	//_Data: this+0x1280, Member, Type: bool, isHeatChargingBatteries
	//_Data: this+0x1284, Member, Type: int, currentERSPowerIndex
	//_Data: this+0x1288, Member, Type: int, currentERSRecovery
	//_Data: this+0x128C, Member, Type: float, lastERSBatteryCharge
	//_Data: this+0x1290, Member, Type: float, currentERSNormalizedRecharge
	//_Data: this+0x1294, Member, Type: bool, showDownShiftProtetion
	//_Data: this+0x1295, Member, Type: bool, mandatoryPitstopDone
	//_Data: this+0x1298, Member, Type: float, lastPitstopTime
	//_Data: this+0x129C, Member, Type: bool, inPitlane
	//_Data: this+0x129D, Member, Type: bool, wasInPitlane
	//_Data: this+0x12A0, Member, Type: float, pitLaneEntryTime
	//_Data: this+0x12A4, Member, Type: float, pitLaneExitTime
	//_Func: protected void forcePosition(mat44f & matrix); @loc=static @len=287 @rva=864048
	//_Func: protected void initCommon(); @loc=static @len=2729 @rva=874208
	//_Func: protected void initCommonPostPhysics(); @loc=static @len=3480 @rva=876944
	//_Func: protected void init3D(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > * skin); @loc=static @len=3110 @rva=867216
	//_Func: protected void initPhysics(); @loc=static @len=5154 @rva=882272
	//_Func: protected void updateSkidMarks(float dt); @loc=static @len=982 @rva=907136
	//_Func: protected void stepTriggers(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: protected void initDriver(); @loc=static @len=399 @rva=881536
	//_Func: protected void initCameraCar(); @loc=static @len=3871 @rva=870336
	//_Func: protected void initMirrorMaterials(); @loc=static @len=103 @rva=882160
	//_Func: protected void initControls(); @loc=static @len=1103 @rva=880432
	//_Func: protected void onNewSession(); @loc=static @len=238 @rva=889280
	//_Func: protected void updateFromChannels(float dt); @loc=static @len=817 @rva=906128
	//_Func: protected void checkACD(); @loc=static @len=679 @rva=861184
	//_Func: protected void updateInPitlaneState(float dt); @loc=static @len=171 @rva=906960
	//_Func: public CarAvatar & operator=(CarAvatar &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(CarAvatar)==4776),"bad size");
		static_assert((offsetof(CarAvatar,evOnGearChanged)==0x58),"bad off");
		static_assert((offsetof(CarAvatar,evOnLapCompleted)==0x88),"bad off");
		static_assert((offsetof(CarAvatar,evOnSectorSplit)==0xA0),"bad off");
		static_assert((offsetof(CarAvatar,evOnTyreCompoundChanged)==0xB8),"bad off");
		static_assert((offsetof(CarAvatar,evOnBackfireTriggered)==0xD0),"bad off");
		static_assert((offsetof(CarAvatar,evOnDownshiftProtection)==0xE8),"bad off");
		static_assert((offsetof(CarAvatar,externalTyreCompoundShortName)==0x100),"bad off");
		static_assert((offsetof(CarAvatar,mirrorPosition)==0x120),"bad off");
		static_assert((offsetof(CarAvatar,sim)==0x130),"bad off");
		static_assert((offsetof(CarAvatar,unixName)==0x138),"bad off");
		static_assert((offsetof(CarAvatar,configName)==0x158),"bad off");
		static_assert((offsetof(CarAvatar,guiName)==0x178),"bad off");
		static_assert((offsetof(CarAvatar,guiShortName)==0x198),"bad off");
		static_assert((offsetof(CarAvatar,bodyTransform)==0x1B8),"bad off");
		static_assert((offsetof(CarAvatar,steerTransformHR)==0x1C0),"bad off");
		static_assert((offsetof(CarAvatar,steerTransformLR)==0x1C8),"bad off");
		static_assert((offsetof(CarAvatar,orgSteerMatrix)==0x1D0),"bad off");
		static_assert((offsetof(CarAvatar,carNode)==0x210),"bad off");
		static_assert((offsetof(CarAvatar,driverEyesPosition)==0x218),"bad off");
		static_assert((offsetof(CarAvatar,bodyMatrix)==0x224),"bad off");
		static_assert((offsetof(CarAvatar,physicsState)==0x268),"bad off");
		static_assert((offsetof(CarAvatar,aiState)==0xDD8),"bad off");
		static_assert((offsetof(CarAvatar,carAudioFMOD)==0xE18),"bad off");
		static_assert((offsetof(CarAvatar,skidMarkBuffers)==0xE20),"bad off");
		static_assert((offsetof(CarAvatar,physicsInfo)==0xE40),"bad off");
		static_assert((offsetof(CarAvatar,raceEngineer)==0xF80),"bad off");
		static_assert((offsetof(CarAvatar,modelBoundaries)==0xF88),"bad off");
		static_assert((offsetof(CarAvatar,wingsStatus)==0xFA0),"bad off");
		static_assert((offsetof(CarAvatar,onBoardExposure)==0xFB8),"bad off");
		static_assert((offsetof(CarAvatar,outBoardExposure)==0xFBC),"bad off");
		static_assert((offsetof(CarAvatar,dashBoardExposure)==0xFC0),"bad off");
		static_assert((offsetof(CarAvatar,cameras)==0xFC8),"bad off");
		static_assert((offsetof(CarAvatar,isDriverHR)==0xFE0),"bad off");
		static_assert((offsetof(CarAvatar,driverModel_HR)==0xFE8),"bad off");
		static_assert((offsetof(CarAvatar,driverModel_LR)==0xFF0),"bad off");
		static_assert((offsetof(CarAvatar,suspensionAvatar)==0xFF8),"bad off");
		static_assert((offsetof(CarAvatar,isBlackFlagged)==0x1000),"bad off");
		static_assert((offsetof(CarAvatar,netCarStateProvider)==0x1008),"bad off");
		static_assert((offsetof(CarAvatar,lodManager)==0x1010),"bad off");
		static_assert((offsetof(CarAvatar,constrainedObjectManager)==0x1018),"bad off");
		static_assert((offsetof(CarAvatar,chTyreCompound)==0x1020),"bad off");
		static_assert((offsetof(CarAvatar,fuelInExhaust)==0x1048),"bad off");
		static_assert((offsetof(CarAvatar,graphicsOffset)==0x104C),"bad off");
		static_assert((offsetof(CarAvatar,graphicsPitchRotation)==0x1058),"bad off");
		static_assert((offsetof(CarAvatar,userFFGain)==0x105C),"bad off");
		static_assert((offsetof(CarAvatar,graphicSteerLockDegrees)==0x1060),"bad off");
		static_assert((offsetof(CarAvatar,backfireParams)==0x1068),"bad off");
		static_assert((offsetof(CarAvatar,meshCollider)==0x1070),"bad off");
		static_assert((offsetof(CarAvatar,pitPosition)==0x1078),"bad off");
		static_assert((offsetof(CarAvatar,driverInfo)==0x10B8),"bad off");
		static_assert((offsetof(CarAvatar,currentSkin)==0x1138),"bad off");
		static_assert((offsetof(CarAvatar,guid)==0x1158),"bad off");
		static_assert((offsetof(CarAvatar,modelLink)==0x1160),"bad off");
		static_assert((offsetof(CarAvatar,physics)==0x1168),"bad off");
		static_assert((offsetof(CarAvatar,lastSkidPosition)==0x1170),"bad off");
		static_assert((offsetof(CarAvatar,eventTriggers)==0x11A0),"bad off");
		static_assert((offsetof(CarAvatar,physicsStateProvider)==0x11B8),"bad off");
		static_assert((offsetof(CarAvatar,lockVirtualSteer)==0x11C0),"bad off");
		static_assert((offsetof(CarAvatar,hideArmsInCockpit)==0x11C1),"bad off");
		static_assert((offsetof(CarAvatar,hideSteer)==0x11C2),"bad off");
		static_assert((offsetof(CarAvatar,raceInfo)==0x11C8),"bad off");
		static_assert((offsetof(CarAvatar,physicsCarStateProvider)==0x11F0),"bad off");
		static_assert((offsetof(CarAvatar,lapQueue)==0x11F8),"bad off");
		static_assert((offsetof(CarAvatar,splitQueue)==0x1220),"bad off");
		static_assert((offsetof(CarAvatar,nonReplayPhysicsStateProvider)==0x1248),"bad off");
		static_assert((offsetof(CarAvatar,beltOffNode)==0x1250),"bad off");
		static_assert((offsetof(CarAvatar,beltOnNode)==0x1258),"bad off");
		static_assert((offsetof(CarAvatar,currentTyreCompoundIndex)==0x1260),"bad off");
		static_assert((offsetof(CarAvatar,carAnimations)==0x1270),"bad off");
		static_assert((offsetof(CarAvatar,filteredSpeed)==0x1278),"bad off");
		static_assert((offsetof(CarAvatar,currentEngineBrakeSetting)==0x127C),"bad off");
		static_assert((offsetof(CarAvatar,isHeatChargingBatteries)==0x1280),"bad off");
		static_assert((offsetof(CarAvatar,currentERSPowerIndex)==0x1284),"bad off");
		static_assert((offsetof(CarAvatar,currentERSRecovery)==0x1288),"bad off");
		static_assert((offsetof(CarAvatar,lastERSBatteryCharge)==0x128C),"bad off");
		static_assert((offsetof(CarAvatar,currentERSNormalizedRecharge)==0x1290),"bad off");
		static_assert((offsetof(CarAvatar,showDownShiftProtetion)==0x1294),"bad off");
		static_assert((offsetof(CarAvatar,mandatoryPitstopDone)==0x1295),"bad off");
		static_assert((offsetof(CarAvatar,lastPitstopTime)==0x1298),"bad off");
		static_assert((offsetof(CarAvatar,inPitlane)==0x129C),"bad off");
		static_assert((offsetof(CarAvatar,wasInPitlane)==0x129D),"bad off");
		static_assert((offsetof(CarAvatar,pitLaneEntryTime)==0x12A0),"bad off");
		static_assert((offsetof(CarAvatar,pitLaneExitTime)==0x12A4),"bad off");
	};
};

//UDT: class Car @len=16032 @vfcount=1
	//_VTable: 
	//_Func: public void Car(Car &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void Car(PhysicsEngine * iengine, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & iunixName, std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & config); @loc=static @len=6315 @rva=2539264
	//_Func: public void ~Car(); @intro @virtual vtpo=0 vfid=0 @loc=static @len=1894 @rva=2547920
	//_Data: this+0x8, Member, Type: float, finalSteerAngleSignal
	//_Data: this+0xC, Member, Type: float, powerClassIndex
	//_Data: this+0x10, Member, Type: class Event<OnStepCompleteEvent>, evOnStepComplete
	//_Data: this+0x28, Member, Type: class Event<OnControlsProviderChanged>, evOnControlsProviderChanged
	//_Data: this+0x40, Member, Type: class Event<OnLapCompletedEvent>, evOnLapCompleted
	//_Data: this+0x58, Member, Type: class Event<OnSectorSplitEvent>, evOnSectorSplit
	//_Data: this+0x70, Member, Type: class Event<vec3f>, evOnForcedPositionCompleted
	//_Data: this+0x88, Member, Type: class Event<std::pair<int,int> >, evOnTyreCompoundChanged
	//_Data: this+0xA0, Member, Type: class Event<OnCollisionEvent>, evOnCollisionEvent
	//_Data: this+0xB8, Member, Type: class Event<double>, evOnJumpStartEvent
	//_Data: this+0xD0, Member, Type: class Event<std::pair<int,int> >, evOnPush2Pass
	//_Data: this+0xE8, Member, Type: float, carHalfWidth
	//_Data: this+0xEC, Member, Type: float, userFFGain
	//_Data: this+0xF0, Member, Type: bool, isRetired
	//_Data: this+0xF8, Member, Type: class std::vector<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >,std::allocator<std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > > >, tyreCompounds
	//_Data: this+0x110, Member, Type: void *, tag
	//_Data: this+0x118, Member, Type: class IRigidBody *, body
	//_Data: this+0x120, Member, Type: class IRigidBody *, fuelTankBody
	//_Data: this+0x128, Member, Type: class IRigidBody *, rigidAxle
	//_Data: this+0x130, Member, Type: class IJoint *, fuelTankJoint
	//_Data: this+0x138, Member, Type: class PhysicsEngine *, ksPhysics
	//_Data: this+0x140, Member, Type: class CarControls, controls
	//_Data: this+0x174, Member, Type: float, steerLock
	//_Data: this+0x178, Member, Type: float, steerRatio
	//_Data: this+0x17C, Member, Type: float, steerLinearRatio
	//_Data: this+0x180, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, unixName
	//_Data: this+0x1A0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, configName
	//_Data: this+0x1C0, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, carDataPath
	//_Data: this+0x1E0, Member, Type: float, mass
	//_Data: this+0x1E4, Member, Type: float, ffMult
	//_Data: this+0x1E8, Member, Type: class vec3f, accG
	//_Data: this+0x1F8, Member, Type: struct TimeTransponder, transponder
	//_Data: this+0x290, Member, Type: struct CarColliderManager, colliderManager
	//_Data: this+0x310, Member, Type: struct Drivetrain, drivetrain
	//_Data: this+0x958, Member, Type: struct ABS, abs
	//_Data: this+0xA00, Member, Type: struct TractionControl, tractionControl
	//_Data: this+0xAA8, Member, Type: struct SpeedLimiter, speedLimiter
	//_Data: this+0xAB8, Member, Type: struct AeroMap, aeroMap
	//_Data: this+0xB28, Member, Type: struct Tyre[0x4], tyres
	//_Data: this+0x2C88, Member, Type: class std::vector<ISuspension *,std::allocator<ISuspension *> >, suspensions
	//_Data: this+0x2CA0, Member, Type: struct BrakeSystem, brakeSystem
	//_Data: this+0x3090, Member, Type: struct Autoclutch, autoClutch
	//_Data: this+0x3238, Member, Type: unsigned int, physicsGUID
	//_Data: this+0x3240, Member, Type: struct Telemetry, telemetry
	//_Data: this+0x3340, Member, Type: struct AutoBlip, autoBlip
	//_Data: this+0x33E8, Member, Type: struct AutoShifter, autoShift
	//_Data: this+0x3410, Member, Type: struct GearChanger, gearChanger
	//_Data: this+0x3428, Member, Type: struct EDL, edl
	//_Data: this+0x3460, Member, Type: struct AntirollBar[0x2], antirollBars
	//_Data: this+0x34F0, Member, Type: struct StabilityControl, stabilityControl
	//_Data: this+0x3508, Member, Type: double, lastCollisionTime
	//_Data: this+0x3510, Member, Type: struct DriftModeComponent, driftMode
	//_Data: this+0x3560, Member, Type: struct PerformanceMeter, performanceMeter
	//_Data: this+0x35D0, Member, Type: struct SetupManager, setupManager
	//_Data: this+0x3620, Member, Type: class std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> >, screenName
	//_Data: this+0x3640, Member, Type: struct DRS, drs
	//_Data: this+0x3670, Member, Type: class Kers, kers
	//_Data: this+0x3768, Member, Type: class ERS, ers
	//_Data: this+0x3A18, Member, Type: struct LapInvalidator, lapInvalidator
	//_Data: this+0x3A38, Member, Type: struct PenaltyManager, penaltyManager
	//_Data: this+0x3A80, Member, Type: bool, isGearboxLocked
	//_Data: this+0x3A81, Member, Type: bool, isGentleStopping
	//_Data: this+0x3A88, Member, Type: double, penaltyPerfTarget
	//_Data: this+0x3A90, Member, Type: class mat44f, pitPosition
	//_Data: this+0x3AD0, Member, Type: struct SplineLocatorData, splineLocatorData
	//_Data: this+0x3AF8, Member, Type: struct FuelLapEvaluator, fuelLapEvaluator
	//_Data: this+0x3B40, Member, Type: struct HeaveSpring[0x2], heaveSprings
	//_Data: this+0x3BF0, Member, Type: struct SteeringSystem, steeringSystem
	//_Data: this+0x3C30, Member, Type: double, lastCollisionWithCarTime
	//_Data: this+0x3C38, Member, Type: enum SuspensionType, suspensionTypeF
	//_Data: this+0x3C3C, Member, Type: enum SuspensionType, suspensionTypeR
	//_Data: this+0x3C40, Member, Type: float, axleTorqueReaction
	//_Data: this+0x3C44, Member, Type: float, lastGyroFF
	//_Data: this+0x3C48, Member, Type: float, lastFF
	//_Data: this+0x3C4C, Member, Type: float, lastPureMZFF
	//_Data: this+0x3C50, Member, Type: class ThermalObject, water
	//_Data: this+0x3C70, Member, Type: float, steerAssist
	//_Data: this+0x3C74, Member, Type: bool, isRequestingPitStop
	//_Data: this+0x3C78, Member, Type: int, aiLapsToComplete
	//_Data: this+0x3C7C, Member, Type: bool, lightsOn
	//_Data: this+0x3C80, Member, Type: struct CarCollisionBounds, bounds
	//_Data: this+0x3CA8, Member, Type: struct SlipStream, slipStream
	//_Data: this+0x3D10, Member, Type: enum TorqueModeEX, torqueModeEx
	//_Func: public void stepPreCacheValues(float dt); @loc=static @len=290 @rva=2582720
	//_Func: public void setBlackFlag(bool isflagged, PenaltyDescription type); @loc=static @len=120 @rva=2578928
	//_Func: public void setSimplePhysics(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public int setFuelForLaps(int laps, float mult); @loc=static @len=321 @rva=2579296
	//_Func: public float getFuelPerLap(); @loc=static @len=43 @rva=2558928
	//_Func: public float getBackDistanceFromCar(Car * car); @loc=static @len=71 @rva=2556768
	//_Func: public float getForwardDistanceFromCar(Car *  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public bool isInPitLane(); @loc=static @len=74 @rva=2573536
	//_Func: public vec3f getRidePickupPoint(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public double getFuel(); @loc=optimized @len=0 @rva=0
	//_Func: public void forceFuel(double  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void setHeadlights(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public double getMaxFuel(); @loc=optimized @len=0 @rva=0
	//_Func: public bool isInPits(); @loc=static @len=147 @rva=2573616
	//_Func: public void setDamageLevel(float v, int index); @loc=static @len=13 @rva=2579280
	//_Func: public void setDamageLevel(float v); @loc=static @len=41 @rva=2579232
	//_Func: public void resetSuspensionDamageLevel(); @loc=static @len=104 @rva=2578800
	//_Func: public void setEngineLifeleft(float  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void onCollisionCallBack(void * userData0, void * shape0, void * userData1, void * shape1, vec3f normal, vec3f pos, float depth); @loc=static @len=1499 @rva=2573904
	//_Func: public void reset(); @loc=static @len=125 @rva=2578656
	//_Func: public void getTyreThermalState(int index, TyreThermalState * state); @loc=static @len=389 @rva=2566064
	//_Func: public bool isSleeping(); @loc=static @len=16 @rva=2573792
	//_Func: public void step(float dt); @loc=static @len=1836 @rva=2579872
	//_Func: public void postStep(float dt); @loc=static @len=605 @rva=2577456
	//_Func: public void getPhysicsState(CarPhysicsState & state); @loc=static @len=4323 @rva=2559344
	//_Func: public void getAIState(AIState * state); @loc=static @len=275 @rva=2556480
	//_Func: public void setControllerProvider(ICarControlsProvider * cp); @loc=static @len=169 @rva=2579056
	//_Func: public void setExternalControl(bool  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public Speed getSpeed(); @loc=static @len=27 @rva=2564448
	//_Func: public float getCGHeight(); @loc=static @len=102 @rva=2556848
	//_Func: public vec3f getVelocity(); @loc=static @len=31 @rva=2566464
	//_Func: public vec3f getLocalVelocity(); @loc=static @len=34 @rva=2559264
	//_Func: public float getSteerFF(); @loc=static @len=865 @rva=2564480
	//_Func: public vec3f getLocalAngularVelocity(); @loc=static @len=34 @rva=2559216
	//_Func: public ICarControlsProvider * getControlsProvider(); @loc=static @len=8 @rva=2557520
	//_Func: public float getTotalMass(bool withFuel); @loc=static @len=303 @rva=2565488
	//_Func: public mat44f getTyreMatrix(int index); @loc=static @len=258 @rva=2565792
	//_Func: public mat44f getSuspensionMatrix(unsigned int index); @loc=static @len=86 @rva=2565360
	//_Func: public plane4f getGroundPlane(); @loc=optimized @len=0 @rva=0
	//_Func: public void getWingState(std::vector<WingState,std::allocator<WingState> > & ws); @loc=static @len=83 @rva=2566688
	//_Func: public float getDamageLevel(unsigned int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void forcePosition(vec3f & pos, bool invalidateTransponder); @loc=static @len=550 @rva=2555408
	//_Func: public void forceRotation(vec3f & ihed); @loc=static @len=499 @rva=2555968
	//_Func: public float getRequestedFuel(); @loc=optimized @len=0 @rva=0
	//_Func: public void setRequestedFuel(float value, bool changeValue); @loc=static @len=93 @rva=2579632
	//_Func: public void initColliderMesh(Mesh * mesh, mat44f & bodyMatrix); @loc=static @len=1046 @rva=2571040
	//_Func: public mat44f getColliderBodyMatrix(); @loc=optimized @len=0 @rva=0
	//_Func: public float getFuelConsumptionK(); @loc=optimized @len=0 @rva=0
	//_Func: public float computeRideHeight(int zone, plane4f & groundPlane); @loc=static @len=370 @rva=2555024
	//_Func: public void setSlipStreamEffects(float receive, float generationSpeedFactor); @loc=static @len=88 @rva=2579776
	//_Func: public float getFrontWheelAngle(); @loc=static @len=571 @rva=2558352
	//_Func: public void addPenalty(double ptime); @loc=static @len=174 @rva=2553504
	//_Func: public void clearPenalty(); @loc=static @len=63 @rva=2554960
	//_Func: public bool isMinSpeedPenaltyClearDisabled(); @loc=static @len=8 @rva=2573776
	//_Func: public double getPenaltyTime(); @loc=static @len=22 @rva=2559312
	//_Func: public bool isBlackFlagged(); @loc=optimized @len=0 @rva=0
	//_Func: public void lockControlsFor(double  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void lockControlsUntil(double time, double start); @loc=static @len=69 @rva=2573824
	//_Func: public void lockControls(bool lock); @loc=static @len=7 @rva=2573808
	//_Func: public bool getControlsLocked(); @loc=optimized @len=0 @rva=0
	//_Func: public float getBallastKG(); @loc=optimized @len=0 @rva=0
	//_Func: public void setBallastKG(float kg); @loc=static @len=9 @rva=2578912
	//_Func: public float getRestrictor(); @loc=static @len=23 @rva=2564416
	//_Func: public void setRestrictor(float value); @loc=static @len=47 @rva=2579728
	//_Func: public void activateP2P(); @loc=optimized @len=0 @rva=0
	//_Func: public void setP2Pactivations(int  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > getConfigPath(std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > & filename); @loc=static @len=552 @rva=2556960
	//_Func: public PitStopTime getPitstopTime(float fuel_to_add, bool changeTyres, bool repairBody, bool repairEngine, bool repairSus, bool useRandomizer); @loc=static @len=733 @rva=2563680
	//_Func: public float getWheelSterAngleDEG(int index); @loc=static @len=180 @rva=2566496
	//_Func: public float getFinalFF(); @loc=static @len=48 @rva=2558304
	//_Func: public void setGridPosition(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public PitStopTimings getPitStopTiming(); @loc=optimized @len=0 @rva=0
	//_Func: public void getEngagement(float engagement_length, float * left, float * right); @loc=static @len=760 @rva=2557536
	//_Func: public void resetSplineLocator(); @loc=static @len=12 @rva=2578784
	//_Func: public float getTotalKM(); @loc=static @len=21 @rva=2565456
	//_Func: public vec3f getGroundWindVector(); @loc=static @len=236 @rva=2558976
	//_Data: this+0x3D14, Member, Type: bool, isControlsLocked
	//_Data: this+0x3D18, Member, Type: double, lockControlsTime
	//_Data: this+0x3D20, Member, Type: bool, blackFlagged
	//_Data: this+0x3D28, Member, Type: double, penaltyTimeAccumulator
	//_Data: this+0x3D30, Member, Type: double, penaltyTime
	//_Data: this+0x3D38, Member, Type: float, expectedFuelPerLap
	//_Data: this+0x3D3C, Member, Type: class mat44f, meshColliderBodyMatrix
	//_Data: this+0x3D80, Member, Type: class ICarControlsProvider *, controlsProvider
	//_Data: this+0x3D88, Member, Type: class vec3f, lastVelocity
	//_Data: this+0x3D94, Member, Type: int, sleepingFrames
	//_Data: this+0x3D98, Member, Type: float, mzCurrent
	//_Data: this+0x3D9C, Member, Type: class vec3f, bodyInertia
	//_Data: this+0x3DA8, Member, Type: class vec3f, explicitInertia
	//_Data: this+0x3DB8, Member, Type: double, fuel
	//_Data: this+0x3DC0, Member, Type: double, fuelConsumptionK
	//_Data: this+0x3DC8, Member, Type: double, maxFuel
	//_Data: this+0x3DD0, Member, Type: float, requestedFuel
	//_Data: this+0x3DD8, Member, Type: double, lastBodyMassUpdateTime
	//_Data: this+0x3DE0, Member, Type: float[0x5], damageZoneLevel
	//_Data: this+0x3DF4, Member, Type: class vec3f[0x2], ridePickupPoint
	//_Data: this+0x3E0C, Member, Type: class vec3f, fuelTankPos
	//_Data: this+0x3E18, Member, Type: float, vibrationPhase
	//_Data: this+0x3E1C, Member, Type: float, slipVibrationPhase
	//_Data: this+0x3E20, Member, Type: float, flatSpotPhase
	//_Data: this+0x3E24, Member, Type: float, slipStreamEffectGain
	//_Data: this+0x3E28, Member, Type: int, framesToSleep
	//_Data: this+0x3E2C, Member, Type: bool, disableMinSpeedPenaltyClear
	//_Data: this+0x3E30, Member, Type: float, ballastKG
	//_Data: this+0x3E34, Member, Type: float, lastSteerPosition
	//_Data: this+0x3E38, Member, Type: struct SplineLocator, splineLocator
	//_Data: this+0x3E68, Member, Type: bool, isCollisionOffForPits
	//_Data: this+0x3E6C, Member, Type: struct PitStopTimings, pitTimings
	//_Data: this+0x3E80, Member, Type: class vec3f, gridPosition
	//_Data: this+0x3E8C, Member, Type: bool, hasGridPosition
	//_Data: this+0x3E8D, Member, Type: bool, lastLigthSwitchState
	//_Data: this+0x3E90, Member, Type: struct PhysicsValueCache, valueCache
	//_Data: this+0x3E94, Member, Type: float, fuelKG
	//_Data: this+0x3E98, Member, Type: bool, externalControl
	//_Func: protected void pollControls(float dt); @loc=static @len=1457 @rva=2575984
	//_Func: protected void initControls(); @loc=optimized @len=0 @rva=0
	//_Func: protected void initCarData(); @loc=static @len=3727 @rva=2566960
	//_Func: protected void initCarDataPath(); @loc=static @len=348 @rva=2570688
	//_Func: protected void buildARBS(); @loc=static @len=1047 @rva=2553680
	//_Func: protected void initAeroMap(); @loc=static @len=171 @rva=2566784
	//_Func: protected float calcBodyMass(); @loc=static @len=102 @rva=2554736
	//_Func: protected void updateBodyMass(); @loc=static @len=381 @rva=2583664
	//_Func: protected void updateAirPressure(); @loc=static @len=396 @rva=2583264
	//_Func: protected void initHeaveSprings(); @loc=static @len=217 @rva=2572096
	//_Func: protected void onTyresStepCompleted(); @loc=static @len=414 @rva=2575568
	//_Func: protected void onNewSession(SessionInfo & si); @loc=static @len=155 @rva=2575408
	//_Func: protected void stepThermalObjects(float dt); @loc=static @len=225 @rva=2583024
	//_Func: protected void stepComponents(float dt); @loc=static @len=675 @rva=2581712
	//_Func: protected void updateColliderStatus(float dt); @loc=static @len=533 @rva=2584048
	//_Func: protected void initPitstopTimings(); @loc=static @len=1203 @rva=2572320
	//_Func: protected void stepJumpStart(float dt); @loc=static @len=311 @rva=2582400
	//_Func: public Car & operator=(Car &  _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void * __vecDelDtor(unsigned int  _arg0); @intro @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
//UDT;

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
	inline void _guard_obj() {
		static_assert((sizeof(Car)==16032),"bad size");
		static_assert((offsetof(Car,finalSteerAngleSignal)==0x8),"bad off");
		static_assert((offsetof(Car,powerClassIndex)==0xC),"bad off");
		static_assert((offsetof(Car,evOnStepComplete)==0x10),"bad off");
		static_assert((offsetof(Car,evOnControlsProviderChanged)==0x28),"bad off");
		static_assert((offsetof(Car,evOnLapCompleted)==0x40),"bad off");
		static_assert((offsetof(Car,evOnSectorSplit)==0x58),"bad off");
		static_assert((offsetof(Car,evOnForcedPositionCompleted)==0x70),"bad off");
		static_assert((offsetof(Car,evOnTyreCompoundChanged)==0x88),"bad off");
		static_assert((offsetof(Car,evOnCollisionEvent)==0xA0),"bad off");
		static_assert((offsetof(Car,evOnJumpStartEvent)==0xB8),"bad off");
		static_assert((offsetof(Car,evOnPush2Pass)==0xD0),"bad off");
		static_assert((offsetof(Car,carHalfWidth)==0xE8),"bad off");
		static_assert((offsetof(Car,userFFGain)==0xEC),"bad off");
		static_assert((offsetof(Car,isRetired)==0xF0),"bad off");
		static_assert((offsetof(Car,tyreCompounds)==0xF8),"bad off");
		static_assert((offsetof(Car,tag)==0x110),"bad off");
		static_assert((offsetof(Car,body)==0x118),"bad off");
		static_assert((offsetof(Car,fuelTankBody)==0x120),"bad off");
		static_assert((offsetof(Car,rigidAxle)==0x128),"bad off");
		static_assert((offsetof(Car,fuelTankJoint)==0x130),"bad off");
		static_assert((offsetof(Car,ksPhysics)==0x138),"bad off");
		static_assert((offsetof(Car,controls)==0x140),"bad off");
		static_assert((offsetof(Car,steerLock)==0x174),"bad off");
		static_assert((offsetof(Car,steerRatio)==0x178),"bad off");
		static_assert((offsetof(Car,steerLinearRatio)==0x17C),"bad off");
		static_assert((offsetof(Car,unixName)==0x180),"bad off");
		static_assert((offsetof(Car,configName)==0x1A0),"bad off");
		static_assert((offsetof(Car,carDataPath)==0x1C0),"bad off");
		static_assert((offsetof(Car,mass)==0x1E0),"bad off");
		static_assert((offsetof(Car,ffMult)==0x1E4),"bad off");
		static_assert((offsetof(Car,accG)==0x1E8),"bad off");
		static_assert((offsetof(Car,transponder)==0x1F8),"bad off");
		static_assert((offsetof(Car,colliderManager)==0x290),"bad off");
		static_assert((offsetof(Car,drivetrain)==0x310),"bad off");
		static_assert((offsetof(Car,abs)==0x958),"bad off");
		static_assert((offsetof(Car,tractionControl)==0xA00),"bad off");
		static_assert((offsetof(Car,speedLimiter)==0xAA8),"bad off");
		static_assert((offsetof(Car,aeroMap)==0xAB8),"bad off");
		static_assert((offsetof(Car,tyres)==0xB28),"bad off");
		static_assert((offsetof(Car,suspensions)==0x2C88),"bad off");
		static_assert((offsetof(Car,brakeSystem)==0x2CA0),"bad off");
		static_assert((offsetof(Car,autoClutch)==0x3090),"bad off");
		static_assert((offsetof(Car,physicsGUID)==0x3238),"bad off");
		static_assert((offsetof(Car,telemetry)==0x3240),"bad off");
		static_assert((offsetof(Car,autoBlip)==0x3340),"bad off");
		static_assert((offsetof(Car,autoShift)==0x33E8),"bad off");
		static_assert((offsetof(Car,gearChanger)==0x3410),"bad off");
		static_assert((offsetof(Car,edl)==0x3428),"bad off");
		static_assert((offsetof(Car,antirollBars)==0x3460),"bad off");
		static_assert((offsetof(Car,stabilityControl)==0x34F0),"bad off");
		static_assert((offsetof(Car,lastCollisionTime)==0x3508),"bad off");
		static_assert((offsetof(Car,driftMode)==0x3510),"bad off");
		static_assert((offsetof(Car,performanceMeter)==0x3560),"bad off");
		static_assert((offsetof(Car,setupManager)==0x35D0),"bad off");
		static_assert((offsetof(Car,screenName)==0x3620),"bad off");
		static_assert((offsetof(Car,drs)==0x3640),"bad off");
		static_assert((offsetof(Car,kers)==0x3670),"bad off");
		static_assert((offsetof(Car,ers)==0x3768),"bad off");
		static_assert((offsetof(Car,lapInvalidator)==0x3A18),"bad off");
		static_assert((offsetof(Car,penaltyManager)==0x3A38),"bad off");
		static_assert((offsetof(Car,isGearboxLocked)==0x3A80),"bad off");
		static_assert((offsetof(Car,isGentleStopping)==0x3A81),"bad off");
		static_assert((offsetof(Car,penaltyPerfTarget)==0x3A88),"bad off");
		static_assert((offsetof(Car,pitPosition)==0x3A90),"bad off");
		static_assert((offsetof(Car,splineLocatorData)==0x3AD0),"bad off");
		static_assert((offsetof(Car,fuelLapEvaluator)==0x3AF8),"bad off");
		static_assert((offsetof(Car,heaveSprings)==0x3B40),"bad off");
		static_assert((offsetof(Car,steeringSystem)==0x3BF0),"bad off");
		static_assert((offsetof(Car,lastCollisionWithCarTime)==0x3C30),"bad off");
		static_assert((offsetof(Car,suspensionTypeF)==0x3C38),"bad off");
		static_assert((offsetof(Car,suspensionTypeR)==0x3C3C),"bad off");
		static_assert((offsetof(Car,axleTorqueReaction)==0x3C40),"bad off");
		static_assert((offsetof(Car,lastGyroFF)==0x3C44),"bad off");
		static_assert((offsetof(Car,lastFF)==0x3C48),"bad off");
		static_assert((offsetof(Car,lastPureMZFF)==0x3C4C),"bad off");
		static_assert((offsetof(Car,water)==0x3C50),"bad off");
		static_assert((offsetof(Car,steerAssist)==0x3C70),"bad off");
		static_assert((offsetof(Car,isRequestingPitStop)==0x3C74),"bad off");
		static_assert((offsetof(Car,aiLapsToComplete)==0x3C78),"bad off");
		static_assert((offsetof(Car,lightsOn)==0x3C7C),"bad off");
		static_assert((offsetof(Car,bounds)==0x3C80),"bad off");
		static_assert((offsetof(Car,slipStream)==0x3CA8),"bad off");
		static_assert((offsetof(Car,torqueModeEx)==0x3D10),"bad off");
		static_assert((offsetof(Car,isControlsLocked)==0x3D14),"bad off");
		static_assert((offsetof(Car,lockControlsTime)==0x3D18),"bad off");
		static_assert((offsetof(Car,blackFlagged)==0x3D20),"bad off");
		static_assert((offsetof(Car,penaltyTimeAccumulator)==0x3D28),"bad off");
		static_assert((offsetof(Car,penaltyTime)==0x3D30),"bad off");
		static_assert((offsetof(Car,expectedFuelPerLap)==0x3D38),"bad off");
		static_assert((offsetof(Car,meshColliderBodyMatrix)==0x3D3C),"bad off");
		static_assert((offsetof(Car,controlsProvider)==0x3D80),"bad off");
		static_assert((offsetof(Car,lastVelocity)==0x3D88),"bad off");
		static_assert((offsetof(Car,sleepingFrames)==0x3D94),"bad off");
		static_assert((offsetof(Car,mzCurrent)==0x3D98),"bad off");
		static_assert((offsetof(Car,bodyInertia)==0x3D9C),"bad off");
		static_assert((offsetof(Car,explicitInertia)==0x3DA8),"bad off");
		static_assert((offsetof(Car,fuel)==0x3DB8),"bad off");
		static_assert((offsetof(Car,fuelConsumptionK)==0x3DC0),"bad off");
		static_assert((offsetof(Car,maxFuel)==0x3DC8),"bad off");
		static_assert((offsetof(Car,requestedFuel)==0x3DD0),"bad off");
		static_assert((offsetof(Car,lastBodyMassUpdateTime)==0x3DD8),"bad off");
		static_assert((offsetof(Car,damageZoneLevel)==0x3DE0),"bad off");
		static_assert((offsetof(Car,ridePickupPoint)==0x3DF4),"bad off");
		static_assert((offsetof(Car,fuelTankPos)==0x3E0C),"bad off");
		static_assert((offsetof(Car,vibrationPhase)==0x3E18),"bad off");
		static_assert((offsetof(Car,slipVibrationPhase)==0x3E1C),"bad off");
		static_assert((offsetof(Car,flatSpotPhase)==0x3E20),"bad off");
		static_assert((offsetof(Car,slipStreamEffectGain)==0x3E24),"bad off");
		static_assert((offsetof(Car,framesToSleep)==0x3E28),"bad off");
		static_assert((offsetof(Car,disableMinSpeedPenaltyClear)==0x3E2C),"bad off");
		static_assert((offsetof(Car,ballastKG)==0x3E30),"bad off");
		static_assert((offsetof(Car,lastSteerPosition)==0x3E34),"bad off");
		static_assert((offsetof(Car,splineLocator)==0x3E38),"bad off");
		static_assert((offsetof(Car,isCollisionOffForPits)==0x3E68),"bad off");
		static_assert((offsetof(Car,pitTimings)==0x3E6C),"bad off");
		static_assert((offsetof(Car,gridPosition)==0x3E80),"bad off");
		static_assert((offsetof(Car,hasGridPosition)==0x3E8C),"bad off");
		static_assert((offsetof(Car,lastLigthSwitchState)==0x3E8D),"bad off");
		static_assert((offsetof(Car,valueCache)==0x3E90),"bad off");
		static_assert((offsetof(Car,fuelKG)==0x3E94),"bad off");
		static_assert((offsetof(Car,externalControl)==0x3E98),"bad off");
	};
};

