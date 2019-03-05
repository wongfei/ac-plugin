
set "dumper=..\tools\pdb-ripper\_bin\x64\Release\Dia2Dump.exe"
set "pdb=..\stuff\acs.pdb"

set "ac_core=ACPlugin; ACPluginContext; ACClient; ACCarState; Game; Sim; Car; Drivetrain; Engine; Suspension; CarAvatar; NetCarStateProvider; Track; TrackAvatar; PhysicsEngine; IPhysicsCore; IRigidBody; IJoint; IRayCaster; ICollisionObject; RayCastResult; CollisionMeshODE; RaceManager; RaceTimingServices; LapDB; ICarControlsProvider; SuspensionAxle; SuspensionML; SuspensionStrut"
set "ac_render=GraphicsManager; ShaderManager; GLRenderer; RenderWindow; RenderContext; RenderTarget; Shader; ShaderVariable; ShaderResource; MaterialResource; MaterialVar; MaterialOption; IndexBuffer"
set "ac_kgl=KGLTexture; KGLRenderTarget; KGLShader; KGLShaderVar; KGLShaderTexture; KGLShaderCBuffer; KGLShaderVarDesc; KGLShaderCBufferDesc; KGLShaderTextureDesc; KGLVertexBuffer; KGLIndexBuffer"
set "ac_gui=ksgui::GUI; ksgui::GameScreen; ksgui::Form; ksgui::ActiveButton; ksgui::CheckBox; ksgui::ConnectedLabel; ksgui::CustomSpinner; ksgui::Graph; ksgui::Label; ksgui::ListBox; ksgui::ListBoxRow; ksgui::TextBox; ksgui::MovingBar; ksgui::PopOver; ksgui::ProgressBar; ksgui::ScrollBar; ksgui::Slider; ksgui::Spinner; ksgui::Taskbar; ksgui::TaskBarIcon; ksgui::TextBox; ksgui::TextInput"
set "ac_excl=-in_addr; -sockaddr_in; -IEventTrigger; -OnValueChanged; -IVertexBuffer; -VertexBuffer"

set "ac_plugin=ACPlugin; ACPluginContext; ACClient; ACCarState"
set "ac_game=Game; RaceManager; RaceTimingServices; LapDB"
set "ac_sim=Sim; PhysicsEngine; IPhysicsCore; IRigidBody; IJoint; ICollisionObject; IRayCaster; RayCastResult; CollisionMeshODE; Track; TrackAvatar"
set "ac_car=CarAvatar; Car; Drivetrain; Engine; Suspension; SuspensionAxle; SuspensionML; SuspensionStrut; ICarControlsProvider; ICarPhysicsStateProvider; NetCarStateProvider"

set "ac_names=%ac_plugin%;%ac_game%;%ac_sim%;%ac_car%;%ac_render%;%ac_kgl%;%ac_gui%;%ac_excl%"
