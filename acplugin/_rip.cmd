
set "dumper=..\tools\pdb-ripper\_bin\x64\Release\Dia2Dump.exe"
set "pdb=..\stuff\acs.pdb"

set "ac_core=ACPlugin; ACPluginContext; ACClient; ACCarState; Game; Sim; PhysicsEngine; IPhysicsCore; IRigidBody; Car; CarAvatar; NetCarStateProvider; RenderWindow; GraphicsManager; GLRenderer"
set "ac_gui=ksgui::GUI; ksgui::GameScreen; ksgui::Form; ksgui::ActiveButton; ksgui::CheckBox; ksgui::ConnectedLabel; ksgui::CustomSpinner; ksgui::Graph; ksgui::Label; ksgui::ListBox; ksgui::ListBoxRow; ksgui::TextBox; ksgui::MovingBar; ksgui::PopOver; ksgui::ProgressBar; ksgui::ScrollBar; ksgui::Slider; ksgui::Spinner; ksgui::Taskbar; ksgui::TaskBarIcon; ksgui::TextBox; ksgui::TextInput;"
set "ac_excl=-in_addr; -IEventTrigger; -OnValueChanged; -IVertexBuffer; -VertexBuffer"

"%dumper%" -rip -cpp -m -g -d -rd -names "%ac_core%;%ac_gui%;%ac_excl%" "%pdb%" > ac_gen.h
