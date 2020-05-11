#pragma once

static Sim* _Sim_instance = nullptr;
static void* _orig_Sim_ctor = nullptr;
static void* _orig_Sim_addCar = nullptr;

BEGIN_HOOK_OBJ(Sim)

	#define RVA_Sim_ctor 1646704
	#define RVA_Sim_addCar 1669280

	static void _hook()
	{
		HOOK_METHOD_RVA_ORIG(Sim, ctor);
		HOOK_METHOD_RVA_ORIG(Sim, addCar);
	}

	Sim* _ctor(Game* pGame);
	CarAvatar* _addCar(const std::wstring& model, const std::wstring& config, const std::wstring& skin);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Sim* _Sim::_ctor(Game* pGame)
{ 
	_Sim_instance = this;
	auto orig = ORIG_METHOD(Sim, ctor);
	return THIS_CALL(orig)(pGame);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CarAvatar* _Sim::_addCar(const std::wstring& model, const std::wstring& config, const std::wstring& skin)
{
	auto orig = ORIG_METHOD(Sim, addCar);
	return THIS_CALL(orig)(model, config, skin);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
