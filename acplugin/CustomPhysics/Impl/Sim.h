#pragma once

static void* _orig_Sim_addCar = nullptr;

BEGIN_HOOK_OBJ(Sim)

	#define RVA_Sim_addCar 1669280

	static void _hook()
	{
		HOOK_METHOD_RVA_ORIG(Sim, addCar);
	}

	CarAvatar* _addCar(const std::wstring& model, const std::wstring& config, const std::wstring& skin);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

CarAvatar* _Sim::_addCar(const std::wstring& model, const std::wstring& config, const std::wstring& skin)
{
	auto orig = ORIG_METHOD(Sim, addCar);
	return THIS_CALL(orig)(model, config, skin);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
