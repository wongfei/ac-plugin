#pragma once

#include "plugin/plugin.h"
#include "utils/common.h"

#define HOOK_FUNC_RVA(func) hook_create(_drva(RVA_##func), &func, nullptr, UT_WSTRING(func))

// Engine

#define RVA_Engine_step 2654432
void Engine_step(Engine* pThis, SACEngineInput* pInput, float dt);

#define RVA_Engine_stepTurbos 2656512
void Engine_stepTurbos(Engine* pThis);

#define RVA_Turbo_step 2811840
void Turbo_step(Turbo* pThis, float gas, float rpms, float dt);

// Drivetrain

#define RVA_Drivetrain_step 2535728
void Drivetrain_step(Drivetrain* pThis, float dt);

#define RVA_Drivetrain_stepControllers 2535936
void Drivetrain_stepControllers(Drivetrain* pThis, float dt);

#define RVA_Drivetrain_step2WD 2528480
void Drivetrain_step2WD(Drivetrain* pThis, float dt);

// Suspension

#define RVA_Suspension_step 2896784
void Suspension_step(Suspension* pThis, float dt);

#define RVA_Damper_getForce 2830976
float Damper_getForce(Damper* pThis, float fSpeed);

// Tyre

#define RVA_Tyre_step 2635776
void Tyre_step(Tyre* pThis, float dt);
