#pragma once

#include "utils/log.h"
#include "utils/common.h"

#define HOOK_FUNC_RVA(func) hook_create(UT_WSTRING(func), _drva(RVA_##func), &::func)
#define HOOK_FUNC_RVA_2(func) hook_create(UT_WSTRING(func), _drva(RVA_##func), &::func, &_orig_##func)

// Core

#define RVA_PhysicsDriveThread_run 1192272
void PhysicsDriveThread_run(PhysicsDriveThread* pThis);

// Car

#define RVA_Car_step 2579872
void Car_step(Car* pThis, float dt);

#define RVA_Car_stepComponents 2581712
void Car_stepComponents(Car* pThis, float dt);

#define RVA_Car_pollControls 2575984
void Car_pollControls(Car* pThis, float dt);

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

#define RVA_Damper_getForce 2830976
float Damper_getForce(Damper* pThis, float fSpeed);

#define RVA_Suspension_step 2896784
void Suspension_step(Suspension* pThis, float dt);

#define RVA_SuspensionAxle_step 2918256
void SuspensionAxle_step(SuspensionAxle* pThis, float dt);

#define RVA_SuspensionML_step 2927360
void SuspensionML_step(SuspensionML* pThis, float dt);

#define RVA_SuspensionStrut_step 2909696
void SuspensionStrut_step(SuspensionStrut* pThis, float dt);

// Tyre

#define RVA_Tyre_step 2635776
void Tyre_step(Tyre* pThis, float dt);

#define RVA_Tyre_addGroundContact 2611584
void Tyre_addGroundContact(Tyre* pThis, vec3f& pos, vec3f& normal);

#define RVA_Tyre_addTyreForcesV10 2616672
void Tyre_addTyreForcesV10(Tyre* pThis, vec3f& pos, vec3f& normal, SurfaceDef* pSurface, float dt);

#define RVA_SCTM_solve 4504608
TyreModelOutput SCTM_solve(SCTM* pThis, TyreModelInput& tmi);

// Brake

#define RVA_BrakeSystem_step 2680384
void BrakeSystem_step(BrakeSystem* pThis, float dt);

#define RVA_BrakeSystem_stepTemps 2681120
void BrakeSystem_stepTemps(BrakeSystem* pThis, float dt);
