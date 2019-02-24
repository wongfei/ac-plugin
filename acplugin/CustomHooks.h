#pragma once

#define Engine_step_rva 2654432
void Engine_step(Engine* pThis, SACEngineInput* pInput, float dt);

#define Engine_stepTurbos_rva 2656512
void Engine_stepTurbos(Engine* pThis);

#define Turbo_step_rva 2811840
void Turbo_step(Turbo* pThis, float gas, float rpms, float dt);
