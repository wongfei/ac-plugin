#pragma once

BEGIN_HOOK_OBJ(PhysicsEngine)
	
	#define RVA_PhysicsEngine_step 2508640
	#define RVA_PhysicsEngine_stepWind 2511744
	#define RVA_PhysicsEngine_onCollisionCallBack 2506784

	void _step(float dt, double currentTime, double gt);
	void _stepWind(float dt);
	void _onCollisionCallBack(void* userData0, void* shape0, void* userData1, void* shape1, const vec3f& normal, const vec3f& pos, float depth);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_step(float dt, double currentTime, double gt)
{
	this->gameTime = gt;
	this->physicsTime = currentTime;
	this->stepCounter++;

	for (auto& h : this->evOnPreStep.handlers)
	{
		if (h.second)
		{
			(h.second)(currentTime);
		}
	}

	this->track->step(dt);
	this->stepWind(dt);

	double t0 = ksGetQPTTime();

	for (auto* pCar : this->cars)
	{
		pCar->stepPreCacheValues(dt);
	}

	//if (this->pool) // TODO: implement multithreading
	//{
	//}
	//else
	{
		for (auto* pCar : this->cars)
		{
			pCar->step(dt);
		}
	}

	this->physicsCPUTimes.carStep = ksGetQPTTime() - t0;

	this->core->step(dt);

	this->physicsCPUTimes.coreCPUTimes = this->core->getCoreCPUTimes();

	for (auto& h : this->evOnStepCompleted.handlers)
	{
		if (h.second)
		{
			(h.second)(this->physicsTime);
		}
	}

	this->physicsCPUTimes.currentCPU = 0;

	//if (PhysicsEngine::isTestMode) // TODO: implement
	//{
	//}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_stepWind(float dt)
{
	if (this->wind.speed.value >= 0.0099999998f)
	{
		float fChange = (float)sin((this->physicsTime - this->sessionInfo.startTimeMS) * 0.0001);
		float fSpeed = this->wind.speed.value * ((fChange * 0.1f) + 1.0f);
		vec3f vWind = this->wind.vector.get_norm() * fSpeed;

		if (isfinite(vWind.x) && isfinite(vWind.y) && isfinite(vWind.z))
		{
			this->wind.vector = vWind;
		}
		else
		{
			SHOULD_NOT_REACH;
			log_printf(L"INF vWind");
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_onCollisionCallBack(
	void* userData0, void* shape0, 
	void* userData1, void* shape1, 
	const vec3f& normal, const vec3f& pos, float depth)
{
	bool bFlag0 = false;
	bool bFlag1 = false;

	for (auto* pCar : this->cars)
	{
		if (pCar->body == userData0)
			bFlag0 = true;

		if (pCar->body == userData1)
			bFlag1 = true;
	}

	if (!bFlag0 && bFlag1)
	{
		std::swap(userData0, userData1);
		std::swap(shape0, shape1);
	}

	for (auto* pCar : this->cars)
	{
		pCar->onCollisionCallBack(userData0, shape0, userData1, shape1, normal, pos, depth);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
