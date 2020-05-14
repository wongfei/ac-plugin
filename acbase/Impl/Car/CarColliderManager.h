#pragma once

BEGIN_HOOK_OBJ(CarColliderManager)

	#define RVA_CarColliderManager_ctor 2545584
	#define RVA_CarColliderManager_init 2766672
	#define RVA_CarColliderManager_loadINI 2766752
	#define RVA_CarColliderManager_step 2768208

	static void _hook()
	{
		HOOK_METHOD_RVA(CarColliderManager, ctor);
		HOOK_METHOD_RVA(CarColliderManager, init);
		HOOK_METHOD_RVA(CarColliderManager, loadINI);
		HOOK_METHOD_RVA(CarColliderManager, step);
	}

	CarColliderManager* _ctor();
	void _init(Car* pCar);
	void _loadINI();
	void _step(float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

CarColliderManager* _CarColliderManager::_ctor()
{
	AC_CTOR_POD(CarColliderManager);
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_init(Car* pCar)
{
	this->isLive = false;
	this->carBody = pCar->body;
	this->car = pCar;
	this->carModel = pCar->unixName;
	this->boxes.clear();

	this->loadINI();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_loadINI()
{
	auto ini(new_udt_unique<INIReader>(this->car->carDataPath + L"colliders.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	for (int id = 0; ; id++)
	{
		auto sectionName = strf(L"COLLIDER_%d", id);
		if (!ini->hasSection(sectionName))
			break;

		vec3f vCentre = ini->getFloat3(sectionName, L"CENTRE");
		vec3f vSize = ini->getFloat3(sectionName, L"SIZE");

		CarCollisionBox box;
		box.centre = vCentre;
		box.size = vSize;

		this->boxes.push_back(box);

		//(const vec3f& pos, const vec3f& size, unsigned int category, unsigned long mask, unsigned int spaceId)
		this->carBody->addBoxCollider(vCentre, vSize, 4, 1, this->car->physicsGUID + 1);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_step(float dt)
{
	#if 0 // Dont need this
	if (this->isLive)
	{
		if (this->observer.hasChanged())
		{
			this->carBody->removeCollisionObjects();
			INIReader::clearCache();
			this->loadINI();
		}
	}
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////
