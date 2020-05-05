#pragma once

BEGIN_HOOK_OBJ(CarColliderManager)

	#define RVA_CarColliderManager_init 2766672
	#define RVA_CarColliderManager_loadINI 2766752

	static void _hook()
	{
		HOOK_METHOD_RVA(CarColliderManager, init);
		HOOK_METHOD_RVA(CarColliderManager, loadINI);
	}

	void _ctor();
	void _init(Car* pCar);
	void _loadINI();
	void _step(float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_ctor()
{
	this->isLive = false;
	this->carBody = nullptr;
	this->car = nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_init(Car* pCar)
{
	this->isLive = false;
	this->carBody = pCar->body;
	this->car = pCar;
	this->carModel = pCar->unixName;
	this->boxes.clear();

	this->_loadINI();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_loadINI()
{
	std::wstring filename(L"content/cars/" + this->carModel + L"/data/colliders.ini");
	auto ini(new_udt_unique<INIReader>(filename));

	if (ini->ready)
	{
		for (int id = 0; ; id++)
		{
			std::wstringstream section;
			section << L"COLLIDER_";
			section << id;
			auto sectionName(section.str());

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
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CarColliderManager::_step(float dt)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////
