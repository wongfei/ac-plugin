#pragma once

inline void forceRotationX(Car* pCar, mat44f& mat)
{
	pCar->body->setRotation(mat);
	pCar->fuelTankBody->setRotation(mat);

	for (auto& s : pCar->suspensions)
	{
		s->attach();
	}

	pCar->body->stop(1.0f);
	pCar->fuelTankBody->stop(1.0f);
}

inline void teleportCar(CarAvatar* pCarAvatar, mat44f& mat)
{
	float height = pCarAvatar->raceEngineer->getBaseCarHeight();
	vec3f pos = makev(mat.M41, mat.M42 - height, mat.M43);
	pCarAvatar->physics->forcePosition(pos, true);
	forceRotationX(pCarAvatar->physics, mat);
}
