#pragma once

inline void Car_forceRotation(Car* pCar, const mat44f& mat)
{
	pCar->body->setRotation(mat);
	pCar->fuelTankBody->setRotation(mat);

	for (auto* susp : pCar->suspensions)
	{
		susp->attach();
	}

	pCar->body->stop(1.0f);
	pCar->fuelTankBody->stop(1.0f);
}

inline void CarAvatar_teleport(CarAvatar* pCarAvatar, const mat44f& mat)
{
	float height = pCarAvatar->raceEngineer->getBaseCarHeight();
	vec3f pos = vec3f(mat.M41, mat.M42 - height, mat.M43);
	pCarAvatar->physics->forcePosition(pos, true);
	Car_forceRotation(pCarAvatar->physics, mat);
}
