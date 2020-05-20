#pragma once

inline void Car_forceRotationMx(Car* pCar, const mat44f& mat)
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
	auto* pCar = pCarAvatar->physics;

	Car_forceRotationMx(pCar, mat);
	//pCar->forceRotation(vec3f(&mat.M31) * -1.0f);

	float fHeight = pCarAvatar->raceEngineer->getBaseCarHeight() + 0.01f;
	vec3f vPos = vec3f(mat.M41, mat.M42 - fHeight, mat.M43);
	pCar->forcePosition(vPos, true);
}
