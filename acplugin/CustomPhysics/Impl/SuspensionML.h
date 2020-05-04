#pragma once

BEGIN_HOOK_OBJ(SuspensionML)

	#define RVA_SuspensionML_step 2927360
	#define RVA_SuspensionML_addForceAtPos 2920912
	#define RVA_SuspensionML_addLocalForceAndTorque 2921200
	#define RVA_SuspensionML_addTorque 2921344

	static void _hook()
	{
		HOOK_METHOD_RVA(SuspensionML, step);
		HOOK_METHOD_RVA(SuspensionML, addForceAtPos);
		HOOK_METHOD_RVA(SuspensionML, addLocalForceAndTorque);
		HOOK_METHOD_RVA(SuspensionML, addTorque);
	}

	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

void _SuspensionML::_step(float dt)
{
	this->steerTorque = 0.0;

	mat44f mxBodyWorld = this->car->body->getWorldMatrix(0.0f);
	vec3f vM2(&mxBodyWorld.M21);

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->car->body->worldToLocal(vHubWorld);

	float fTravel = (vHubLocal.y - this->basePosition.y) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = (fTravel * this->progressiveK + this->k) * fTravel;
	if (this->packerRange != 0.0f && fTravel > this->packerRange && this->k != 0.0f)
	{
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);
	}

	this->addForceAtPos(vM2 * -fForce, vHubWorld, false, false);
	this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), this->basePosition);

	vec3f vPointVel = this->car->body->getLocalPointVelocity(this->basePosition);
	vec3f vDeltaVel = this->hub->getVelocity() - vPointVel;

	float fDamperSpeed = vDeltaVel * vM2;
	this->status.damperSpeedMS = fDamperSpeed;

	float fDamperForce = this->damper.getForce(fDamperSpeed);
	vec3f vForce = vM2 * fDamperForce;
	this->addForceAtPos(vForce, vHubWorld, false, false);
	this->car->body->addForceAtLocalPos(vForce * -1.0f, this->basePosition);
}

void _SuspensionML::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
{
	this->hub->addForceAtPos(force, pos);

	if (addToSteerTorque)
	{
		vec3f vCenter, vAxis;
		this->getSteerBasis(vCenter, vAxis);

		this->steerTorque += (pos - vCenter).cross(force) * vAxis;
	}
}

void _SuspensionML::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
{
	this->hub->addForceAtLocalPos(force, vec3f(0, 0, 0));
	this->hub->addTorque(torque);

	if (driveTorque.x != 0.0f || driveTorque.y != 0.0f || driveTorque.z != 0.0f)
		this->car->body->addTorque(driveTorque);
}

void _SuspensionML::_addTorque(const vec3f& torque)
{
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	this->steerTorque += torque * vAxis;
}
