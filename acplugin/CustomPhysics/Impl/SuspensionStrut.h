#pragma once

BEGIN_HOOK_OBJ(SuspensionStrut)

	#define RVA_SuspensionStrut_step 2909696
	#define RVA_SuspensionStrut_addForceAtPos 2899296
	#define RVA_SuspensionStrut_addLocalForceAndTorque 2899584
	#define RVA_SuspensionStrut_addTorque 2900064

	static void _hook()
	{
		HOOK_METHOD_RVA(SuspensionStrut, step);
		HOOK_METHOD_RVA(SuspensionStrut, addForceAtPos);
		HOOK_METHOD_RVA(SuspensionStrut, addLocalForceAndTorque);
		HOOK_METHOD_RVA(SuspensionStrut, addTorque);
	}

	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

void _SuspensionStrut::_step(float dt)
{
	this->steerTorque = 0.0f;

	mat44f mxBodyWorld = this->carBody->getWorldMatrix(0.0f);

	vec3f vCarStrut = this->carBody->localToWorld(this->dataRelToBody.carStrut);
	vec3f vTyreStrut = this->hub->localToWorld(this->dataRelToWheel.tyreStrut);

	vec3f vDelta = vTyreStrut - vCarStrut;
	float fDeltaLen = vDelta.len();
	vDelta.norm(fDeltaLen);

	float fDefaultLength = this->strutBaseLength + this->rodLength;
	float fTravel = fDefaultLength - fDeltaLen;
	this->status.travel = fTravel;

	float fForce = ((fTravel * this->progressiveK) + this->k) * fTravel;
	if (fForce < 0) 
		fForce = 0;

	if (this->packerRange != 0.0f && fTravel > this->packerRange)
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);

	if (fForce > 0)
	{
		vec3f vForce = vDelta * fForce;
		this->addForceAtPos(vForce, vTyreStrut, false, false);
		this->carBody->addForceAtPos(vForce * -1.0f, vCarStrut);
	}

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->carBody->worldToLocal(vHubWorld);
	float fHubDelta = vHubLocal.y - this->dataRelToWheel.refPoint.y;

	if (fHubDelta > this->bumpStopUp)
	{
		fForce = (fHubDelta - this->bumpStopUp) * 500000.0f;
		this->addForceAtPos(vec3f(&mxBodyWorld.M21) * -fForce, this->hub->getPosition(0.0f), false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocal);
	}

	if (fHubDelta < this->bumpStopDn)
	{
		fForce = (fHubDelta - this->bumpStopDn) * 500000.0f;
		this->addForceAtPos(vec3f(&mxBodyWorld.M21) * -fForce, this->hub->getPosition(0.0f), false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocal);
	}

	vec3f vTyreStrutVel = this->hub->getLocalPointVelocity(this->dataRelToWheel.tyreStrut);
	vec3f vCarStrutVel = this->carBody->getLocalPointVelocity(this->dataRelToBody.carStrut);
	vec3f vDamperDelta = vTyreStrutVel - vCarStrutVel;

	float fDamperSpeed = vDamperDelta * vDelta;
	this->status.damperSpeedMS = fDamperSpeed;

	float fDamperForce = this->damper.getForce(fDamperSpeed);
	vec3f vDamperForce = vDelta * fDamperForce;
	this->addForceAtPos(vDamperForce, vTyreStrut, false, false);
	this->carBody->addForceAtPos(vDamperForce * -1.0f, vCarStrut);
}

void _SuspensionStrut::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
{
	this->hub->addForceAtPos(force, pos);

	if (addToSteerTorque)
	{
		vec3f vCenter, vAxis;
		this->getSteerBasis(vCenter, vAxis);

		this->steerTorque -= force.cross(pos - vCenter) * vAxis;
	}
}

void _SuspensionStrut::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
{
	this->hub->addForceAtLocalPos(force, vec3f(0, 0, 0));
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	vec3f vHubPos = this->hub->getPosition(0);

	this->steerTorque += (vHubPos - vCenter).cross(force) * vAxis;
	this->steerTorque += torque * vAxis;

	if (driveTorque.x != 0.0f || driveTorque.y != 0.0f || driveTorque.z != 0.0f)
		this->car->body->addTorque(driveTorque);
}

void _SuspensionStrut::_addTorque(const vec3f& torque)
{
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	this->steerTorque += torque * vAxis;
}
