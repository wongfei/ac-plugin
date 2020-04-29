#pragma once

BEGIN_HOOK_OBJ(Suspension)

	#define RVA_Suspension_step 2896784
	#define RVA_Suspension_addForceAtPos 2886640
	#define RVA_Suspension_addLocalForceAndTorque 2886976
	#define RVA_Suspension_addTorque 2887440

	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

void _Suspension::_step(float dt)
{
	this->steerTorque = 0.0f;

	mat44f mxBodyWorld = this->carBody->getWorldMatrix(0.0f);
	vec3f vBodyM2(&mxBodyWorld.M21);

	vec3f vHubWorldPos = this->hub->getPosition(0.0f);
	vec3f vHubLocalPos = this->carBody->worldToLocal(vHubWorldPos);

	float fHubDeltaY = vHubLocalPos.y - this->dataRelToWheel.refPoint.y;
	float fTravel = fHubDeltaY + this->rodLength;
	this->status.travel = fTravel;

	if (this->useActiveActuator)
	{
		this->activeActuator.targetTravel = 0.0f;
		float fActiveActuator = this->activeActuator.eval(dt, fHubDeltaY);

		this->car->antirollBars[0].k = 0.0f;
		this->car->antirollBars[1].k = 0.0f;

		vec3f vForce = vBodyM2 * fActiveActuator;
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);

		this->status.travel = fHubDeltaY;
	}
	else
	{
		float fForce = ((fTravel * this->progressiveK) + this->k) * fTravel;
		if (this->packerRange != 0.0f && fTravel > this->packerRange && this->k != 0.0f)
		{
			fForce += (((fTravel - this->packerRange) * this->bumpStopProgressive) + this->bumpStopRate) * (fTravel - this->packerRange);
		}

		if (fForce > 0.0f)
		{
			this->addForceAtPos(vBodyM2 * -fForce, vHubWorldPos, false, false);
			this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), this->dataRelToWheel.refPoint);
		}

		vec3f vHubVel = this->hub->getVelocity();
		vec3f vPointVel = this->carBody->getLocalPointVelocity(this->dataRelToWheel.refPoint);
		vec3f vDeltaVel = vHubVel - vPointVel;

		float fDamperSpeed = vDeltaVel * vBodyM2;
		this->status.damperSpeedMS = fDamperSpeed;

		float fDamperForce = this->damper.getForce(fDamperSpeed);
		vec3f vForce = vBodyM2 * fDamperForce;
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);
	}

	if (this->bumpStopUp != 0.0f && fHubDeltaY > this->bumpStopUp && 0.0f != this->k)
	{
		float fForce = (((fHubDeltaY - this->bumpStopUp) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - this->bumpStopUp);
		this->addForceAtPos(vBodyM2 * -fForce, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocalPos);
	}

	if (this->bumpStopDn != 0.0f && fHubDeltaY < this->bumpStopDn && 0.0f != this->k)
	{
		float fForce = (((fHubDeltaY - this->bumpStopDn) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - this->bumpStopDn);
		this->addForceAtPos(vBodyM2 * -fForce, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocalPos);
	}
}

void _Suspension::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
{
	this->hub->addForceAtPos(force, pos);

	if (addToSteerTorque)
	{
		vec3f vCenter, vAxis;
		this->getSteerBasis(vCenter, vAxis);

		this->steerTorque += (pos - vCenter).cross(force) * vAxis;
	}
}

void _Suspension::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
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

void _Suspension::_addTorque(const vec3f& torque)
{
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	this->steerTorque += torque * vAxis;
}
