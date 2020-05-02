#pragma once

BEGIN_HOOK_OBJ(Wing)

	#define RVA_Wing_step 2829248
	#define RVA_Wing_addDrag 2827296
	#define RVA_Wing_addLift 2828080

	void _step(float dt);
	void _addDrag(const vec3f& lv);
	void _addLift(const vec3f& lv);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_step(float dt)
{
	if (!this->overrideStatus.isActive)
		this->stepDynamicControllers(dt);

	vec3f vGroundWind = this->car->getGroundWindVector();
	vec3f vWorldVel = this->car->body->getLocalPointVelocity(this->data.position);
	vec3f vLocalVel = this->car->body->worldToLocalNormal(vWorldVel + vGroundWind);
	vec3f vWingWorld = this->car->body->localToWorld(this->data.position);
	this->status.groundHeight = this->engineer.getPointGroundHeight(vWingWorld);
  
	float fAngle = this->status.angle;
	if (this->overrideStatus.isActive)
		this->status.angle = this->overrideStatus.overrideAngle;
	
	if (vLocalVel.z == 0.0f)
	{
		this->status.aoa = 0;
		this->status.cd = 0;
		this->status.cl = 0;
		this->status.yawAngle = 0;
	}
	else
	{
		this->status.aoa = atanf((1.0f / vLocalVel.z) * vLocalVel.y) * 57.29578f;
		this->status.yawAngle = atanf((1.0f / vLocalVel.z) * vLocalVel.x) * 57.29578f;
		this->addDrag(vLocalVel);
		this->addLift(vLocalVel);
	}
  
	if (this->overrideStatus.isActive)
		this->status.angle = fAngle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_addDrag(const vec3f& lv)
{
	float fAngleOff = this->data.isVertical ? this->status.yawAngle : this->status.aoa;
	this->status.cd = this->data.lutAOA_CD.getValue((this->status.angleMult * this->status.angle) + fAngleOff) * this->data.cdGain;

	if (this->hasDamage) // TODO: implement
	{
	}

	if (this->data.lutGH_CD.getCount())
	{
		float fLut = this->data.lutGH_CD.getValue(this->status.groundHeight);
		this->status.groundEffectDrag = fLut;
		this->status.cd *= fLut;
	}

	float fDot = lv.sqlen();
	float fDrag = (((fDot * this->status.cd) * this->car->aeroMap.airDensity) * this->data.area) * 0.5f;
	this->status.dragKG = fDrag * 0.10197838f;

	if (fDot != 0.0f)
	{
		this->car->body->addLocalForceAtLocalPos(lv.get_norm() * -fDrag, this->data.position);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_addLift(const vec3f& lv)
{
	float fAngleOff, fAxis;

	if (this->data.isVertical)
	{
		fAngleOff = this->status.yawAngle;
		fAxis = lv.x;
	}
	else
	{
		fAngleOff = this->status.aoa;
		fAxis = lv.y;
	}

	this->status.cl = this->data.lutAOA_CL.getValue((this->status.angleMult * this->status.angle) + fAngleOff) * this->data.clGain;

	if (lv.z < 0.0f)
		this->status.cl = 0.0f;

	if (!this->data.isVertical && this->data.yawGain != 0.0f)
	{
		float v8 = (sinf(fabsf(this->status.yawAngle) * 0.017453f) * this->data.yawGain) + 1.0f;
		this->status.cl *= tclamp(v8, 0.0f, 1.0f);
	}
  
	if (this->data.lutGH_CL.getCount())
	{
		float fLut = this->data.lutGH_CL.getValue(this->status.groundHeight);
		this->status.groundEffectLift = fLut;
		this->status.cl *= fLut;
	}
  
	if (this->hasDamage) // TODO: implement
	{
	}

	float fDot = (fAxis * fAxis) + (lv.z * lv.z);
	float fLift = (((fDot * this->status.cl) * this->car->aeroMap.airDensity) * this->data.area) * 0.5f;
	this->status.liftKG = fLift * 0.10197838f;
  
	if (fDot != 0.0f)
	{
		// TODO: check

		vec3f vNorm = lv.get_norm();
		vec3f vOut = this->data.isVertical ? vec3f(-vNorm.z, 0, vNorm.x) : vec3f(0, vNorm.z, -vNorm.y);
		vec3f vForce = vOut * -fLift;

		this->status.liftVector = vForce;
		this->car->body->addLocalForceAtLocalPos(vForce, this->data.position);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
