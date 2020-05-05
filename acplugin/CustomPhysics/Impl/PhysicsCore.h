#pragma once

AC_GREF_DECL(int, rayCollideTestCount, 0x0155AA5C)
AC_GREF_DECL(int, geomCollideTestCount, 0x0155AA60)
AC_GREF_DECL(int, broadTestCount, 0x0155AA64)

BEGIN_HOOK_OBJ(PhysicsCore)

	#define RVA_PhysicsCore_ctor 2931328
	#define RVA_PhysicsCore_dtor 2931744
	#define RVA_PhysicsCore_release 2937952
	#define RVA_PhysicsCore_step 2938512
	#define RVA_PhysicsCore_collisionStep 2932624
	#define RVA_PhysicsCore_onCollision 2936224
	#define RVA_PhysicsCore_rayCastR 2936944
	#define RVA_PhysicsCore_rayCastL 2937248

	static void _hook()
	{
		AC_GREF_INIT(rayCollideTestCount)
		AC_GREF_INIT(geomCollideTestCount)
		AC_GREF_INIT(broadTestCount)

		HOOK_METHOD_RVA(PhysicsCore, step);
		HOOK_METHOD_RVA(PhysicsCore, collisionStep);
		HOOK_METHOD_RVA(PhysicsCore, onCollision);
		HOOK_METHOD_RVA(PhysicsCore, rayCastR);
		HOOK_METHOD_RVA(PhysicsCore, rayCastL);
	}

	void _ctor();
	void _dtor();
	void _release();

	void _step(float dt);
	void _collisionStep(float dt);
	void _onCollision(dContactGeom* contacts, int numContacts, dxGeom* g0, dxGeom* g1);

	RayCastHit _rayCastR(const vec3f& pos, const vec3f& dir, dxGeom* ray);
	RayCastHit _rayCastL(const vec3f& pos, const vec3f& dir, float length);

	IRigidBody* _createRigidBody();
	IJoint* _createBallJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& pos);
	IJoint* _createBumpJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& p1, float rangeUp, float rangeDn);
	IJoint* _createDistanceJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& p1, const vec3f& p2);
	IJoint* _createFixedJoint(IRigidBody* rb1, IRigidBody* rb2);
	IJoint* _createSliderJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& axis);

	IRayCaster* _createRayCaster(float length);

	ICollisionObject* _createCollisionMesh(
		float* vertices, unsigned int numVertices, unsigned short* indices, int indexCount, 
		const mat44f& worldMatrix, IRigidBody* body, unsigned long group, unsigned long mask, unsigned int space_id);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsCore::_ctor()
{
	memset(&this->coreCPUTimes, 0, sizeof(CoreCPUTimes));
	this->noCollisionCounter = 0;
	this->totRayTime = 0;
	this->currentFrame = 0;
	this->collisionCallback = nullptr;
	this->threading = nullptr;
	this->pool = nullptr;

	ODE_CALL(dInitODE2)(0);
	ODE_CALL(dAllocateODEDataForThread)(0xFFFFFFFF);
	log_printf(L"ODE BUILD FLAGS: %S", ODE_CALL(dGetConfiguration)());

	this->id = ODE_CALL(dWorldCreate)();
	ODE_CALL(dWorldSetGravity)(this->id, 0.0f, -9.805999f, 0.0f);
	ODE_CALL(dWorldSetERP)(this->id, 0.3f);
	ODE_CALL(dWorldSetCFM)(this->id, 1.0e-7f);
	ODE_CALL(dWorldSetContactMaxCorrectingVel)(this->id, 3.0f);
	ODE_CALL(dWorldSetContactSurfaceLayer)(this->id, 0.0f);

	this->contactGroup = ODE_CALL(dJointGroupCreate)(0);
	this->contactGroupDynamic = ODE_CALL(dJointGroupCreate)(0);
	this->currentContactGroup = this->contactGroup;

	this->spaceStatic = ODE_CALL(dSimpleSpaceCreate)(nullptr);
	this->spaceDynamic = ODE_CALL(dSimpleSpaceCreate)(nullptr);

	this->ray = ODE_CALL(dCreateRay)(nullptr, 100.0f);
	ODE_CALL(dGeomRaySetFirstContact)(this->ray, 1);
	ODE_CALL(dGeomRaySetBackfaceCull)(this->ray, 1);

	ODE_CALL(dWorldSetDamping)(this->id, 0.0f, 0.0f);

	this->id->qs.num_iterations = 48; // TODO: check
	// IDA guess -> dGeomSetCategoryBits(this->id, 48i64)
	// same as dWorldSetQuickStepNumIterations
}

void _PhysicsCore::_dtor()
{
	TODO_NOT_IMPLEMENTED;
}

void _PhysicsCore::_release()
{
	TODO_NOT_IMPLEMENTED;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsCore::_step(float dt)
{
	if (this->noCollisionCounter)
		this->noCollisionCounter--;
	else
		this->collisionStep(dt);

	ODE_CALL(dWorldStep)(this->id, dt);

	rayCollideTestCount = 0;
	geomCollideTestCount = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

static void collisionNearCallback(void *data, dxGeom *o1, dxGeom *o2);

void _PhysicsCore::_collisionStep(float dt)
{
	auto* pCore = (PhysicsCore*)this;

	this->coreCPUTimes.contactPoints = 0;
	broadTestCount = 0;

	if (this->currentFrame & 1)
	{
		ODE_CALL(dJointGroupEmpty)(this->contactGroupDynamic);
		this->currentContactGroup = this->contactGroupDynamic;
		ODE_CALL(dSpaceCollide2)((dGeomID)this->spaceDynamic, (dGeomID)this->spaceStatic, pCore, collisionNearCallback);
	}
	else
	{
		ODE_CALL(dJointGroupEmpty)(this->contactGroup);
		this->currentContactGroup = this->contactGroup;
		ODE_CALL(dSpaceCollide)(this->spaceDynamic, pCore, collisionNearCallback);
	}

	this->currentFrame++;
	this->coreCPUTimes.narrowPhaseTests = geomCollideTestCount;
}

static void collisionNearCallback(void *data, dxGeom *o1, dxGeom *o2)
{
	if (ODE_CALL(dGeomIsSpace)(o1) || ODE_CALL(dGeomIsSpace)(o2))
	{
		++broadTestCount;
		ODE_CALL(dSpaceCollide2)(o1, o2, data, collisionNearCallback);
	}
	else
	{
		if ((ODE_CALL(dGeomGetCategoryBits)(o1) & ODE_CALL(dGeomGetCollideBits)(o2))
			&& (ODE_CALL(dGeomGetCollideBits)(o1) & ODE_CALL(dGeomGetCategoryBits)(o2)))
		{
			++geomCollideTestCount;

			int flags = (ODE_CALL(dGeomGetBody)(o1) && ODE_CALL(dGeomGetBody)(o2)) ? 4 : 32;

			const int maxContacts = 32;
			dContactGeom contacts[maxContacts];

			int n = ODE_CALL(dCollide)(o1, o2, flags, &contacts[0], sizeof(contacts[0]));
			if (n > 0)
				((PhysicsCore*)data)->onCollision(&contacts[0], n, o1, o2);
		}
	}
}

void _PhysicsCore::_onCollision(dContactGeom* contacts, int numContacts, dxGeom* g0, dxGeom* g1)
{
	dBodyID body0 = ODE_CALL(dGeomGetBody)(g0);
	dBodyID body1 = ODE_CALL(dGeomGetBody)(g1);
	int geomClass0 = ODE_CALL(dGeomGetClass)(g0);
	int geomClass1 = ODE_CALL(dGeomGetClass)(g1);
	void* shape0 = ODE_CALL(dGeomGetData)(g0); // CollisionMeshODE*
	void* shape1 = ODE_CALL(dGeomGetData)(g1);
	void* userData0 = body0 ? ODE_CALL(dBodyGetData)(body0) : nullptr; // RigidBodyODE*
	void* userData1 = body1 ? ODE_CALL(dBodyGetData)(body1) : nullptr;

	for (int i = 0; i < numContacts; ++i)
	{
		const dContactGeom& cg = contacts[i];

		dContact cj;
		memset(&cj, 0, sizeof(cj));
		cj.geom = cg;

		if ((geomClass1 != 8 || geomClass0 != 1) 
			&& (geomClass0 != 8 || geomClass1 != 1))
		{
			cj.surface.mode = 28692; // dContactBounce | dContactSoftCFM | dContactApprox1
			cj.surface.mu = 0.25f;
			cj.surface.bounce = 0.00999999978f;
			cj.surface.soft_cfm = 9.99999975e-005f;
		}
		else
		{
			if (body0 || body1)
			{
				dVector3 loc_normal;
				ODE_CALL(dBodyVectorFromWorld)((body0 ? body0 : body1), 
					cg.normal[0], cg.normal[1], cg.normal[2], loc_normal);

				if (loc_normal[1] < 0.8999999f)
					continue;
			}

			cj.surface.mode = 28700; // dContactBounce | dContactSoftERP | dContactSoftCFM | dContactApprox1
			cj.surface.mu = 0.1f;
			cj.surface.bounce = 0.0f;
			cj.surface.soft_cfm = 0.000952380942f;
			cj.surface.soft_erp = 0.714285731f;
		}

		dJointID j = ODE_CALL(dJointCreateContact)(this->id, this->currentContactGroup, &cj);
		ODE_CALL(dJointAttach)(j, body0, body1);

		if (this->collisionCallback)
		{
			this->collisionCallback->onCollisionCallBack(
				userData0, shape0, 
				userData1, shape1, 
				vec3f(cg.normal), vec3f(cg.pos), cg.depth);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

static void rayNearCallback(void* data, dxGeom* o1, dxGeom* o2);

RayCastHit _PhysicsCore::_rayCastL(const vec3f& pos, const vec3f& dir, float length)
{
	ODE_CALL(dGeomRaySetLength)(this->ray, length); 
	return _rayCastR(pos, dir, this->ray);
}

RayCastHit _PhysicsCore::_rayCastR(const vec3f& pos, const vec3f& dir, dxGeom* ray)
{
	RayCastHit hit;
	memset(&hit, 0, sizeof(hit));

	dContactGeom cg;
	cg.depth = -1.0f;

	broadTestCount = 0;
	rayCollideTestCount = 0;

	ODE_CALL(dGeomRaySet)(ray, ODE_V3(pos), ODE_V3(dir));
	ODE_CALL(dSpaceCollide2)(ray, this->spaceStatic, &cg, rayNearCallback);

	if (cg.depth >= 0.0f)
	{
		hit.pos = vec3f(cg.pos);
		hit.normal = vec3f(cg.normal);
		hit.collisionObject = (ICollisionObject*)ODE_CALL(dGeomGetData)(cg.g2);
		hit.hasContact = true;
	}

	return hit;
}

static void rayNearCallback(void* data, dxGeom* o1, dxGeom* o2)
{
	if (ODE_CALL(dGeomIsSpace)(o1) || ODE_CALL(dGeomIsSpace)(o2))
	{
		++broadTestCount;
		ODE_CALL(dSpaceCollide2)(o1, o2, data, rayNearCallback);
	}
	else if (!ODE_CALL(dGeomGetBody)(o1) && !ODE_CALL(dGeomGetBody)(o2))
	{
		++rayCollideTestCount;

		auto* result = (dContactGeom*)data;

		const int maxContacts = 32;
		dContactGeom contacts[maxContacts];

		int n = ODE_CALL(dCollide)(o1, o2, 1i64, &contacts[0], sizeof(contacts[0]));

		// TODO: check (orig code is fkn mess here)
		for (int i = 0; i < n; ++i)
		{
			const dContactGeom& cg = contacts[i];

			if (result->depth < 0.0f || result->depth > cg.depth)
			{
				*result = cg;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#if 0

IRigidBody* _PhysicsCore::_createRigidBody()
{
	return new _RigidBodyODE(this);
}

IJoint* _PhysicsCore::_createBallJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& pos)
{
	dJointID j = dJointCreateBall(this->id, 0);
	dJointAttach(j, ((RigidBodyODE*)rb1)->id, ((RigidBodyODE*)rb2)->id);
	dJointSetBallAnchor(j, ODE_V3(pos));
	return new _JointODE(this, j); // BallJointODE
}

IJoint* _PhysicsCore::_createBumpJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& p1, float rangeUp, float rangeDn)
{
	return nullptr;
}

IJoint* _PhysicsCore::_createDistanceJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& p1, const vec3f& p2)
{
	dJointID j = dJointCreateDBall(this->id, 0);
	dJointAttach(j, ((RigidBodyODE*)rb1)->id, ((RigidBodyODE*)rb2)->id);
	dJointSetDBallAnchor1(j, ODE_V3(p1));
	dJointSetDBallAnchor2(j, ODE_V3(p2));
	float d = dJointGetDBallDistance(j);
	return new _JointODE(this, j); // DistanceJointODE
}

IJoint* _PhysicsCore::_createFixedJoint(IRigidBody* rb1, IRigidBody* rb2)
{
	dJointID j = dJointCreateFixed(this->id, 0);
	dJointAttach(j, ((RigidBodyODE*)rb1)->id, ((RigidBodyODE*)rb2)->id);
	dJointSetFixed(j);
	return new _JointODE(this, j); // FixedJointODE
}

IJoint* _PhysicsCore::_createSliderJoint(IRigidBody* rb1, IRigidBody* rb2, const vec3f& axis)
{
	dJointID j = dJointCreateSlider(this->id, 0);
	dJointAttach(j, ((RigidBodyODE*)rb1)->id, ((RigidBodyODE*)rb2)->id);
	dJointSetSliderAxis(j, ODE_V3(axis));
	return new _JointODE(this, j); // SliderJointODE
}

IRayCaster* _PhysicsCore::_createRayCaster(float length)
{
	return new _RayCaster(length);
}

ICollisionObject* _PhysicsCore::_createCollisionMesh(
	float* vertices, unsigned int numVertices, unsigned short* indices, int indexCount, 
	const mat44f& worldMatrix, IRigidBody* body, unsigned long group, unsigned long mask, unsigned int space_id)
{
	return new _CollisionMeshODE(
		vertices, numVertices, indices, indexCount, 
		worldMatrix, body, group, mask, space_id);
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
