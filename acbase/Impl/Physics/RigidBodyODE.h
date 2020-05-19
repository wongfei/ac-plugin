#pragma once

static void* _orig_RigidBodyODE_addBoxCollider = nullptr;
static void* _orig_RigidBodyODE_addSphereCollider = nullptr;
static void* _orig_RigidBodyODE_addMeshCollider = nullptr;

BEGIN_HOOK_OBJ(RigidBodyODE)
	
	#define RVA_RigidBodyODE_vtable 0x5009C0
	#define RVA_RigidBodyODE_ctor 2938880
	#define RVA_RigidBodyODE_dtor 2939104
	#define RVA_RigidBodyODE_release 2942976

	#define RVA_RigidBodyODE_setEnabled 2943088
	#define RVA_RigidBodyODE_isEnabled 2942816
	#define RVA_RigidBodyODE_setAutoDisable 2943040
	#define RVA_RigidBodyODE_stop 2943696

	#define RVA_RigidBodyODE_setMassBox 2943120
	#define RVA_RigidBodyODE_getMass 2942320
	#define RVA_RigidBodyODE_setMassExplicitInertia 2943248
	#define RVA_RigidBodyODE_getLocalInertia 2942080

	#define RVA_RigidBodyODE_localToWorld 2942848
	#define RVA_RigidBodyODE_worldToLocal 2943792
	#define RVA_RigidBodyODE_localToWorldNormal 2942912
	#define RVA_RigidBodyODE_worldToLocalNormal 2943856

	#define RVA_RigidBodyODE_setPosition 2943456
	#define RVA_RigidBodyODE_getPosition 2942528
	#define RVA_RigidBodyODE_setRotation 2943488
	#define RVA_RigidBodyODE_getWorldMatrix 2942656

	#define RVA_RigidBodyODE_setVelocity 2943664
	#define RVA_RigidBodyODE_getVelocity 2942592
	#define RVA_RigidBodyODE_getLocalVelocity 2942240
	#define RVA_RigidBodyODE_getPointVelocity 2942464
	#define RVA_RigidBodyODE_getLocalPointVelocity 2942176

	#define RVA_RigidBodyODE_setAngularVelocity 2943008
	#define RVA_RigidBodyODE_getAngularVelocity 2941936
	#define RVA_RigidBodyODE_getLocalAngularVelocity 2942000

	#define RVA_RigidBodyODE_addForceAtPos 2940720
	#define RVA_RigidBodyODE_addForceAtLocalPos 2940640
	#define RVA_RigidBodyODE_addLocalForce 2940800
	#define RVA_RigidBodyODE_addLocalForceAtPos 2940944
	#define RVA_RigidBodyODE_addLocalForceAtLocalPos 2940864
	#define RVA_RigidBodyODE_addTorque 2941904
	#define RVA_RigidBodyODE_addLocalTorque 2941024

	#define RVA_RigidBodyODE_addBoxCollider 2940336
	#define RVA_RigidBodyODE_addSphereCollider 96368
	#define RVA_RigidBodyODE_addMeshCollider 2941056

	static void _hook()
	{
		HOOK_METHOD_RVA(RigidBodyODE, ctor);

		HOOK_METHOD_RVA(RigidBodyODE, setEnabled);
		HOOK_METHOD_RVA(RigidBodyODE, isEnabled);
		HOOK_METHOD_RVA(RigidBodyODE, setAutoDisable);
		HOOK_METHOD_RVA(RigidBodyODE, stop);

		HOOK_METHOD_RVA(RigidBodyODE, setMassBox);
		HOOK_METHOD_RVA(RigidBodyODE, getMass);
		HOOK_METHOD_RVA(RigidBodyODE, setMassExplicitInertia);
		HOOK_METHOD_RVA(RigidBodyODE, getLocalInertia);

		HOOK_METHOD_RVA(RigidBodyODE, localToWorld);
		HOOK_METHOD_RVA(RigidBodyODE, worldToLocal);
		HOOK_METHOD_RVA(RigidBodyODE, localToWorldNormal);
		HOOK_METHOD_RVA(RigidBodyODE, worldToLocalNormal);

		HOOK_METHOD_RVA(RigidBodyODE, setPosition);
		HOOK_METHOD_RVA(RigidBodyODE, getPosition);
		HOOK_METHOD_RVA(RigidBodyODE, setRotation);
		HOOK_METHOD_RVA(RigidBodyODE, getWorldMatrix);

		HOOK_METHOD_RVA(RigidBodyODE, setVelocity);
		HOOK_METHOD_RVA(RigidBodyODE, getVelocity);
		HOOK_METHOD_RVA(RigidBodyODE, getLocalVelocity);
		HOOK_METHOD_RVA(RigidBodyODE, getPointVelocity);
		HOOK_METHOD_RVA(RigidBodyODE, getLocalPointVelocity);

		HOOK_METHOD_RVA(RigidBodyODE, setAngularVelocity);
		HOOK_METHOD_RVA(RigidBodyODE, getAngularVelocity);
		HOOK_METHOD_RVA(RigidBodyODE, getLocalAngularVelocity);

		HOOK_METHOD_RVA(RigidBodyODE, addForceAtPos);
		HOOK_METHOD_RVA(RigidBodyODE, addForceAtLocalPos);
		HOOK_METHOD_RVA(RigidBodyODE, addLocalForce);
		HOOK_METHOD_RVA(RigidBodyODE, addLocalForceAtPos);
		HOOK_METHOD_RVA(RigidBodyODE, addLocalForceAtLocalPos);
		HOOK_METHOD_RVA(RigidBodyODE, addTorque);
		HOOK_METHOD_RVA(RigidBodyODE, addLocalTorque);

		HOOK_METHOD_RVA_ORIG(RigidBodyODE, addBoxCollider);
		//HOOK_METHOD_RVA_ORIG(RigidBodyODE, addSphereCollider);
		HOOK_METHOD_RVA_ORIG(RigidBodyODE, addMeshCollider);
	}

	RigidBodyODE* _ctor(PhysicsCore* core);
	void _dtor();
	void _release();

	void _setEnabled(bool value);
	bool _isEnabled();
	void _setAutoDisable(bool mode);
	void _stop(float amount);

	void _setMassBox(float m, float x, float y, float z);
	float _getMass();
	void _setMassExplicitInertia(float totalMass, float x, float y, float z);
	vec3f _getLocalInertia();

	vec3f _localToWorld(const vec3f& p);
	vec3f _worldToLocal(const vec3f& p);
	vec3f _localToWorldNormal(const vec3f& p);
	vec3f _worldToLocalNormal(const vec3f& p);

	void _setPosition(const vec3f& pos);
	vec3f _getPosition(float interpolationT);
	void _setRotation(const mat44f& mat);
	mat44f _getWorldMatrix(float interpolationT);

	void _setVelocity(const vec3f& vel);
	vec3f _getVelocity();
	vec3f _getLocalVelocity();
	vec3f _getPointVelocity(const vec3f& p);
	vec3f _getLocalPointVelocity(const vec3f& p);

	void _setAngularVelocity(const vec3f& vel);
	vec3f _getAngularVelocity();
	vec3f _getLocalAngularVelocity();
	
	void _addForceAtPos(const vec3f& f, const vec3f& p);
	void _addForceAtLocalPos(const vec3f& f, const vec3f& p);
	void _addLocalForce(const vec3f& f);
	void _addLocalForceAtPos(const vec3f& f, const vec3f& p);
	void _addLocalForceAtLocalPos(const vec3f& f, const vec3f& p);
	void _addTorque(const vec3f& t);
	void _addLocalTorque(const vec3f& t);
	
	uint64_t _addBoxCollider(const vec3f& pos, const vec3f& size, unsigned int category, unsigned long mask, unsigned int spaceId);
	void _setBoxColliderMask(uint64_t box, unsigned long mask);
	void _addSphereCollider(const vec3f& pos, float radius, unsigned int group, ISphereCollisionCallback* callback);

	void _addMeshCollider(float* vertices, unsigned int verticesCount, unsigned short* indices, unsigned int indicesCount, mat44f* mat, unsigned long category, unsigned long collideMask, unsigned int spaceId);
	void _setMeshCollideMask(unsigned int meshIndex, unsigned long mask);
	void _setMeshCollideCategory(unsigned int meshIndex, unsigned long category);
	unsigned long _getMeshCollideMask(unsigned int meshIndex);
	unsigned long _getMeshCollideCategory(unsigned int meshIndex);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

RigidBodyODE* _RigidBodyODE::_ctor(PhysicsCore* core)
{
	AC_CTOR_THIS_VT(RigidBodyODE);

	this->core = core;
	this->id = ODE_CALL(dBodyCreate)(core->id);

	ODE_CALL(dBodySetFiniteRotationMode)(this->id, 1);
	ODE_CALL(dBodySetFiniteRotationAxis)(this->id, 0, 0, 0);
	ODE_CALL(dBodySetLinearDamping)(this->id, 0);
	ODE_CALL(dBodySetAngularDamping)(this->id, 0);
	ODE_CALL(dBodySetData)(this->id, this);

	return this;
}

void _RigidBodyODE::_dtor()
{
	if (this->id)
	{
		ODE_CALL(dBodyDestroy)(this->id);
		this->id = nullptr;
	}

	this->collisionMeshes.clear();
	this->geoms.clear(); // TODO: delete elements?
}

void _RigidBodyODE::_release()
{
	TODO_NOT_IMPLEMENTED;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_setEnabled(bool value)
{
	if (value)
		ODE_CALL(dBodyEnable)(this->id);
	else
		ODE_CALL(dBodyDisable)(this->id);
}

bool _RigidBodyODE::_isEnabled()
{
	return ODE_CALL(dBodyIsEnabled)(this->id) == 1;
}

void _RigidBodyODE::_setAutoDisable(bool mode)
{
	ODE_CALL(dBodySetAutoDisableFlag)(this->id, mode ? 1 : 0);
}

void _RigidBodyODE::_stop(float amount)
{
	ODE_CALL(dBodySetLinearVel)(this->id, 0, 0, 0);
	ODE_CALL(dBodySetAngularVel)(this->id, 0, 0, 0);
	ODE_CALL(dBodySetForce)(this->id, 0, 0, 0);
	ODE_CALL(dBodySetTorque)(this->id, 0, 0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_setMassBox(float m, float x, float y, float z)
{
	dMass d;
	ODE_CALL(dMassSetZero)(&d);
	ODE_CALL(dMassSetBoxTotal)(&d, m, x, y, z);
	ODE_CALL(dBodySetMass)(this->id, &d);
}

float _RigidBodyODE::_getMass()
{
	dMass d;
	ODE_CALL(dMassSetZero)(&d);
	ODE_CALL(dBodyGetMass)(this->id, &d);
	return d.mass;
}

void _RigidBodyODE::_setMassExplicitInertia(float totalMass, float x, float y, float z)
{
	TODO_WTF_IS_THIS;

	dMass d;
	ODE_CALL(dMassSetZero)(&d);
	d.mass = totalMass;
	d.I[0] = x; // TODO: why not [0] [5] [10]?
	d.I[4] = y;
	d.I[8] = z;
	ODE_CALL(dBodySetMass)(this->id, &d);
}

vec3f _RigidBodyODE::_getLocalInertia()
{
	dMass d;
	ODE_CALL(dMassSetZero)(&d);
	ODE_CALL(dBodyGetMass)(this->id, &d);
	return vec3f(d.I[0], d.I[5], d.I[10]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

vec3f _RigidBodyODE::_localToWorld(const vec3f& p)
{
	dVector3 res;
	ODE_CALL(dBodyGetRelPointPos)(this->id, ODE_V3(p), res);
	return vec3f(res);
}

vec3f _RigidBodyODE::_worldToLocal(const vec3f& p)
{
	dVector3 res;
	ODE_CALL(dBodyGetPosRelPoint)(this->id, ODE_V3(p), res);
	return vec3f(res);
}

vec3f _RigidBodyODE::_localToWorldNormal(const vec3f& p)
{
	dVector3 res;
	ODE_CALL(dBodyVectorToWorld)(this->id, ODE_V3(p), res);
	return vec3f(res);
}

vec3f _RigidBodyODE::_worldToLocalNormal(const vec3f& p)
{
	dVector3 res;
	ODE_CALL(dBodyVectorFromWorld)(this->id, ODE_V3(p), res);
	return vec3f(res);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_setPosition(const vec3f& pos)
{
	ODE_CALL(dBodySetPosition)(this->id, ODE_V3(pos));
}

vec3f _RigidBodyODE::_getPosition(float interpolationT)
{
	const dReal* res = ODE_CALL(dBodyGetPosition)(this->id);
	return vec3f(res);
}

void _RigidBodyODE::_setRotation(const mat44f& mat)
{
	dMatrix3 r;
	ODE_CALL(dRSetIdentity)(r);
	r[0] = mat.M11;
	r[1] = mat.M21;
	r[2] = mat.M31;
	r[4] = mat.M12;
	r[5] = mat.M22;
	r[6] = mat.M32;
	r[8] = mat.M13;
	r[9] = mat.M23;
	r[10] = mat.M33;
	ODE_CALL(dBodySetRotation)(this->id, r);
}

mat44f _RigidBodyODE::_getWorldMatrix(float interpolationT)
{
	const dReal* r = ODE_CALL(dBodyGetRotation)(this->id);
	const dReal* p = ODE_CALL(dBodyGetPosition)(this->id);
	mat44f m;
	m.M11 = r[0];
	m.M12 = r[4];
	m.M13 = r[8];
	m.M14 = 0.0;
	m.M21 = r[1];
	m.M22 = r[5];
	m.M23 = r[9];
	m.M24 = 0.0;
	m.M31 = r[2];
	m.M32 = r[6];
	m.M33 = r[10];
	m.M34 = 0.0;
	m.M41 = p[0];
	m.M42 = p[1];
	m.M43 = p[2];
	m.M44 = 1.0;
	return m;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_setVelocity(const vec3f& vel)
{
	ODE_CALL(dBodySetLinearVel)(this->id, ODE_V3(vel));
}

vec3f _RigidBodyODE::_getVelocity()
{
	dVector3 res;
	ODE_CALL(dBodyGetRelPointVel)(this->id, 0, 0, 0, res);
	return vec3f(res);
}

vec3f _RigidBodyODE::_getLocalVelocity()
{
	return _worldToLocalNormal(_getVelocity());
}

vec3f _RigidBodyODE::_getPointVelocity(const vec3f& p)
{
	dVector3 res;
	ODE_CALL(dBodyGetPointVel)(this->id, ODE_V3(p), res);
	return vec3f(res);
}

vec3f _RigidBodyODE::_getLocalPointVelocity(const vec3f& p)
{
	dVector3 res;
	ODE_CALL(dBodyGetRelPointVel)(this->id, ODE_V3(p), res);
	return vec3f(res);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_setAngularVelocity(const vec3f& vel)
{
	ODE_CALL(dBodySetAngularVel)(this->id, ODE_V3(vel));
}

vec3f _RigidBodyODE::_getAngularVelocity()
{
	const dReal* res = ODE_CALL(dBodyGetAngularVel)(this->id);
	return vec3f(res);
}

vec3f _RigidBodyODE::_getLocalAngularVelocity()
{
	return _worldToLocalNormal(_getAngularVelocity());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_addForceAtPos(const vec3f& f, const vec3f& p)
{
	ODE_CALL(dBodyAddForceAtPos)(this->id, ODE_V3(f), ODE_V3(p));
}

void _RigidBodyODE::_addForceAtLocalPos(const vec3f& f, const vec3f& p)
{
	ODE_CALL(dBodyAddForceAtRelPos)(this->id, ODE_V3(f), ODE_V3(p));
}

void _RigidBodyODE::_addLocalForce(const vec3f& f)
{
	ODE_CALL(dBodyAddRelForceAtRelPos)(this->id, ODE_V3(f), 0, 0, 0);
}

void _RigidBodyODE::_addLocalForceAtPos(const vec3f& f, const vec3f& p)
{
	ODE_CALL(dBodyAddRelForceAtPos)(this->id, ODE_V3(f), ODE_V3(p));
}

void _RigidBodyODE::_addLocalForceAtLocalPos(const vec3f& f, const vec3f& p)
{
	ODE_CALL(dBodyAddRelForceAtRelPos)(this->id, ODE_V3(f), ODE_V3(p));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_addTorque(const vec3f& t)
{
	ODE_CALL(dBodyAddTorque)(this->id,  ODE_V3(t));
}

void _RigidBodyODE::_addLocalTorque(const vec3f& t)
{
	ODE_CALL(dBodyAddRelTorque)(this->id,  ODE_V3(t));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

uint64_t _RigidBodyODE::_addBoxCollider(const vec3f& pos, const vec3f& size, unsigned int category, unsigned long mask, unsigned int spaceId)
{
	auto pSubSpace = this->core->getDynamicSubSpace(spaceId);
	auto pGeom = ODE_CALL(dCreateBox)(pSubSpace, ODE_V3(size));

	ODE_CALL(dGeomSetBody)(pGeom, this->id);

	ODE_CALL(dGeomSetOffsetPosition)(pGeom, ODE_V3(pos));
	const dReal* r = ODE_CALL(dBodyGetRotation)(this->id);
	ODE_CALL(dGeomSetRotation)(pGeom, r);

	ODE_CALL(dGeomSetCollideBits)(pGeom, mask);
	ODE_CALL(dGeomSetCategoryBits)(pGeom, category);

	this->geoms.push_back(pGeom);

	return (uint64_t)pGeom; // BRUTAL!
}

void _RigidBodyODE::_setBoxColliderMask(uint64_t box, unsigned long mask)
{
	ODE_CALL(dGeomSetCollideBits)((dGeomID)box, mask); // BRUTAL!
}

void _RigidBodyODE::_addSphereCollider(const vec3f& pos, float radius, unsigned int group, ISphereCollisionCallback* callback)
{
	// TODO: seems to be empty
	//auto orig = ORIG_METHOD(RigidBodyODE, addSphereCollider);
	//THIS_CALL(orig)(pos, radius, group, callback);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _RigidBodyODE::_addMeshCollider(float* vertices, unsigned int verticesCount, unsigned short* indices, unsigned int indicesCount, mat44f* mat, unsigned long category, unsigned long collideMask, unsigned int spaceId)
{
	//auto orig = ORIG_METHOD(RigidBodyODE, addMeshCollider);
	//THIS_CALL(orig)(vertices, verticesCount, indices, indicesCount, mat, category, collideMask, spaceId);

	int iVertexSize = 3 * sizeof(float);
	int iIndexSize = sizeof(unsigned short);
	int iTriangleSize = 3 * iIndexSize;

	int iVbSize = verticesCount * iVertexSize;
	int iIbSize = indicesCount * iIndexSize;

	auto pMesh(new_udt_shared<BodyCollisionMesh>());

	pMesh->vertices = (float*)malloc(iVbSize);
	memcpy(pMesh->vertices, vertices, iVbSize);

	pMesh->indices = (unsigned short*)malloc(iIbSize);
	memcpy(pMesh->indices, indices, iIbSize);

	auto pTriMeshData = ODE_CALL(dGeomTriMeshDataCreate)();

	ODE_CALL(dGeomTriMeshDataBuildSingle)(pTriMeshData, 
		pMesh->vertices, iVertexSize, verticesCount,
		pMesh->indices, indicesCount, iTriangleSize);

	pMesh->geomID = ODE_CALL(dCreateTriMesh)(nullptr, pTriMeshData, nullptr, nullptr, nullptr);

	auto pSpace = this->core->getDynamicSubSpace(spaceId);
	ODE_CALL(dSpaceAdd)(pSpace, pMesh->geomID);
	ODE_CALL(dGeomSetBody)(pMesh->geomID, this->id);

	auto pRbMesh(new_udt<RBCollisionMesh>());
	pRbMesh->_vtable = _drva(0x500B18); // TODO: ctor optimized
	pRbMesh->group = category;
	pRbMesh->mask = collideMask;
	pRbMesh->body = this;
	pRbMesh->userPointer = nullptr;
	ODE_CALL(dGeomSetData)(pMesh->geomID, pRbMesh);

	dMatrix3 r;
	ODE_CALL(dRSetIdentity)(r);
	r[0] = mat->M11;
	r[1] = mat->M21;
	r[2] = mat->M31;
	r[4] = mat->M12;
	r[5] = mat->M22;
	r[6] = mat->M32;
	r[8] = mat->M13;
	r[9] = mat->M23;
	r[10] = mat->M33;

	ODE_CALL(dGeomSetOffsetRotation)(pMesh->geomID, r);
	ODE_CALL(dGeomSetOffsetPosition)(pMesh->geomID, 0, 0, 0); // TODO: check
	ODE_CALL(dGeomSetCollideBits)(pMesh->geomID, collideMask);
	ODE_CALL(dGeomSetCategoryBits)(pMesh->geomID, category);

	this->collisionMeshes.push_back(pMesh);
}

void _RigidBodyODE::_setMeshCollideMask(unsigned int meshIndex, unsigned long mask)
{
	GUARD_DEBUG(meshIndex < this->collisionMeshes.size());
	ODE_CALL(dGeomSetCollideBits)(this->collisionMeshes[meshIndex]->geomID, mask);
}

void _RigidBodyODE::_setMeshCollideCategory(unsigned int meshIndex, unsigned long category)
{
	GUARD_DEBUG(meshIndex < this->collisionMeshes.size());
	ODE_CALL(dGeomSetCategoryBits)(this->collisionMeshes[meshIndex]->geomID, category);
}

unsigned long _RigidBodyODE::_getMeshCollideMask(unsigned int meshIndex)
{
	if (meshIndex < this->collisionMeshes.size())
		return ODE_CALL(dGeomGetCollideBits)(this->collisionMeshes[meshIndex]->geomID);
	return 0;
}

unsigned long _RigidBodyODE::_getMeshCollideCategory(unsigned int meshIndex)
{
	if (meshIndex < this->collisionMeshes.size())
		return ODE_CALL(dGeomGetCategoryBits)(this->collisionMeshes[meshIndex]->geomID);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
