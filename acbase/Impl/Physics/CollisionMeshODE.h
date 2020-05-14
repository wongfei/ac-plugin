#pragma once

BEGIN_HOOK_OBJ(CollisionMeshODE)

	#define RVA_CollisionMeshODE_vtable 0x500BC0
	#define RVA_CollisionMeshODE_ctor 2943920

	static void _hook()
	{
		HOOK_METHOD_RVA(CollisionMeshODE, ctor);
	}

	CollisionMeshODE* _ctor(PhysicsCore* core, 
		float* vertices, int numVertices, unsigned short* indices, int indexCount, 
		unsigned long group, unsigned long mask, unsigned int space_id);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

CollisionMeshODE* _CollisionMeshODE::_ctor(PhysicsCore* core, 
	float* vertices, int numVertices, unsigned short* indices, int indexCount, 
	unsigned long group, unsigned long mask, unsigned int space_id)
{
	AC_CTOR_VCLASS(CollisionMeshODE);

	int vertexStride = 3 * sizeof(float); // 12
	int triStride = 3 * sizeof(unsigned short); // 6

	this->lvertices = (float*)malloc(numVertices * vertexStride);
	memcpy(this->lvertices, vertices, numVertices * vertexStride);

	this->lindices = (unsigned short*)malloc(indexCount * sizeof(unsigned short));
	memcpy(this->lindices, indices, indexCount * sizeof(unsigned short));

	this->trimeshData = ODE_CALL(dGeomTriMeshDataCreate)();

	ODE_CALL(dGeomTriMeshDataBuildSingle)(this->trimeshData, 
		this->lvertices, vertexStride, numVertices,
		this->lindices, indexCount, triStride);

	auto* pSpace = core->getStaticSubSpace(space_id);
	this->trimesh = ODE_CALL(dCreateTriMesh)(pSpace, this->trimeshData, nullptr, nullptr, nullptr);

	ODE_CALL(dGeomSetData)(this->trimesh, this);
	ODE_CALL(dGeomSetCategoryBits)(this->trimesh, group);
	ODE_CALL(dGeomSetCollideBits)(this->trimesh, mask);

	this->userPointer = nullptr;
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
