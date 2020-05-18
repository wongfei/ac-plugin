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

	int iVertexSize = 3 * sizeof(float);
	int iIndexSize = sizeof(unsigned short);
	int iTriangleSize = 3 * iIndexSize;

	int iVbSize = numVertices * iVertexSize;
	int iIbSize = indexCount * iIndexSize;

	this->lvertices = (float*)malloc(iVbSize);
	memcpy(this->lvertices, vertices, iVbSize);

	this->lindices = (unsigned short*)malloc(iIbSize);
	memcpy(this->lindices, indices, iIbSize);

	this->trimeshData = ODE_CALL(dGeomTriMeshDataCreate)();

	ODE_CALL(dGeomTriMeshDataBuildSingle)(this->trimeshData, 
		this->lvertices, iVertexSize, numVertices,
		this->lindices, indexCount, iTriangleSize);

	auto* pSpace = core->getStaticSubSpace(space_id);
	this->trimesh = ODE_CALL(dCreateTriMesh)(pSpace, this->trimeshData, nullptr, nullptr, nullptr);

	ODE_CALL(dGeomSetData)(this->trimesh, this);
	ODE_CALL(dGeomSetCategoryBits)(this->trimesh, group);
	ODE_CALL(dGeomSetCollideBits)(this->trimesh, mask);

	this->userPointer = nullptr;
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
