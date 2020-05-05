#pragma once

BEGIN_HOOK_OBJ(CollisionMeshODE)

	#define RVA_CollisionMeshODE_ctor 2943920
	#define RVA_CollisionMeshODE_dtor 0 // TODO: rva -> 0
	#define RVA_CollisionMeshODE_release 2944400

	static void _hook()
	{
		HOOK_METHOD_RVA(CollisionMeshODE, ctor);
	}

	void _ctor(PhysicsCore* core, 
		float* vertices, int numVertices, unsigned short* indices, int indexCount, 
		unsigned long group, unsigned long mask, unsigned int space_id);

	void _dtor();
	void _release();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _CollisionMeshODE::_ctor(PhysicsCore* core, 
	float* vertices, int numVertices, unsigned short* indices, int indexCount, 
	unsigned long group, unsigned long mask, unsigned int space_id)
{
	this->lvertices = (float*)malloc(numVertices * 3 * sizeof(float));
	memcpy(this->lvertices, vertices, numVertices * 3 * sizeof(float));

	this->lindices = (unsigned short*)malloc(indexCount * sizeof(unsigned short));
	memcpy(this->lindices, indices, indexCount * sizeof(unsigned short));

	this->trimeshData = ODE_CALL(dGeomTriMeshDataCreate)();

	ODE_CALL(dGeomTriMeshDataBuildSingle)(this->trimeshData, 
		this->lvertices, 12, numVertices, 
		this->lindices, indexCount, 6);

	auto* pSpace = core->getStaticSubSpace(space_id);
	this->trimesh = ODE_CALL(dCreateTriMesh)(pSpace, this->trimeshData, nullptr, nullptr, nullptr);

	ODE_CALL(dGeomSetData)(this->trimesh, this);
	ODE_CALL(dGeomSetCategoryBits)(this->trimesh, group);
	ODE_CALL(dGeomSetCollideBits)(this->trimesh, mask);

	this->userPointer = nullptr;
}

void _CollisionMeshODE::_dtor()
{
	TODO_NOT_IMPLEMENTED;
}

void _CollisionMeshODE::_release()
{
	TODO_NOT_IMPLEMENTED;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
