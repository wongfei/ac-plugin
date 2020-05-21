#pragma once

static void* _orig_KN5IO_load = nullptr;

BEGIN_HOOK_OBJ(KN5IO)

	#define RVA_KN5IO_load 2183584

	static void _hook()
	{
		HOOK_METHOD_RVA_ORIG(KN5IO, load);
	}

	Node* _load(const std::wstring& filename);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Node* _KN5IO::_load(const std::wstring& filename)
{
	auto orig = ORIG_METHOD(KN5IO, load);
	auto pNode = THIS_CALL(orig)(filename);

	#if 0
	if (filename.find(L"toyota_ae86_drift.kn5") != std::wstring::npos)
	{
		Mesh* m = findNodeOfType<Mesh>(pNode, RVA_Mesh_RTTI);

		std::vector<Mesh*> meshes;
		findAllNodesOfType<Mesh>(pNode, RVA_Mesh_RTTI, meshes);
	}
	#endif

	return pNode;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
