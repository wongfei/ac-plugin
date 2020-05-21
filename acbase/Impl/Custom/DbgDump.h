#pragma once

static void obj_save(const wchar_t* filename, 
	const float* vertices, int numVertices, 
	const unsigned short* indices, int numIndices)
{
	FILE* fd = nullptr;
	_wfopen_s(&fd, filename, L"wb");
	if (!fd)
	{
		SHOULD_NOT_REACH_WARN;
		return;
	}

	const float* pv = vertices;
	const unsigned short* pi = indices;

	for (int id = 0; id < numVertices; id++, pv += 3)
	{
		fprintf(fd, "v %f %f %f\n", pv[0], pv[1], pv[2]);
	}

	for (int id = 0; id < numIndices; id += 3, pi += 3)
	{
		fprintf(fd, "f %d %d %d\n", pi[0]+1, pi[1]+1, pi[2]+1);
	}

	fclose(fd);
}

static int _dbg_surf_id = 0;

static void dumpTrackSurface(
	Track* track,
	const std::wstring& iname, 
	float* vertices, int numVertices, 
	unsigned short* indices, int indexCount, 
	const SurfaceDef& surfaceDef, unsigned int sector_id)
{
	auto strName = strf(L"surf_%05d", (int)_dbg_surf_id);
	auto strPath = L"_dump/" + track->dataFolder + L"/" + strName + L".bin";
	if (!osFileExists(strPath.c_str()))
	{
		struct SurfaceHeader
		{
			uint32_t headerSize;
			int32_t numVertices;
			int32_t numIndices;
			int32_t sectorId;
			SurfaceDef surfaceDef;
		};
		SurfaceHeader header = { sizeof(SurfaceHeader), numVertices, indexCount, (int)sector_id, surfaceDef };

		FILE* fd = nullptr;
		_wfopen_s(&fd, strPath.c_str(), L"wb");
		if (fd)
		{
			fwrite(&header, sizeof(header), 1, fd);
			fwrite(vertices, numVertices * 3 * sizeof(float), 1, fd);
			fwrite(indices, indexCount * sizeof(unsigned short), 1, fd);
			fclose(fd);
		}
	}

	strPath = strPath = L"_dump/" + track->dataFolder + L"/" + strName + L".obj";
	if (!osFileExists(strPath.c_str()))
	{
		obj_save(strPath.c_str(), vertices, numVertices, indices, indexCount);
	}

	_dbg_surf_id++;
}

static void dumpCarCollider(Car* car, Mesh* mesh, const mat44f& bodyMatrix, const std::vector<vec3f>& vertices)
{
	int iNumVertices = (int)vertices.size();
	auto pVertices = &vertices[0].x;

	int iNumIndices = (int)mesh->indices.size();
	auto pIndices = &mesh->indices[0];

	auto strPath = L"_dump/" + car->carDataPath + L"/collider.bin";
	if (!osFileExists(strPath.c_str()))
	{
		struct ColliderHeader
		{
			uint32_t headerSize;
			int32_t numVertices;
			int32_t numIndices;
		};
		ColliderHeader header = { sizeof(ColliderHeader), iNumVertices, iNumIndices };

		FILE* fd = nullptr;
		_wfopen_s(&fd, strPath.c_str(), L"wb");
		if (fd)
		{
			fwrite(&header, sizeof(header), 1, fd);
			fwrite(pVertices, iNumVertices * 3 * sizeof(float), 1, fd);
			fwrite(pIndices, iNumIndices * sizeof(unsigned short), 1, fd);
			fclose(fd);
		}
	}

	strPath = L"_dump/" + car->carDataPath + L"/collider.obj";
	if (!osFileExists(strPath.c_str()))
	{
		obj_save(strPath.c_str(), pVertices, iNumVertices, pIndices, iNumIndices);
	}
}
