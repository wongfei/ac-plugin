#pragma once

AC_GVAR_DECL(bool, INIReader_useCache, 0x0151D0F9);

static void* _orig_INIReader_load = nullptr;
static void* _orig_INIReader_loadEncrypt = nullptr;
static void* _orig_INIReader_parse = nullptr;

BEGIN_HOOK_OBJ(INIReader)

	#define RVA_INIReader_load 2322752
	#define RVA_INIReader_loadEncrypt 2323824
	#define RVA_INIReader_parse 2324096

	static void _hook()
	{
		AC_GVAR_INIT(INIReader_useCache);
		//log_printf(L"INIReader::useCache=%d", (int)AC_GVAR(INIReader_useCache));

		HOOK_METHOD_RVA_ORIG(INIReader, load);
		HOOK_METHOD_RVA_ORIG(INIReader, loadEncrypt);
		HOOK_METHOD_RVA_ORIG(INIReader, parse);
	}

	void _load(const std::wstring& filename);
	void _loadEncrypt(const std::wstring& filename, const std::wstring& dataFile);
	void _parse();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

static void ini_save(INIReader* ini, const std::wstring& dst, bool rewriteExisting = false)
{
	//log_printf(L"save \"%s\"", dst.c_str());

	if (dst.empty())
	{
		SHOULD_NOT_REACH_WARN;
		return;
	}

	if (rewriteExisting || !osFileExists(dst.c_str()))
	{
		auto sep = dst.find_last_of(L"\\/");
		if (sep != std::wstring::npos)
			osCreateDirectoryTree(dst.substr(0, sep));

		FILE* fd = nullptr;
		_wfopen_s(&fd, dst.c_str(), L"wt");
		if (fd)
		{
			for (const auto& s : ini->sections)
			{
				fwprintf(fd, L"[%s]\n", s.first.c_str());
				for (const auto& kv : s.second.keys)
				{
					fwprintf(fd, L"%s=%s\n", kv.first.c_str(), kv.second.c_str());
				}
				fwprintf(fd, L"\n");
			}
			fclose(fd);
		}
		else
		{
			log_printf(L"ini_save: fopen failed: \"%s\"", dst.c_str());
		}
	}
}

static void ini_dump(const std::wstring& filename)
{
	auto ini(new_udt_unique<INIReader>(filename));
	ini_save(ini.get(), L"_dump/" + ini->filename);
}

static void ini_load_event(INIReader* ini)
{
	#if defined(AC_DBG_DUMP_INI)
	if (ini->filename.find(L"content") == 0)
	{
		ini_save(ini, L"_dump/" + ini->filename);
	}
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _INIReader::_load(const std::wstring& filename)
{
	auto orig = ORIG_METHOD(INIReader, load);
	THIS_CALL(orig)(filename);
	ini_load_event(this);
}

void _INIReader::_loadEncrypt(const std::wstring& filename, const std::wstring& dataFile)
{
	auto orig = ORIG_METHOD(INIReader, loadEncrypt);
	THIS_CALL(orig)(filename, dataFile);
	ini_load_event(this);
}

void _INIReader::_parse()
{
	auto orig = ORIG_METHOD(INIReader, parse);
	THIS_CALL(orig)();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
