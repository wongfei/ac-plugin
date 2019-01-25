#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include "vthook.h"

#define AC_GUARD_METHOD_SIG(_class, _method, _ret, _args)\
struct guard_##_class##_##_method {\
	template <typename T, _ret (T::*)_args> struct enable { typedef T type; };\
	inline guard_##_class##_##_method() { static_assert(sizeof(enable<_class, &_class::_method>::type) == sizeof(_class), "invalid method"); }\
};

#define AC_GET_PTHIS(_hook_class, _base_class, _pthis)\
	((_hook_class*)get_udt_tag((_base_class*)((void*)_pthis)))

// _base_class->vfptr->thunk->_hook_class->vfptr->method
#define AC_OVERRIDE_METHOD(_hook_class, _base_class, _sig_class, _method, _vfid, _ret, _args, _arg_names)\
	AC_GUARD_METHOD_SIG(_sig_class, _method, _ret, _args)\
	virtual _ret _method _args;\
	__declspec(noinline) _ret thunk_##_method _args {\
		auto pthis = AC_GET_PTHIS(_hook_class, _base_class, this);\
		return pthis->##_method##_arg_names;\
	}\
	inline void hook_##_method(_base_class* pbase) { vtablehook_hook(pbase, method_ptr(&_hook_class::thunk_##_method), _vfid); }

class _RawVft
{
public:
	void* dbg[64];
};

class _RawClass
{
public:
	union {
		void* vfptr;
		_RawVft* vfptrd;
	};
};

template<typename T>
inline T tmin(T a, T b) { return (a < b ? a : b); }

template<typename T>
inline T tmax(T a, T b) { return (a > b ? a : b); }

inline void* mem_off(void* mem, size_t off) {
	return (void*)(((uint8_t*)mem) + off);
}

template<typename T>
inline void set_udt_tag(T* udt, void* tag) {
	*((void**) mem_off(udt, sizeof(T)) ) = tag;
}

template<typename T>
inline void* get_udt_tag(T* udt) {
	return *((void**) mem_off(udt, sizeof(T)) );
}

template<typename T, typename ...Args>
inline T* new_udt(Args&&... params) {
	T* obj = (T*)malloc(sizeof(T) + sizeof(void*));
	set_udt_tag(obj, nullptr);
	obj->ctor(std::forward<Args>(params)...);
	return obj;
}

template<typename T>
inline void del_udt(T* obj) {
	obj->dtor();
	free(obj);
}

template<typename T>
inline void safe_delete(T*& obj) {
	if (obj) {
		delete obj;
		obj = nullptr;
	}
}

template<typename T>
inline void* method_ptr(T f) {
	auto fref = f;
	void* fptr = (void*&)fref;
	return fptr;
}

template<typename T, typename E>
inline void remove_elem(std::vector<T>& vec, E elem) {
	auto it = std::find(vec.begin(), vec.end(), static_cast<T>(elem));
	if (it != vec.end()) {
		vec.erase(it);
	}
}

std::wstring strf(const wchar_t* format, ...);
std::wstring getDocumentsPath();
bool fileExists(LPCWSTR Path);
bool dirExists(LPCWSTR Path);
void ensureDirExists(const std::wstring& path);
