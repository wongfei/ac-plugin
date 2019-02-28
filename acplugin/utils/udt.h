#pragma once

#include "vthook.h"

#define UDT_GUARD_METHOD_SIG(_class, _method, _ret, _args)\
struct guard_##_class##_##_method {\
	template <typename T, _ret (T::*)_args> struct enable { typedef T type; };\
	inline guard_##_class##_##_method() { static_assert(sizeof(enable<_class, &_class::_method>::type) == sizeof(_class), "invalid method"); }\
};

#define UDT_GET_PTHIS(_hook_class, _base_class, _pthis)\
	((_hook_class*)get_udt_tag((_base_class*)((void*)_pthis)))

// _base_class->vfptr->thunk->_hook_class->vfptr->method
#define UDT_OVERRIDE_METHOD(_hook_class, _base_class, _sig_class, _method, _vfid, _ret, _args, _arg_names)\
	UDT_GUARD_METHOD_SIG(_sig_class, _method, _ret, _args)\
	virtual _ret _method _args;\
	__declspec(noinline) _ret thunk_##_method _args {\
		auto pthis = UDT_GET_PTHIS(_hook_class, _base_class, this);\
		return pthis->##_method##_arg_names;\
	}\
	inline void hook_##_method(_base_class* pbase) { vtablehook_hook(pbase, method_ptr(&_hook_class::thunk_##_method), _vfid); }

inline void* mem_off(void* mem, size_t off) {
	return (void*)(((uint8_t*)mem) + off);
}

template<typename T>
inline void* method_ptr(T f) {
	auto fref = f;
	void* fptr = (void*&)fref;
	return fptr;
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
	if (obj) {
		obj->dtor();
		free(obj);
	}
}

template<typename T>
class scoped_udt
{
public:
	inline scoped_udt() : _udt(nullptr) {}
	inline scoped_udt(T* udt) : _udt(udt) {}
	inline ~scoped_udt() { del_udt(_udt); }

	inline T* operator->() { return _udt; }
	inline const T* operator->() const { return _udt; }

	inline T* get() { return _udt; }
	inline const T* get() const { return _udt; }

private:
	T* _udt;
};
