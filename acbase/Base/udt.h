#pragma once

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
	inline void hook_##_method(_base_class* pbase) { vthook_set(UT_WSTRING(_base_class), _vfid, pbase, method_ptr(&_hook_class::thunk_##_method)); }

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

template<typename T>
struct stack_udt
{
	inline stack_udt(T* p) : _ptr(p) {}
	inline ~stack_udt() { if (_ptr) { _ptr->dtor(); _freea(_ptr); } }

	inline T** operator&() { return &_ptr; }

	inline T& operator*() { return *_ptr; }
	inline const T& operator*() const { return *_ptr; }

	inline T* operator->() { return _ptr; }
	inline const T* operator->() const { return _ptr; }

	inline T* ptr() { return _ptr; }
	inline const T* ptr() const { return _ptr; }

	inline T& ref() { return *_ptr; }
	inline const T& ref() const { return *_ptr; }

	inline T byval() const { return *_ptr; }

	T* _ptr;
};

template<typename T, typename ...Args>
inline stack_udt<T> stackalloc_udt(Args&&... params) {
	T* obj = (T*)_malloca(sizeof(T) + sizeof(void*));
	set_udt_tag(obj, nullptr);
	obj->ctor(std::forward<Args>(params)...);
	return stack_udt<T>(obj);
}

template<typename T, typename ...Args>
inline T* new_udt(Args&&... params) {
	T* obj = (T*)malloc(sizeof(T) + sizeof(void*));
	set_udt_tag(obj, nullptr);
	obj->ctor(std::forward<Args>(params)...);
	//new (obj) T(std::forward<Args>(params)...);
	return obj;
}

template<typename T>
inline void del_udt(T* obj) {
	if (obj) {
		obj->dtor();
		//obj->~T();
		free(obj);
	}
}

template<typename T, typename ...Args>
inline std::unique_ptr<T, void(*)(T*)> new_udt_unique(Args&&... params)
{
	return std::unique_ptr<T, void(*)(T*)>(new_udt<T>(std::forward<Args>(params)...), &del_udt<T>);
}

template<typename T, typename ...Args>
inline std::shared_ptr<T> new_udt_shared(Args&&... params)
{
	return std::shared_ptr<T>(new_udt<T>(std::forward<Args>(params)...), &del_udt<T>);
}
