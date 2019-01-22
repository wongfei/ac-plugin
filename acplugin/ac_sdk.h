#pragma once

#include <windows.h>
#include <shlobj.h>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <functional>
#include <vector>
#include <map>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <concurrent_queue.h>

#define AC_GUARD_METHOD_SIG(_class, _method, _ret, _args)\
struct guard_##_class##_##_method {\
	template <typename T, _ret (T::*)_args> struct enable { typedef T type; };\
	inline guard_##_class##_##_method() { static_assert(sizeof(enable<_class, &_class::_method>::type) == sizeof(_class), "invalid method"); }\
};

#define AC_OVERRIDE_METHOD(_class, _method, _ret, _args)\
	AC_GUARD_METHOD_SIG(_class, _method, _ret, _args)\
	_ret _method _args;

extern void* _g_module;
inline void* _drva(size_t off) { return ((uint8_t*)_g_module) + off; }

namespace acsdk {

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

template<typename T, typename ...Args>
inline T* new_udt(Args&&... params) {
	T* obj = (T*)malloc(sizeof(T));
	obj->ctor(std::forward<Args>(params)...);
	return obj;
}

template<typename T>
inline void del_udt(T* obj) {
	obj->dtor();
	free(obj);
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

inline std::wstring getDocumentsPath() {
	wchar_t buf[MAX_PATH];
	SHGetFolderPathW(0, 5, 0, 0, buf);
	return std::wstring(buf);
}

inline bool dirExists(LPCWSTR Path) {
	DWORD dwAttrib = GetFileAttributesW(Path);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

inline bool fileExists(LPCWSTR Path) {
	DWORD dwAttrib = GetFileAttributesW(Path);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && !(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

inline void ensureDirExists(const std::wstring& path) {
	if (!dirExists(path.c_str())) {
		CreateDirectoryW(path.c_str(), nullptr);
	}
}

//
// AC TYPES
//

struct _object {};

template<typename T>
class BufferedChannel
{
public:
	Concurrency::concurrent_queue<T> queue;
};

template<typename T>
class Event
{
public:
	std::vector<std::pair<void*, std::function<void (const T&)> > > handlers;
};

class OnValueChanged
{
	float currentValue;
	float oldValue;
};

class IEventTrigger
{
public:
	virtual void update(float) = 0;
};

template<typename T>
class EventTriggerOnChange : IEventTrigger, Event<OnValueChanged>
{
public:
	T *attached;
	T oldValue;
	virtual void update(float);
};

template<typename T>
class ksgui_EventReplicator
{
public:
	Event<T> *destEvent;
	Event<T> *srcEvent;
};

template<typename A, typename B>
class CubicSpline
{
public:
	class Element
	{
		float x;
		float a;
		float b;
		float c;
		float d;
	};
	std::vector<Element> mElements;
	virtual ~CubicSpline();
};

class IVertexBuffer {
public:
	void * kid;
	inline void _guard_obj() {
		static_assert((sizeof(IVertexBuffer)==8),"bad size");
		static_assert((offsetof(IVertexBuffer,kid)==0x0),"bad off");
	};
};

template<typename T>
class VertexBuffer : public IVertexBuffer {
public:
	virtual ~VertexBuffer();
};

#pragma warning(push)
#pragma warning(disable: 4512) // warning C4512: assignment operator could not be generated
#include "ac_gen.h"
#pragma warning(pop)

}
