#pragma once

#include <windows.h>
#include <d3d11.h>
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

extern void* _g_module;
inline void* _drva(size_t off) { return ((uint8_t*)_g_module) + off; }

namespace acsdk {

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

	inline void addHandler(void* key, std::function<void (const T&)> value) {
		handlers.push_back(std::pair<void*, std::function<void (const T&)> >(key, value));
	}
};

class OnValueChanged
{
public:
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
	T* attached;
	T oldValue;
	virtual void update(float);
};

template<typename T>
class ksgui_EventReplicator
{
public:
	Event<T>* destEvent;
	Event<T>* srcEvent;
};

class IVertexBuffer {
public:
	void* kid;
};

template<typename T>
class VertexBuffer : public IVertexBuffer {
public:
	virtual ~VertexBuffer();
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

#pragma warning(push)
#pragma warning(disable: 4512) // warning C4512: assignment operator could not be generated
#include "ac_gen.h"
#pragma warning(pop)

inline void vset(vec3f& v, float x, float y, float z) { v.x = x, v.y = y, v.z = z; }
inline vec3f makev(float x, float y, float z) { vec3f r; r.x = x, r.y = y, r.z = z; return r; }
inline vec3f vmul(const vec3f& a, float f) { return makev(a.x * f, a.y * f, a.z * f); }
inline vec3f vdiv(const vec3f& a, float f) { return makev(a.x / f, a.y / f, a.z / f); }
inline vec3f vadd(const vec3f& a, const vec3f& b) { return makev(a.x + b.x, a.y + b.y, a.z + b.z); }
inline vec3f vsub(const vec3f& a, const vec3f& b) { return makev(a.x - b.x, a.y - b.y, a.z - b.z); }
inline float vdot(const vec3f& a, const vec3f& b) { return (a.x * b.x + a.y * b.y + a.z * b.z); }
inline float vlen(const vec3f& v) { const float sqlen = vdot(v, v); return (sqlen > 0.001f ? sqrtf(sqlen) : 0); }
inline vec3f vnorm(const vec3f& v) { const float l = vlen(v); if (l > 0) return vmul(v, 1 / l); else return makev(0, 0, 0); }

} // namespace acsdk
