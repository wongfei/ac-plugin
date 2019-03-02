#pragma once

extern void* _ac_module;
inline void* _drva(size_t off) { return ((uint8_t*)_ac_module) + off; }

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
	typedef std::function<void (const T&)> Callback;
	typedef std::pair<void*, Callback> Entry;
	typedef std::vector<Entry> Vec;
	Vec handlers;

	inline void add(void* key, Callback value) {
		handlers.push_back(Entry(key, value));
	}

	inline void swapAndPop(void* key) {
		if (handlers.size() > 1) {
			for (auto iter = handlers.begin(); iter != handlers.end(); ++iter) {
				if (iter->first == key) {
					std::iter_swap(iter, handlers.end() - 1);
					handlers.pop_back();
					break;
				}
			}
		}
		else {
			handlers.clear();
		}
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

template<typename T1, typename T2>
class CubicSpline
{
public:
	class Element
	{
		T1 x;
		T2 a;
		T2 b;
		T2 c;
		T2 d;
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
inline float vlen(const vec3f& v) { const float sqlen = vdot(v, v); return (sqlen != 0.0f ? sqrtf(sqlen) : 0.0f); }
inline vec3f vnorm(const vec3f& v) { const float len = vlen(v); return (len != 0.0f ? vmul(v, 1.0f / len) : makev(0, 0, 0)); }

inline vec4f rgba(uint8_t r, uint8_t g, uint8_t b, float a) { 
	const float s = 1 / 255.0f;
	vec4f v; (v.x = r*s), (v.y = g*s), (v.z = b*s), (v.w = a);
	return v;
}
