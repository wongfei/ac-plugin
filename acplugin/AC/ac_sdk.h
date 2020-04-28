#pragma once

#define DEBUG_BREAK __debugbreak()
#define DEBUG_ASSERT assert

#define NOT_IMPLEMENTED DEBUG_ASSERT(false)
#define SHOULD_NOT_REACH DEBUG_ASSERT(false)

// IDA types

typedef uint8_t _BYTE;
typedef uint16_t _WORD;
typedef uint32_t _DWORD;
typedef uint64_t _QWORD;
typedef __m128 _OWORD;

// Proxy hacks

extern void* _ac_module;
__forceinline void* _drva(size_t off) { return ((uint8_t*)_ac_module) + off; }

// THIS IS SPARTA!!!
__forceinline intptr_t get_vtp(void* obj) { return *((intptr_t*)obj); }
__forceinline intptr_t get_vfp(void* obj, size_t id) { return *((intptr_t*)(get_vtp(obj) + id * sizeof(intptr_t))); }

// NUCLEAR MINEFIELD!!!
template<typename TOUT, typename TIN>
__forceinline TOUT xcast(TIN in)
{
    union
    {
        TIN in;
        TOUT out;
    }
    u = { in };
    return u.out;
}

// Common

struct _object {};

template<typename T>
class BufferedChannel
{
public:
	Concurrency::concurrent_queue<T> queue;
};

// Math

class vec2f {
public:
	float x;
	float y;

	inline vec2f() {}
	inline vec2f(const vec2f& v) : x(v.x), y(v.y) {}
	inline vec2f(float ix, float iy) : x(ix), y(iy) {}
	inline explicit vec2f(const float* v) : x(v[0]), y(v[1]) {}

	inline vec2f clone() const { return *this; }

	inline vec2f& operator=(const vec2f& v) { x = v.x, y = v.y; return *this; }
	inline float& operator[](const int id) { return (&x)[id]; }
	inline const float& operator[](const int id) const { return (&x)[id]; }

	inline vec2f operator*(const float f) const { return vec2f(x * f, y * f); }
	inline vec2f operator/(const float f) const { return vec2f(x / f, y / f); }
	inline vec2f operator+(const vec2f& v) const { return vec2f(x + v.x, y + v.y); }
	inline vec2f operator-(const vec2f& v) const { return vec2f(x - v.x, y - v.y); }
	inline float operator*(const vec2f& v) const { return (x * v.x + y * v.y); }

	inline vec2f& operator*=(const float f) { x *= f, y *= f; return *this; }
	inline vec2f& operator/=(const float f) { x /= f, y /= f; return *this; }
	inline vec2f& operator+=(const vec2f& v) { x += v.x, y += v.y; return *this; }
	inline vec2f& operator-=(const vec2f& v) { x -= v.x, y -= v.y; return *this; }

	inline float sqlen() const { return (x * x + y * y); }
	inline float len() const { return sqrtf(sqlen()); }

	inline vec2f& norm(float l) { if (l != 0.0f) { (*this) *= (1.0f / l); } return *this; }
	inline vec2f& norm() { return norm(len()); }

	inline vec2f get_norm(float l) const { return clone().norm(l); }
	inline vec2f get_norm() const { return clone().norm(); }
};

class vec3f {
public:
	float x;
	float y;
	float z;

	inline vec3f() {}
	inline vec3f(const vec3f& v) : x(v.x), y(v.y), z(v.z) {}
	inline vec3f(float ix, float iy, float iz) : x(ix), y(iy), z(iz) {}
	inline explicit vec3f(const float* v) : x(v[0]), y(v[1]), z(v[2]) {}

	inline vec3f clone() const { return *this; }

	inline vec3f& operator=(const vec3f& v) { x = v.x, y = v.y, z = v.z; return *this; }
	inline float& operator[](const int id) { return (&x)[id]; }
	inline const float& operator[](const int id) const { return (&x)[id]; }

	inline vec3f operator*(const float f) const { return vec3f(x * f, y * f, z * f); }
	inline vec3f operator/(const float f) const { return vec3f(x / f, y / f, z / f); }
	inline vec3f operator+(const vec3f& v) const { return vec3f(x + v.x, y + v.y, z + v.z); }
	inline vec3f operator-(const vec3f& v) const { return vec3f(x - v.x, y - v.y, z - v.z); }
	inline float operator*(const vec3f& v) const { return (x * v.x + y * v.y + z * v.z); }

	inline vec3f& operator*=(const float f) { x *= f, y *= f, z *= f; return *this; }
	inline vec3f& operator/=(const float f) { x /= f, y /= f, z /= f; return *this; }
	inline vec3f& operator+=(const vec3f& v) { x += v.x, y += v.y, z += v.z; return *this; }
	inline vec3f& operator-=(const vec3f& v) { x -= v.x, y -= v.y, z -= v.z; return *this; }

	inline float sqlen() const { return (x * x + y * y + z * z); }
	inline float len() const { return sqrtf(sqlen()); }

	inline vec3f& norm(float l) { if (l != 0.0f) { (*this) *= (1.0f / l); } return *this; }
	inline vec3f& norm() { return norm(len()); }

	inline vec3f get_norm(float l) const { return clone().norm(l); }
	inline vec3f get_norm() const { return clone().norm(); }

	inline vec3f cross(const vec3f& v) const { return vec3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
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

template<typename T>
class SignalGenerator3D
{
public:
	SignalGenerator3D() {}
	virtual ~SignalGenerator3D() {}
	vec3f freqScale;
	vec3f scale;
	float randomBlend;
	T sins[3];
};

// Event

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

// VB

class IVertexBuffer {
public:
	void* kid;
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

inline vec4f rgba(uint8_t r, uint8_t g, uint8_t b, float a) { 
	const float s = 1 / 255.0f;
	vec4f v; (v.x = r*s), (v.y = g*s), (v.z = b*s), (v.w = a);
	return v;
}

inline DirectX::XMMATRIX xmload(const mat44f& m) { return DirectX::XMMATRIX(&m.M11); }
inline mat44f xmstore(const DirectX::XMMATRIX& m) { DirectX::XMFLOAT4X4 f44; DirectX::XMStoreFloat4x4(&f44, m); return *(mat44f*)&f44._11; }

inline Speed getSpeed(Car* pCar) { return pCar->valueCache.speed; }
inline float getSpeedMS(Car* pCar) { return pCar->valueCache.speed.value; }
inline float getSpeedKMH(Car* pCar) { return getSpeedMS(pCar) * 3.6f; }
