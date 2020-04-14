#pragma once

// IDA types
typedef uint8_t _BYTE;
typedef uint16_t _WORD;
typedef uint32_t _DWORD;
typedef uint64_t _QWORD;
typedef __m128 _OWORD;

#define DEBUG_BREAK __debugbreak()
#define DEBUG_ASSERT assert

#define NOT_IMPLEMENTED DEBUG_ASSERT(false)
#define SHOULD_NOT_REACH DEBUG_ASSERT(false)

extern void* _ac_module;
inline void* _drva(size_t off) { return ((uint8_t*)_ac_module) + off; }

struct _object {};

//UDT: class vec3f @len=12
//_Data: this+0x0, Member, Type: float, x
//_Data: this+0x4, Member, Type: float, y
//_Data: this+0x8, Member, Type: float, z
//_Func: public void vec3f(double *  _arg0); @loc=optimized @len=0 @rva=0
//_Func: public void vec3f(float *  _arg0); @loc=optimized @len=0 @rva=0
//_Func: public void vec3f(float ix, float iy, float iz); @loc=static @len=18 @rva=147424
//_Func: public void vec3f(float  _arg0); @loc=optimized @len=0 @rva=0
//_Func: public void vec3f(); @loc=optimized @len=0 @rva=0
//_Func: public void normalize(); @loc=static @len=136 @rva=147456
//_Func: public bool isZero(); @loc=optimized @len=0 @rva=0
//_Func: public std::basic_string<wchar_t,std::char_traits<wchar_t>,std::allocator<wchar_t> > toString(); @loc=static @len=322 @rva=340976
//_Func: public float length(); @loc=optimized @len=0 @rva=0
//_Func: public float lengthSquared(); @loc=optimized @len=0 @rva=0
//_Func: public bool isFinite(); @loc=static @len=105 @rva=418800
//_Func: public vec3f negate(); @loc=optimized @len=0 @rva=0
//_Func: public void operator+=(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
//_Func: public void operator-=(vec3f &  _arg0); @loc=optimized @len=0 @rva=0
//_Func: public void operator*=(float  _arg0); @loc=optimized @len=0 @rva=0
//_Func: public void operator/=(float m); @loc=static @len=47 @rva=908800
//_Func: public void print(char * name); @loc=static @len=61 @rva=918176
//UDT;

class vec3f {
public:
	float x;
	float y;
	float z;

	inline vec3f() {}
	inline vec3f(const vec3f& v) : x(v.x), y(v.y), z(v.z) {}
	inline vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

	inline vec3f& operator=(const vec3f& v) { x = v.x, y = v.y, z = v.z; return *this; }

	inline vec3f operator*(const float f) const { return vec3f(x * f, y * f, z * f); }
	inline vec3f operator/(const float f) const { return vec3f(x / f, y / f, z / f); }

	inline vec3f operator+(const vec3f& v) const { return vec3f(x + v.x, y + v.y, z + v.z); }
	inline vec3f operator-(const vec3f& v) const { return vec3f(x - v.x, y - v.y, z - v.z); }
	inline float operator*(const vec3f& v) const { return (x * v.x + y * v.y + z * v.z); }

	inline vec3f& operator*=(const float f) { x *= f, y *= f, z *= f; return *this; }
	inline vec3f& operator/=(const float f) { x /= f, y /= f, z /= f; return *this; }

	inline vec3f& operator+=(const vec3f& v) { x += v.x, y += v.y, z += v.z; return *this; }
	inline vec3f& operator-=(const vec3f& v) { x -= v.x, y -= v.y, z -= v.z; return *this; }
};

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
	//uint8_t dummy[0x58];
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

inline DirectX::XMMATRIX xmload(const mat44f& m) { return DirectX::XMMATRIX(&m.M11); }
inline mat44f xmstore(const DirectX::XMMATRIX& m) { DirectX::XMFLOAT4X4 f44; DirectX::XMStoreFloat4x4(&f44, m); return *(mat44f*)&f44._11; }

inline float getSpeedV(Car* pCar) { return pCar->valueCache.speed.value; }
