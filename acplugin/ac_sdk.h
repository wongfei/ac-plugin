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
		handlers.push_back(std::pair<void*, std::function<void (const ksgui_OnControlClicked&)> >(key, value));
	}
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
