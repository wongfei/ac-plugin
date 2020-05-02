#pragma once

#pragma warning(push)
#pragma warning(disable: 4244) // conversion from 'double' to 'float'
#include "CubicSpline.h"
#pragma warning(pop)

BEGIN_HOOK_OBJ(Curve)

	#define RVA_Curve_getValue 2124176
	#define RVA_Curve_getCubicSplineValue 2124016

	float _getValue(float ref);
	float _getCubicSplineValue(float ref);

END_HOOK_OBJ()

float _Curve::_getValue(float ref)
{
	if (references.empty())
		return 0.0f;

	if (ref <= references[0])
		return values[0];

	const size_t n = references.size();

	for (size_t id = 1; id < n; ++id)
	{
		if (ref <= references[id])
		{
			return (((values[id] - values[id - 1]) * (ref - references[id - 1])) / (references[id] - references[id - 1])) + values[id - 1];
		}
	}

	return values[n - 1];
}

struct HaxPtr
{
	union {
		struct {
			float f1, f2;
		};
		void* ptr;
	};
	inline HaxPtr(void* a) : ptr(a) {}
	inline HaxPtr(float a, float b) : f1(a), f2(b) {}
};

struct CubicSplineImpl
{
	tk::spline s;
};

float _Curve::_getCubicSplineValue(float ref)
{
	TODO_WTF_IS_THIS;

	const float magic = 666.0f;
	CubicSplineImpl* spline = nullptr;

	if (!this->cSpline.mElements.empty())
	{
		auto& elem = this->cSpline.mElements[0];
		if (elem.a == magic && elem.d == magic)
		{
			spline = (CubicSplineImpl*)(HaxPtr(elem.b, elem.c).ptr); // LOL
		}
	}

	if (!this->cubicSplineReady || !spline)
	{
		if (!spline)
			spline = new CubicSplineImpl();

		spline->s.set_points(this->references, this->values);
		
		HaxPtr p(spline);
		this->cSpline.mElements.clear();
		this->cSpline.mElements.push_back(CubicSpline<float, float>::Element { 0, magic, p.f1, p.f2, magic }); // x a b c d
		this->cubicSplineReady = true;
	}

	return spline ? (spline->s)(ref) : 0;
}
