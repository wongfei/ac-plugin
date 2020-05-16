#pragma once

BEGIN_HOOK_OBJ(SlipStream)

	#define RVA_SlipStream_getSlipEffect 2796640
	#define RVA_SlipStream_setPosition 2797184

	static void _hook()
	{
		HOOK_METHOD_RVA(SlipStream, getSlipEffect); // TODO: how to test?
		HOOK_METHOD_RVA(SlipStream, setPosition); // TODO: how to test?
	}

	float _getSlipEffect(const vec3f& p);
	void _setPosition(const vec3f& pos, const vec3f& vel);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

float _SlipStream::_getSlipEffect(const vec3f& p)
{
	float fSlip = 0;

	vec3f vDelta = (p - this->triangle.points[0]);
	float fDeltaLen = vDelta.len();

	if (fDeltaLen < this->length)
	{
		vDelta.norm(fDeltaLen);
		float fDot = vDelta * this->dir;

		if (fDot <= 0.7f)
			fSlip = 0.0f;
		else
			fSlip = (((1.0f - (fDeltaLen / this->length)) * (fDot - 0.7f)) * 3.3333333f) * this->effectGainMult;
	}

	return fSlip;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SlipStream::_setPosition(const vec3f& pos, const vec3f& vel) // TODO: check
{
	this->triangle.points[0] = pos;

	float fVelLen = vel.len();
	this->dir = vel.get_norm(fVelLen) * -1.0f;

	float fLen = (fVelLen * this->speedFactor) * this->speedFactorMult;
	this->length = fLen;

	vec2f vNormXZ(vel.x, vel.z);
	vNormXZ.norm();

	/*
	fOffX = (((fZero * 0.0) - vn_z) * fLength) * 0.25;
	fOffY = (((vn_z * 0.0) - (vn_x * 0.0)) * fLength) * 0.25;
	fOffZ = ((vn_x - (fZero * 0.0)) * fLength) * 0.25;
  	*/
	vec3f vOff1(-vNormXZ[1], 0, vNormXZ[0]);
	vOff1 *= (fLen * 0.25f);

	/*
	this->triangle.points[1]
	v21 = ((vn_x * fLengthNeg) + pos->x) + fOffX;
	v22 = ((fZero * fLengthNeg) + pos->y) + fOffY;
	v24 = (vn_z * fLengthNeg + pos->z) + fOffZ;

	this->triangle.points[2]
	v25 = ((vn_x * fLengthNeg) + pos->x) - fOffX;
	v25y = ((fZero * fLengthNeg) + pos->y) - fOffY;
	v26 = (vn_z * fLengthNeg + pos->z) - fOffZ;
	*/
	vec3f vOff2((vNormXZ[0] * -fLen), 0, (vNormXZ[1] * -fLen));

	this->triangle.points[1] = pos + vOff2 + vOff1;
	this->triangle.points[2] = pos + vOff2 - vOff1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
