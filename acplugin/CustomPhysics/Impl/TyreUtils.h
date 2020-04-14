#pragma once

inline float calcSlipAngleRAD(float vy, float vx)
{
	if (vx != 0.0f)
		return atanf(-(vy / fabsf(vx)));
	return 0;
}

inline float calcCamberRAD(vec3f& roadNormal, mat44f& susMatrix)
{
	float fTmp = ((susMatrix.M12 * roadNormal.y) + (susMatrix.M11 * roadNormal.x)) + (susMatrix.M13 * roadNormal.z);
	if (fTmp <= -1.0f || fTmp >= 1.0f)
		return -1.5707964f;
	else
		return -asinf(fTmp);
}

inline float calcContactPatchLength(float radius, float deflection)
{
	float v = radius - deflection;
	if (v <= 0.0f || radius <= v)
		return 0.0f;
	else
		return sqrtf((radius * radius) - (v * v)) * 2.0f;
}
