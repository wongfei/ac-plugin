#pragma once

#define RVA_mat44f_createFromAxisAngle 356768

mat44f mat44f_createFromAxisAngle(const vec3f& axis, float angle)
{
	mat44f res;

	float sin_a = sinf(angle);
	float cos_a = cosf(angle);
	float om_cos_a = 1.0f - cos_a;

	res.M11 = ((axis.x * axis.x) * om_cos_a) + cos_a;
	res.M22 = ((axis.y * axis.y) * om_cos_a) + cos_a;
	res.M33 = ((axis.z * axis.z) * om_cos_a) + cos_a;

	res.M12 = (axis.z * sin_a) + (axis.y * axis.x) * om_cos_a;
	res.M23 = (axis.x * sin_a) + (axis.z * axis.y) * om_cos_a;
	res.M31 = (axis.y * sin_a) + (axis.z * axis.x) * om_cos_a;

	res.M13 = (axis.z * axis.x) * om_cos_a - (axis.y * sin_a);
	res.M21 = (axis.y * axis.x) * om_cos_a - (axis.z * sin_a);
	res.M32 = (axis.z * axis.y) * om_cos_a - (axis.x * sin_a);
	
	res.M14 = 0;
	res.M24 = 0;
	res.M34 = 0;
	
	res.M41 = 0;
	res.M42 = 0;
	res.M43 = 0;
	res.M44 = 1;

	return res;
}
