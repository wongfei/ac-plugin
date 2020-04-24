#pragma once

BEGIN_HOOK_OBJ(AntirollBar)

	#define RVA_AntirollBar_step 2864704

	void _step(float dt);

END_HOOK_OBJ()

void _AntirollBar::_step(float dt)
{
	if (this->ctrl.ready)
		this->k = this->ctrl.eval();

	if (this->k > 0.0f)
	{
		mat44f mxBody = this->carBody->getWorldMatrix(0);
		vec3f vBodyM2(mxBody.M21, mxBody.M22, mxBody.M23);

		mat44f mxHub0 = this->hubs[0]->getHubWorldMatrix();
		mat44f mxHub1 = this->hubs[1]->getHubWorldMatrix();
		vec3f vHubWorld0(mxHub0.M41, mxHub0.M42, mxHub0.M43);
		vec3f vHubWorld1(mxHub1.M41, mxHub1.M42, mxHub1.M43);
		vec3f vHubLoc0 = this->carBody->worldToLocal(vHubWorld0);
		vec3f vHubLoc1 = this->carBody->worldToLocal(vHubWorld1);

		float fM2Len = sqrtf(vBodyM2 * vBodyM2);
		if (fM2Len != 0.0f) {
			vBodyM2 /= fM2Len;
		}

		float fDelta = vHubLoc1.y - vHubLoc0.y; // TODO: check this!
		float fDeltaK = fDelta * this->k;

		vec3f force(vBodyM2 * fDeltaK);
		this->hubs[0]->addForceAtPos(force, vHubWorld0, false, false);
		this->hubs[1]->addForceAtPos(force * -1.0f, vHubWorld1, false, false);

		this->carBody->addLocalForceAtLocalPos(vec3f(0, -fDeltaK, 0), vHubLoc0);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fDeltaK, 0), vHubLoc1);
	}
}
