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
		mat44f mxBodyWorld = this->carBody->getWorldMatrix(0);
		vec3f vBodyM2(&mxBodyWorld.M21);

		mat44f mxHub0 = this->hubs[0]->getHubWorldMatrix();
		vec3f vHubWorld0(&mxHub0.M41);
		vec3f vHubLoc0 = this->carBody->worldToLocal(vHubWorld0);

		mat44f mxHub1 = this->hubs[1]->getHubWorldMatrix();
		vec3f vHubWorld1(&mxHub1.M41);
		vec3f vHubLoc1 = this->carBody->worldToLocal(vHubWorld1);

		float fDelta = vHubLoc1.y - vHubLoc0.y;
		float fDeltaK = fDelta * this->k;

		vec3f vForce = vBodyM2.get_norm() * fDeltaK;

		this->hubs[0]->addForceAtPos(vForce, vHubWorld0, false, false);
		this->hubs[1]->addForceAtPos(vForce * -1.0f, vHubWorld1, false, false);

		this->carBody->addLocalForceAtLocalPos(vec3f(0, -fDeltaK, 0), vHubLoc0);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fDeltaK, 0), vHubLoc1);
	}
}
