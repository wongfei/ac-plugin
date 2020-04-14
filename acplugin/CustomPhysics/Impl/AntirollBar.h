#pragma once

#define RVA_AntirollBar_step 2864704

void AntirollBar_step(AntirollBar* pThis, float dt)
{
	if (pThis->ctrl.ready)
		pThis->k = pThis->ctrl.eval();

	if (pThis->k > 0.0f)
	{
		mat44f mxBody = pThis->carBody->getWorldMatrix(0);
		vec3f vBodyM2(mxBody.M21, mxBody.M22, mxBody.M23);

		mat44f mxHub0 = pThis->hubs[0]->getHubWorldMatrix();
		mat44f mxHub1 = pThis->hubs[1]->getHubWorldMatrix();
		vec3f vHubWorld0(mxHub0.M41, mxHub0.M42, mxHub0.M43);
		vec3f vHubWorld1(mxHub1.M41, mxHub1.M42, mxHub1.M43);
		vec3f vHubLoc0 = pThis->carBody->worldToLocal(vHubWorld0);
		vec3f vHubLoc1 = pThis->carBody->worldToLocal(vHubWorld1);

		float fM2Len = sqrtf(vBodyM2 * vBodyM2);
		if (fM2Len != 0.0f) {
			vBodyM2 /= fM2Len;
		}

		float fDelta = vHubLoc1.y - vHubLoc0.y; // TODO: check this!
		float fDeltaK = fDelta * pThis->k;

		vec3f force(vBodyM2 * fDeltaK);
		pThis->hubs[0]->addForceAtPos(force, vHubWorld0, false, false);
		pThis->hubs[1]->addForceAtPos(force * -1.0f, vHubWorld1, false, false);

		pThis->carBody->addLocalForceAtLocalPos(vec3f(0, -fDeltaK, 0), vHubLoc0);
		pThis->carBody->addLocalForceAtLocalPos(vec3f(0, fDeltaK, 0), vHubLoc1);
	}
}
