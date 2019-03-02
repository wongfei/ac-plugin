#include "precompiled.h"
#include "GameHooks.h"

void Tyre_step(Tyre* pThis, float dt)
{
	pThis->status.feedbackTorque = 0.0f;
	pThis->status.Fx = 0.0f;
	pThis->status.Mz = 0.0f;
	pThis->status.slipFactor = 0.0f;
	pThis->status.rollingResistence = 0.0f;
	pThis->slidingVelocityY = 0.0f;
	pThis->slidingVelocityX = 0.0f;
	pThis->totalSlideVelocity = 0.0f;
	pThis->totalHubVelocity = 0.0f;
	pThis->surfaceDef = nullptr;

	auto hubM = pThis->hub->getHubWorldMatrix();
	pThis->worldRotation = hubM;

	pThis->worldPosition.x = pThis->worldRotation.M41;
	pThis->worldPosition.y = pThis->worldRotation.M42;
	pThis->worldPosition.z = pThis->worldRotation.M43;
	pThis->worldRotation.M41 = 0.0f;
	pThis->worldRotation.M42 = 0.0f;
	pThis->worldRotation.M43 = 0.0f;

	float fFdTest = pThis->status.angularVelocity;
	if (_fdtest(&fFdTest) > 0)
	{
		log_printf(L"ERR: Tyre_step: angularVelocity is NAN");
		pThis->status.angularVelocity = 0.0f;
	}

	if (!pThis->status.isLocked)
	{
		if (pThis->car)
		{
			if (pThis->car->torqueModeEx == TorqueModeEX::reactionTorques)
			{
				float fTorq = (pThis->inputs.electricTorque + pThis->inputs.brakeTorque) + pThis->inputs.handBrakeTorque;
				vec3f torq;
				torq.x = pThis->worldRotation.M11 * fTorq;
				torq.y = pThis->worldRotation.M12 * fTorq;
				torq.z = pThis->worldRotation.M13 * fTorq;
				pThis->hub->addTorque(torq);
			}
		}
	}

	vec3f worldPos = pThis->worldPosition;
	vec3f hitPos = makev(0, 0, 0);
	vec3f hitNorm = makev(0, 0, 0);
	ICollisionObject* pCollisionObject = nullptr;
	SurfaceDef* pSurface = nullptr;
	bool bHasContact = false;

	auto pCollisionProvider = pThis->rayCollisionProvider;
	if (pCollisionProvider)
	{
		vec3f rayPos = makev(worldPos.x, worldPos.y + 2.0f, worldPos.z);
		vec3f rayDir = makev(0.0f, -1.0f, 0.0f);

		if (pThis->rayCaster)
		{
			auto hit = pThis->rayCaster->rayCast(rayPos, rayDir);
			bHasContact = hit.hasContact;

			if (bHasContact)
			{
				hitPos = hit.pos;
				hitNorm = hit.normal;
				pCollisionObject = hit.collisionObject;
				pSurface = (SurfaceDef*)pCollisionObject->getUserPointer();
			}
		}
		else
		{
			RayCastResult hit;
			pCollisionProvider->rayCast(rayPos, rayDir, &hit, 2.0f);
			bHasContact = hit.hasHit;

			if (bHasContact)
			{
				hitPos = hit.pos;
				hitNorm = hit.normal;
				pCollisionObject = (ICollisionObject*)hit.collisionObject;
				pSurface = hit.surfaceDef;
			}
		}
	}

	if (!bHasContact || pThis->worldRotation.M22 <= 0.34999999f)
	{
		pThis->status.ndSlip = 0.0f;
		pThis->status.Fy = 0.0f;
		goto LB_COMPUTE_TORQ;
	}

	pThis->surfaceDef = pSurface;
	pThis->unmodifiedContactPoint = hitPos;

	float fTest = hitNorm.x * pThis->worldRotation.M21 + hitNorm.y * pThis->worldRotation.M22 + hitNorm.z * pThis->worldRotation.M23;
	if (fTest <= 0.95999998f)
	{
		float fTestAcos;
		if (fTest <= -1.0f || fTest >= 1.0f)
			fTestAcos = 0.0f;
		else
			fTestAcos = acosf(fTest);

		float fAngle = fTestAcos - acosf(0.95999998f);

		vec3f axis;
		axis.x = (pThis->worldRotation.M23 * hitNorm.y) - (pThis->worldRotation.M22 * hitNorm.z);
		axis.y = (pThis->worldRotation.M21 * hitNorm.z) - (pThis->worldRotation.M23 * hitNorm.x);
		axis.z = (pThis->worldRotation.M22 * hitNorm.x) - (pThis->worldRotation.M21 * hitNorm.y);
		axis = vnorm(axis);

		auto mat = mat44f::createFromAxisAngle(axis, fAngle);

		float nx = (((mat.M11 * hitNorm.x) + (mat.M21 * hitNorm.y)) + (mat.M31 * hitNorm.z)) + mat.M41;
		float ny = (((mat.M12 * hitNorm.x) + (mat.M22 * hitNorm.y)) + (mat.M32 * hitNorm.z)) + mat.M42;
		float nz = (((mat.M13 * hitNorm.x) + (mat.M23 * hitNorm.y)) + (mat.M33 * hitNorm.z)) + mat.M43;

		hitNorm = makev(nx, ny, nz);

		pThis->contactPoint = hitPos;
	}
	else
	{
		vec3f hitOff = vsub(hitPos, pThis->worldPosition);
		float fDot = (hitNorm.x * hitOff.x) + (hitNorm.y * hitOff.y) + (hitNorm.z * hitOff.z);

		hitPos.x = (hitNorm.x * fDot) + pThis->worldPosition.x;
		hitPos.y = (hitNorm.y * fDot) + pThis->worldPosition.y;
		hitPos.z = (hitNorm.z * fDot) + pThis->worldPosition.z;

		pThis->contactPoint = hitPos;
	}

	if (pSurface)
	{
		float fSinHeight = pSurface->sinHeight;
		if (fSinHeight != 0.0f)
		{
			float fSinLength = pSurface->sinLength;
			pThis->contactPoint.y -= (((sinf(fSinLength * pThis->contactPoint.x) * cosf(fSinLength * pThis->contactPoint.z)) + 1.0f) * fSinHeight);
		}
	}

	if (pSurface->granularity != 0.0f)
	{
		float v1[3] = { 1.0f, 5.8000002f, 11.4f };
		float v2[3] = { 0.0049999999f, 0.0049999999f, 0.0099999998f };

		float cx = pThis->contactPoint.x;
		float cy = pThis->contactPoint.y;
		float cz = pThis->contactPoint.z;

		for (int id = 0; id < 3; ++id)
		{
			float v = v1[id];
			cy = cy + ((((sinf(v * cx) * cosf(v * cz)) + 1.0f) * v2[id]) * -0.60000002f);
		}

		pThis->contactPoint.y = cy;
	}

	pThis->contactNormal = hitNorm;
	pThis->addGroundContact(pThis->contactPoint, pThis->contactNormal);

	if (pThis->modelData.version < 10)
		pThis->addTyreForces(pThis->contactPoint, pThis->contactNormal, pSurface, dt);
	else
		pThis->addTyreForcesV10(pThis->contactPoint, pThis->contactNormal, pSurface, dt);

	if (pSurface)
	{
		float fDamping = pSurface->damping;
		if (fDamping > 0.0f)
		{
			auto bodyVel = pThis->car->body->getVelocity();
			float fMass = pThis->car->body->getMass();
			vec3f force = vmul(bodyVel, fMass * -fDamping);
			vec3f pos = makev(0, 0, 0);

			pThis->car->body->addForceAtLocalPos(force, pos);
		}
	}

LB_COMPUTE_TORQ:

	float fHandBrakeTorque = pThis->inputs.handBrakeTorque;
	float fBrakeTorque = pThis->inputs.brakeTorque * pThis->absOverride;

	if (fBrakeTorque <= fHandBrakeTorque)
		fBrakeTorque = fHandBrakeTorque;

	float fAngularVelocitySign = signf(pThis->status.angularVelocity);
	float fTorq = 0.0f;

	if (pThis->modelData.version < 10)
	{
		fTorq = ((pThis->status.loadedRadius * pThis->status.Fx) - (fAngularVelocitySign * fBrakeTorque)) + pThis->status.rollingResistence;
	}
	else
	{
		fTorq = pThis->status.rollingResistence - ((fAngularVelocitySign * fBrakeTorque) + pThis->localMX);
	}

	float fFeedbackTorque = fTorq + pThis->inputs.electricTorque;
	fFdTest = fFeedbackTorque;
	if (_fdtest(&fFdTest) > 0)
	{
		log_printf(L"ERR: Tyre_step: feedbackTorque is NAN");
		fFeedbackTorque = 0.0f;
	}

	pThis->status.feedbackTorque = fFeedbackTorque;

	if (pThis->driven)
	{
		pThis->updateLockedState(dt);

		float fOldAngularVelocitySign = signf(pThis->oldAngularVelocity);
		fAngularVelocitySign = signf(pThis->status.angularVelocity);

		if (fOldAngularVelocitySign != fAngularVelocitySign && pThis->totalHubVelocity < 1.0)
			pThis->status.isLocked = 1;

		pThis->oldAngularVelocity = pThis->status.angularVelocity;
	}
	else
	{
		pThis->updateAngularSpeed(dt);
		pThis->stepRotationMatrix(dt);
	}

	float fTotalHubVelocity = pThis->totalHubVelocity;
	if (fTotalHubVelocity < 10.0f)
		pThis->status.slipFactor = fabsf(fTotalHubVelocity * 0.1f) * pThis->status.slipFactor;

	pThis->stepThermalModel(dt);

	pThis->status.pressureDynamic = ((pThis->thermalModel.coreTemp - 26.0f) * pThis->pressureTemperatureGain) + pThis->status.pressureStatic;

	pThis->stepGrainBlister(dt, pThis->totalHubVelocity);
	pThis->stepFlatSpot(dt, pThis->totalHubVelocity);

	if (pThis->onStepCompleted)
	{
		pThis->onStepCompleted();
	}
}
