#include "precompiled.h"
#include "GameHooks.h"
#include "Tyre.inl"

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

	mat44f hubM = pThis->hub->getHubWorldMatrix();
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
			RayCastHit hit = pThis->rayCaster->rayCast(rayPos, rayDir);
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

		mat44f mat = mat44f::createFromAxisAngle(axis, fAngle);

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

void Tyre_addGroundContact(Tyre* pThis, vec3f& pos, vec3f& normal)
{
	vec3f offset = vsub(pThis->worldPosition, pos);
	float fDistToGround = vlen(offset);
	pThis->status.distToGround = fDistToGround;

	float fRadius;
	if (pThis->data.radiusRaiseK == 0.0f)
		fRadius = pThis->data.radius;
	else
		fRadius = (fabsf(pThis->status.angularVelocity) * pThis->data.radiusRaiseK) + pThis->data.radius;

	if (pThis->status.inflation < 1.0f)
		fRadius = ((fRadius - pThis->data.rimRadius) * pThis->status.inflation) + pThis->data.rimRadius;

	pThis->status.liveRadius = fRadius;
	pThis->status.effectiveRadius = fRadius;

	if (fDistToGround > fRadius)
	{
		pThis->status.loadedRadius = fRadius;
		pThis->status.depth = 0;
		pThis->status.load = 0;
		pThis->status.Fy = 0.0;
		pThis->status.Fx = 0;
		pThis->status.Mz = 0;
		pThis->rSlidingVelocityX = 0;
		pThis->rSlidingVelocityY = 0;
		pThis->status.ndSlip = 0;
	}
	else
	{
		float fDepth = fRadius - fDistToGround;
		float fLoadedRadius = fRadius - fDepth;

		pThis->status.depth = fDepth;
		pThis->status.loadedRadius = fLoadedRadius;

		float fMaybePressure;
		if (fLoadedRadius <= pThis->data.rimRadius)
		{
			fMaybePressure = 200000.0f;
		}
		else
		{
			fMaybePressure = ((pThis->status.pressureDynamic - pThis->modelData.pressureRef) * pThis->modelData.pressureSpringGain) + pThis->data.k;
			if (fMaybePressure < 0.0f)
				fMaybePressure = 0.0f;
		}

		vec3f hubVel = pThis->hub->getPointVelocity(pos);
		float fLoad = -(vdot(hubVel, normal) * pThis->data.d) + (fDepth * fMaybePressure);
		pThis->status.load = fLoad;

		vec3f force = vmul(normal, fLoad);
		pThis->hub->addForceAtPos(force, pos, pThis->driven, false);

		if (pThis->status.load < 0.0f)
			pThis->status.load = 0.0f;
	}

	if (pThis->externalInputs.isActive)
		pThis->status.load = pThis->externalInputs.load;
}

void Tyre_addTyreForcesV10(Tyre* pThis, vec3f& pos, vec3f& normal, SurfaceDef* pSurface, float dt)
{
	mat44f& worldRotation = pThis->worldRotation;
	vec3f& roadHeading = pThis->roadHeading;
	vec3f& roadRight = pThis->roadRight;

	float fTmp = ((-worldRotation.M31 * normal.x) + (-worldRotation.M32 * normal.y)) + (-worldRotation.M33 * normal.z);
	roadHeading.x = -worldRotation.M31 - fTmp * normal.x;
	roadHeading.y = -worldRotation.M32 - fTmp * normal.y;
	roadHeading.z = -worldRotation.M33 - fTmp * normal.z;
	roadHeading = vnorm(roadHeading);

	fTmp = ((worldRotation.M11 * normal.x) + (worldRotation.M12 * normal.y)) + (worldRotation.M13 * normal.z);
	roadRight.x = worldRotation.M11 - fTmp * normal.x;
	roadRight.y = worldRotation.M12 - fTmp * normal.y;
	roadRight.z = worldRotation.M13 - fTmp * normal.z;
	roadRight = vnorm(roadRight);

	vec3f hubAngVel = pThis->hub->getHubAngularVelocity();
	vec3f hubPointVel = pThis->hub->getPointVelocity(pos);

	pThis->slidingVelocityY = ((hubPointVel.y * roadRight.y) + (hubPointVel.x * roadRight.x)) + (hubPointVel.z * roadRight.z);
	pThis->roadVelocityX = -(((hubPointVel.y * roadHeading.y) + (hubPointVel.x * roadHeading.x)) + (hubPointVel.z * roadHeading.z));
	float fSlipAngleTmp = calcSlipAngleRAD(pThis->slidingVelocityY, pThis->roadVelocityX);

	fTmp = (((hubAngVel.y * worldRotation.M12) + (hubAngVel.x * worldRotation.M11)) + (hubAngVel.z * worldRotation.M13)) + pThis->status.angularVelocity;
	pThis->slidingVelocityX = (fTmp * pThis->status.effectiveRadius) - pThis->roadVelocityX;
	float fRoadVelocityXAbs = fabsf(pThis->roadVelocityX);
	float fSlipRatioTmp = ((fRoadVelocityXAbs == 0.0f) ? 0.0f : (pThis->slidingVelocityX / fRoadVelocityXAbs));

	if (pThis->externalInputs.isActive)
	{
		fSlipAngleTmp = pThis->externalInputs.slipAngle;
		fSlipRatioTmp = pThis->externalInputs.slipRatio;
	}

	pThis->status.camberRAD = calcCamberRAD(pThis->contactNormal, worldRotation);
	pThis->totalHubVelocity = sqrtf((pThis->roadVelocityX * pThis->roadVelocityX) + (pThis->slidingVelocityY * pThis->slidingVelocityY));

	float fNdSlip = tclamp(pThis->status.ndSlip, 0.0f, 1.0f);
	float fLoadDivFz0 = pThis->status.load / pThis->modelData.Fz0;
	float fRelaxLen = pThis->modelData.relaxationLength;
	float fRelax1 = (((fLoadDivFz0 * fRelaxLen) - fRelaxLen) * 0.30000001f) + fRelaxLen;
	float fRelax2 = ((fRelaxLen - (fRelax1 * 2.0f)) * fNdSlip) + (fRelax1 * 2.0f);

	if (pThis->totalHubVelocity < 1.0f)
	{
		fSlipRatioTmp = pThis->slidingVelocityX * 0.5f;
		fSlipRatioTmp = tclamp(fSlipRatioTmp, -1.0f, 1.0f);

		fSlipAngleTmp = pThis->slidingVelocityY * -5.5f;
		fSlipAngleTmp = tclamp(fSlipAngleTmp, -1.0f, 1.0f);
	}

	float fSlipRatio = pThis->status.slipRatio;
	float fSlipRatioDelta = fSlipRatioTmp - fSlipRatio;
	float fNewSlipRatio = fSlipRatioTmp;

	if (fRelax2 != 0.0f)
	{
		float fSlipRatioScale = (pThis->totalHubVelocity * dt) / fRelax2;
		if (fSlipRatioScale <= 1.0f)
		{
			if (fSlipRatioScale < 0.039999999f)
				fNewSlipRatio = (0.039999999f * fSlipRatioDelta) + fSlipRatio;
			else
				fNewSlipRatio = (fSlipRatioScale * fSlipRatioDelta) + fSlipRatio;
		}
	}

	float fSlipAngle = pThis->status.slipAngleRAD;
	float fSlipAngleDelta = fSlipAngleTmp - fSlipAngle;
	float fNewSlipAngle = fSlipAngleTmp;

	if (fRelax2 != 0.0f)
	{
		float fSlipAngleScale = (pThis->totalHubVelocity * dt) / fRelax2;
		if (fSlipAngleScale <= 1.0f)
		{
			if (fSlipAngleScale < 0.039999999f)
				fNewSlipAngle = (0.039999999f * fSlipAngleDelta) + fSlipAngle;
			else
				fNewSlipAngle = (fSlipAngleScale * fSlipAngleDelta) + fSlipAngle;
		}
	}

	pThis->status.slipAngleRAD = fNewSlipAngle;
	pThis->status.slipRatio = fNewSlipRatio;

	if (pThis->status.load <= 0.0f)
	{
		pThis->status.slipAngleRAD = 0;
		pThis->status.slipRatio = 0;
	}

	float fCorrectedD = pThis->getCorrectedD(1.0, &pThis->status.wearMult);
	float fDynamicGripLevel = pThis->car ? pThis->car->ksPhysics->track->dynamicGripLevel : 1.0f;

	TyreModelInput tmi;
	tmi.load = pThis->status.load;
	tmi.slipAngleRAD = pThis->status.slipAngleRAD;
	tmi.slipRatio = pThis->status.slipRatio;
	tmi.camberRAD = pThis->status.camberRAD;
	tmi.speed = pThis->totalHubVelocity;
	tmi.u = (fCorrectedD * pSurface->gripMod) * fDynamicGripLevel;
	tmi.tyreIndex = pThis->index;
	tmi.cpLength = calcContactPatchLength(pThis->status.liveRadius, pThis->status.depth);
	tmi.grain = (float)pThis->status.grain;
	tmi.blister = (float)pThis->status.blister;
	tmi.pressureRatio = (pThis->status.pressureDynamic / pThis->modelData.idealPressure) - 1.0f;
	tmi.useSimpleModel = 1.0f < pThis->aiMult;

	TyreModelOutput tmo = pThis->tyreModel->solve(tmi);

	pThis->status.Fy = tmo.Fy * pThis->aiMult;
	pThis->status.Fx = -tmo.Fx;
	pThis->status.Dy = tmo.Dy;
	pThis->status.Dx = tmo.Dx;

	float fHubSpeed = pThis->status.effectiveRadius * pThis->status.angularVelocity;

	pThis->stepDirtyLevel(dt, fabsf(fHubSpeed));
	pThis->stepPuncture(dt, pThis->totalHubVelocity);
	pThis->status.Mz = tmo.Mz;

	vec3f force;
	force.x = pThis->status.Fy * roadRight.x + pThis->status.Fx * roadHeading.x;
	force.y = pThis->status.Fy * roadRight.y + pThis->status.Fx * roadHeading.y;
	force.z = pThis->status.Fy * roadRight.z + pThis->status.Fx * roadHeading.z;

	vec3f test = force;
	if (_fdtest(&test.x) > 0 || _fdtest(&test.y) > 0 || _fdtest(&test.z) > 0)
	{
		log_printf(L"ERR: Tyre_addTyreForcesV10: force is NAN");
	}

	if (pThis->car && pThis->car->torqueModeEx != TorqueModeEX::original)
	{
		pThis->addTyreForceToHub(pos, force);
	}
	else
	{
		pThis->hub->addForceAtPos(force, pos, pThis->driven, true);
		pThis->localMX = -(pThis->status.loadedRadius * pThis->status.Fx);
	}

	vec3f torq = vmul(normal, tmo.Mz);
	pThis->hub->addTorque(torq);

	float fAngularVelocityAbs = fabsf(pThis->status.angularVelocity);
	if (fAngularVelocityAbs > 1.0f)
	{
		fHubSpeed = pThis->status.effectiveRadius * pThis->status.angularVelocity;
		float fHubSpeedSign = signf(fHubSpeed);

		float fPressureDynamic = pThis->status.pressureDynamic;
		float fPressureUnk = (((pThis->modelData.idealPressure / fPressureDynamic) - 1.0f) * pThis->modelData.pressureRRGain) + 1.0f;

		if (fPressureDynamic <= 0.0f)
			fPressureUnk = 0.0f;

		float fRrUnk = ((((fHubSpeed * fHubSpeed) * pThis->modelData.rr1) + pThis->modelData.rr0) * fHubSpeedSign) * fPressureUnk;

		if (fAngularVelocityAbs > 20.0f)
		{
			float fSlipUnk = 0;
			if (pThis->modelData.version < 2)
			{
				float fRrSrUnk = fPressureUnk * pThis->modelData.rr_sr;
				float fRrSaUnk = fabsf(pThis->status.slipAngleRAD * 57.29578f) * (fPressureUnk * pThis->modelData.rr_sa);

				float fSlipRatioAbs = fabsf(pThis->status.slipRatio);
				float fSlipRatioNorm = tclamp(fSlipRatioAbs, 0.0f, 1.0f);

				fSlipUnk = (fSlipRatioNorm * fRrSrUnk) + fRrSaUnk;
			}
			else
			{
				float fNdSlipNorm = tclamp(pThis->status.ndSlip, 0.0f, 1.0f);
				fSlipUnk = (fNdSlipNorm * pThis->modelData.rr_slip) * fPressureUnk;
			}

			fRrUnk = fRrUnk * ((fSlipUnk * 0.001f) + 1.0f);
		}

		pThis->status.rollingResistence = -(((pThis->status.load * 0.001f) * fRrUnk) * pThis->status.effectiveRadius);
	}

	if (pThis->car)
	{
		float svx = pThis->slidingVelocityX;
		float svy = pThis->slidingVelocityY;
		float fSlidingVelocity = sqrtf(svx * svx + svy * svy);

		float fLoadVKM = 1.0f;
		if (pThis->useLoadForVKM)
			fLoadVKM = pThis->status.load / pThis->modelData.Fz0;

		pThis->status.virtualKM = (((fSlidingVelocity * dt) * pThis->car->ksPhysics->tyreConsumptionRate) * fLoadVKM) * 0.001 + pThis->status.virtualKM;
	}

	float fStaticDy = pThis->scTM.getStaticDY(pThis->status.load);

	pThis->status.ndSlip = tmo.ndSlip;
	pThis->status.D = fStaticDy;
}
