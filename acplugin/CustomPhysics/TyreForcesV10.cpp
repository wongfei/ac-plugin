#include "precompiled.h"
#include "GameHooks.h"
#include "Tyre.inl"

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
