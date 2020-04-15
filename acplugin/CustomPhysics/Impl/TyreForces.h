#pragma once

#define RVA_Tyre_addTyreForcesV10 2616672
#define RVA_Tyre_stepRelaxationLength 2640448
#define RVA_Tyre_getCorrectedD 2621840
#define RVA_Tyre_stepDirtyLevel 2638800
#define RVA_Tyre_stepPuncture 2640304
#define RVA_Tyre_addTyreForceToHub 2612224

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_addTyreForcesV10(Tyre* pThis, const vec3f& pos, const vec3f& normal, SurfaceDef* pSurface, float dt)
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
	float fRelax1 = (((fLoadDivFz0 * fRelaxLen) - fRelaxLen) * 0.3f) + fRelaxLen;
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
			if (fSlipRatioScale < 0.04f)
				fNewSlipRatio = (0.04f * fSlipRatioDelta) + fSlipRatio;
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
			if (fSlipAngleScale < 0.04f)
				fNewSlipAngle = (0.04f * fSlipAngleDelta) + fSlipAngle;
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

	// TODO: ITyreModel::solve crashes
	TyreModelOutput tmo = ((SCTM*)(pThis->tyreModel))->solve_impl(tmi);

	pThis->status.Fy = tmo.Fy * pThis->aiMult;
	pThis->status.Fx = -tmo.Fx;
	pThis->status.Dy = tmo.Dy;
	pThis->status.Dx = tmo.Dx;

	float fHubSpeed = pThis->status.effectiveRadius * pThis->status.angularVelocity;

	pThis->stepDirtyLevel(dt, fabsf(fHubSpeed));
	pThis->stepPuncture(dt, pThis->totalHubVelocity);
	pThis->status.Mz = tmo.Mz;

	vec3f force(
		pThis->status.Fy * roadRight.x + pThis->status.Fx * roadHeading.x,
		pThis->status.Fy * roadRight.y + pThis->status.Fx * roadHeading.y,
		pThis->status.Fy * roadRight.z + pThis->status.Fx * roadHeading.z);

	if (!(isfinite(force.x) && isfinite(force.y) && isfinite(force.z)))
	{
		log_printf(L"INF vTyreForce");
		force = vec3f(0, 0, 0);
	}

	if (pThis->car && pThis->car->torqueModeEx != TorqueModeEX::original)
	{
		DEBUG_BREAK; // TODO: what car uses this code?

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

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepRelaxationLength(Tyre* pThis, float svx, float svy, float hubVelocity, float dt)
{
	float fRelaxLen = pThis->modelData.relaxationLength;
	float fNdSlip = tclamp<float>(pThis->status.ndSlip, 0.0f, 1.0f);

	float v8 = ((((pThis->status.load / pThis->modelData.Fz0) * fRelaxLen) - fRelaxLen) * 0.3f) + fRelaxLen;
	float v9 = ((fRelaxLen - (v8 * 2.0f)) * fNdSlip) + (v8 * 2.0f);

	if (v9 == 0.0f)
	{
		pThis->rSlidingVelocityX = 0;
		pThis->rSlidingVelocityY = 0;
	}
	else
	{
		float v10 = (hubVelocity * dt) / v9;
		if (v10 < 1.0f) // TODO: not sure
		{
			if (v10 < 0.04f)
				v10 = 0.04f;

			pThis->rSlidingVelocityY = (v10 * (svy - pThis->rSlidingVelocityY)) + pThis->rSlidingVelocityY;
			pThis->rSlidingVelocityX = (v10 * (svx - pThis->rSlidingVelocityX)) + pThis->rSlidingVelocityX;
		}
		else
		{
			pThis->rSlidingVelocityX = svx;
			pThis->rSlidingVelocityY = svy;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float Tyre_getCorrectedD(Tyre* pThis, float d, float* wear_mult)
{
	float fD = pThis->thermalModel.getCorrectedD(d, pThis->status.camberRAD) / ((fabsf(pThis->status.pressureDynamic - pThis->modelData.idealPressure) * pThis->modelData.pressureGainD) + 1.0f);

	if (pThis->modelData.wearCurve.getCount())
	{
		float fWear = pThis->modelData.wearCurve.getValue((float)pThis->status.virtualKM);
		fD = fD * fWear;

		if (wear_mult)
			*wear_mult = fWear;
	}

	return fD;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepDirtyLevel(Tyre* pThis, float dt, float hubSpeed)
{
	if (pThis->status.dirtyLevel < 5.0f)
		pThis->status.dirtyLevel = (((hubSpeed * pThis->surfaceDef->dirtAdditiveK) * 0.03f) * dt) + pThis->status.dirtyLevel;

	if (pThis->surfaceDef->dirtAdditiveK == 0.0f)
	{
		if (pThis->status.dirtyLevel > 0.0f)
			pThis->status.dirtyLevel = pThis->status.dirtyLevel - ((hubSpeed * 0.015f) * dt);

		if (pThis->status.dirtyLevel < 0.0f)
			pThis->status.dirtyLevel = 0.0f;
	}

	if (pThis->aiMult == 1.0f)
	{
		float fDirty = 1.0f - tclamp<float>(pThis->status.dirtyLevel * 0.05f, 0.0f, 1.0f);
		float fScale = tmax<float>(0.8f, fDirty);

		pThis->status.Mz = fScale * pThis->status.Mz;
		pThis->status.Fx = fScale * pThis->status.Fx;
		pThis->status.Fy = fScale * pThis->status.Fy;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepPuncture(Tyre* pThis, float dt, float hubSpeed)
{
	if (pThis->car && pThis->car->ksPhysics->mechanicalDamageRate > 0.0f)
	{
		float imo[3];
		pThis->thermalModel.getIMO(imo);

		float fTemp = pThis->explosionTemperature;

		if (imo[0] > fTemp || imo[1] > fTemp || imo[2] > fTemp)
			pThis->status.inflation = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_addTyreForceToHub(Tyre* pThis, const vec3f& pos, const vec3f& force) // TODO: epic shit
{
	vec3f vWP = pThis->worldPosition;
	float fWorldX = pThis->worldPosition.x;
	float fWorldY = pThis->worldPosition.y;
	float fWorldZ = pThis->worldPosition.z;

	mat44f mxWR = pThis->worldRotation;
	float* M1 = &mxWR.M11;
	float* M2 = &mxWR.M21;
	float* M3 = &mxWR.M31;
	float* M4 = &mxWR.M41;

	//M4.m128_u64[0] = __PAIR64__(LODWORD(fWorldY), LODWORD(fWorldX));
	//M4.m128_f32[2] = fWorldZ;
	M4[0] = fWorldX;
	M4[1] = fWorldY;
	M4[2] = fWorldZ;

	DirectX::XMVECTOR xmvDet;
	mat44f mxInv = xmstore(DirectX::XMMatrixInverse(&xmvDet, xmload(mxWR)));

	float fNewForceX = (force.y * mxInv.M21) + (force.x * mxInv.M11) + (force.z * mxInv.M31);
	float fNewForceY = (force.y * mxInv.M22) + (force.x * mxInv.M12) + (force.z * mxInv.M32);
	float fNewForceZ = (force.y * mxInv.M23) + (force.x * mxInv.M13) + (force.z * mxInv.M33);

	float fNewPosX = (pos.y * mxInv.M21) + (pos.x * mxInv.M11) + (pos.z * mxInv.M31) + mxInv.M41;
	float fNewPosY = (pos.y * mxInv.M22) + (pos.x * mxInv.M12) + (pos.z * mxInv.M32) + mxInv.M42;
	float fNewPosZ = (pos.y * mxInv.M23) + (pos.x * mxInv.M13) + (pos.z * mxInv.M33) + mxInv.M43;

	float fZY = (fNewForceZ * fNewPosY) - (fNewForceY * fNewPosZ);
	float fXZ = (fNewForceX * fNewPosZ) - (fNewForceZ * fNewPosX);
	float fYX = (fNewForceY * fNewPosX) - (fNewForceX * fNewPosY);

	vec3f vTorq(0, 0, 0);
	vec3f vDriveTorq(0, 0, 0);

	if (pThis->car->torqueModeEx == TorqueModeEX::reactionTorques) // (torqueModeEx - 1) == 0
	{
		vTorq.x = ((M2[0] * fXZ) + (M1[0] * 0.0f)) + (M3[0] * fYX);
		vTorq.y = ((M2[1] * fXZ) + (M1[1] * 0.0f)) + (M3[1] * fYX);
		vTorq.z = ((M2[2] * fXZ) + (M1[2] * 0.0f)) + (M3[2] * fYX);
	}
	else if (pThis->car->torqueModeEx == TorqueModeEX::driveTorques) // (torqueModeEx - 1) == 1
	{
		if (pThis->driven)
		{
			vDriveTorq.x = ((M1[0] * fZY) + (M2[0] * 0.0f)) + (M3[0] * 0.0f);
			vDriveTorq.y = ((M1[1] * fZY) + (M2[1] * 0.0f)) + (M3[1] * 0.0f);
			vDriveTorq.z = ((M1[2] * fZY) + (M2[2] * 0.0f)) + (M3[2] * 0.0f);

			vTorq.x = ((M2[0] * fXZ) + (M1[0] * 0.0f)) + (M3[0] * fYX);
			vTorq.y = ((M2[1] * fXZ) + (M1[1] * 0.0f)) + (M3[1] * fYX);
			vTorq.z = ((M2[2] * fXZ) + (M1[2] * 0.0f)) + (M3[2] * fYX);
		}
		else
		{
			vTorq.x = ((M2[0] * fXZ) + (M1[0] * fZY)) + (M3[0] * fYX);
			vTorq.y = ((M2[1] * fXZ) + (M1[1] * fZY)) + (M3[1] * fYX);
			vTorq.z = ((M2[2] * fXZ) + (M1[2] * fZY)) + (M3[2] * fYX);
		}
	}

	pThis->hub->addLocalForceAndTorque(force, vTorq, vDriveTorq);
	pThis->localMX = fabsf(fZY); // TODO: wtf?
}

///////////////////////////////////////////////////////////////////////////////////////////////////
