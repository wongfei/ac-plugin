#pragma once

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_addTyreForcesV10(const vec3f& pos, const vec3f& normal, SurfaceDef* pSurface, float dt)
{
	mat44f& worldRotation = this->worldRotation;
	vec3f& roadHeading = this->roadHeading;
	vec3f& roadRight = this->roadRight;

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

	vec3f hubAngVel = this->hub->getHubAngularVelocity();
	vec3f hubPointVel = this->hub->getPointVelocity(pos);

	this->slidingVelocityY = ((hubPointVel.y * roadRight.y) + (hubPointVel.x * roadRight.x)) + (hubPointVel.z * roadRight.z);
	this->roadVelocityX = -(((hubPointVel.y * roadHeading.y) + (hubPointVel.x * roadHeading.x)) + (hubPointVel.z * roadHeading.z));
	float fSlipAngleTmp = calcSlipAngleRAD(this->slidingVelocityY, this->roadVelocityX);

	fTmp = (((hubAngVel.y * worldRotation.M12) + (hubAngVel.x * worldRotation.M11)) + (hubAngVel.z * worldRotation.M13)) + this->status.angularVelocity;
	this->slidingVelocityX = (fTmp * this->status.effectiveRadius) - this->roadVelocityX;
	float fRoadVelocityXAbs = fabsf(this->roadVelocityX);
	float fSlipRatioTmp = ((fRoadVelocityXAbs == 0.0f) ? 0.0f : (this->slidingVelocityX / fRoadVelocityXAbs));

	if (this->externalInputs.isActive)
	{
		fSlipAngleTmp = this->externalInputs.slipAngle;
		fSlipRatioTmp = this->externalInputs.slipRatio;
	}

	this->status.camberRAD = calcCamberRAD(this->contactNormal, worldRotation);
	this->totalHubVelocity = sqrtf((this->roadVelocityX * this->roadVelocityX) + (this->slidingVelocityY * this->slidingVelocityY));

	float fNdSlip = tclamp(this->status.ndSlip, 0.0f, 1.0f);
	float fLoadDivFz0 = this->status.load / this->modelData.Fz0;
	float fRelaxLen = this->modelData.relaxationLength;
	float fRelax1 = (((fLoadDivFz0 * fRelaxLen) - fRelaxLen) * 0.3f) + fRelaxLen;
	float fRelax2 = ((fRelaxLen - (fRelax1 * 2.0f)) * fNdSlip) + (fRelax1 * 2.0f);

	if (this->totalHubVelocity < 1.0f)
	{
		fSlipRatioTmp = this->slidingVelocityX * 0.5f;
		fSlipRatioTmp = tclamp(fSlipRatioTmp, -1.0f, 1.0f);

		fSlipAngleTmp = this->slidingVelocityY * -5.5f;
		fSlipAngleTmp = tclamp(fSlipAngleTmp, -1.0f, 1.0f);
	}

	float fSlipRatio = this->status.slipRatio;
	float fSlipRatioDelta = fSlipRatioTmp - fSlipRatio;
	float fNewSlipRatio = fSlipRatioTmp;

	if (fRelax2 != 0.0f)
	{
		float fSlipRatioScale = (this->totalHubVelocity * dt) / fRelax2;
		if (fSlipRatioScale <= 1.0f)
		{
			if (fSlipRatioScale < 0.04f)
				fNewSlipRatio = (0.04f * fSlipRatioDelta) + fSlipRatio;
			else
				fNewSlipRatio = (fSlipRatioScale * fSlipRatioDelta) + fSlipRatio;
		}
	}

	float fSlipAngle = this->status.slipAngleRAD;
	float fSlipAngleDelta = fSlipAngleTmp - fSlipAngle;
	float fNewSlipAngle = fSlipAngleTmp;

	if (fRelax2 != 0.0f)
	{
		float fSlipAngleScale = (this->totalHubVelocity * dt) / fRelax2;
		if (fSlipAngleScale <= 1.0f)
		{
			if (fSlipAngleScale < 0.04f)
				fNewSlipAngle = (0.04f * fSlipAngleDelta) + fSlipAngle;
			else
				fNewSlipAngle = (fSlipAngleScale * fSlipAngleDelta) + fSlipAngle;
		}
	}

	this->status.slipAngleRAD = fNewSlipAngle;
	this->status.slipRatio = fNewSlipRatio;

	if (this->status.load <= 0.0f)
	{
		this->status.slipAngleRAD = 0;
		this->status.slipRatio = 0;
	}

	float fCorrectedD = this->getCorrectedD(1.0, &this->status.wearMult);
	float fDynamicGripLevel = this->car ? this->car->ksPhysics->track->dynamicGripLevel : 1.0f;

	TyreModelInput tmi;
	tmi.load = this->status.load;
	tmi.slipAngleRAD = this->status.slipAngleRAD;
	tmi.slipRatio = this->status.slipRatio;
	tmi.camberRAD = this->status.camberRAD;
	tmi.speed = this->totalHubVelocity;
	tmi.u = (fCorrectedD * pSurface->gripMod) * fDynamicGripLevel;
	tmi.tyreIndex = this->index;
	tmi.cpLength = calcContactPatchLength(this->status.liveRadius, this->status.depth);
	tmi.grain = (float)this->status.grain;
	tmi.blister = (float)this->status.blister;
	tmi.pressureRatio = (this->status.pressureDynamic / this->modelData.idealPressure) - 1.0f;
	tmi.useSimpleModel = 1.0f < this->aiMult;

	TyreModelOutput tmo = this->tyreModel->solve(tmi);
	//TyreModelOutput tmo = ((SCTM*)(this->tyreModel))->solve_impl(tmi);

	this->status.Fy = tmo.Fy * this->aiMult;
	this->status.Fx = -tmo.Fx;
	this->status.Dy = tmo.Dy;
	this->status.Dx = tmo.Dx;

	float fHubSpeed = this->status.effectiveRadius * this->status.angularVelocity;

	this->stepDirtyLevel(dt, fabsf(fHubSpeed));
	this->stepPuncture(dt, this->totalHubVelocity);
	this->status.Mz = tmo.Mz;

	vec3f force(
		this->status.Fy * roadRight.x + this->status.Fx * roadHeading.x,
		this->status.Fy * roadRight.y + this->status.Fx * roadHeading.y,
		this->status.Fy * roadRight.z + this->status.Fx * roadHeading.z);

	if (!(isfinite(force.x) && isfinite(force.y) && isfinite(force.z)))
	{
		log_printf(L"INF vTyreForce");
		force = vec3f(0, 0, 0);
	}

	if (this->car && this->car->torqueModeEx != TorqueModeEX::original)
	{
		DEBUG_BREAK; // TODO: what car uses this code?

		this->addTyreForceToHub(pos, force);
	}
	else
	{
		this->hub->addForceAtPos(force, pos, this->driven, true);
		this->localMX = -(this->status.loadedRadius * this->status.Fx);
	}

	vec3f torq = vmul(normal, tmo.Mz);
	this->hub->addTorque(torq);

	float fAngularVelocityAbs = fabsf(this->status.angularVelocity);
	if (fAngularVelocityAbs > 1.0f)
	{
		fHubSpeed = this->status.effectiveRadius * this->status.angularVelocity;
		float fHubSpeedSign = signf(fHubSpeed);

		float fPressureDynamic = this->status.pressureDynamic;
		float fPressureUnk = (((this->modelData.idealPressure / fPressureDynamic) - 1.0f) * this->modelData.pressureRRGain) + 1.0f;

		if (fPressureDynamic <= 0.0f)
			fPressureUnk = 0.0f;

		float fRrUnk = ((((fHubSpeed * fHubSpeed) * this->modelData.rr1) + this->modelData.rr0) * fHubSpeedSign) * fPressureUnk;

		if (fAngularVelocityAbs > 20.0f)
		{
			float fSlipUnk = 0;
			if (this->modelData.version < 2)
			{
				float fRrSrUnk = fPressureUnk * this->modelData.rr_sr;
				float fRrSaUnk = fabsf(this->status.slipAngleRAD * 57.29578f) * (fPressureUnk * this->modelData.rr_sa);

				float fSlipRatioAbs = fabsf(this->status.slipRatio);
				float fSlipRatioNorm = tclamp(fSlipRatioAbs, 0.0f, 1.0f);

				fSlipUnk = (fSlipRatioNorm * fRrSrUnk) + fRrSaUnk;
			}
			else
			{
				float fNdSlipNorm = tclamp(this->status.ndSlip, 0.0f, 1.0f);
				fSlipUnk = (fNdSlipNorm * this->modelData.rr_slip) * fPressureUnk;
			}

			fRrUnk = fRrUnk * ((fSlipUnk * 0.001f) + 1.0f);
		}

		this->status.rollingResistence = -(((this->status.load * 0.001f) * fRrUnk) * this->status.effectiveRadius);
	}

	if (this->car)
	{
		float svx = this->slidingVelocityX;
		float svy = this->slidingVelocityY;
		float fSlidingVelocity = sqrtf(svx * svx + svy * svy);

		float fLoadVKM = 1.0f;
		if (this->useLoadForVKM)
			fLoadVKM = this->status.load / this->modelData.Fz0;

		this->status.virtualKM = (((fSlidingVelocity * dt) * this->car->ksPhysics->tyreConsumptionRate) * fLoadVKM) * 0.001 + this->status.virtualKM;
	}

	float fStaticDy = this->scTM.getStaticDY(this->status.load);

	this->status.ndSlip = tmo.ndSlip;
	this->status.D = fStaticDy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _Tyre::_getCorrectedD(float d, float* outWearMult)
{
	float fD = this->thermalModel.getCorrectedD(d, this->status.camberRAD) / ((fabsf(this->status.pressureDynamic - this->modelData.idealPressure) * this->modelData.pressureGainD) + 1.0f);

	if (this->modelData.wearCurve.getCount())
	{
		float fWearMult = this->modelData.wearCurve.getValue((float)this->status.virtualKM);
		fD *= fWearMult;

		if (outWearMult)
			*outWearMult = fWearMult;
	}

	return fD;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepDirtyLevel(float dt, float hubSpeed)
{
	if (this->status.dirtyLevel < 5.0f)
		this->status.dirtyLevel = (((hubSpeed * this->surfaceDef->dirtAdditiveK) * 0.03f) * dt) + this->status.dirtyLevel;

	if (this->surfaceDef->dirtAdditiveK == 0.0f)
	{
		if (this->status.dirtyLevel > 0.0f)
			this->status.dirtyLevel = this->status.dirtyLevel - ((hubSpeed * 0.015f) * dt);

		if (this->status.dirtyLevel < 0.0f)
			this->status.dirtyLevel = 0.0f;
	}

	if (this->aiMult == 1.0f)
	{
		float fDirty = 1.0f - tclamp((this->status.dirtyLevel * 0.05f), 0.0f, 1.0f);
		float fScale = tmax(0.8f, fDirty);

		this->status.Mz = fScale * this->status.Mz;
		this->status.Fx = fScale * this->status.Fx;
		this->status.Fy = fScale * this->status.Fy;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepPuncture(float dt, float hubSpeed)
{
	if (this->car && this->car->ksPhysics->mechanicalDamageRate > 0.0f)
	{
		float imo[3];
		this->thermalModel.getIMO(imo);

		float fTemp = this->explosionTemperature;

		if (imo[0] > fTemp || imo[1] > fTemp || imo[2] > fTemp)
			this->status.inflation = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_addTyreForceToHub(const vec3f& pos, const vec3f& force) // TODO: epic shit
{
	vec3f vWP = this->worldPosition;
	float fWorldX = this->worldPosition.x;
	float fWorldY = this->worldPosition.y;
	float fWorldZ = this->worldPosition.z;

	mat44f mxWR = this->worldRotation;
	float* M1 = &mxWR.M11;
	float* M2 = &mxWR.M21;
	float* M3 = &mxWR.M31;
	float* M4 = &mxWR.M41;

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

	if (this->car->torqueModeEx == TorqueModeEX::reactionTorques) // (torqueModeEx - 1) == 0
	{
		vTorq.x = ((M2[0] * fXZ) + (M1[0] * 0.0f)) + (M3[0] * fYX);
		vTorq.y = ((M2[1] * fXZ) + (M1[1] * 0.0f)) + (M3[1] * fYX);
		vTorq.z = ((M2[2] * fXZ) + (M1[2] * 0.0f)) + (M3[2] * fYX);
	}
	else if (this->car->torqueModeEx == TorqueModeEX::driveTorques) // (torqueModeEx - 1) == 1
	{
		if (this->driven)
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

	this->hub->addLocalForceAndTorque(force, vTorq, vDriveTorq);
	this->localMX = fabsf(fZY); // TODO: wtf?
}

///////////////////////////////////////////////////////////////////////////////////////////////////
