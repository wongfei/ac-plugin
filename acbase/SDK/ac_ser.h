#pragma once

// TODO: STUPID MONKEY SHIT

typedef std::wostringstream ac_ostream;

#define X_PASTE(x, y) x##y
#define X_WIDE(x) X_PASTE(L, x)

// udt

#define AC_SER_BEGIN(name)\
	out << L"{";\
	out << L"\"UDT\":\"";\
	out << X_WIDE(#name) << L"\",";

#define AC_SER_END()\
	out << L"}";

// field

#define AC_SER_FIELD(name)\
	out << L"\"" << X_WIDE(#name) << L"\":";\
	ser_field(out, o.name);

#define AC_SER_FIELD_T(name, type)\
	out << L"\"" << X_WIDE(#name) << L"\":";\
	ser_field(out, (type)o.name);

// array

#define AC_SER_ARR_R(name)\
	out << L"\"" << X_WIDE(#name) << L"\":[";\
	for (auto& iter : o.name) {\
		ser_field(out, iter); }\
	out << L"],";

#define AC_SER_ARR_P(name)\
	out << L"\"" << X_WIDE(#name) << L"\":[";\
	for (auto* iter : o.name) {\
		ser_field(out, iter); }\
	out << L"],";

///////////////////////////////////////////////////////////////////////////////////////////////////
// COMMON
///////////////////////////////////////////////////////////////////////////////////////////////////

inline std::wstring quote(const std::wstring& s) { return L"\"" + s + L"\""; }

template<typename T>
inline void ser_field(ac_ostream& out, const T& field)
{
	out << field << L",";
}

template<>
inline void ser_field(ac_ostream& out, const std::wstring& field)
{
	out << quote(field) << L",";
}

inline ac_ostream& operator<<(ac_ostream& out, const vec3f& o)
{
	out << L"[" << o.x << L"," << o.y << L"," << o.z << L"]";
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const mat44f& o)
{
	const float* m = &o.M11;
	out << L"[";
	for (int i = 0; i < 16; ++i)
	{
		out << m[i] << L",";
	}
	out << L"]";
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Curve& o)
{
	AC_SER_BEGIN(Curve);
	AC_SER_ARR_R(references);
	AC_SER_ARR_R(values);
	AC_SER_FIELD(fastStep);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const DynamicControllerStage& o)
{
	AC_SER_BEGIN(DynamicControllerStage);
	AC_SER_FIELD_T(inputVar, int);
	AC_SER_FIELD_T(combinatorMode, int);
	AC_SER_FIELD(lut);
	AC_SER_FIELD(filter);
	AC_SER_FIELD(upLimit);
	AC_SER_FIELD(downLimit);
	AC_SER_FIELD(currentValue);
	AC_SER_FIELD(constValue);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const DynamicController& o)
{
	AC_SER_BEGIN(DynamicController);
	AC_SER_FIELD(ready);
	AC_SER_ARR_R(stages);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SinSignalGenerator& o) { return out; }

///////////////////////////////////////////////////////////////////////////////////////////////////
// BRAKE
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const BrakeDisc& o)
{
	AC_SER_BEGIN(BrakeDisc);
	AC_SER_FIELD(t);
	AC_SER_FIELD(coolTransfer);
	AC_SER_FIELD(torqueK);
	AC_SER_FIELD(coolSpeedFactor);
	AC_SER_FIELD(perfCurve);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SteerBrake& o)
{
	AC_SER_BEGIN(SteerBrake);
	AC_SER_FIELD(isActive);
	AC_SER_FIELD(controller);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const BrakeSystem& o)
{
	AC_SER_BEGIN(BrakeSystem);
	AC_SER_FIELD(frontBias);
	AC_SER_FIELD(brakePowerMultiplier);
	AC_SER_FIELD(electronicOverride);
	AC_SER_FIELD(handBrakeTorque);
	AC_SER_FIELD(ebbInstant);
	AC_SER_ARR_R(discs);
	AC_SER_FIELD(limitDown);
	AC_SER_FIELD(limitUp);
	AC_SER_FIELD(rearCorrectionTorque);
	//Car* car;
	AC_SER_FIELD(brakePower);
	AC_SER_FIELD(biasOverride);
	AC_SER_FIELD(hasCockpitBias);
	AC_SER_FIELD(biasStep);
	AC_SER_FIELD_T(ebbMode, int);
	AC_SER_FIELD(ebbFrontMultiplier);
	AC_SER_FIELD(steerBrake);
	AC_SER_FIELD(hasBrakeTempsData);
	//std::basic_ofstream<char,std::char_traits<char> > tempRunFile;
	AC_SER_FIELD(ebbController);
	AC_SER_END();
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// ENGINE
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const acEngineData& o)
{
	AC_SER_BEGIN(acEngineData);
	AC_SER_FIELD(powerCurve);
	AC_SER_FIELD(coastCurve);
	AC_SER_FIELD(coast2);
	AC_SER_FIELD(coast1);
	AC_SER_FIELD(coast0);
	AC_SER_FIELD(useCoastCurve);
	AC_SER_FIELD(minimum);
	AC_SER_FIELD(limiter);
	AC_SER_FIELD(limiterCycles);
	AC_SER_FIELD(overlapFreq);
	AC_SER_FIELD(overlapGain);
	AC_SER_FIELD(overlapIdealRPM);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const EngineStatus& o)
{
	AC_SER_BEGIN(EngineStatus);
	AC_SER_FIELD(outTorque);
	AC_SER_FIELD(externalCoastTorque);
	AC_SER_FIELD(turboBoost);
	AC_SER_FIELD(isLimiterOn);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SACEngineInput& o)
{
	AC_SER_BEGIN(SACEngineInput);
	AC_SER_FIELD(gasInput);
	AC_SER_FIELD(carSpeed);
	AC_SER_FIELD(altitude);
	AC_SER_FIELD(rpm);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TurboDef& o)
{
	AC_SER_BEGIN(TurboDef);
	AC_SER_FIELD(maxBoost);
	AC_SER_FIELD(lagUP);
	AC_SER_FIELD(lagDN);
	AC_SER_FIELD(rpmRef);
	AC_SER_FIELD(gamma);
	AC_SER_FIELD(wastegate);
	AC_SER_FIELD(isAdjustable);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Turbo& o)
{
	AC_SER_BEGIN(Turbo);
	AC_SER_FIELD(userSetting);
	AC_SER_FIELD(rotation);
	AC_SER_FIELD(data);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TurboDynamicController& o)
{
	AC_SER_BEGIN(TurboDynamicController);
	AC_SER_FIELD(controller);
	AC_SER_FIELD(isWastegate);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Engine& o)
{
	AC_SER_BEGIN(Engine);
	AC_SER_FIELD(data);
	AC_SER_FIELD(status);
	AC_SER_FIELD(coastTorqueMultiplier);
	AC_SER_FIELD(limiterMultiplier);
	AC_SER_FIELD(fuelPressure);
	AC_SER_FIELD(bov);
	AC_SER_ARR_R(turbos);
	AC_SER_FIELD(isEngineStallEnabled);
	AC_SER_FIELD(starterTorque);
	AC_SER_FIELD(rpmDamageThreshold);
	AC_SER_FIELD(restrictor);
	//AC_SER_FIELD(p2p);
	AC_SER_ARR_P(torqueGenerators);
	AC_SER_ARR_P(coastGenerators);
	AC_SER_FIELD(turboAdjustableFromCockpit);
	AC_SER_FIELD(lastInput);
	AC_SER_FIELD(defaultEngineLimiter);
	AC_SER_FIELD(inertia);
	AC_SER_FIELD(limiterOn);
	AC_SER_FIELD(electronicOverride);
	AC_SER_FIELD(maxPowerW_Dynamic);
	AC_SER_FIELD(maxPowerW);
	AC_SER_FIELD(maxTorqueNM);
	AC_SER_FIELD(maxPowerRPM);
	AC_SER_FIELD(maxTorqueRPM);
	//PhysicsEngine* physicsEngine;
	AC_SER_FIELD(throttleResponseCurve);
	AC_SER_FIELD(throttleResponseCurveMax);
	AC_SER_FIELD(throttleResponseCurveMaxRef);
	AC_SER_FIELD(gasUsage);
	AC_SER_FIELD(lifeLeft);
	AC_SER_FIELD(turboBoostDamageThreshold);
	AC_SER_FIELD(turboBoostDamageK);
	AC_SER_FIELD(rpmDamageK);
	AC_SER_FIELD(bovThreshold);
	//Car* car;
	AC_SER_ARR_R(turboControllers);
	AC_SER_FIELD(gasCoastOffset);
	AC_SER_FIELD(gasCoastOffsetCurve);
	AC_SER_FIELD(coastSettingsDefaultIndex);
	AC_SER_FIELD(coastEntryRpm);
	AC_SER_END();
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// DRIVETRAIN
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const SGearRatio& o)
{
	out << o.ratio;
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const GearElement& o)
{
	AC_SER_BEGIN(GearElement);
	AC_SER_FIELD(velocity);
	AC_SER_FIELD(inertia);
	AC_SER_FIELD(oldVelocity);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const DifferentialSetting& o)
{
	AC_SER_BEGIN(DifferentialSetting);
	AC_SER_FIELD(power);
	AC_SER_FIELD(coast);
	AC_SER_FIELD(preload);
	AC_SER_FIELD_T(type, int);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const GearRequestStatus& o)
{
	AC_SER_BEGIN(GearRequestStatus);
	AC_SER_FIELD_T(request, int);
	AC_SER_FIELD(timeAccumulator);
	AC_SER_FIELD(timeout);
	AC_SER_FIELD(requestedGear);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const DrivetrainControllers& o)
{
	AC_SER_BEGIN(DrivetrainControllers);
	if (o.awdFrontShare.get())
		out <<*o.awdFrontShare.get();
	if (o.awdCenterLock.get())
		out <<*o.awdCenterLock.get();
	if (o.singleDiffLock.get())
		out <<*o.singleDiffLock.get();
	if (o.awd2.get())
		out <<*o.awd2.get();
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Drivetrain& o)
{
	AC_SER_BEGIN(Drivetrain);
	AC_SER_FIELD(isGearGrinding);
	AC_SER_FIELD(finalRatio);
	AC_SER_FIELD(engine);
	AC_SER_FIELD(drive);
	AC_SER_FIELD(outShaftL);
	AC_SER_FIELD(outShaftR);
	AC_SER_FIELD(outShaftLF);
	AC_SER_FIELD(outShaftRF);
	AC_SER_ARR_R(gears);
	AC_SER_FIELD(rootVelocity);
	AC_SER_FIELD(clutchOpenState);
	AC_SER_FIELD(ratio);
	AC_SER_FIELD(diffPowerRamp);
	AC_SER_FIELD(diffCoastRamp);
	AC_SER_FIELD(diffPreLoad);
	AC_SER_FIELD_T(diffType, int);
	AC_SER_FIELD(cutOff);
	AC_SER_FIELD(acEngine);
	AC_SER_FIELD(isShifterSupported);
	AC_SER_FIELD(clutchMaxTorque);
	AC_SER_FIELD(totalTorque);
	AC_SER_FIELD(awdFrontShare);
	AC_SER_FIELD(awdFrontDiff);
	AC_SER_FIELD(awdRearDiff);
	AC_SER_FIELD(awdCenterDiff);
	//DownshiftProtection downshiftProtection;
	//AWD2Data awd2;
	AC_SER_FIELD(currentClutchTorque);
	//std::function<void __cdecl(void)> downshiftProtectionFunction;
	//Event<OnGearRequestEvent> evOnGearRequest;
	//Car* car;
	AC_SER_FIELD_T(tractionType, int);
	AC_SER_FIELD(currentGear);
	AC_SER_FIELD(lastRatio);
	//Tyre* tyreLeft;
	//Tyre* tyreRight;
	AC_SER_FIELD(gearRequest);
	AC_SER_FIELD(gearUpTime);
	AC_SER_FIELD(gearDnTime);
	AC_SER_FIELD(autoCutOffTime);
	AC_SER_FIELD(validShiftRPMWindow);
	AC_SER_FIELD(controlsWindowGain);
	AC_SER_ARR_P(wheelTorqueGenerators);
	AC_SER_FIELD(damageRpmWindow);
	AC_SER_FIELD(orgRpmWindow);
	AC_SER_ARR_R(lockCounter);
	AC_SER_FIELD(controllers);
	AC_SER_FIELD(clutchInertia);
	AC_SER_FIELD(locClutch);
	AC_SER_END();
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// SUSPENSION
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const Damper& o)
{
	AC_SER_BEGIN(Damper);
	AC_SER_FIELD(reboundSlow);
	AC_SER_FIELD(reboundFast);
	AC_SER_FIELD(bumpSlow);
	AC_SER_FIELD(bumpFast);
	AC_SER_FIELD(fastThresholdBump);
	AC_SER_FIELD(fastThresholdRebound);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const ActiveActuator& o)
{
	AC_SER_BEGIN(ActiveActuator);
	AC_SER_FIELD(targetTravel);
	//PIDController pid;
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SDWSuspensionData& o)
{
	AC_SER_BEGIN(SDWSuspensionData);
	AC_SER_FIELD(carTopWB_F);
	AC_SER_FIELD(carTopWB_R);
	AC_SER_FIELD(carBottomWB_F);
	AC_SER_FIELD(carBottomWB_R);
	AC_SER_FIELD(tyreTopWB);
	AC_SER_FIELD(tyreBottomWB);
	AC_SER_FIELD(carSteer);
	AC_SER_FIELD(tyreSteer);
	AC_SER_FIELD(refPoint);
	AC_SER_FIELD(hubMass);
	AC_SER_FIELD(hubInertiaBox);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SuspensionStatus& o)
{
	AC_SER_BEGIN(SuspensionStatus);
	AC_SER_FIELD(travel);
	AC_SER_FIELD(damperSpeedMS);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SusDamageDef& o)
{
	AC_SER_BEGIN(SusDamageDef);
	AC_SER_FIELD(damageAmount);
	AC_SER_FIELD(damageDirection);
	AC_SER_FIELD(minVelocity);
	AC_SER_FIELD(damageGain);
	AC_SER_FIELD(maxDamage);
	AC_SER_FIELD(isDebug);
	AC_SER_FIELD(lastAmount);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Suspension& o)
{
	AC_SER_BEGIN(Suspension);
	//IRigidBody* carBody;
	//IRigidBody* hub;
	AC_SER_FIELD(basePosition);
	//Car* car;
	AC_SER_FIELD(useActiveActuator);
	//IJoint* joints[5];
	//IJoint* bumpStopJoint;
	AC_SER_FIELD(dataRelToWheel);
	AC_SER_FIELD(dataRelToBody);
	AC_SER_FIELD(damper);
	AC_SER_FIELD(activeActuator);
	AC_SER_FIELD(status);
	AC_SER_FIELD(index);
	//PhysicsEngine* physicsEngine;
	AC_SER_FIELD(steerLinkBaseLength);
	AC_SER_FIELD(steerTorque);
	AC_SER_FIELD(baseCarSteerPosition);
	AC_SER_FIELD(steerAngle);
	AC_SER_FIELD(damageData);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SStrutSuspensionData& o)
{
	AC_SER_BEGIN(SStrutSuspensionData);
	AC_SER_FIELD(carStrut);
	AC_SER_FIELD(tyreStrut);
	AC_SER_FIELD(carBottomWB_F);
	AC_SER_FIELD(carBottomWB_R);
	AC_SER_FIELD(tyreBottomWB);
	AC_SER_FIELD(carSteer);
	AC_SER_FIELD(tyreSteer);
	AC_SER_FIELD(refPoint);
	AC_SER_FIELD(hubMass);
	AC_SER_FIELD(hubInertiaBox);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SusStrutDamageDef& o)
{
	AC_SER_BEGIN(SusStrutDamageDef);
	AC_SER_FIELD(damageAmount);
	AC_SER_FIELD(damageDirection);
	AC_SER_FIELD(minVelocity);
	AC_SER_FIELD(damageGain);
	AC_SER_FIELD(maxDamage);
	AC_SER_FIELD(isDebug);
	AC_SER_FIELD(lastAmount);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SuspensionStrut& o)
{
	AC_SER_BEGIN(SuspensionStrut);
	//IRigidBody* carBody;
	//IRigidBody* hub;
	AC_SER_FIELD(basePosition);
	//IJoint* joints[5];
	//IJoint* bumpStopJoint;
	AC_SER_FIELD(dataRelToWheel);
	AC_SER_FIELD(dataRelToBody);
	AC_SER_FIELD(damper);
	AC_SER_FIELD(status);
	AC_SER_FIELD(index);
	//PhysicsEngine* physicsEngine;
	AC_SER_FIELD(steerLinkBaseLength);
	AC_SER_FIELD(steerTorque);
	AC_SER_FIELD(baseCarSteerPosition);
	AC_SER_FIELD(steerAngle);
	AC_SER_FIELD(strutBaseLength);
	//IRigidBody* strutBody;
	AC_SER_FIELD(strutBodyLength);
	AC_SER_FIELD(damageData);
	//Car* car;
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const AxleBall& o)
{
	AC_SER_BEGIN(AxleBall);
	AC_SER_FIELD(relToAxle);
	AC_SER_FIELD(relToCar);
	//IJoint* joint;
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const AxleJoint& o)
{
	AC_SER_BEGIN(AxleJoint);
	AC_SER_FIELD(ballCar);
	AC_SER_FIELD(ballAxle);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SuspensionAxle& o)
{
	AC_SER_BEGIN(SuspensionAxle);
	AC_SER_FIELD_T(side, int);
	AC_SER_FIELD(damper);
	//Car* car;
	//IRigidBody* axle;
	AC_SER_FIELD(status);
	AC_SER_FIELD(axleBasePos);
	AC_SER_FIELD(track);
	AC_SER_FIELD(referenceY);
	AC_SER_ARR_R(joints);
	AC_SER_FIELD(leafSpringK);
	AC_SER_FIELD(attachRelativePos);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const MLBall& o)
{
	AC_SER_BEGIN(MLBall);
	AC_SER_FIELD(relToTyre);
	AC_SER_FIELD(relToCar);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const MLJoint& o)
{
	AC_SER_BEGIN(MLJoint);
	AC_SER_FIELD(ballCar);
	AC_SER_FIELD(ballTyre);
	//IJoint* joint;
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SuspensionML& o)
{
	AC_SER_BEGIN(SuspensionML);
	//IRigidBody* hub;
	AC_SER_FIELD(status);
	AC_SER_FIELD(damper);
	//Car* car;
	AC_SER_FIELD(index);
	AC_SER_ARR_R(joints);
	AC_SER_FIELD(hubMass);
	AC_SER_FIELD(basePosition);
	AC_SER_FIELD(steerTorque);
	AC_SER_FIELD(damageData);
	AC_SER_FIELD(baseCarSteerPosition);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const HeaveSpringStatus& o)
{
	AC_SER_BEGIN(HeaveSpringStatus);
	AC_SER_FIELD(travel);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const HeaveSpring& o)
{
	AC_SER_BEGIN(HeaveSpring);
	AC_SER_FIELD(isPresent);
	AC_SER_FIELD(rodLength);
	AC_SER_FIELD(status);
	AC_SER_FIELD(k);
	AC_SER_FIELD(progressiveK);
	AC_SER_FIELD(packerRange);
	AC_SER_FIELD(bumpStopRate);
	AC_SER_FIELD(bumpStopUp);
	AC_SER_FIELD(bumpStopDn);
	AC_SER_FIELD(damper);
	AC_SER_FIELD(isFront);
	//Suspension* suspensions[2];
	//Car* car;
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const AntirollBar& o)
{
	AC_SER_BEGIN(AntirollBar);
	//IRigidBody* carBody;
	//ISuspension* hubs[2];
	AC_SER_FIELD(ctrl);
	AC_SER_FIELD(k);
	AC_SER_END();
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// TYRE
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const TyreInputs& o)
{
	AC_SER_BEGIN(TyreInputs);
	AC_SER_FIELD(brakeTorque);
	AC_SER_FIELD(handBrakeTorque);
	AC_SER_FIELD(electricTorque);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreData& o)
{
	AC_SER_BEGIN(TyreData);
	AC_SER_FIELD(width);
	AC_SER_FIELD(radius);
	AC_SER_FIELD(k);
	AC_SER_FIELD(d);
	AC_SER_FIELD(angularInertia);
	AC_SER_FIELD(thermalFrictionK);
	AC_SER_FIELD(thermalRollingK);
	AC_SER_FIELD(thermalRollingSurfaceK);
	AC_SER_FIELD(grainThreshold);
	AC_SER_FIELD(blisterThreshold);
	AC_SER_FIELD(grainGamma);
	AC_SER_FIELD(blisterGamma);
	AC_SER_FIELD(grainGain);
	AC_SER_FIELD(blisterGain);
	AC_SER_FIELD(rimRadius);
	AC_SER_FIELD(optimumTemp);
	AC_SER_FIELD(softnessIndex);
	AC_SER_FIELD(radiusRaiseK);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreModelData& o)
{
	AC_SER_BEGIN(TyreModelData);
	AC_SER_FIELD(version);
	AC_SER_FIELD(Dy0);
	AC_SER_FIELD(Dy1);
	AC_SER_FIELD(Dx0);
	AC_SER_FIELD(Dx1);
	AC_SER_FIELD(Fz0);
	AC_SER_FIELD(flexK);
	AC_SER_FIELD(speedSensitivity);
	AC_SER_FIELD(relaxationLength);
	AC_SER_FIELD(rr0);
	AC_SER_FIELD(rr1);
	AC_SER_FIELD(rr_sa);
	AC_SER_FIELD(rr_sr);
	AC_SER_FIELD(rr_slip);
	AC_SER_FIELD(camberGain);
	AC_SER_FIELD(pressureSpringGain);
	AC_SER_FIELD(pressureFlexGain);
	AC_SER_FIELD(pressureRRGain);
	AC_SER_FIELD(pressureGainD);
	AC_SER_FIELD(idealPressure);
	AC_SER_FIELD(pressureRef);
	AC_SER_FIELD(wearCurve);
	AC_SER_FIELD(dcamber0);
	AC_SER_FIELD(dcamber1);
	AC_SER_FIELD(dyLoadCurve);
	AC_SER_FIELD(dxLoadCurve);
	AC_SER_FIELD(lsMultY);
	AC_SER_FIELD(lsExpY);
	AC_SER_FIELD(lsMultX);
	AC_SER_FIELD(lsExpX);
	AC_SER_FIELD(maxWearKM);
	AC_SER_FIELD(maxWearMult);
	AC_SER_FIELD(asy);
	AC_SER_FIELD(cfXmult);
	AC_SER_FIELD(brakeDXMod);
	AC_SER_FIELD(dCamberCurve);
	AC_SER_FIELD(useSmoothDCamberCurve);
	AC_SER_FIELD(combinedFactor);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreStatus& o)
{
	AC_SER_BEGIN(TyreStatus);
	AC_SER_FIELD(depth);
	AC_SER_FIELD(load);
	AC_SER_FIELD(camberRAD);
	AC_SER_FIELD(slipAngleRAD);
	AC_SER_FIELD(slipRatio);
	AC_SER_FIELD(angularVelocity);
	AC_SER_FIELD(Fy);
	AC_SER_FIELD(Fx);
	AC_SER_FIELD(Mz);
	AC_SER_FIELD(isLocked);
	AC_SER_FIELD(slipFactor);
	AC_SER_FIELD(ndSlip);
	AC_SER_FIELD(distToGround);
	AC_SER_FIELD(Dy);
	AC_SER_FIELD(Dx);
	AC_SER_FIELD(D);
	AC_SER_FIELD(dirtyLevel);
	AC_SER_FIELD(rollingResistence);
	AC_SER_FIELD(thermalInput);
	AC_SER_FIELD(feedbackTorque);
	AC_SER_FIELD(loadedRadius);
	AC_SER_FIELD(effectiveRadius);
	AC_SER_FIELD(liveRadius);
	AC_SER_FIELD(pressureStatic);
	AC_SER_FIELD(pressureDynamic);
	AC_SER_FIELD(virtualKM);
	AC_SER_ARR_R(lastTempIMO);
	AC_SER_FIELD(peakSA);
	AC_SER_FIELD(grain);
	AC_SER_FIELD(blister);
	AC_SER_FIELD(inflation);
	AC_SER_FIELD(flatSpot);
	AC_SER_FIELD(lastGrain);
	AC_SER_FIELD(lastBlister);
	AC_SER_FIELD(normalizedSlideX);
	AC_SER_FIELD(normalizedSlideY);
	AC_SER_FIELD(finalDY);
	AC_SER_FIELD(wearMult);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreThermalPatch& o)
{
	AC_SER_BEGIN(TyreThermalPatch);
	//std::vector<TyreThermalPatch*,std::allocator<TyreThermalPatch*> > connections;
	AC_SER_FIELD(T);
	AC_SER_FIELD(inputT);
	AC_SER_FIELD(elementIndex);
	AC_SER_FIELD(stripeIndex);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyrePatchData& o)
{
	AC_SER_BEGIN(TyrePatchData);
	AC_SER_FIELD(surfaceTransfer);
	AC_SER_FIELD(patchTransfer);
	AC_SER_FIELD(patchCoreTransfer);
	AC_SER_FIELD(internalCoreTransfer);
	AC_SER_FIELD(coolFactorGain);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreThermalModel& o)
{
	AC_SER_BEGIN(TyreThermalModel);
	AC_SER_FIELD(elements);
	AC_SER_FIELD(stripes);
	AC_SER_ARR_R(patches);
	AC_SER_FIELD(phase);
	AC_SER_FIELD(patchData);
	AC_SER_FIELD(coreTemp);
	AC_SER_FIELD(performanceCurve);
	AC_SER_FIELD(isActive);
	AC_SER_FIELD(thermalMultD);
	AC_SER_FIELD(practicalTemp);
	AC_SER_FIELD(camberSpreadK);
	//Car* car;
	AC_SER_FIELD(coreTInput);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const BrushTyreModelData& o)
{
	AC_SER_BEGIN(BrushTyreModelData);
	AC_SER_FIELD(CF);
	AC_SER_FIELD(xu);
	AC_SER_FIELD(CF1);
	AC_SER_FIELD(Fz0);
	AC_SER_FIELD(maxSlip0);
	AC_SER_FIELD(maxSlip1);
	AC_SER_FIELD(falloffSpeed);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const BrushTyreModel& o)
{
	AC_SER_BEGIN(BrushTyreModel);
	AC_SER_FIELD(data);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const BrushSlipProvider& o)
{
	AC_SER_BEGIN(BrushSlipProvider);
	AC_SER_FIELD(brushModel);
	AC_SER_FIELD(asy);
	AC_SER_FIELD(version);
	AC_SER_FIELD(maximum);
	AC_SER_FIELD(maxSlip);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreCompoundDef& o)
{
	AC_SER_BEGIN(TyreCompoundDef);
	AC_SER_FIELD(index);
	AC_SER_FIELD(name);
	AC_SER_FIELD(shortName);
	AC_SER_FIELD(modelData);
	AC_SER_FIELD(data);
	AC_SER_FIELD(slipProvider);
	AC_SER_FIELD(pressureStatic);
	AC_SER_FIELD(thermalPatchData);
	AC_SER_FIELD(thermalPerformanceCurve);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const TyreExternalInputs& o)
{
	AC_SER_BEGIN(TyreExternalInputs);
	AC_SER_FIELD(isActive);
	AC_SER_FIELD(load);
	AC_SER_FIELD(slipAngle);
	AC_SER_FIELD(slipRatio);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const SCTM& o)
{
	AC_SER_BEGIN(SCTM);
	AC_SER_FIELD(lsMultY);
	AC_SER_FIELD(lsExpY);
	AC_SER_FIELD(lsMultX);
	AC_SER_FIELD(lsExpX);
	AC_SER_FIELD(Fz0);
	AC_SER_FIELD(maxSlip0);
	AC_SER_FIELD(maxSlip1);
	AC_SER_FIELD(asy);
	AC_SER_FIELD(falloffSpeed);
	AC_SER_FIELD(speedSensitivity);
	AC_SER_FIELD(camberGain);
	AC_SER_FIELD(dcamber0);
	AC_SER_FIELD(dcamber1);
	AC_SER_FIELD(cfXmult);
	AC_SER_FIELD(dyLoadCurve);
	AC_SER_FIELD(dxLoadCurve);
	AC_SER_FIELD(pressureCfGain);
	AC_SER_FIELD(brakeDXMod);
	AC_SER_FIELD(dCamberCurve);
	AC_SER_FIELD(useSmoothDCamberCurve);
	AC_SER_FIELD(dCamberBlend);
	AC_SER_FIELD(combinedFactor);
	AC_SER_FIELD(dy0);
	AC_SER_FIELD(dx0);
	AC_SER_FIELD(pacE);
	AC_SER_FIELD(pacCf);
	AC_SER_FIELD(pacFlex);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Tyre& o)
{
	AC_SER_BEGIN(Tyre);
	AC_SER_FIELD(inputs);
	AC_SER_FIELD(data);
	AC_SER_FIELD(modelData);
	AC_SER_FIELD(status);
	//ISuspension* hub;
	AC_SER_FIELD(worldRotation);
	AC_SER_FIELD(unmodifiedContactPoint);
	AC_SER_FIELD(contactPoint);
	AC_SER_FIELD(contactNormal);
	//SurfaceDef* surfaceDef;
	AC_SER_FIELD(debugOutput);
	AC_SER_FIELD(absOverride);
	AC_SER_FIELD(thermalModel);
	AC_SER_ARR_R(compoundDefs);
	//std::function<void __cdecl(void)> onStepCompleted;
	AC_SER_FIELD(aiMult);
	AC_SER_FIELD(slipProvider);
	AC_SER_FIELD(externalInputs);
	AC_SER_FIELD(roadRight);
	AC_SER_FIELD(roadHeading);
	AC_SER_FIELD(useLoadForVKM);
	AC_SER_FIELD(driven);
	//IRayTrackCollisionProvider* rayCollisionProvider;
	AC_SER_FIELD(oldAngularVelocity);
	AC_SER_FIELD(totalSlideVelocity);
	AC_SER_FIELD(localWheelRotation);
	AC_SER_FIELD(worldPosition);
	AC_SER_FIELD(slidingVelocityY);
	AC_SER_FIELD(slidingVelocityX);
	AC_SER_FIELD(roadVelocityX);
	AC_SER_FIELD(roadVelocityY);
	AC_SER_FIELD(totalHubVelocity);
	AC_SER_FIELD(rSlidingVelocityX);
	AC_SER_FIELD(rSlidingVelocityY);
	//IRayCaster* rayCaster;
	AC_SER_FIELD(index);
	AC_SER_FIELD(shakeGenerator);
	//Car* car;
	AC_SER_FIELD(currentCompoundIndex);
	AC_SER_FIELD(tyreBlanketsOn);
	AC_SER_FIELD(flatSpotK);
	//ITyreModel* tyreModel;
	AC_SER_FIELD(scTM);
	AC_SER_FIELD(explosionTemperature);
	AC_SER_FIELD(blanketTemperature);
	AC_SER_FIELD(pressureTemperatureGain);
	AC_SER_FIELD(localMX);
	AC_SER_END();
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// AERO
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const WingData& o)
{
	AC_SER_BEGIN(WingData);
	AC_SER_FIELD(name);
	AC_SER_FIELD(chord);
	AC_SER_FIELD(span);
	AC_SER_FIELD(position);
	AC_SER_FIELD(lutAOA_CL);
	AC_SER_FIELD(lutAOA_CD);
	AC_SER_FIELD(lutGH_CL);
	AC_SER_FIELD(lutGH_CD);
	AC_SER_FIELD(clGain);
	AC_SER_FIELD(cdGain);
	AC_SER_FIELD(hasController);
	AC_SER_FIELD(yawGain);
	AC_SER_FIELD(area);
	AC_SER_FIELD(isVertical);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const WingState& o)
{
	AC_SER_BEGIN(WingState);
	AC_SER_FIELD(aoa);
	AC_SER_FIELD(cd);
	AC_SER_FIELD(cl);
	AC_SER_FIELD(angle);
	AC_SER_FIELD(inputAngle);
	AC_SER_FIELD(groundHeight);
	AC_SER_FIELD(frontShare);
	AC_SER_FIELD(dragKG);
	AC_SER_FIELD(liftKG);
	AC_SER_FIELD(angleMult);
	AC_SER_FIELD(groundEffectLift);
	AC_SER_FIELD(groundEffectDrag);
	AC_SER_FIELD(yawAngle);
	AC_SER_FIELD(isVertical);
	AC_SER_FIELD(liftVector);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const DynamicWingController& o)
{
	AC_SER_BEGIN(DynamicWingController);
	AC_SER_FIELD(outputAngle);
	AC_SER_FIELD(upLimit);
	AC_SER_FIELD(downLimit);
	AC_SER_FIELD_T(combinatorMode, int);
	AC_SER_FIELD_T(inputVar, int);
	AC_SER_FIELD(lut);
	AC_SER_FIELD(filter);
	//const CarPhysicsState* state;
	//Car* car;
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const WingOverrideDef& o)
{
	AC_SER_BEGIN(WingOverrideDef);
	AC_SER_FIELD(overrideAngle);
	AC_SER_FIELD(isActive);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Wing& o)
{
	AC_SER_BEGIN(Wing);
	AC_SER_FIELD(data);
	AC_SER_FIELD(status);
	AC_SER_ARR_R(dynamicControllers);
	//Car* car;
	//RaceEngineer engineer;
	AC_SER_ARR_R(damageCL);
	AC_SER_ARR_R(damageCD);
	AC_SER_FIELD(hasDamage);
	AC_SER_FIELD(overrideStatus);
	AC_SER_FIELD(SPEED_DAMAGE_COEFF);
	AC_SER_FIELD(SURFACE_DAMAGE_COEFF);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const AeroMap& o)
{
	AC_SER_BEGIN(AeroMap);
	AC_SER_FIELD(referenceArea);
	AC_SER_FIELD(CD);
	AC_SER_FIELD(CL);
	AC_SER_FIELD(frontShare);
	AC_SER_FIELD(CDX);
	AC_SER_FIELD(CDY);
	AC_SER_FIELD(CDA);
	//IRigidBody* carBody;
	AC_SER_FIELD(dynamicCD);
	AC_SER_FIELD(dynamicCL);
	AC_SER_FIELD(airDensity);
	AC_SER_ARR_R(wings);
	AC_SER_FIELD(frontApplicationPoint);
	AC_SER_FIELD(rearApplicationPoint);
	//Car* car;
	AC_SER_END();
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CAR
///////////////////////////////////////////////////////////////////////////////////////////////////

inline ac_ostream& operator<<(ac_ostream& out, const SteeringSystem& o)
{
	AC_SER_BEGIN(SteeringSystem);
	AC_SER_FIELD(linearRatio);
	//Car* car;
	AC_SER_FIELD(has4ws);
	AC_SER_FIELD(ctrl4ws);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const ThermalObject& o)
{
	AC_SER_BEGIN(ThermalObject);
	AC_SER_FIELD(tmass);
	AC_SER_FIELD(coolSpeedK);
	AC_SER_FIELD(coolFactor);
	AC_SER_FIELD(heatFactor);
	AC_SER_FIELD(t);
	AC_SER_FIELD(heatAccumulator);
	AC_SER_END();
	return out;
}

inline ac_ostream& operator<<(ac_ostream& out, const Car& o)
{
	AC_SER_BEGIN(Car);
	AC_SER_FIELD(finalSteerAngleSignal);
	AC_SER_FIELD(powerClassIndex);
	//Event<OnStepCompleteEvent> evOnStepComplete;
	//Event<OnControlsProviderChanged> evOnControlsProviderChanged;
	//Event<OnLapCompletedEvent> evOnLapCompleted;
	//Event<OnSectorSplitEvent> evOnSectorSplit;
	//Event<vec3f> evOnForcedPositionCompleted;
	//Event<std::pair<int,int> > evOnTyreCompoundChanged;
	//Event<OnCollisionEvent> evOnCollisionEvent;
	//Event<double> evOnJumpStartEvent;
	//Event<std::pair<int,int> > evOnPush2Pass;
	AC_SER_FIELD(carHalfWidth);
	AC_SER_FIELD(userFFGain);
	AC_SER_FIELD(isRetired);
	AC_SER_ARR_R(tyreCompounds);
	//void* tag;
	//IRigidBody* body;
	//IRigidBody* fuelTankBody;
	//IRigidBody* rigidAxle;
	//IJoint* fuelTankJoint;
	//PhysicsEngine* ksPhysics;
	//CarControls controls;
	AC_SER_FIELD(steerLock);
	AC_SER_FIELD(steerRatio);
	AC_SER_FIELD(steerLinearRatio);
	AC_SER_FIELD(unixName);
	AC_SER_FIELD(configName);
	AC_SER_FIELD(carDataPath);
	AC_SER_FIELD(mass);
	AC_SER_FIELD(ffMult);
	AC_SER_FIELD(accG);
	//TimeTransponder transponder;
	//CarColliderManager colliderManager;
	AC_SER_FIELD(drivetrain);
	//ABS abs;
	//TractionControl tractionControl;
	//SpeedLimiter speedLimiter;
	AC_SER_FIELD(aeroMap);
	AC_SER_ARR_R(tyres);
	AC_SER_ARR_P(suspensions);
	AC_SER_FIELD(brakeSystem);
	//Autoclutch autoClutch;
	AC_SER_FIELD(physicsGUID);
	//Telemetry telemetry;
	//AutoBlip autoBlip;
	//AutoShifter autoShift;
	//GearChanger gearChanger;
	//EDL edl;
	AC_SER_ARR_R(antirollBars);
	//StabilityControl stabilityControl;
	AC_SER_FIELD(lastCollisionTime);
	//DriftModeComponent driftMode;
	//PerformanceMeter performanceMeter;
	//SetupManager setupManager;
	AC_SER_FIELD(screenName);
	//DRS drs;
	//Kers kers;
	//ERS ers;
	//LapInvalidator lapInvalidator;
	//PenaltyManager penaltyManager;
	AC_SER_FIELD(isGearboxLocked);
	AC_SER_FIELD(isGentleStopping);
	AC_SER_FIELD(penaltyPerfTarget);
	AC_SER_FIELD(pitPosition);
	//SplineLocatorData splineLocatorData;
	//FuelLapEvaluator fuelLapEvaluator;
	AC_SER_ARR_R(heaveSprings);
	AC_SER_FIELD(steeringSystem);
	AC_SER_FIELD(lastCollisionWithCarTime);
	AC_SER_FIELD_T(suspensionTypeF, int);
	AC_SER_FIELD_T(suspensionTypeR, int);
	AC_SER_FIELD(axleTorqueReaction);
	AC_SER_FIELD(lastGyroFF);
	AC_SER_FIELD(lastFF);
	AC_SER_FIELD(lastPureMZFF);
	AC_SER_FIELD(water);
	AC_SER_FIELD(steerAssist);
	AC_SER_FIELD(isRequestingPitStop);
	AC_SER_FIELD(aiLapsToComplete);
	AC_SER_FIELD(lightsOn);
	//CarCollisionBounds bounds;
	//SlipStream slipStream;
	AC_SER_FIELD_T(torqueModeEx, int);
	AC_SER_FIELD(isControlsLocked);
	AC_SER_FIELD(lockControlsTime);
	AC_SER_FIELD(blackFlagged);
	AC_SER_FIELD(penaltyTimeAccumulator);
	AC_SER_FIELD(penaltyTime);
	AC_SER_FIELD(expectedFuelPerLap);
	AC_SER_FIELD(meshColliderBodyMatrix);
	//ICarControlsProvider* controlsProvider;
	AC_SER_FIELD(lastVelocity);
	AC_SER_FIELD(sleepingFrames);
	AC_SER_FIELD(mzCurrent);
	AC_SER_FIELD(bodyInertia);
	AC_SER_FIELD(explicitInertia);
	AC_SER_FIELD(fuel);
	AC_SER_FIELD(fuelConsumptionK);
	AC_SER_FIELD(maxFuel);
	AC_SER_FIELD(requestedFuel);
	AC_SER_FIELD(lastBodyMassUpdateTime);
	AC_SER_ARR_R(damageZoneLevel);
	AC_SER_ARR_R(ridePickupPoint);
	AC_SER_FIELD(fuelTankPos);
	AC_SER_FIELD(vibrationPhase);
	AC_SER_FIELD(slipVibrationPhase);
	AC_SER_FIELD(flatSpotPhase);
	AC_SER_FIELD(slipStreamEffectGain);
	AC_SER_FIELD(framesToSleep);
	AC_SER_FIELD(disableMinSpeedPenaltyClear);
	AC_SER_FIELD(ballastKG);
	AC_SER_FIELD(lastSteerPosition);
	//SplineLocator splineLocator;
	AC_SER_FIELD(isCollisionOffForPits);
	//PitStopTimings pitTimings;
	AC_SER_FIELD(gridPosition);
	AC_SER_FIELD(hasGridPosition);
	AC_SER_FIELD(lastLigthSwitchState);
	//PhysicsValueCache valueCache;
	AC_SER_FIELD(fuelKG);
	AC_SER_FIELD(externalControl);
	AC_SER_END();
	return out;
}
