#pragma once

#define RVA_TyreThermalModel_step 2810688

///////////////////////////////////////////////////////////////////////////////////////////////////

void TyreThermalModel_step(TyreThermalModel* pThis, float dt, float angularSpeed, float camberRAD)
{
	Car* pCar = pThis->car;

	float fPhase = (float)pThis->phase + (angularSpeed * dt);
	if (fPhase > 100000.0) fPhase -= 100000.0;
	else if (fPhase < 0.0) fPhase += 100000.0;
	pThis->phase = fPhase;

	float fAmbientTemp = pCar ? pCar->ksPhysics->ambientTemperature : 26.0f;
	float fCoreTempInput = tmax(fAmbientTemp, pThis->coreTInput);
	pThis->coreTemp += ((fCoreTempInput - pThis->coreTemp) * (pThis->patchData.internalCoreTransfer * dt));
	pThis->coreTInput = 0;

	float fSpeed = pCar ? Car_getSpeedValue(pCar) : 17.32f; // sqrt(300)
	float fAmbientFactor = ((((fSpeed * fSpeed) * pThis->patchData.coolFactorGain) + 1.0f) * pThis->patchData.surfaceTransfer) * dt;
	float fPctDt = pThis->patchData.patchCoreTransfer * dt;

	if (pCar)
	{
		for (auto& patch : pThis->patches)
		{
			float fInputT = patch.inputT;
			float fPatchT = patch.T;

			if (fInputT <= fAmbientTemp)
				fPatchT += ((fAmbientTemp - fPatchT) * fAmbientFactor);
			else
				fPatchT += ((fInputT - fPatchT) * (pThis->patchData.surfaceTransfer * dt));

			for (auto* conn : patch.connections)
			{
				fPatchT += (conn->T - fPatchT) * (pThis->patchData.patchTransfer * dt);
			}

			fPatchT += (pThis->coreTemp - fPatchT) * fPctDt;
			patch.T = fPatchT;
			patch.inputT = 0;

			pThis->coreTemp += ((fPatchT - pThis->coreTemp) * fPctDt);
		}
	}

	if (pThis->isActive)
	{
		if (pThis->performanceCurve.getCount() > 0)
		{
			float fPracT = ((pThis->getCurrentCPTemp(camberRAD) - pThis->coreTemp) * 0.25f) + pThis->coreTemp;
			pThis->practicalTemp = fPracT;
			pThis->thermalMultD = pThis->performanceCurve.getValue(fPracT);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TyreThermalModel_addThermalCoreInput(TyreThermalModel* pThis, float temp)
{
	pThis->coreTInput += temp;
}

void TyreThermalModel_addThermalInput(TyreThermalModel* pThis, float xpos, float pressureRel, float temp)
{
	// TODO
}

float TyreThermalModel_getAvgSurfaceTemp(TyreThermalModel* pThis)
{
	float fSum = 0;
	for (auto& patch : pThis->patches)
	{
		fSum += patch.T;
	}
	return fSum / 36.0f;
}

float TyreThermalModel_getCorrectedD(TyreThermalModel* pThis, float d, float camberRAD)
{
	if (pThis->isActive)
		return d * pThis->thermalMultD;
	return d;
}

float TyreThermalModel_getCurrentCPTemp(TyreThermalModel* pThis, float camber)
{
	// TODO
	return 0;
}

void TyreThermalModel_getIMO(TyreThermalModel* pThis, float* pfOut)
{
	for (int i = 0; i < 3; ++i)
	{
		float fSum = 0;
		for (int j = 0; j < 12; ++j)
		{
			auto& patch = pThis->patches[j + i * pThis->elements];
			fSum += patch.T;
		}
		*pfOut++ = fSum / 12.0f;
	}
}

TyreThermalPatch* TyreThermalModel_getPatchAt(TyreThermalModel* pThis, int x, int y)
{
	if (x >= 0 && x < pThis->stripes && y >= 0 && y < pThis->elements)
		return &pThis->patches[y + x * pThis->elements];

	SHOULD_NOT_REACH;
	return &pThis->patches[0];
}

float TyreThermalModel_getPracticalTemp(TyreThermalModel* pThis, float camberRAD)
{
	return ((pThis->getCurrentCPTemp(camberRAD) - pThis->coreTemp) * 0.25f) + pThis->coreTemp;
}

void TyreThermalModel_setTemperature(TyreThermalModel* pThis, float optimumTemp)
{
	pThis->coreTemp = optimumTemp;
	for (auto& patch : pThis->patches)
	{
		patch.T = optimumTemp;
	}
}
