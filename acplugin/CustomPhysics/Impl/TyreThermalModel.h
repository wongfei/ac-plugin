#pragma once

#define RVA_TyreThermalModel_step 2810688
#define RVA_TyreThermalModel_getPatchAt 2809776
#define RVA_TyreThermalModel_getCorrectedD 2808768
#define RVA_TyreThermalModel_getIMO 2809264
#define RVA_TyreThermalModel_addThermalCoreInput 2805872
#define RVA_TyreThermalModel_addThermalInput 2805904
#define RVA_TyreThermalModel_getCurrentCPTemp 2808800
#define RVA_TyreThermalModel_getPracticalTemp 2810176
#define RVA_TyreThermalModel_getAvgSurfaceTemp 2808720
#define RVA_TyreThermalModel_setTemperature 2810640
#define RVA_TyreThermalModel_reset 2810304

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

TyreThermalPatch* TyreThermalModel_getPatchAt(TyreThermalModel* pThis, int x, int y)
{
	if (x >= 0 && x < pThis->stripes && y >= 0 && y < pThis->elements)
		return &pThis->patches[y + x * pThis->elements];

	SHOULD_NOT_REACH;
	return &pThis->patches[0];
}

float TyreThermalModel_getCorrectedD(TyreThermalModel* pThis, float d, float camberRAD)
{
	if (pThis->isActive)
		return d * pThis->thermalMultD;
	return d;
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

void TyreThermalModel_addThermalCoreInput(TyreThermalModel* pThis, float temp)
{
	pThis->coreTInput += temp;
}

void TyreThermalModel_addThermalInput(TyreThermalModel* pThis, float xpos, float pressureRel, float temp)
{
	float fNormXcs = tclamp((xpos * pThis->camberSpreadK), -1.0f, 1.0f);

	float fPhase = (float)(pThis->phase * 0.1591549430964443);
	int iElemY = ((int)(fPhase * pThis->elements)) % pThis->elements;

	float fT = (pThis->car ? pThis->car->ksPhysics->roadTemperature : 26.0f) + temp;
	float fPr1 = pressureRel * 0.1f;
	float fPr2 = (pressureRel * -0.5f) + 1.0f;

	auto& patch0 = pThis->getPatchAt(0, iElemY);
	patch0.inputT += ((((fNormXcs + 1.0f) - (fPr1 * 0.5f)) * fPr2) * fT);

	auto& patch1 = pThis->getPatchAt(1, iElemY);
	patch1.inputT += (((fPr1 + 1.0f) * fPr2) * fT);

	auto& patch2 = pThis->getPatchAt(2, iElemY);
	patch2.inputT += ((((1.0f - fNormXcs) - (fPr1 * 0.5f)) * fPr2) * fT);
}

float TyreThermalModel_getCurrentCPTemp(TyreThermalModel* pThis, float camber)
{
	float fNormCsk = tclamp((camber * pThis->camberSpreadK), -1.0f, 1.0f);

	float fPhase = (float)(pThis->phase * 0.1591549430964443);
	int iElemY = ((int)(fPhase * pThis->elements)) % pThis->elements;

	auto& patch0 = pThis->getPatchAt(0, iElemY);
	auto& patch1 = pThis->getPatchAt(1, iElemY);
	auto& patch2 = pThis->getPatchAt(2, iElemY);

	return ((((fNormCsk + 1.0f) * patch0.T) + patch1.T) + ((1.0f - fNormCsk) * patch2.T)) * 0.33333334f;
}

float TyreThermalModel_getPracticalTemp(TyreThermalModel* pThis, float camberRAD)
{
	return ((pThis->getCurrentCPTemp(camberRAD) - pThis->coreTemp) * 0.25f) + pThis->coreTemp;
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

void TyreThermalModel_setTemperature(TyreThermalModel* pThis, float optimumTemp)
{
	pThis->coreTemp = optimumTemp;
	for (auto& patch : pThis->patches)
	{
		patch.T = optimumTemp;
	}
}

void TyreThermalModel_reset(TyreThermalModel* pThis)
{
	float fT = pThis->car ? pThis->car->ksPhysics->ambientTemperature : 26.0f;
	pThis->setTemperature(fT);
	pThis->phase = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
