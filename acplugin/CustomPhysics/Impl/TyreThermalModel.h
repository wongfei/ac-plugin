#pragma once

BEGIN_HOOK_OBJ(TyreThermalModel)

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

	void _step(float dt, float angularSpeed, float camberRAD);
	TyreThermalPatch* _getPatchAt(int x, int y);
	float _getCorrectedD(float d, float camberRAD);
	void _getIMO(float* pfOut);
	void _addThermalCoreInput(float temp);
	void _addThermalInput(float xpos, float pressureRel, float temp);
	float _getCurrentCPTemp(float camber);
	float _getPracticalTemp(float camberRAD);
	float _getAvgSurfaceTemp(TyreThermalModel* pThis);
	void _setTemperature(float optimumTemp);
	void _reset(TyreThermalModel* pThis);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _TyreThermalModel::_step(float dt, float angularSpeed, float camberRAD)
{
	Car* pCar = this->car;

	float fPhase = (float)this->phase + (angularSpeed * dt);
	if (fPhase > 100000.0) fPhase -= 100000.0;
	else if (fPhase < 0.0) fPhase += 100000.0;
	this->phase = fPhase;

	float fAmbientTemp = pCar ? pCar->ksPhysics->ambientTemperature : 26.0f;
	float fCoreTempInput = tmax(fAmbientTemp, this->coreTInput);
	this->coreTemp += ((fCoreTempInput - this->coreTemp) * (this->patchData.internalCoreTransfer * dt));
	this->coreTInput = 0;

	float fSpeed = pCar ? getSpeedMS(pCar) : 17.32f; // sqrt(300)
	float fAmbientFactor = ((((fSpeed * fSpeed) * this->patchData.coolFactorGain) + 1.0f) * this->patchData.surfaceTransfer) * dt;
	float fPctDt = this->patchData.patchCoreTransfer * dt;

	if (pCar)
	{
		for (auto& patch : this->patches)
		{
			float fInputT = patch.inputT;
			float fPatchT = patch.T;

			if (fInputT <= fAmbientTemp)
				fPatchT += ((fAmbientTemp - fPatchT) * fAmbientFactor);
			else
				fPatchT += ((fInputT - fPatchT) * (this->patchData.surfaceTransfer * dt));

			for (auto* conn : patch.connections)
			{
				fPatchT += (conn->T - fPatchT) * (this->patchData.patchTransfer * dt);
			}

			fPatchT += (this->coreTemp - fPatchT) * fPctDt;
			patch.T = fPatchT;
			patch.inputT = 0;

			this->coreTemp += ((fPatchT - this->coreTemp) * fPctDt);
		}
	}

	if (this->isActive)
	{
		if (this->performanceCurve.getCount() > 0)
		{
			float fPracT = ((this->getCurrentCPTemp(camberRAD) - this->coreTemp) * 0.25f) + this->coreTemp;
			this->practicalTemp = fPracT;
			this->thermalMultD = this->performanceCurve.getValue(fPracT);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TyreThermalPatch* _TyreThermalModel::_getPatchAt(int x, int y)
{
	if (x >= 0 && x < this->stripes && y >= 0 && y < this->elements)
		return &this->patches[y + x * this->elements];

	SHOULD_NOT_REACH;
	return &this->patches[0];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _TyreThermalModel::_getCorrectedD(float d, float camberRAD)
{
	if (this->isActive)
		return d * this->thermalMultD;
	return d;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _TyreThermalModel::_getIMO(float* pfOut)
{
	for (int i = 0; i < 3; ++i)
	{
		float fSum = 0;
		for (int j = 0; j < 12; ++j)
		{
			auto& patch = this->patches[j + i * this->elements];
			fSum += patch.T;
		}
		*pfOut++ = fSum / 12.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _TyreThermalModel::_addThermalCoreInput(float temp)
{
	this->coreTInput += temp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _TyreThermalModel::_addThermalInput(float xpos, float pressureRel, float temp)
{
	float fNormXcs = tclamp((xpos * this->camberSpreadK), -1.0f, 1.0f);

	float fPhase = (float)(this->phase * 0.1591549430964443);
	int iElemY = ((int)(fPhase * this->elements)) % this->elements;

	float fT = (this->car ? this->car->ksPhysics->roadTemperature : 26.0f) + temp;
	float fPr1 = pressureRel * 0.1f;
	float fPr2 = (pressureRel * -0.5f) + 1.0f;

	auto& patch0 = this->getPatchAt(0, iElemY);
	patch0.inputT += ((((fNormXcs + 1.0f) - (fPr1 * 0.5f)) * fPr2) * fT);

	auto& patch1 = this->getPatchAt(1, iElemY);
	patch1.inputT += (((fPr1 + 1.0f) * fPr2) * fT);

	auto& patch2 = this->getPatchAt(2, iElemY);
	patch2.inputT += ((((1.0f - fNormXcs) - (fPr1 * 0.5f)) * fPr2) * fT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _TyreThermalModel::_getCurrentCPTemp(float camber)
{
	float fNormCsk = tclamp((camber * this->camberSpreadK), -1.0f, 1.0f);

	float fPhase = (float)(this->phase * 0.1591549430964443);
	int iElemY = ((int)(fPhase * this->elements)) % this->elements;

	auto& patch0 = this->getPatchAt(0, iElemY);
	auto& patch1 = this->getPatchAt(1, iElemY);
	auto& patch2 = this->getPatchAt(2, iElemY);

	return ((((fNormCsk + 1.0f) * patch0.T) + patch1.T) + ((1.0f - fNormCsk) * patch2.T)) * 0.33333334f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _TyreThermalModel::_getPracticalTemp(float camberRAD)
{
	return ((this->getCurrentCPTemp(camberRAD) - this->coreTemp) * 0.25f) + this->coreTemp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _TyreThermalModel::_getAvgSurfaceTemp(TyreThermalModel* pThis)
{
	float fSum = 0;
	for (auto& patch : this->patches)
	{
		fSum += patch.T;
	}
	return fSum / 36.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _TyreThermalModel::_setTemperature(float optimumTemp)
{
	this->coreTemp = optimumTemp;
	for (auto& patch : this->patches)
	{
		patch.T = optimumTemp;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _TyreThermalModel::_reset(TyreThermalModel* pThis)
{
	float fT = this->car ? this->car->ksPhysics->ambientTemperature : 26.0f;
	this->setTemperature(fT);
	this->phase = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
