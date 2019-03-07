#include "precompiled.h"
#include "Timer.h"

static unsigned int* startTGT;
static LARGE_INTEGER* startTime;
static LARGE_INTEGER* frequency;
static bool* isUsingQPT;

void ksInitTimerVars()
{
	startTGT = (unsigned int*)_drva(0x155A5A8);
	startTime = (LARGE_INTEGER*)_drva(0x155A590);
	frequency = (LARGE_INTEGER*)_drva(0x155A598);
	isUsingQPT = (bool*)_drva(0x151D140);
}

void ksReInitTimer()
{
	*startTGT = timeGetTime();
	QueryPerformanceCounter(startTime);
}

double ksGetSystemTime()
{
	return (double)timeGetTime();
}

double ksGetTime()
{
	if (!*isUsingQPT)
		return (double)(timeGetTime() - *startTGT);

	LARGE_INTEGER PerformanceCount;
	QueryPerformanceCounter(&PerformanceCount);
	return (double)(PerformanceCount.QuadPart - startTime->QuadPart) * 1000.0 / (double)frequency->QuadPart;
}

double ksGetQPTTime()
{
	LARGE_INTEGER PerformanceCount;
	QueryPerformanceCounter(&PerformanceCount);
	return (double)(PerformanceCount.QuadPart - startTime->QuadPart) * 1000.0 / (double)frequency->QuadPart;
}
