#include "precompiled.h"
#include "Timer.h"

AC_GREF_DECL(bool, isUsingQPT, 0x151D140)
AC_GREF_DECL(LARGE_INTEGER, startTime, 0x155A590)
AC_GREF_DECL(LARGE_INTEGER, frequency, 0x155A598)
AC_GREF_DECL(unsigned int, startTGT, 0x155A5A8)

void ksInitTimerVars()
{
	AC_GREF_INIT(isUsingQPT)
	AC_GREF_INIT(startTime)
	AC_GREF_INIT(frequency)
	AC_GREF_INIT(startTGT)
}

void ksReInitTimer()
{
	startTGT = timeGetTime();
	QueryPerformanceCounter(&startTime);
}

double ksGetSystemTime()
{
	return (double)timeGetTime();
}

double ksGetTime()
{
	if (!isUsingQPT)
		return (double)(timeGetTime() - startTGT);

	LARGE_INTEGER qpc;
	QueryPerformanceCounter(&qpc);
	return (double)(qpc.QuadPart - startTime.QuadPart) * 1000.0 / (double)frequency.QuadPart;
}

double ksGetQPTTime()
{
	LARGE_INTEGER qpc;
	QueryPerformanceCounter(&qpc);
	return (double)(qpc.QuadPart - startTime.QuadPart) * 1000.0 / (double)frequency.QuadPart;
}
