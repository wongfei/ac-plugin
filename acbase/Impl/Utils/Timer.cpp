#include "precompiled.h"
#include "Timer.h"

AC_GVAR_DECL(bool, isUsingQPT, 0x151D140)
AC_GVAR_DECL(LARGE_INTEGER, startTime, 0x155A590)
AC_GVAR_DECL(LARGE_INTEGER, frequency, 0x155A598)
AC_GVAR_DECL(unsigned int, startTGT, 0x155A5A8)

void ksInitTimerVars()
{
	AC_GVAR_INIT(isUsingQPT)
	AC_GVAR_INIT(startTime)
	AC_GVAR_INIT(frequency)
	AC_GVAR_INIT(startTGT)
}

void ksReInitTimer()
{
	AC_GVAR(startTGT) = timeGetTime();
	QueryPerformanceCounter(AC_GPTR(startTime));
}

double ksGetSystemTime()
{
	return (double)timeGetTime();
}

double ksGetTime()
{
	if (!AC_GVAR(isUsingQPT))
		return (double)(timeGetTime() - AC_GVAR(startTGT));

	LARGE_INTEGER qpc;
	QueryPerformanceCounter(&qpc);
	return (double)(qpc.QuadPart - AC_GVAR(startTime).QuadPart) * 1000.0 / (double)AC_GVAR(frequency).QuadPart;
}

double ksGetQPTTime()
{
	LARGE_INTEGER qpc;
	QueryPerformanceCounter(&qpc);
	return (double)(qpc.QuadPart - AC_GVAR(startTime).QuadPart) * 1000.0 / (double)AC_GVAR(frequency).QuadPart;
}
