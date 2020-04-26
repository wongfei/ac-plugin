#pragma once

#include "DriverPerf.h"

#define DPERF_UPDATE_FREQ_MS 100
#define DPERF_DATALOG_FREQ_MS 500
#define DPERF_DATALOG_BUFFER_MAX 1000

#define DPERF_AVG_DATALOG (DPERF_DATALOG_FREQ_MS / DPERF_UPDATE_FREQ_MS)

struct DriverState
{
	CarAvatar* avatar = nullptr;
	struct CarIni* carIni = nullptr;
	bool online = false;

	std::wstring driverName;
	std::wstring carName;
	std::function<void()> onDestroy;

	PerfTable<float> perf;
	PerfTable< AvgValue<float, DPERF_AVG_DATALOG> > avg;
	std::vector<PerfTable<float> > datalog;
	vec3f pos;
	vec3f vel;
	size_t complLapTime;

	DriverState() { resetPerf(); }

	void resetPerf() {
		perf.reset();
		avg.reset();
		datalog.clear();
		pos = vec3f(0, 0, 0);
		vel = vec3f(0, 0, 0);
		complLapTime = 0;
	}

	void computeAvg() {
		for (int i = 0; i < perf.count; ++i) {
			avg[i] = perf[i];
		}
	}
};
