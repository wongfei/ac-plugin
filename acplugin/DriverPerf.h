#pragma once

enum class ePerfParam : int {
	Rpm = 0,
	Vel,
	Acc,
	Power,
	COUNT
};

template<typename T, int N>
struct AvgValue
{
	enum { count = N };

	T values[N];
	T sum;
	int id;

	AvgValue() { reset(); }
	AvgValue(const AvgValue& other) { *this = other; }

	void reset() {
		for (int i = 0; i < N; ++i) {
			values[i] = T(0);
		}
		sum = T(0);
		id = 0;
	}

	AvgValue& operator=(const AvgValue& other) {
		for (int i = 0; i < N; ++i) {
			values[i] = other.values[i];
		}
		return *this;
	}

	AvgValue& operator=(const T& value) { 
		sum = sum - values[id];
		values[id] = value;
		sum = sum + value;
		if (++id >= N) {
			id = 0;
		}
		return *this;
	}

	const T val() const {
		return (sum / (T)N);
	}

	const T maxv() const {
		T v(0);
		for (int i = 0; i < N; ++i) {
			v = tmax(v, values[i]);
		}
		return v;
	}

	inline operator const T() const { return val(); }
};

template<typename T, int N>
inline void resetv(AvgValue<T, N>& v) { v.reset(); }
inline void resetv(float& v) { v = 0; }
inline void resetv(double& v) { v = 0; }

template<typename T, int N = (int)ePerfParam::COUNT>
struct PerfTable
{
	enum { count = N };

	T values[N];

	PerfTable() { reset(); }
	PerfTable(const PerfTable& other) { *this = other; }

	void reset() {
		for (int i = 0; i < N; ++i) {
			resetv(values[i]);
		}
	}

	PerfTable& operator=(const PerfTable& other) {
		for (int i = 0; i < N; ++i) {
			values[i] = other.values[i];
		}
		return *this;
	}

	template<int S>
	PerfTable& operator=(const PerfTable< AvgValue<T, S> >& other) {
		for (int i = 0; i < N; ++i) {
			values[i] = other.values[i];
		}
		return *this;
	}

	inline T& operator[](int i) { return values[i]; }
	inline const T& operator[](int i) const { return values[i]; }

	inline T& operator[](ePerfParam i) { return values[(int)i]; }
	inline const T& operator[](ePerfParam i) const { return values[(int)i]; }
};
