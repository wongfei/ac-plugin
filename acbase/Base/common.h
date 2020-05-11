#pragma once

#define UT_CONCAT(A, B) A##B
#define UT_WSTRING(A) UT_CONCAT(L, #A)

template<typename T>
inline T tmin(const T& a, const T& b) { return (a < b ? a : b); }

template<typename T>
inline T tmax(const T& a, const T& b) { return (a > b ? a : b); }

template<typename T>
inline T tclamp(const T& x, const T& a, const T& b) { return (x < a ? a : (x > b ? b : x)); }

inline float signf(const float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }
//inline float signf(const float x) { return x < 0.0f ? -1.0f : 1.0f; }

template<typename T, typename E>
inline void remove_elem(std::vector<T>& vec, E elem) {
	auto it = std::find(vec.begin(), vec.end(), static_cast<T>(elem));
	if (it != vec.end()) {
		vec.erase(it);
	}
}

void* ac_get_module();
void ac_set_module(void* base);

std::wstring strf(const wchar_t* format, ...);
