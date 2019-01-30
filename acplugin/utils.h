#pragma once

template<typename T>
inline T tmin(const T& a, const T& b) { return (a < b ? a : b); }

template<typename T>
inline T tmax(const T& a, const T& b) { return (a > b ? a : b); }

template<typename T>
inline T tclamp(const T& x, const T& a, const T& b) { return (x < a ? a : (x > b ? b : x)); }

template<typename T>
inline void safe_delete(T*& obj) { if (obj) { delete obj; obj = nullptr; } }

template<typename T, typename E>
inline void remove_elem(std::vector<T>& vec, E elem) {
	auto it = std::find(vec.begin(), vec.end(), static_cast<T>(elem));
	if (it != vec.end()) {
		vec.erase(it);
	}
}

std::wstring strf(const wchar_t* format, ...);
std::wstring getDocumentsPath();
bool fileExists(LPCWSTR Path);
bool dirExists(LPCWSTR Path);
void ensureDirExists(const std::wstring& path);
