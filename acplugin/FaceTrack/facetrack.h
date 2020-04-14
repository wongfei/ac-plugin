#ifndef _FACETRACK_H_
#define _FACETRACK_H_

#include <stdint.h>

#define FACETRACK_VER_MAJOR 0
#define FACETRACK_VER_MINOR 1

#define FACETRACK_OK 0
#define FACETRACK_ERROR 1
#define FACETRACK_INVALID_OPERATION 2
#define FACETRACK_INVALID_PARAM 3

#ifdef __cplusplus
#define FACETRACK_LINKAGE extern "C"
#else
#define FACETRACK_LINKAGE
#endif

#ifdef FACETRACK_EXPORTS
#define FACETRACK_API FACETRACK_LINKAGE __declspec(dllexport)
#else
#define FACETRACK_API FACETRACK_LINKAGE __declspec(dllimport)
#endif

typedef int facetrack_err;
typedef void* facetrack_handle;

struct facetrack_version {
	uint8_t major;
	uint8_t minor;
};

struct facetrack_config {
	uint8_t draw_debug_info;
	uint8_t cam_id;
	uint16_t resx;
	uint16_t resy;
	uint16_t fps;
	float cx;
	float cy;
	float focal_length;
};

struct facetrack_pose {
	float matrix[16];
	float euler_raw[3];
	float euler_kf[3];
	uint16_t resx;
	uint16_t resy;
	uint16_t rect[4];
	uint8_t is_tracking;
};

struct facetrack_feature2d {
	float x, y;
};

typedef facetrack_version(*pft_get_version)();
typedef facetrack_err(*pft_initialize)(facetrack_config* config, facetrack_handle* out_handle);
typedef facetrack_err(*pft_shutdown)(facetrack_handle handle);
typedef facetrack_err(*pft_get_pose)(facetrack_handle handle, facetrack_pose* out_pose);
typedef facetrack_err(*pft_get_features2d)(facetrack_handle handle, int* count, facetrack_feature2d* out_features);

struct facetrack_api {
	pft_get_version get_version;
	pft_initialize initialize;
	pft_shutdown shutdown;
	pft_get_pose get_pose;
	pft_get_features2d get_features2d;
};

typedef facetrack_err(*pft_get_api)(facetrack_api* out_api);
FACETRACK_API facetrack_err ft_get_api(facetrack_api* out_api);

#endif
