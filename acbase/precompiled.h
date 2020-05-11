#pragma once

#define NOMINMAX
#define DIRECTINPUT_VERSION 0x0800

// win
#include <windows.h>
#include <winsock.h>
#include <d3d11.h>
#include <dinput.h>
#include <DirectXMath.h>
#include <shlobj.h>
#include <tlhelp32.h>

// c
#include <stdio.h>
#include <stdarg.h>

// cpp
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <functional>
#include <vector>
#include <deque>
#include <map>
#include <unordered_map>

// threading
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <concurrent_queue.h>

// third party
#include <MinHook.h>

// shared stuff
#include "Base/common.h"
#include "Base/os.h"
#include "Base/log.h"
#include "Base/mempatch.h"
#include "Base/hook.h"
#include "Base/vthook.h"
#include "Base/udt.h"

// AC SDK
#include "SDK/ac_sdk.h"
