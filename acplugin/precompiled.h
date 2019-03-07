#pragma once

#define DIRECTINPUT_VERSION 0x0800

// win
#include <windows.h>
#include <winsock.h>
#include <d3d11.h>
#include <dinput.h>
#include <shlobj.h>
#include <tlhelp32.h>

// c
#include <stdio.h>
#include <stdarg.h>

// cpp
#include <memory>
#include <string>
#include <iostream>
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

// deps
#include "MinHook.h"

// AC
#include "AC/ac_sdk.h"
