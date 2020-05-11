#pragma once

void log_init(const wchar_t* filename);
void log_printf(const wchar_t* format, ...);
void log_close();
