#pragma once
#include "api.h"
#include <math.h>

#define TIME_SECONDS (int)(round(pros::c::millis() / 1000))

// log with arguments
#define LOG(mode, format, ...) printf("[%i:%02d " mode "] " LOGGER ": " format "\n", (int)floor(TIME_SECONDS / 60), TIME_SECONDS % 60, __VA_ARGS__)
// log without arguments
#define LOG_TEXT(mode, format) printf("[%i:%02d " mode "] " LOGGER ": " format "\n", (int)floor(TIME_SECONDS / 60), TIME_SECONDS % 60)

// log levels with arguments
#define DEBUG(format, ...) LOG("DEBUG", format, __VA_ARGS__)
#define INFO(format, ...) LOG("\033[34mINFO\033[0m", format, __VA_ARGS__)
#define WARN(format, ...) LOG("\033[33mWARN\033[0m", format, __VA_ARGS__)
#define ERROR(format, ...) LOG("\033[31mERROR\033[0m", format, __VA_ARGS__)

// logs levels without arguments
#define DEBUG_TEXT(format) LOG_TEXT("DEBUG", format)
#define INFO_TEXT(format) LOG_TEXT("\033[34mINFO\033[0m", format)
#define WARN_TEXT(format) LOG_TEXT("\033[33mWARN\033[0m", format)
#define ERROR_TEXT(format) LOG_TEXT("\033[31mERROR\033[0m", format)