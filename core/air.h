#pragma once

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <cstring>
#include <sstream>
#include <errno.h>
#include <chrono>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "air_config.h"

#define GETEKYDIR ("core/air.h")
#define PROJECTID '~'
#define SHMSIZE (4096)

struct thread_struct {
    double* array;
    const air_config* cfg;
};

uint64_t current_time_microseconds();
int hertz_to_microseconds(double hertz);

void* estimation_loop(void* arguments);
void* control_loop(void* arguments);
void* logger_loop(void* arguments);
void* imu_loop(void* arguments);
void* gps_baro_loop(void* arguments);
void* servo_loop(void* arguments);
void* telemetry_loop(void* arguments);
void* logger_loop(void* arguments);