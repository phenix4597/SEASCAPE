#include "air.h"

#include "Navio2/C++/Ublox.h"
#include "Navio2/C++/MS5611.h"


Ublox* initialize_gps(int milliseconds) {
    Ublox* gps = new Ublox();
    printf("Initializing GPS.\n");
    if (gps->testConnection()) {
        gps->configureSolutionRate(milliseconds);
    } else {
        printf("GPS initialization FAILED.\n");
        return nullptr;
    }
    return gps;
}

MS5611* initialize_baro() {
    MS5611* barometer = new MS5611();
    printf("Initializing Barometer.\t\t\t[MS5611]\n");
    barometer->initialize();
    if (!barometer->testConnection()) {
        printf("Barometer initialization FAILED.\n");
        return nullptr;
    }
    barometer->refreshPressure();
    return barometer;
}


void* gps_baro_loop(void* arguments) {
    thread_struct* args = (thread_struct*)arguments;
    double* array = args->array;
    const air_config* cfg = args->cfg;
    std::map<std::string, int> keys = cfg->keys;
    usleep(150000);
    Ublox* gps;
    MS5611* barometer;
    if (cfg->GPS_ENABLED) {
        gps = initialize_gps(200);
    }
    if (cfg->MS5611_ENABLED) {
        barometer = initialize_baro();
    }
    usleep(350000);
    int max_sleep = hertz_to_microseconds(cfg->GPS_LOOP_RATE);
    uint64_t start_time, now;

    while (true) {
        start_time = current_time_microseconds();
        if (cfg->GPS_ENABLED && gps->decodeMessages(array, keys)) {
            array[keys["y_GPS_UPDATES"]]++;
        }
        if (cfg->MS5611_ENABLED) {
            barometer->refreshPressure();
            usleep(10000);
            barometer->readPressure();
            barometer->calculatePressureAndTemperature();
            array[keys["y_BARO_PRES"]] = barometer->getPressure();
        }
        now = current_time_microseconds();
        int remaining = (int)(max_sleep - (now - start_time));
        if (remaining > 0) {
            usleep(remaining);
        } else {
            // If we're behind schedule, yield briefly to avoid busy-wait
            usleep(1000);
        }
    }
}