#include "air.h"

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

bool hasEnding(std::string const& fullString, std::string const& ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

std::map<int, std::string> get_log_keys(const air_config* cfg, std::map<std::string, int> keys) {
    std::vector<std::string> vec_list;
    if (cfg->LOG_SENSOR_DATA) {
        vec_list.push_back("y_");
    }
    if (cfg->LOG_ESTIMATOR_0 and cfg->ESTIMATOR_0_ENABLED) {
        vec_list.push_back("xh_0");
    }
    if (cfg->LOG_ESTIMATOR_1 and cfg->ESTIMATOR_1_ENABLED) {
        vec_list.push_back("xh_1");
    }
    if (cfg->LOG_CONTROLLER_0 and cfg->CONTROLLER_0_ENABLED) {
        vec_list.push_back("controller_0");
    }
    if (cfg->LOG_CONTROLLER_1 and cfg->CONTROLLER_1_ENABLED) {
        vec_list.push_back("controller_1");
    }
    if (cfg->LOG_RCIN_SERVO and cfg->SERVO_LOOP_ENABLED) {
        vec_list.push_back("rcin_");
        vec_list.push_back("servo_");
    }

    std::map<int, std::string> log_keys;

    for (auto const& k : keys) {
        for (std::string v : vec_list) {
            if (k.first.rfind(v, 0) == 0) {
                if (k.first.find("ADC") != std::string::npos) {
                    continue;
                }
                if (k.first.find("IMU_2") != std::string::npos && (!cfg->MPU_ENABLED || !cfg->LSM_ENABLED)) {
                    continue;
                }
                if (k.first.find("GPS") != std::string::npos && !cfg->GPS_ENABLED) {
                    continue;
                }
                if (k.first.find("BARO") != std::string::npos && !cfg->MS5611_ENABLED) {
                    continue;
                }
                std::string column_name = k.first;
                if (hasEnding(column_name, "CHANNEL_" + std::to_string(cfg->THROTTLE_CHANNEL))) {
                    replace(column_name, "CHANNEL_" + std::to_string(cfg->THROTTLE_CHANNEL), "THROTTLE");
                } else if (hasEnding(column_name, "CHANNEL_" + std::to_string(cfg->ELEVATOR_CHANNEL))) {
                    replace(column_name, "CHANNEL_" + std::to_string(cfg->ELEVATOR_CHANNEL), "ELEVATOR");
                } else if (hasEnding(column_name, "CHANNEL_" + std::to_string(cfg->AILERON_CHANNEL))) {
                    replace(column_name, "CHANNEL_" + std::to_string(cfg->AILERON_CHANNEL), "AILERON");
                } else if (hasEnding(column_name, "CHANNEL_" + std::to_string(cfg->RUDDER_CHANNEL))) {
                    replace(column_name, "CHANNEL_" + std::to_string(cfg->RUDDER_CHANNEL), "RUDDER");
                } else if (hasEnding(column_name, "CHANNEL_" + std::to_string(cfg->FLAPS_CHANNEL))) {
                    replace(column_name, "CHANNEL_" + std::to_string(cfg->FLAPS_CHANNEL), "FLAPS");
                } else if (hasEnding(column_name, "CHANNEL_" + std::to_string(cfg->FLIGHT_MODE_CHANNEL))) {
                    replace(column_name, "CHANNEL_" + std::to_string(cfg->FLIGHT_MODE_CHANNEL), "MODE");
                } else if (column_name.find("CHANNEL_") != std::string::npos) {
                    std::stringstream ss(column_name);
                    std::string segment;
                    while (std::getline(ss, segment, '_'));
                    try {
                        int c = std::stoi(segment) + 1; // increment channels for 1-based indexing in log.
                        replace(column_name, segment, std::to_string(c));
                    }
                    catch (...) { continue; }
                }
                log_keys.insert(std::pair<int, std::string>(k.second, column_name));
            }
        }
    }
    return log_keys;
}

void* logger_loop(void* arguments) {
    thread_struct* args = (thread_struct*)arguments;
    double* array = args->array;
    const air_config* cfg = args->cfg;
    std::map<std::string, int> keys = cfg->keys;
    usleep(3000000);
    int max_sleep = hertz_to_microseconds(cfg->LOGGER_LOOP_RATE);
    static char filename[36];
    time_t filename_time = time(0);
    uint64_t first_timestamp = current_time_microseconds();
    strftime(filename, sizeof(filename), "data/logs/%Y-%m-%d_%H.%M.%S.csv", localtime(&filename_time));
    std::ofstream log;
    log.open(filename);
    std::map<int, std::string> log_keys = get_log_keys(cfg, keys);
    log << "TIMESTAMP" << ", ";
    for (auto const& k : log_keys) {
        log << k.second << ", ";
    }
    log << std::endl;
    uint64_t t0, now;
    while (true) {
        t0 = current_time_microseconds();
        double stamp = (t0 - first_timestamp) / 1000000.0;
        log << stamp << ", ";
        for (auto const& k : log_keys) {
            if (k.second == "servo_MODE_FLAG") {
                if (array[k.first] == 0) {
                    log << "MANUAL" << ", ";
                } else if (array[k.first] == 1) {
                    log << "SEMI-AUTO" << ", ";
                } else if (array[k.first] == 2) {
                    log << "AUTO" << ", ";
                }
            } else {
                log << array[k.first] << ", ";
            }
        }
        log << std::endl;
        log.flush();
        now = current_time_microseconds();
        int sleep_time = (int)(max_sleep - (now - t0));
        usleep(std::max(sleep_time, 0));
    }
}