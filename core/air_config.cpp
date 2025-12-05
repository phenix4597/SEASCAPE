#include "air.h"
#include "air_config.h"

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

air_config::air_config() {

    Json::Reader reader;
    Json::Value cfg;

    std::ifstream file("config.json"); // we can assume config file has been verified by prelaunch_checks.py

    if (!reader.parse(file, cfg)) {
        std::cout << reader.getFormattedErrorMessages();
        exit(1);
    }

    auto threads = cfg["THREADS"];

    IMU_LOOP_RATE = threads["IMU_ADC"]["RATE"].asDouble();
    GPS_LOOP_RATE = threads["GPS_BAROMETER"]["RATE"].asDouble();
    SERVO_LOOP_RATE = threads["RCIN_SERVO"]["RATE"].asDouble();
    ESTIMATION_LOOP_RATE = threads["ESTIMATOR_0"]["RATE"].asDouble();
    CONTROL_LOOP_RATE = threads["CONTROLLER_0"]["RATE"].asDouble();
    TELEMETRY_LOOP_RATE = threads["TELEMETRY"]["RATE"].asDouble();
    LOGGER_LOOP_RATE = threads["LOGGER"]["RATE"].asDouble();

    ESTIMATOR_0_ENABLED = threads["ESTIMATOR_0"]["ENABLED"].asBool();
    CONTROLLER_0_ENABLED = threads["CONTROLLER_0"]["ENABLED"].asBool();
    ESTIMATOR_1_ENABLED = threads["ESTIMATOR_1"]["ENABLED"].asBool();
    CONTROLLER_1_ENABLED = threads["CONTROLLER_1"]["ENABLED"].asBool();
    CONTROLLER_XH = threads["CONTROLLER_0"]["XH_VECTOR_TO_USE"].asInt();
    SERVO_CONTROLLER = threads["RCIN_SERVO"]["CONTROLLER_VECTOR_TO_USE"].asInt();
    SERVO_LOOP_ENABLED = threads["RCIN_SERVO"]["ENABLED"].asBool();
    TELEMETRY_LOOP_ENABLED = threads["TELEMETRY"]["ENABLED"].asBool();
    GPS_LOOP_ENABLED = threads["GPS_BAROMETER"]["ENABLED"].asBool();
    IMU_LOOP_ENABLED = threads["IMU_ADC"]["ENABLED"].asBool();
    LSM_ENABLED = threads["IMU_ADC"]["USE_LSM9DS1"].asBool();
    MPU_ENABLED = threads["IMU_ADC"]["USE_MPU9250"].asBool();
    ADC_ENABLED = threads["IMU_ADC"]["USE_ADC"].asBool();
    GPS_ENABLED = threads["GPS_BAROMETER"]["USE_GPS"].asBool();
    MS5611_ENABLED = threads["GPS_BAROMETER"]["USE_MS5611"].asBool();
    PRIMARY_IMU = threads["IMU_ADC"]["PRIMARY_IMU"].asString();
    if (PRIMARY_IMU != "LSM9DS1" && PRIMARY_IMU != "MPU9250") {
        if (!LSM_ENABLED && MPU_ENABLED) {
            PRIMARY_IMU = "MPU9250";
        } else {
            PRIMARY_IMU = "LSM9DS1";
        }
    }

    LOGGER_LOOP_ENABLED = threads["LOGGER"]["ENABLED"].asBool();
    LOG_ESTIMATOR_0 = threads["LOGGER"]["LOG_ESTIMATOR_0"].asBool();
    LOG_ESTIMATOR_1 = threads["LOGGER"]["LOG_ESTIMATOR_1"].asBool();
    LOG_CONTROLLER_0 = threads["LOGGER"]["LOG_CONTROLLER_0"].asBool();
    LOG_CONTROLLER_1 = threads["LOGGER"]["LOG_CONTROLLER_1"].asBool();
    LOG_RCIN_SERVO = threads["LOGGER"]["LOG_RCIN_SERVO"].asBool();

    PWM_FREQUENCY = threads["RCIN_SERVO"]["PWM_FREQUENCY"].asInt();
    MIN_THROTTLE = threads["RCIN_SERVO"]["MIN_THROTTLE"].asInt();
    MAX_THROTTLE = threads["RCIN_SERVO"]["MAX_THROTTLE"].asInt();
    MIN_SERVO = threads["RCIN_SERVO"]["MIN_SERVO"].asInt();
    MAX_SERVO = threads["RCIN_SERVO"]["MAX_SERVO"].asInt();

    // subtract one because board shows 1-based indexing, but i think drivers are 0-based.
    THROTTLE_CHANNEL = threads["RCIN_SERVO"]["THROTTLE_CHANNEL"].asInt() - 1;
    AILERON_CHANNEL = threads["RCIN_SERVO"]["AILERON_CHANNEL"].asInt() - 1;
    ELEVATOR_CHANNEL = threads["RCIN_SERVO"]["ELEVATOR_CHANNEL"].asInt() - 1;
    RUDDER_CHANNEL = threads["RCIN_SERVO"]["RUDDER_CHANNEL"].asInt() - 1;
    FLAPS_CHANNEL = threads["RCIN_SERVO"]["FLAPS_CHANNEL"].asInt() - 1;
    FLIGHT_MODE_CHANNEL = threads["RCIN_SERVO"]["FLIGHT_MODES"]["MODE_CHANNEL"].asInt() - 1;
    MANUAL_MODE_MIN = threads["RCIN_SERVO"]["FLIGHT_MODES"]["MANUAL_RANGE"]["LOW"].asInt();
    MANUAL_MODE_MAX = threads["RCIN_SERVO"]["FLIGHT_MODES"]["MANUAL_RANGE"]["HIGH"].asInt();
    SEMI_MODE_MIN = threads["RCIN_SERVO"]["FLIGHT_MODES"]["SEMI-AUTO_RANGE"]["LOW"].asInt();
    SEMI_MODE_MAX = threads["RCIN_SERVO"]["FLIGHT_MODES"]["SEMI-AUTO_RANGE"]["HIGH"].asInt();
    AUTO_MODE_MIN = threads["RCIN_SERVO"]["FLIGHT_MODES"]["AUTO_RANGE"]["LOW"].asInt();
    AUTO_MODE_MAX = threads["RCIN_SERVO"]["FLIGHT_MODES"]["AUTO_RANGE"]["HIGH"].asInt();
    SEMI_DEADZONE = threads["RCIN_SERVO"]["FLIGHT_MODES"]["SEMI-AUTO_DEADZONE"].asInt();

    Json::Value v;
    std::ifstream key_file("core/keys.json"); // read memory keys from a separate file

    if (!reader.parse(key_file, v)) {
        std::cout << reader.getFormattedErrorMessages();
        exit(1);
    }

    int i = 1; // leave index 0 empty for default when key not found
    auto vectors = v["VECTORS"];

    for (int v = 0; v < vectors.size(); v++) {
        auto vector = vectors[v];
        auto ks = vector["KEYS"];
        for (int k = 0; k < ks.size(); k++) {
            std::string key = vector["ID"].asString() + "_" + ks[k].asString();
            keys.insert(std::pair<std::string, int>(key, i++));
        }
    }

    keys.insert(std::pair<std::string, int>("rcin_THROTTLE", keys.at("rcin_CHANNEL_" + std::to_string(THROTTLE_CHANNEL))));
    keys.insert(std::pair<std::string, int>("rcin_ELEVATOR", keys.at("rcin_CHANNEL_" + std::to_string(ELEVATOR_CHANNEL))));
    keys.insert(std::pair<std::string, int>("rcin_AILERON", keys.at("rcin_CHANNEL_" + std::to_string(AILERON_CHANNEL))));
    keys.insert(std::pair<std::string, int>("rcin_RUDDER", keys.at("rcin_CHANNEL_" + std::to_string(RUDDER_CHANNEL))));
    keys.insert(std::pair<std::string, int>("rcin_FLAPS", keys.at("rcin_CHANNEL_" + std::to_string(FLAPS_CHANNEL))));
    keys.insert(std::pair<std::string, int>("servo_THROTTLE", keys.at("servo_CHANNEL_" + std::to_string(THROTTLE_CHANNEL))));
    keys.insert(std::pair<std::string, int>("servo_ELEVATOR", keys.at("servo_CHANNEL_" + std::to_string(ELEVATOR_CHANNEL))));
    keys.insert(std::pair<std::string, int>("servo_AILERON", keys.at("servo_CHANNEL_" + std::to_string(AILERON_CHANNEL))));
    keys.insert(std::pair<std::string, int>("servo_RUDDER", keys.at("servo_CHANNEL_" + std::to_string(RUDDER_CHANNEL))));
    keys.insert(std::pair<std::string, int>("servo_FLAPS", keys.at("servo_CHANNEL_" + std::to_string(FLAPS_CHANNEL))));
    keys.insert(std::pair<std::string, int>("controller_0_THROTTLE", keys.at("controller_0_CHANNEL_" + std::to_string(THROTTLE_CHANNEL))));
    keys.insert(std::pair<std::string, int>("controller_0_ELEVATOR", keys.at("controller_0_CHANNEL_" + std::to_string(ELEVATOR_CHANNEL))));
    keys.insert(std::pair<std::string, int>("controller_0_AILERON", keys.at("controller_0_CHANNEL_" + std::to_string(AILERON_CHANNEL))));
    keys.insert(std::pair<std::string, int>("controller_0_RUDDER", keys.at("controller_0_CHANNEL_" + std::to_string(RUDDER_CHANNEL))));
    keys.insert(std::pair<std::string, int>("controller_0_FLAPS", keys.at("controller_0_CHANNEL_" + std::to_string(FLAPS_CHANNEL))));
}