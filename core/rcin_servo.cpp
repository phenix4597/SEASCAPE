#include "air.h"

#include "Navio2/C++/RCInput_Navio2.h"
#include "Navio2/C++/RCOutput_Navio2.h"

RCInput* initialize_rcin() {
    RCInput* rcin = new RCInput_Navio2();
    printf("Initializing RCIN.\n");
    rcin->initialize();
    return rcin;
}

RCOutput* initialize_pwm(float freq) {
    RCOutput* pwm = new RCOutput_Navio2();
    printf("Initializing Servos.\n");
    for (int i = 0; i < 14; i++) {
        if (!pwm->initialize(i)) {
            pwm = nullptr;
            break;
        } else if (!pwm->set_frequency(i, freq)) {
            pwm = nullptr;
            break;
        } else if (!pwm->enable(i)) {
            pwm = nullptr;
            break;
        }
    }
    if (!pwm) {
        printf("Failed to initialize servo-rail PWM. Could be lacking root privilege.\n");
    }
    return pwm;
}

float clip_throttle(const air_config* cfg, int pwm) {
    return (float)(std::max(cfg->MIN_THROTTLE, std::min(pwm, cfg->MAX_THROTTLE)));
}

float clip_servo(const air_config* cfg, int pwm) {
    return (float)(std::max(cfg->MIN_SERVO, std::min(pwm, cfg->MAX_SERVO)));
}

int read_rcin(RCInput* rcin, double* array, std::map<std::string, int>& keys) {
    for (int i = 0; i < 14; i++) {
        array[keys["rcin_CHANNEL_" + std::to_string(i)]] = (double)(rcin->read(i));
    }
    return 0;
}

int write_servo(RCOutput* pwm, double* array, const air_config* cfg) {
    std::map<std::string, int> keys = cfg->keys;
    std::string controller_vec = "controller_" + std::to_string(cfg->SERVO_CONTROLLER) + "_";
    int mode_value = (int)array[keys["rcin_CHANNEL_" + std::to_string(cfg->FLIGHT_MODE_CHANNEL)]];
    array[keys["servo_MODE_FLAG"]] = 0; // manual
    if ((cfg->SERVO_CONTROLLER == 0 && !cfg->CONTROLLER_0_ENABLED) || (cfg->SERVO_CONTROLLER == 1 && !cfg->CONTROLLER_1_ENABLED)) {
        // stay in manual mode
    } else if (cfg->AUTO_MODE_MIN <= mode_value && mode_value < cfg->AUTO_MODE_MAX) {
        array[keys["servo_MODE_FLAG"]] = 2; // auto
    } else if (cfg->SEMI_MODE_MIN <= mode_value && mode_value < cfg->SEMI_MODE_MAX) {
        array[keys["servo_MODE_FLAG"]] = 1; // semi-auto
    }
    if (array[keys["servo_MODE_FLAG"]] == 0) { // manual
        for (int i = 0; i < 14; i++) {
            float new_pwm = array[keys["rcin_CHANNEL_" + std::to_string(i)]];
            if (i == cfg->THROTTLE_CHANNEL) {
                new_pwm = clip_throttle(cfg, (int)new_pwm);
            } else {
                new_pwm = clip_servo(cfg, (int)new_pwm);
            }
            pwm->set_duty_cycle(i, new_pwm);
            array[keys["servo_CHANNEL_" + std::to_string(i)]] = (double)new_pwm;
        }
    } else if (array[keys["servo_MODE_FLAG"]] == 2) { // auto
        for (int i = 0; i < 14; i++) {
            float new_pwm = array[keys[controller_vec + std::to_string(i)]];
            if (i == cfg->THROTTLE_CHANNEL) {
                new_pwm = clip_throttle(cfg, (int)new_pwm);
            } else {
                new_pwm = clip_servo(cfg, (int)new_pwm);
            }
            pwm->set_duty_cycle(i, new_pwm);
            array[keys["servo_CHANNEL_" + std::to_string(i)]] = (double)new_pwm;
        }
    } else if (array[keys["servo_MODE_FLAG"]] == 1) { // semi-auto
        bool man_override = false;
        double el = array[keys["rcin_CHANNEL_" + std::to_string(cfg->ELEVATOR_CHANNEL)]];
        double al = array[keys["rcin_CHANNEL_" + std::to_string(cfg->AILERON_CHANNEL)]];
        if ((!(std::abs(1500 - el) > cfg->SEMI_DEADZONE)) || (!(std::abs(1500 - al) > cfg->SEMI_DEADZONE))) {
            man_override = true;
        }
        if (man_override) {
            for (int i = 0; i < 14; i++) {
                float new_pwm = array[keys["rcin_CHANNEL_" + std::to_string(i)]];
                if (i == cfg->THROTTLE_CHANNEL) {
                    new_pwm = clip_throttle(cfg, (int)new_pwm);
                } else {
                    new_pwm = clip_servo(cfg, (int)new_pwm);
                }
                pwm->set_duty_cycle(i, new_pwm);
                array[keys["servo_CHANNEL_" + std::to_string(i)]] = (double)new_pwm;
            }
        } else {
            for (int i = 0; i < 14; i++) {
                float new_pwm = array[keys[controller_vec + std::to_string(i)]];
                if (i == cfg->THROTTLE_CHANNEL) {
                    new_pwm = clip_throttle(cfg, (int)new_pwm);
                } else {
                    new_pwm = clip_servo(cfg, (int)new_pwm);
                }
                pwm->set_duty_cycle(i, new_pwm);
                array[keys["servo_CHANNEL_" + std::to_string(i)]] = (double)new_pwm;
            }
        }
    }
    return 0;
}


void* servo_loop(void* arguments) {
    thread_struct* args = (thread_struct*)arguments;
    double* array = args->array;
    const air_config* cfg = args->cfg;
    std::map<std::string, int> keys = cfg->keys;
    usleep(250000);
    RCInput* rcin = initialize_rcin();
    RCOutput* pwm = initialize_pwm(cfg->PWM_FREQUENCY);
    usleep(250000);
    int max_sleep = hertz_to_microseconds(cfg->SERVO_LOOP_RATE);
    uint64_t start_time, now;

    while (true) {
        start_time = current_time_microseconds();
        read_rcin(rcin, array, keys);
        if (pwm) {
            write_servo(pwm, array, cfg);
        }
        now = current_time_microseconds();
        int sleep_time = (int)(max_sleep - (now - start_time));
        usleep(std::max(sleep_time, 0));
    }
}