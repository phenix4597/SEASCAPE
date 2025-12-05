#include "air.h"


uint64_t current_time_microseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int hertz_to_microseconds(double hertz) {
    return (int)(1000000 / hertz);
}


int main(int argc, char* argv[]) {
    const air_config cfg;
    double* array;

    key_t key = ftok(GETEKYDIR, PROJECTID);
    if (key < 0) {
        printf("ftok error\n");
        exit(1);
    }

    int shmid;
    shmid = shmget(key, SHMSIZE, IPC_CREAT | IPC_EXCL | 0664);
    if (shmid == -1) {
        if (errno == EEXIST) {
            printf("shared memory already exist\n");
            shmid = shmget(key, 0, 0);
            printf("reference shmid = %d\n", shmid);
        } else {
            printf("shmget error\n");
            exit(1);
        }
    }

    if ((array = (double*)shmat(shmid, 0, 0)) == (void*)-1) {
        if (shmctl(shmid, IPC_RMID, NULL) == -1) {
            printf("shmctl error\n");
            exit(1);
        } else {
            printf("Attach shared memory failed\n");
            printf("remove shared memory identifier successful\n");
        }

        printf("shmat error\n");
        exit(1);
    }

    memset(array, 0, SHMSIZE); // clear shared vector just in case

    for (int i = 0; i < 14; i++) { // set initial pwm values
        if (i == cfg.THROTTLE_CHANNEL) {
            array[cfg.keys.at("rcin_CHANNEL_" + std::to_string(i))] = 1000.0;
            array[cfg.keys.at("controller_0_CHANNEL_" + std::to_string(i))] = 1000.0;
            array[cfg.keys.at("controller_1_CHANNEL_" + std::to_string(i))] = 1000.0;
            array[cfg.keys.at("servo_CHANNEL_" + std::to_string(i))] = 1000.0;
        } else {
            array[cfg.keys.at("rcin_CHANNEL_" + std::to_string(i))] = 1500.0;
            array[cfg.keys.at("controller_0_CHANNEL_" + std::to_string(i))] = 1500.0;
            array[cfg.keys.at("controller_1_CHANNEL_" + std::to_string(i))] = 1500.0;
            array[cfg.keys.at("servo_CHANNEL_" + std::to_string(i))] = 1500.0;
        }
    }

    pthread_t imu_thread;
    pthread_t gps_baro_thread;
    pthread_t servo_thread;
    pthread_t estimation_thread;
    pthread_t control_thread;
    pthread_t telemetry_thread;
    pthread_t logger_thread;

    struct thread_struct thread_args;
    thread_args.array = array;
    thread_args.cfg = &cfg;

    printf("\n");

    if (cfg.IMU_LOOP_ENABLED) {
        pthread_create(&imu_thread, NULL, &imu_loop, (void*)&thread_args);
    }
    if (cfg.GPS_LOOP_ENABLED) {
        pthread_create(&gps_baro_thread, NULL, &gps_baro_loop, (void*)&thread_args);
    }
    if (cfg.SERVO_LOOP_ENABLED) {
        pthread_create(&servo_thread, NULL, &servo_loop, (void*)&thread_args);
    }
    usleep(2000000);
    if (cfg.ESTIMATOR_0_ENABLED) {
        pthread_create(&estimation_thread, NULL, &estimation_loop, (void*)&thread_args);
        printf("Starting Estimator_0 thread.\n");
    }
    if (cfg.CONTROLLER_0_ENABLED) {
        pthread_create(&control_thread, NULL, &control_loop, (void*)&thread_args);
        printf("Starting Controller_0 thread.\n");
    }
    if (cfg.TELEMETRY_LOOP_ENABLED) {
        pthread_create(&telemetry_thread, NULL, &telemetry_loop, (void*)&thread_args);
        printf("Starting Telemetry thread.\n");
    }
    if (cfg.LOGGER_LOOP_ENABLED) {
        pthread_create(&logger_thread, NULL, &logger_loop, (void*)&thread_args);
        printf("Starting Logger thread.\n");
    }

    if (cfg.IMU_LOOP_ENABLED) {
        pthread_join(imu_thread, NULL);
    }
    if (cfg.GPS_LOOP_ENABLED) {
        pthread_join(gps_baro_thread, NULL);
    }
    if (cfg.SERVO_LOOP_ENABLED) {
        pthread_join(servo_thread, NULL);
    }
    if (cfg.ESTIMATOR_0_ENABLED) {
        pthread_join(estimation_thread, NULL);
    }
    if (cfg.CONTROLLER_0_ENABLED) {
        pthread_join(control_thread, NULL);
    }
    if (cfg.TELEMETRY_LOOP_ENABLED) {
        pthread_join(telemetry_thread, NULL);
    }
    if (cfg.LOGGER_LOOP_ENABLED) {
        pthread_join(logger_thread, NULL);
    }

    return 0;
}