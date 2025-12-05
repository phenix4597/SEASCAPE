#ifndef _INERTIAL_SENSOR_H
#define _INERTIAL_SENSOR_H

struct imu_calibration_profile {
    double* offsets;
    double* matrix;
};

class InertialSensor {
public:
    virtual bool initialize() = 0;
    virtual bool probe() = 0;
    virtual void update() = 0;
    void adjust() {
        if (!_calibration_profile) {
            return;
        }
        _ax -= _calibration_profile->offsets[0];
        _ay -= _calibration_profile->offsets[1];
        _az -= _calibration_profile->offsets[2];
        _gx -= _calibration_profile->offsets[3];
        _gy -= _calibration_profile->offsets[4];
        _gz -= _calibration_profile->offsets[5];
        _mx -= _calibration_profile->offsets[6];
        _my -= _calibration_profile->offsets[7];
        _mz -= _calibration_profile->offsets[8];

        // Apply soft-iron 3x3 matrix with temporaries to avoid overwriting inputs
        double mx0 = _mx, my0 = _my, mz0 = _mz;
        _mx = mx0 * _calibration_profile->matrix[0] + my0 * _calibration_profile->matrix[3] + mz0 * _calibration_profile->matrix[6];
        _my = mx0 * _calibration_profile->matrix[1] + my0 * _calibration_profile->matrix[4] + mz0 * _calibration_profile->matrix[7];
        _mz = mx0 * _calibration_profile->matrix[2] + my0 * _calibration_profile->matrix[5] + mz0 * _calibration_profile->matrix[8];

        return;
    };

    double read_temperature() { return temperature; };
    void read_accelerometer(double* ax, double* ay, double* az) { *ax = _ax; *ay = _ay; *az = _az; };
    void read_gyroscope(double* gx, double* gy, double* gz) { *gx = _gx; *gy = _gy; *gz = _gz; };
    void read_magnetometer(double* mx, double* my, double* mz) { *mx = _mx; *my = _my; *mz = _mz; };
    void set_calibration_profile(struct imu_calibration_profile* p) { _calibration_profile = p; };

protected:
    double temperature;
    double _ax, _ay, _az;
    double _gx, _gy, _gz;
    double _mx, _my, _mz;
    struct imu_calibration_profile* _calibration_profile;
};

#endif //_INERTIAL_SENSOR_H
