#include "air.h"
#include <cmath>

// PID Controller structure
struct pid_controller {
    double kp;           // Proportional gain
    double ki;           // Integral gain
    double kd;           // Derivative gain
    double integral;     // Integral accumulator
    double prev_error;   // Previous error for derivative
    double integral_limit; // Anti-windup limit
    bool initialized;
};

// Initialize PID controller
void pid_init(pid_controller* pid, double kp, double ki, double kd, double integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->integral_limit = integral_limit;
    pid->initialized = false;
}

// Reset PID controller
void pid_reset(pid_controller* pid) {
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->initialized = false;
}

// PID update
double pid_update(pid_controller* pid, double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;
    
    // Proportional term
    double p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    double i_term = pid->ki * pid->integral;
    
    // Derivative term (with initialization check)
    double d_term = 0.0;
    if (pid->initialized) {
        double derivative = (error - pid->prev_error) / dt;
        d_term = pid->kd * derivative;
    } else {
        pid->initialized = true;
    }
    pid->prev_error = error;
    
    return p_term + i_term + d_term;
}

// Normalize angle to [-PI, PI]
static inline double normalize_angle(double angle) {
    const double PI = 3.14159265358979323846;
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// Convert PWM microseconds to normalized value [-1, 1]
double pwm_to_normalized(double pwm) {
    return (pwm - 1500.0) / 500.0;
}

// Convert normalized value [-1, 1] to PWM microseconds
double normalized_to_pwm(double normalized) {
    return 1500.0 + normalized * 500.0;
}

// Constrain value to range
double constrain(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void* control_loop(void* arguments) {
    thread_struct* args = (thread_struct*)arguments;
    double* array = args->array;
    const air_config* cfg = args->cfg;
    std::map<std::string, int> keys = cfg->keys;
    std::string xh_vec = "xh_" + std::to_string(cfg->CONTROLLER_XH) + "_";
    int updates = 0;
    int max_sleep = hertz_to_microseconds(cfg->CONTROL_LOOP_RATE);
    uint64_t start_time, now, last_time;
    
    // Initialize PID controllers for SAS
    pid_controller roll_pid, pitch_pid, yaw_rate_pid;
    
    // Roll attitude hold: kp, ki, kd, integral_limit
    pid_init(&roll_pid, 1.5, 0.2, 0.3, 0.5);
    
    // Pitch attitude hold: kp, ki, kd, integral_limit
    pid_init(&pitch_pid, 1.5, 0.2, 0.3, 0.5);
    
    // Yaw rate damping: kp, ki, kd, integral_limit
    pid_init(&yaw_rate_pid, 0.8, 0.05, 0.1, 0.3);
    
    // Attitude setpoints (will be set from RC when engaged)
    double phi_setpoint = 0.0;
    double theta_setpoint = 0.0;
    double r_setpoint = 0.0;  // Yaw rate setpoint (usually 0 for damping)
    
    // SAS engagement flag
    bool sas_engaged = false;
    bool sas_initialized = false;
    
    last_time = current_time_microseconds();
    
    printf("Controller_0: SAS Attitude Hold System initialized\n");
    
    while (true) {
        start_time = current_time_microseconds();
        if (updates == array[keys[xh_vec + "UPDATES"]]) {
            usleep(100);
            continue; // wait for new data
        }
        updates = array[keys[xh_vec + "UPDATES"]];
        
        // Calculate time step
        double dt = (start_time - last_time) / 1000000.0;
        if (dt > 0.5 || dt <= 0.0) {
            dt = 1.0 / cfg->CONTROL_LOOP_RATE;
        }
        last_time = start_time;

        ////////////////////////////////////////////////////////////////
        // READ STATE ESTIMATES
        ////////////////////////////////////////////////////////////////
        
        double phi = array[keys[xh_vec + "PHI"]];      // Roll angle (rad)
        double theta = array[keys[xh_vec + "THETA"]];  // Pitch angle (rad)
        double psi = array[keys[xh_vec + "PSI"]];      // Yaw angle (rad)
        double p = array[keys[xh_vec + "P"]];          // Roll rate (rad/s)
        double q = array[keys[xh_vec + "Q"]];          // Pitch rate (rad/s)
        double r = array[keys[xh_vec + "R"]];          // Yaw rate (rad/s)
        
        ////////////////////////////////////////////////////////////////
        // READ RC INPUTS
        ////////////////////////////////////////////////////////////////
        
        double rc_throttle = array[keys["rcin_CHANNEL_" + std::to_string(cfg->THROTTLE_CHANNEL)]];
        double rc_aileron = array[keys["rcin_CHANNEL_" + std::to_string(cfg->AILERON_CHANNEL)]];
        double rc_elevator = array[keys["rcin_CHANNEL_" + std::to_string(cfg->ELEVATOR_CHANNEL)]];
        double rc_rudder = array[keys["rcin_CHANNEL_" + std::to_string(cfg->RUDDER_CHANNEL)]];
        double rc_flaps = array[keys["rcin_CHANNEL_" + std::to_string(cfg->FLAPS_CHANNEL)]];
        
        // Check if pilot wants manual override (stick deflection > 10%)
        double aileron_norm = pwm_to_normalized(rc_aileron);
        double elevator_norm = pwm_to_normalized(rc_elevator);
        double rudder_norm = pwm_to_normalized(rc_rudder);
        
        bool manual_override = (fabs(aileron_norm) > 0.1 || 
                               fabs(elevator_norm) > 0.1 || 
                               fabs(rudder_norm) > 0.1);

        ////////////////////////////////////////////////////////////////
        // SAS LOGIC
        ////////////////////////////////////////////////////////////////
        
        double aileron_cmd, elevator_cmd, rudder_cmd;
        
        if (manual_override) {
            // Manual control - capture current attitude as setpoint
            phi_setpoint = phi;
            theta_setpoint = theta;
            r_setpoint = 0.0;
            
            // Pass through RC inputs
            aileron_cmd = rc_aileron;
            elevator_cmd = rc_elevator;
            rudder_cmd = rc_rudder;
            
            // Reset integrators when in manual
            if (sas_engaged) {
                pid_reset(&roll_pid);
                pid_reset(&pitch_pid);
                pid_reset(&yaw_rate_pid);
                sas_engaged = false;
                sas_initialized = false;
            }
            
        } else {
            // SAS engaged - attitude hold
            if (!sas_initialized) {
                // First engagement - set current attitude as target
                phi_setpoint = phi;
                theta_setpoint = theta;
                r_setpoint = 0.0;
                sas_initialized = true;
                printf("Controller_0: SAS engaged - holding phi=%.2f deg, theta=%.2f deg\n", 
                       phi * 180.0 / 3.14159, theta * 180.0 / 3.14159);
            }
            sas_engaged = true;
            
            // Roll attitude hold (output is roll rate command)
            double roll_rate_cmd = pid_update(&roll_pid, phi_setpoint, phi, dt);
            
            // Convert roll rate command to aileron deflection
            // Inner loop: damping actual roll rate
            double roll_damping = -p * 0.5;  // Proportional damping
            double aileron_norm_cmd = constrain(roll_rate_cmd + roll_damping, -1.0, 1.0);
            aileron_cmd = normalized_to_pwm(aileron_norm_cmd);
            
            // Pitch attitude hold (output is pitch rate command)
            double pitch_rate_cmd = pid_update(&pitch_pid, theta_setpoint, theta, dt);
            
            // Convert pitch rate command to elevator deflection
            // Inner loop: damping actual pitch rate
            double pitch_damping = -q * 0.5;  // Proportional damping
            double elevator_norm_cmd = constrain(pitch_rate_cmd + pitch_damping, -1.0, 1.0);
            elevator_cmd = normalized_to_pwm(elevator_norm_cmd);
            
            // Yaw rate damping (no heading hold, just stability)
            double yaw_damping_cmd = pid_update(&yaw_rate_pid, r_setpoint, r, dt);
            double rudder_norm_cmd = constrain(yaw_damping_cmd, -1.0, 1.0);
            rudder_cmd = normalized_to_pwm(rudder_norm_cmd);
        }

        ////////////////////////////////////////////////////////////////
        // OUTPUT CONTROL COMMANDS
        ////////////////////////////////////////////////////////////////
        
        // Throttle is always pass-through
        array[keys["controller_0_CHANNEL_" + std::to_string(cfg->THROTTLE_CHANNEL)]] = rc_throttle;
        
        // Control surface commands
        array[keys["controller_0_CHANNEL_" + std::to_string(cfg->AILERON_CHANNEL)]] = aileron_cmd;
        array[keys["controller_0_CHANNEL_" + std::to_string(cfg->ELEVATOR_CHANNEL)]] = elevator_cmd;
        array[keys["controller_0_CHANNEL_" + std::to_string(cfg->RUDDER_CHANNEL)]] = rudder_cmd;
        array[keys["controller_0_CHANNEL_" + std::to_string(cfg->FLAPS_CHANNEL)]] = rc_flaps;
        
        // Pass through remaining channels
        for (int i = 0; i < 14; i++) {
            if (i != cfg->THROTTLE_CHANNEL && 
                i != cfg->AILERON_CHANNEL && 
                i != cfg->ELEVATOR_CHANNEL && 
                i != cfg->RUDDER_CHANNEL && 
                i != cfg->FLAPS_CHANNEL) {
                array[keys["controller_0_CHANNEL_" + std::to_string(i)]] = 
                    array[keys["rcin_CHANNEL_" + std::to_string(i)]];
            }
        }

        ////////////////////////////////////////////////////////////////

        now = current_time_microseconds();
        int sleep_time = (int)(max_sleep - (now - start_time));
        usleep(std::max(sleep_time, 0));
    }
}