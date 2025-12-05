import time
import numpy as np
from core import shared_mem_helper_controller_1 as shm


class PIDController:
    """Simple PID controller for roll stabilization."""
    
    def __init__(self, kp, ki, kd, dt=0.01):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            dt: Time step for integration/differentiation
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, setpoint, measurement):
        """
        Calculate PID output.
        
        Args:
            setpoint: Desired value (e.g., desired roll angle)
            measurement: Current value (e.g., current roll angle)
            
        Returns:
            PID output (control signal)
        """
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / self.dt
        
        # Store for next iteration
        self.prev_error = error
        
        # Return total output
        return p_term + i_term + d_term
    
    def reset(self):
        """Reset integral and derivative terms."""
        self.integral = 0.0
        self.prev_error = 0.0


def controller_loop(mem: shm.helper, max_sleep: float):
    # mem object is defined in 'core/shared_mem_helper_controller_1.py'
    # mem.read_xh() will return a list of all values from the xh vector specified in config.json
    # mem.read_rcin() will update the object's list of current RC inputs from each channel.
    # ---- these values can be addressed by channel, such as mem.rcin[3]
    # ---- or by channel standard channel names, such as mem.rcin.throttle or mem.rcin.flaps
    # mem.read_servo() will update the object's list of current PWM values for each servo-rail channel.
    # ---- these values can be addressed by channel, such as mem.servos[3]
    # ---- or by channel standard channel names, such as mem.servos.throttle or mem.servos.rudder
    # mem.write_controller_1() will apply the modified PWM values to the controller's output vector.
    # ---- these outputs do not go directly to the servo-rail. they are instead handled by the RCIN_SERVO thread.

    # Initialize PID controller for roll (PHI)
    # Tune these gains (kp, ki, kd) based on your aircraft's response
    roll_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, dt=0.02)  # Match the controller loop time step with CONFIG

    while True:
        t0 = time.time()

        # read latest data
        [X, Y, Z, VT, ALPHA, BETA, PHI, THETA, PSI, P, Q, R] = mem.read_xh()
        mem.read_rcin()
        mem.read_servos()

        # \/ \/ \/ \/ \/ \/ \/ \/ \/  CONTROLLER CODE STARTS HERE  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ #

        # manual passthrough for throttle and flaps
        mem.servos.throttle = mem.rcin.throttle
        mem.servos.flaps = mem.rcin.flaps

        # PID controller for roll stabilization (using aileron input)
        # Convert aileron RC input to desired roll angle (setpoint)
        # Assuming aileron input ranges from 1000-2000 PWM, convert to degrees
        aileron_norm = (mem.rcin.aileron - 1500) / 500.0  # normalize to [-1, 1] # assuming 1500 is neutral
        desired_roll = aileron_norm * 15.0  # map to ±15 degrees max roll angle (SAFETY CRITICAL!!!!)
        
        # Calculate PID output for roll control
        roll_correction = roll_pid.update(desired_roll, PHI) # Make sure PHI is in degrees!! if in radians, convert it first.
        
        # Apply correction to aileron servo (convert from degrees/s to PWM units)
        mem.servos.aileron = 1500 + roll_correction # assuming 1500 is neutral
        
        # passthrough for remaining channels (TEST BEFORE FLIGHT!!!!, if it doesn't work as expected, CHECK CONFIG FILE! )
        mem.servos.elevator = mem.rcin.elevator
        mem.servos.rudder = mem.rcin.rudder
        mem.servos[5] = mem.rcin[5]
        mem.servos[6] = mem.rcin[6]
        mem.servos[7] = mem.rcin[7]
        mem.servos[8] = mem.rcin[8]

        # /\ /\ /\ /\ /\ /\ /\ /\ /\  CONTROLLER CODE STOPS HERE  /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ #

        # update controller data
        mem.write_controller_1()

        # sleep to maintain frequency
        sleep_time = max_sleep - (time.time() - t0)
        time.sleep(max(sleep_time, 0))
