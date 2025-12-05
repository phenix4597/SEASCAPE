import time
import numpy as np
from core import shared_mem_helper_controller_1 as shm

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

        # passthrough for remaining channels (TEST BEFORE FLIGHT!!!!, if it doesn't work as expected, CHECK CONFIG FILE! )
        mem.servos.aileron = mem.rcin.aileron
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
