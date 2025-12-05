import sys
import time
import subprocess
import multiprocessing
from core import prelaunch_checks
from core import shared_mem_helper_estimator_1
from core import shared_mem_helper_controller_1
import estimator_1
import controller_1

if __name__ == "__main__":

    print()

    prelaunch_checks.check_dependencies()
    time.sleep(0.5)
    prelaunch_checks.check_core()
    time.sleep(0.5)
    cfg, keys, channels = prelaunch_checks.check_config()

    core_process = subprocess.Popen(["sudo", "./core/air"])
    print(f"Core threads have launched.\t\t[PID:{core_process.pid}]")

    time.sleep(5)

    import sysv_ipc as ipc

    ftok_path = "core/air.h"
    key = ipc.ftok(ftok_path, ord("~"), silence_warning=True)
    shm = ipc.SharedMemory(key, 0, 0)
    shm.attach(0, 0)  # attach to shared memory created by core/air

    print()

    if cfg["THREADS"]["ESTIMATOR_1"]["ENABLED"]:
        mem_est1 = shared_mem_helper_estimator_1.helper(shm, keys)
        max_sleep_est1 = 1.0 / cfg["THREADS"]["ESTIMATOR_1"]["RATE"]
        estimator_1_process = multiprocessing.Process(
            target=estimator_1.estimator_loop, args=(mem_est1, max_sleep_est1)
        )
        estimator_1_process.daemon = True
        estimator_1_process.start()
        print(f"Estimator_1 process has launched.\t[PID:{estimator_1_process.pid}]")

    if cfg["THREADS"]["CONTROLLER_1"]["ENABLED"]:

        con1_xh_index = cfg["THREADS"]["CONTROLLER_1"]["XH_VECTOR_TO_USE"]
        mem_con1 = shared_mem_helper_controller_1.helper(shm, keys, con1_xh_index, channels)
        max_sleep_con1 = 1.0 / cfg["THREADS"]["CONTROLLER_1"]["RATE"]
        controller_1_process = multiprocessing.Process(
            target=controller_1.controller_loop, args=(mem_con1, max_sleep_con1)
        )
        controller_1_process.daemon = True
        controller_1_process.start()
        print(f"Controller_1 process has launched.\t[PID:{controller_1_process.pid}]")

    print()

    shm.remove()  # when all processes detach, the memory will be cleared.

    core_process.wait()
    print("\nCore process has died.\nShutting down...")
    sys.exit()
