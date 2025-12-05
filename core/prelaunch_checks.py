import json
import os
import sys
import apt
import datetime
from dateutil.relativedelta import relativedelta


def check_dependencies():
    cache = apt.Cache()
    if not cache["libjsoncpp-dev"].is_installed:
        print("C++ library libjsoncpp-dev was not found.\nAttempting to install...\n")
        os.system("sudo apt-get install libjsoncpp-dev")
        cache = apt.Cache()
        if not cache["libjsoncpp-dev"].is_installed:
            print("\nInstallation failed. Launch canceled.\n")
            sys.exit()
        else:
            print("Installation successful.\n")

    try:
        import sysv_ipc
    except ImportError as e:
        print("Python module sysv_ipc was not found.\nAttempting to install...\n")
        os.system("sudo pip3 install sysv_ipc")
        try:
            import sysv_ipc

            print("")
        except ImportError as e:
            print("\nInstallation failed. Launch canceled.\n")
            sys.exit()

    try:
        import jsonschema
    except ImportError as e:
        print("Python module jsonschema was not found.\nAttempting to install...\n")
        os.system("sudo pip3 install jsonschema")
        try:
            import jsonschema

            print("")
        except ImportError as e:
            print("\nInstallation failed. Launch canceled.\n")
            sys.exit()

    if not os.path.exists("core/mavlink/common/"):
        print("Seems like the mavlink submodule is not present.\nAttempting to clone...\n")
        
        # Check if this is a git repository
        if os.path.exists(".git"):
            ret = os.system("git submodule update --init")
            if ret == 0 and os.path.exists("core/mavlink/common/"):
                print("\nClone successful.\n")
            else:
                print("\nGit submodule clone failed. Launch canceled.\n")
                sys.exit()
        else:
            # Not a git repo, try direct clone
            print("Not a git repository. Attempting direct clone of mavlink...\n")
            ret = os.system("git clone --depth 1 https://github.com/mavlink/mavlink.git core/mavlink")
            if ret == 0 and os.path.exists("core/mavlink/common/"):
                print("\nClone successful.\n")
            else:
                print("\nDirect clone failed. Launch canceled.\n")
                print("Please manually download mavlink headers to core/mavlink/\n")
                sys.exit()


def check_core():
    needs_build = False
    if not os.path.exists("core/air"):
        needs_build = True
        print("Looks like the core files have not been compiled yet.\nAttempting to compile...\n")
    elif not os.path.exists("core/calibrate_imu"):
        needs_build = True
        print("Looks like calibrate_imu has not been compiled yet.\nAttempting to compile...\n")
    
    if needs_build:
        os.system("make -s -C core/")
        if not os.path.exists("core/air"):
            print("Build failed. Launch canceled.\n")
            sys.exit()
        else:
            print("Compilation successful.\n")
            if not os.path.exists("core/calibrate_imu"):
                print("Warning: calibrate_imu binary not built. You may need to run 'make -C core/ calibrate_imu' manually.\n")


def check_config(debug=False):
    cfg = json.load(open("config.json"))
    check_property_types(cfg)
    verify_thead_rates(cfg)
    check_vector_dependencies(cfg)
    check_imu_enabled(cfg)
    check_imu_calibration(cfg, debug)
    check_mode_ranges(cfg)
    keys = check_keys()
    channels = check_servo_channels(cfg)
    return cfg, keys, channels


def check_property_types(cfg):
    path = ""
    structure = {
        "type": "object",
        "properties": {
            "THREADS": {
                "type": "object",
                "properties": {
                    "LOGGER": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                            "LOG_SENSOR_DATA": {"type": "boolean"},
                            "LOG_ESTIMATOR_0": {"type": "boolean"},
                            "LOG_ESTIMATOR_1": {"type": "boolean"},
                            "LOG_CONTROLLER_0": {"type": "boolean"},
                            "LOG_CONTROLLER_1": {"type": "boolean"},
                            "LOG_RCIN_SERVO": {"type": "boolean"},
                        },
                        "required": [
                            "ENABLED",
                            "RATE",
                            "LOG_SENSOR_DATA",
                            "LOG_ESTIMATOR_0",
                            "LOG_ESTIMATOR_1",
                            "LOG_CONTROLLER_0",
                            "LOG_CONTROLLER_1",
                            "LOG_RCIN_SERVO",
                        ],
                    },
                    "ESTIMATOR_0": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                        },
                        "required": ["ENABLED", "RATE"],
                    },
                    "ESTIMATOR_1": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                        },
                        "required": ["ENABLED", "RATE"],
                    },
                    "CONTROLLER_0": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                            "XH_VECTOR_TO_USE": {"type": "integer"},
                        },
                        "required": ["ENABLED", "RATE", "XH_VECTOR_TO_USE"],
                    },
                    "CONTROLLER_1": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                            "XH_VECTOR_TO_USE": {"type": "integer"},
                        },
                        "required": ["ENABLED", "RATE", "XH_VECTOR_TO_USE"],
                    },
                    "IMU_ADC": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                            "USE_LSM9DS1": {"type": "boolean"},
                            "USE_MPU9250": {"type": "boolean"},
                            "PRIMARY_IMU": {"type": "string"},
                        },
                        "required": [
                            "ENABLED",
                            "RATE",
                            "USE_LSM9DS1",
                            "USE_MPU9250",
                            "PRIMARY_IMU",
                        ],
                    },
                    "GPS_BAROMETER": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                            "USE_GPS": {"type": "boolean"},
                            "USE_MS5611": {"type": "boolean"},
                        },
                        "required": ["ENABLED", "RATE", "USE_GPS", "USE_MS5611"],
                    },
                    "RCIN_SERVO": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                            "PWM_FREQUENCY": {"type": "number"},
                            "CONTROLLER_VECTOR_TO_USE": {"type": "integer"},
                            "THROTTLE_CHANNEL": {"type": "integer"},
                            "AILERON_CHANNEL": {"type": "integer"},
                            "ELEVATOR_CHANNEL": {"type": "integer"},
                            "RUDDER_CHANNEL": {"type": "integer"},
                            "FLAPS_CHANNEL": {"type": "integer"},
                            "FLIGHT_MODES": {
                                "type": "object",
                                "properties": {
                                    "MODE_CHANNEL": {"type": "integer"},
                                    "MANUAL_RANGE": {
                                        "type": "object",
                                        "properties": {
                                            "LOW": {"type": "integer"},
                                            "HIGH": {"type": "integer"},
                                        },
                                        "required": ["LOW", "HIGH"],
                                    },
                                    "SEMI-AUTO_RANGE": {
                                        "type": "object",
                                        "properties": {
                                            "LOW": {"type": "integer"},
                                            "HIGH": {"type": "integer"},
                                        },
                                        "required": [
                                            "LOW",
                                            "HIGH",
                                        ],
                                    },
                                    "AUTO_RANGE": {
                                        "type": "object",
                                        "properties": {
                                            "LOW": {"type": "integer"},
                                            "HIGH": {"type": "integer"},
                                        },
                                        "required": ["LOW", "HIGH"],
                                    },
                                    "SEMI-AUTO_DEADZONE": {"type": "integer"},
                                },
                                "required": [
                                    "MODE_CHANNEL",
                                    "MANUAL_RANGE",
                                    "SEMI-AUTO_RANGE",
                                    "AUTO_RANGE",
                                    "SEMI-AUTO_DEADZONE",
                                ],
                            },
                            "MIN_THROTTLE": {"type": "integer"},
                            "MAX_THROTTLE": {"type": "integer"},
                            "MIN_SERVO": {"type": "integer"},
                            "MAX_SERVO": {"type": "integer"},
                        },
                        "required": [
                            "ENABLED",
                            "RATE",
                            "PWM_FREQUENCY",
                            "CONTROLLER_VECTOR_TO_USE",
                            "THROTTLE_CHANNEL",
                            "AILERON_CHANNEL",
                            "ELEVATOR_CHANNEL",
                            "RUDDER_CHANNEL",
                            "FLAPS_CHANNEL",
                            "FLIGHT_MODES",
                            "MIN_THROTTLE",
                            "MAX_THROTTLE",
                            "MIN_SERVO",
                            "MAX_SERVO",
                        ],
                    },
                    "TELEMETRY": {
                        "type": "object",
                        "properties": {
                            "ENABLED": {"type": "boolean"},
                            "RATE": {"type": "number"},
                        },
                        "required": ["ENABLED", "RATE"],
                    },
                },
            }
        },
    }

    import jsonschema

    try:
        jsonschema.validate(instance=cfg, schema=structure)
    except jsonschema.ValidationError as e:
        path = "/".join(str(item) for item in e.absolute_path)
        print(f"Invalid config at {path}\n\t{e.message}\n")
        sys.exit()


def verify_thead_rates(cfg):
    threads = cfg["THREADS"]

    for name in [
        "LOGGER",
        "ESTIMATOR_0",
        "ESTIMATOR_1",
        "CONTROLLER_0",
        "CONTROLLER_1",
        "IMU_ADC",
        "GPS_BAROMETER",
        "RCIN_SERVO",
        "TELEMETRY",
    ]:
        t = threads[name]
        if t["RATE"] <= 0.0 and t["ENABLED"]:
            print(
                f"CONFIG ERROR: Invalid rate specified for {name} thread. Rate must be greater than 0 hertz.\nIf you wish to disable the thread, set ENABLED to false.\n"
            )
            sys.exit()


def check_vector_dependencies(cfg):
    threads = cfg["THREADS"]
    for name in [
        "CONTROLLER_0",
        "CONTROLLER_1",
    ]:
        t = threads[name]
        if not t["ENABLED"]:
            continue
        xh = t["XH_VECTOR_TO_USE"]
        if xh not in [0, 1]:
            print(f"CONFIG ERROR: Invalid value for THREADS/{name}/XH_VECTOR_TO_USE. Should be either 0 or 1.\n")
            sys.exit()
        if not threads["ESTIMATOR_" + str(xh)]["ENABLED"]:
            print(
                f"CONFIG ERROR: {name} is expecting to read values from ESTIMATOR_{xh} which is disabled.\nYou can resolve this by either enabling ESTIMATOR_{xh}, disabling {name}, or by changing the {name}/XH_VECTOR_TO_USE in config.json.\n"
            )
            sys.exit()

    if threads["RCIN_SERVO"]["ENABLED"]:
        i = threads["RCIN_SERVO"]["CONTROLLER_VECTOR_TO_USE"]
        if i not in [0, 1]:
            print(
                f"CONFIG ERROR: Invalid value for THREADS/RCIN_SERVO/CONTROLER_VECTOR_TO_USE. Should be either 0 or 1.\n"
            )
            sys.exit()
        if not threads["CONTROLLER_" + str(i)]["ENABLED"]:
            print(
                f"CONFIG WARNING: RCIN_SERVO thread is expecting to read values from CONTROLLER_{i} which is disabled.\nFor this reason, the only flight mode available will be MANUAL."
            )
            if not ask_proceed():
                sys.exit()


def check_imu_enabled(cfg):
    imu_cfg = cfg["THREADS"]["IMU_ADC"]
    if not imu_cfg["ENABLED"]:
        print(
            f"CONFIG WARNING: IMU_ADC thread is disabled.\nHaving the no IMU values will likely yield dangerously innacurate estimations."
        )
        if not ask_proceed():
            sys.exit()
    elif not (imu_cfg["USE_LSM9DS1"] or imu_cfg["USE_MPU9250"]):
        print(
            f"CONFIG WARNING: Both IMU sensors are disabled.\nHaving the no IMU values will likely yield dangerously innacurate estimations."
        )
        if not ask_proceed():
            sys.exit()
    elif imu_cfg["PRIMARY_IMU"] == "LSM9DS1" and not imu_cfg["USE_LSM9DS1"]:
        print(f"CONFIG WARNING: LSM9DS1 is listed as PRIMARY_IMU but it is disabled.")
        if imu_cfg["USE_MPU9250"]:
            imu_cfg["PRIMARY_IMU"] = "MPU9250"
        print("This value will be switched to MPU9250.")
        if not ask_proceed():
            sys.exit()
    elif imu_cfg["PRIMARY_IMU"] == "MPU9250" and not imu_cfg["USE_MPU9250"]:
        print(f"CONFIG WARNING: MPU9250 is listed as PRIMARY_IMU but it is disabled.")
        if imu_cfg["USE_LSM9DS1"]:
            imu_cfg["PRIMARY_IMU"] = "LSM9DS1"
        print("This value will be switched to LSM9DS1.")
        if not ask_proceed():
            sys.exit()
    elif not imu_cfg["PRIMARY_IMU"] in ["LSM9DS1", "MPU9250"]:
        new_value = "LSM9DS1"
        if imu_cfg["USE_MPU9250"] and not imu_cfg["USE_LSM9DS1"]:
            new_value = "MPU9250"
        print(f"CONFIG WARNING: Invalid PRIMARY_IMU.\nThis value will default to {new_value}.")
        if not ask_proceed():
            sys.exit()
        else:
            imu_cfg["PRIMARY_IMU"] = new_value


def check_imu_calibration(cfg, debug=False):
    imu_cfg = cfg["THREADS"]["IMU_ADC"]
    if cfg["THREADS"]["ESTIMATOR_0"]["ENABLED"] or imu_cfg.get("APPLY_CALIBRATION_PROFILE", False):
        sensors_to_calibrate = []
        
        if imu_cfg["USE_LSM9DS1"]:
            if not os.path.exists("data/calibration/LSM9DS1_calibration.bin"):
                sensors_to_calibrate.append("LSM9DS1")
            else:
                modified = check_calibration_date("data/calibration/LSM9DS1_calibration.bin")
                if modified is not None:
                    print(
                        f"Looks like you haven't calibrated your LSM9DS1 IMU in over a month. [{modified.strftime('%m/%d/%Y, %H:%M:%S')}]"
                    )
                    if not ask_proceed():
                        sys.exit()
        
        if imu_cfg["USE_MPU9250"]:
            if not os.path.exists("data/calibration/MPU9250_calibration.bin"):
                sensors_to_calibrate.append("MPU9250")
            else:
                modified = check_calibration_date("data/calibration/MPU9250_calibration.bin")
                if modified is not None:
                    print(
                        f"Looks like you haven't calibrated your MPU9250 IMU in over a month. [{modified.strftime('%m/%d/%Y, %H:%M:%S')}]"
                    )
                    if not ask_proceed():
                        sys.exit()
        
        # If any sensors need calibration, offer to run it now
        if sensors_to_calibrate:
            print("\n" + "="*70)
            print(f"CALIBRATION REQUIRED: {', '.join(sensors_to_calibrate)}")
            print("="*70)
            print("The following IMU sensor(s) need to be calibrated before launch:")
            for sensor in sensors_to_calibrate:
                print(f"  - {sensor}")
            print("\nWould you like to run the calibration now?")
            print("This will calibrate the accelerometer, gyroscope, and magnetometer.")
            reply = str(input("Run calibration now? [Y/N]: ")).upper().strip()
            
            if reply == "Y":
                # Run calibration script
                if not os.path.exists("calibrate_sensor.py"):
                    print("\nERROR: calibrate_sensor.py not found.\n")
                    sys.exit()
                
                for sensor in sensors_to_calibrate:
                    print(f"\nStarting calibration for {sensor}...")
                    debug_flag = " --debug" if debug else ""
                    cmd = f"sudo python3 calibrate_sensor.py --sensor {sensor} --no-sudo{debug_flag}"
                    ret = os.system(cmd)
                    if ret != 0:
                        print(f"\nCalibration failed for {sensor}. Launch canceled.\n")
                        sys.exit()
                
                print("\n" + "="*70)
                print("Calibration complete! Continuing with launch...")
                print("="*70 + "\n")
            else:
                print("\nCalibration skipped. Launch canceled.")
                print("You can run calibration manually with: sudo python3 calibrate_sensor.py --all\n")
                sys.exit()


def check_servo_channels(cfg):
    t = cfg["THREADS"]["RCIN_SERVO"]
    channels = [
        t["THROTTLE_CHANNEL"],
        t["ELEVATOR_CHANNEL"],
        t["AILERON_CHANNEL"],
        t["FLAPS_CHANNEL"],
        t["RUDDER_CHANNEL"],
        t["FLIGHT_MODES"]["MODE_CHANNEL"],
    ]
    for c in channels:
        if c < 1 or c > 14:
            print("CONFIG ERROR: One or more servo channels was out of bounds.")
            print("Check the RCIN_SERVO section of config.json, and ensure all channel values are within [1,14].\n")
            sys.exit()
    if len(set(channels)) != len(channels):
        print("CONFIG ERROR: Two or more servo types were mapped to the same channel.")
        print(
            "Check the RCIN_SERVO section of config.json, and ensure all channel values are unique and within [1,14].\n"
        )
        sys.exit()


def check_mode_ranges(cfg):
    r = cfg["THREADS"]["RCIN_SERVO"]["FLIGHT_MODES"]

    if (
        r["MANUAL_RANGE"]["LOW"] > r["MANUAL_RANGE"]["HIGH"]
        or r["SEMI-AUTO_RANGE"]["LOW"] > r["SEMI-AUTO_RANGE"]["HIGH"]
        or r["AUTO_RANGE"]["LOW"] > r["AUTO_RANGE"]["HIGH"]
    ):
        print(
            "CONFIG ERROR: An invalid flight mode range was detected. Upper limit cannot be larger than lower limit."
        )
        print("Check the RCIN_SERVO section of config.json.\n")
        sys.exit()

    manual = set(range(r["MANUAL_RANGE"]["LOW"], r["MANUAL_RANGE"]["HIGH"]))
    semi = set(range(r["SEMI-AUTO_RANGE"]["LOW"], r["SEMI-AUTO_RANGE"]["HIGH"]))
    auto = set(range(r["AUTO_RANGE"]["LOW"], r["AUTO_RANGE"]["HIGH"]))

    if manual.intersection(semi) or manual.intersection(auto) or semi.intersection(auto):
        print("CONFIG ERROR: An overlap was detected between flight mode ranges.")
        print("Check the RCIN_SERVO section of config.json, and ensure all flight mode ranges are distinct.\n")
        sys.exit()


def check_keys():
    v = json.load(open("core/keys.json"))
    i = 1
    vectors = v["VECTORS"]
    keys = {}
    for v in vectors:
        ks = v["KEYS"]
        for k in ks:
            k = v["ID"] + "_" + k
            if k in keys.keys():
                print(f"CONFIG ERROR: Duplicate keys '{k}' found in core/keys.json.\n")
                sys.exit()
            keys[k] = i
            i += 1
    return keys


def check_servo_channels(cfg):
    t = cfg["THREADS"]["RCIN_SERVO"]
    channel_list = [
        t["THROTTLE_CHANNEL"],
        t["ELEVATOR_CHANNEL"],
        t["AILERON_CHANNEL"],
        t["FLAPS_CHANNEL"],
        t["RUDDER_CHANNEL"],
        t["FLIGHT_MODES"]["MODE_CHANNEL"],
    ]
    for c in channel_list:
        if c < 1 or c > 14:
            print("CONFIG ERROR: One or more servo channels was out of bounds.")
            print("Check the RCIN_SERVO section of config.json, and ensure all channel values are within [1,14].\n")
            sys.exit()
    if len(set(channel_list)) != len(channel_list):
        print("CONFIG ERROR: Two or more servo types were mapped to the same channel.")
        print(
            "Check the RCIN_SERVO section of config.json, and ensure all channel values are unique and within [1,14].\n"
        )
        sys.exit()

    zero_based_channel_dict = {}
    # subtract one because board shows 1-based indexing, but i think drivers are 0-based.
    zero_based_channel_dict["THROTTLE"] = t["THROTTLE_CHANNEL"]
    zero_based_channel_dict["ELEVATOR"] = t["ELEVATOR_CHANNEL"]
    zero_based_channel_dict["AILERON"] = t["AILERON_CHANNEL"]
    zero_based_channel_dict["RUDDER"] = t["RUDDER_CHANNEL"]
    zero_based_channel_dict["FLAPS"] = t["FLAPS_CHANNEL"]
    zero_based_channel_dict["MODE"] = t["FLIGHT_MODES"]["MODE_CHANNEL"]
    return zero_based_channel_dict


def check_calibration_date(filepath):
    mtime = os.path.getmtime(filepath)
    modified = datetime.datetime.fromtimestamp(mtime)
    a_month_ago = datetime.datetime.now() - relativedelta(months=1)
    if modified < a_month_ago:
        return modified
    return None


def ask_proceed():
    print()
    while "the answer is invalid":
        reply = str(input("Would you like to continue the launch? [Y/N]: ")).upper().strip()
        if reply == "Y":
            print("Ok. Continuing...\n")
            return True
        if reply == "N":
            return False
