#!/usr/bin/env python3
"""
Calibration script for SEASCAPE IMU sensors and magnetometers.
Wraps the core/calibrate_imu C++ utility to calibrate both LSM9DS1 and MPU9250 sensors.
"""

import os
import sys
import subprocess
import argparse
import logging
from pathlib import Path


def setup_logging(debug: bool):
    """Configure logging output."""
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def check_calibrate_imu_binary():
    """Ensure the calibrate_imu binary exists, attempt to build if missing."""
    calibrate_path = Path("core/calibrate_imu")
    
    if not calibrate_path.exists():
        logging.info("calibrate_imu binary not found. Attempting to build...")
        try:
            result = subprocess.run(
                ["make", "-C", "core/", "calibrate_imu"],
                capture_output=True,
                text=True,
                check=False
            )
            if result.returncode != 0:
                logging.error("Failed to build calibrate_imu:")
                logging.error(result.stderr)
                return False
            logging.info("Successfully built calibrate_imu")
        except FileNotFoundError:
            logging.error("'make' command not found. Please install build tools.")
            return False
        except Exception as e:
            logging.error(f"Error building calibrate_imu: {e}")
            return False
    
    if not calibrate_path.exists():
        logging.error("calibrate_imu binary still not found after build attempt")
        return False
    
    return True


def check_sudo():
    """Check if running with sudo privileges."""
    if os.geteuid() != 0:
        logging.warning("This script requires sudo privileges to access IMU hardware.")
        logging.warning("Please run with: sudo python3 calibrate_sensor.py")
        return False
    return True


def calibrate_sensor(sensor_name: str, use_sudo: bool = False):
    """
    Run calibration for a specific sensor.
    
    Args:
        sensor_name: Name of the sensor (LSM9DS1 or MPU9250)
        use_sudo: Whether to use sudo (for non-root execution)
    
    Returns:
        True if calibration succeeded, False otherwise
    """
    calibrate_path = "./core/calibrate_imu"
    
    cmd = [calibrate_path, sensor_name]
    if use_sudo:
        cmd = ["sudo"] + cmd
    
    logging.info("=" * 70)
    logging.info(f"Starting calibration for {sensor_name}")
    logging.info("=" * 70)
    
    try:
        # Run with real-time output
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Stream output in real-time
        for line in process.stdout:
            print(line, end='')
        
        process.wait()
        
        if process.returncode == 0:
            logging.info(f"Successfully calibrated {sensor_name}")
            return True
        else:
            logging.error(f"Calibration failed for {sensor_name} (exit code: {process.returncode})")
            return False
            
    except FileNotFoundError:
        logging.error(f"calibrate_imu binary not found at {calibrate_path}")
        return False
    except KeyboardInterrupt:
        logging.warning(f"\nCalibration interrupted by user during {sensor_name} calibration")
        if process:
            process.terminate()
        return False
    except Exception as e:
        logging.error(f"Error during {sensor_name} calibration: {e}")
        return False


def verify_calibration_files():
    """Check that calibration files were created."""
    calib_dir = Path("data/calibration")
    
    if not calib_dir.exists():
        logging.error(f"Calibration directory not found: {calib_dir}")
        return False
    
    results = {}
    for sensor in ["LSM9DS1", "MPU9250"]:
        json_file = calib_dir / f"{sensor}_calibration.json"
        bin_file = calib_dir / f"{sensor}_calibration.bin"
        
        if json_file.exists() and bin_file.exists():
            results[sensor] = True
            logging.info(f"✓ {sensor} calibration files found")
        else:
            results[sensor] = False
            logging.warning(f"✗ {sensor} calibration files missing")
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate SEASCAPE IMU sensors (accelerometer, gyroscope, magnetometer)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate both sensors (recommended):
  sudo python3 calibrate_sensor.py --all
  
  # Calibrate only LSM9DS1:
  sudo python3 calibrate_sensor.py --sensor LSM9DS1
  
  # Calibrate only MPU9250:
  sudo python3 calibrate_sensor.py --sensor MPU9250

Notes:
  - This script must be run with sudo privileges
  - Each calibration takes ~40-60 seconds
  - Follow the on-screen instructions carefully
  - Step 1: Keep vehicle flat and still for ~10 seconds
  - Step 2: Rotate vehicle slowly through all orientations for ~30-60 seconds
        """
    )
    
    parser.add_argument(
        "--sensor",
        choices=["LSM9DS1", "MPU9250"],
        help="Calibrate a specific sensor only"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Calibrate all available sensors (LSM9DS1 and MPU9250)"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )
    parser.add_argument(
        "--no-sudo",
        action="store_true",
        help="Don't prefix commands with sudo (use if already running as root)"
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Only verify existing calibration files, don't run calibration"
    )
    
    args = parser.parse_args()
    
    setup_logging(args.debug)
    
    # Verify we're in the right directory
    if not Path("core").is_dir():
        logging.error("core/ directory not found. Please run this script from the SEASCAPE root directory.")
        sys.exit(1)
    
    # Verify calibration files only
    if args.verify_only:
        logging.info("Verifying calibration files...")
        results = verify_calibration_files()
        if all(results.values()):
            logging.info("All calibration files present.")
            sys.exit(0)
        else:
            logging.warning("Some calibration files are missing.")
            sys.exit(1)
    
    # Check sudo if needed
    if not args.no_sudo and not check_sudo():
        logging.error("Sudo privileges required. Exiting.")
        sys.exit(1)
    
    # Check binary
    if not check_calibrate_imu_binary():
        logging.error("Cannot proceed without calibrate_imu binary. Exiting.")
        sys.exit(1)
    
    # Determine which sensors to calibrate
    sensors_to_calibrate = []
    
    if args.all:
        sensors_to_calibrate = ["LSM9DS1", "MPU9250"]
    elif args.sensor:
        sensors_to_calibrate = [args.sensor]
    else:
        # Default: ask user or calibrate both
        logging.info("No sensor specified. Use --all to calibrate both, or --sensor to choose one.")
        parser.print_help()
        sys.exit(1)
    
    # Perform calibration
    logging.info(f"Will calibrate: {', '.join(sensors_to_calibrate)}")
    logging.info("")
    
    success_count = 0
    for i, sensor in enumerate(sensors_to_calibrate):
        if calibrate_sensor(sensor, use_sudo=False):  # Already checked sudo above
            success_count += 1
        
        # Add delay between calibrations (except after the last one)
        if i < len(sensors_to_calibrate) - 1:
            import time
            delay_seconds = 5
            logging.info("")
            logging.info(f"Waiting {delay_seconds} seconds before next calibration...")
            time.sleep(delay_seconds)
        
        logging.info("")
    
    # Summary
    logging.info("=" * 70)
    logging.info(f"Calibration Summary: {success_count}/{len(sensors_to_calibrate)} successful")
    logging.info("=" * 70)
    
    # Verify files were created
    logging.info("Verifying calibration files...")
    verify_calibration_files()
    
    if success_count == len(sensors_to_calibrate):
        logging.info("✓ All calibrations completed successfully!")
        logging.info("You can now launch SEASCAPE with: python3 launch.py")
        sys.exit(0)
    else:
        logging.error("✗ Some calibrations failed. Please review the output above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
