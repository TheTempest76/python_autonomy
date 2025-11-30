#!/usr/bin/env python3
# fly a few meters and land using pymavli
from pymavlink import mavutil
import time
import math


def wait_for_mode(master, mode_name):
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.custom_mode == master.mode_mapping().get(mode_name, -1):
            print(f"[OK] Mode changed to {mode_name}")
            break


def arm_and_takeoff(master, altitude):
    print("[INFO] Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[OK] Armed.")

    print("[INFO] Switching to GUIDED mode...")
    master.set_mode_apm("GUIDED")
    wait_for_mode(master, "GUIDED")

    print(f"[INFO] Taking off to {altitude} m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, altitude
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0

        print(f"Altitude: {current_alt:.2f} m")

        if current_alt >= altitude * 0.95:
            print("[OK] Target altitude reached.")
            break

        time.sleep(0.5)


def send_velocity(master, vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


def travel_distance(master, distance_m, speed_mps):
    """
    Fly forward 'distance_m' meters and then return.
    """
    print(f"[INFO] Flying forward for {distance_m} meters at {speed_mps} m/s...")

    travel_time = distance_m / speed_mps
    print(f"[INFO] Estimated forward travel time: {travel_time:.1f} sec")

    # Forward
    start = time.time()
    while time.time() - start < travel_time:
        send_velocity(master, speed_mps, 0, 0)
        time.sleep(0.2)

    print("[OK] Forward distance reached.")

    # Stop first
    send_velocity(master, 0, 0, 0)
    time.sleep(1)

    # Return
    print("[INFO] Returning to start position...")
    start = time.time()
    while time.time() - start < travel_time:
        send_velocity(master, -speed_mps, 0, 0)
        time.sleep(0.2)

    print("[OK] Returned close to original point.")

    send_velocity(master, 0, 0, 0)
    time.sleep(1)


def main():
    print("[INFO] Connecting to SITL...")
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print("[OK] Heartbeat received.")

    master.set_mode_apm("GUIDED")
    arm_and_takeoff(master, altitude=10)

    # Fly 30 meters forward and then return
    travel_distance(master, distance_m=30, speed_mps=5)

    print("[INFO] Landing...")
    master.set_mode_apm("LAND")
    wait_for_mode(master, "LAND")

    master.motors_disarmed_wait()
    print("[OK] Landed and disarmed.")


if __name__ == "__main__":
    main()
