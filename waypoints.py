#!/usr/bin/env python3
# Navigate drone to waypoints using GPS coordinates with pymavlink
from pymavlink import mavutil
import time
import math

def wait_for_mode(master, mode_name):
    """Wait until the drone enters the specified mode."""
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.custom_mode == master.mode_mapping().get(mode_name, -1):
            print(f"[OK] Mode changed to {mode_name}")
            break

def arm_and_takeoff(master, altitude):
    """Arm the drone and take off to specified altitude."""
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

def get_current_position(master):
    """Get current GPS position."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0
    return lat, lon, alt

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates using Haversine formula."""
    R = 6371000  # Earth's radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    distance = R * c
    return distance

def goto_waypoint(master, lat, lon, alt, acceptance_radius=2.0):
    """
    Navigate to a waypoint using GPS coordinates.
    
    Args:
        master: MAVLink connection
        lat: Target latitude in degrees
        lon: Target longitude in degrees
        alt: Target altitude in meters (relative)
        acceptance_radius: Distance in meters to consider waypoint reached
    """
    print(f"[INFO] Going to waypoint: Lat={lat}, Lon={lon}, Alt={alt}m")
    
    # Send position target in global frame
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # Position only (ignore velocity and acceleration)
        int(lat * 1e7),  # Latitude in degrees * 1e7
        int(lon * 1e7),  # Longitude in degrees * 1e7
        alt,  # Altitude in meters
        0, 0, 0,  # vx, vy, vz (not used)
        0, 0, 0,  # afx, afy, afz (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    
    # Wait until waypoint is reached
    while True:
        current_lat, current_lon, current_alt = get_current_position(master)
        distance = calculate_distance(current_lat, current_lon, lat, lon)
        
        print(f"Distance to waypoint: {distance:.2f} m, Altitude: {current_alt:.2f} m")
        
        if distance <= acceptance_radius:
            print(f"[OK] Waypoint reached (within {acceptance_radius}m)")
            break
        
        # Resend command periodically to maintain target
        master.mav.set_position_target_global_int_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        
        time.sleep(1)

def main():
    print("[INFO] Connecting to drone...")
    # For SITL simulation
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    
    # For real drone via serial/USB:
    # master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)
    
    # For real drone via network:
    # master = mavutil.mavlink_connection("udp:<drone_ip>:14550")
    
    master.wait_heartbeat()
    print("[OK] Heartbeat received.")
    
    # Get home position
    print("[INFO] Getting home position...")
    home_lat, home_lon, home_alt = get_current_position(master)
    print(f"[INFO] Home position: Lat={home_lat:.7f}, Lon={home_lon:.7f}")
    
    # Arm and takeoff
    master.set_mode_apm("GUIDED")
    arm_and_takeoff(master, altitude=10)
    
    # Define waypoints (modify these with your actual coordinates)
    # Example waypoints - these create a small square pattern
    # You should replace these with your actual GPS coordinates
    waypoint1_lat = home_lat + 0.0001  # ~11 meters north
    waypoint1_lon = home_lon + 0.0001  # ~11 meters east
    waypoint1_alt = 10
    
    waypoint2_lat = home_lat + 0.0001  # ~11 meters north
    waypoint2_lon = home_lon - 0.0001  # ~11 meters west
    waypoint2_alt = 15
    
    # Navigate to waypoint 1
    print("\n[INFO] ===== Navigating to Waypoint 1 =====")
    goto_waypoint(master, waypoint1_lat, waypoint1_lon, waypoint1_alt)
    time.sleep(2)  # Hover at waypoint 1
    
    # Navigate to waypoint 2
    print("\n[INFO] ===== Navigating to Waypoint 2 =====")
    goto_waypoint(master, waypoint2_lat, waypoint2_lon, waypoint2_alt)
    time.sleep(2)  # Hover at waypoint 2
    
    # Return to home
    print("\n[INFO] ===== Returning to Home =====")
    goto_waypoint(master, home_lat, home_lon, 10)
    
    # Land
    print("\n[INFO] Landing...")
    master.set_mode_apm("LAND")
    wait_for_mode(master, "LAND")
    master.motors_disarmed_wait()
    print("[OK] Landed and disarmed.")

if __name__ == "__main__":
    main()