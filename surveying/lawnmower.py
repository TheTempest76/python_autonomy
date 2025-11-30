#!/usr/bin/env python3
# Fly drone in lawnmower pattern using pymavlink
# Supports manual coordinate input or KML file import

from pymavlink import mavutil
import time
import math
import argparse
import xml.etree.ElementTree as ET

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
    """
    print(f"[INFO] Going to waypoint: Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt}m")
    
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
    
    while True:
        current_lat, current_lon, current_alt = get_current_position(master)
        distance = calculate_distance(current_lat, current_lon, lat, lon)
        
        print(f"Distance to waypoint: {distance:.2f} m, Altitude: {current_alt:.2f} m")
        
        if distance <= acceptance_radius:
            print(f"[OK] Waypoint reached (within {acceptance_radius}m)")
            break
        
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

def parse_kml_file(kml_file):
    """
    Parse KML file and extract polygon coordinates.
    Returns list of (lat, lon) tuples.
    """
    print(f"[INFO] Parsing KML file: {kml_file}")
    
    tree = ET.parse(kml_file)
    root = tree.getroot()
    
    # KML namespace
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    
    # Try to find coordinates in Polygon
    coordinates = None
    for placemark in root.findall('.//kml:Placemark', ns):
        polygon = placemark.find('.//kml:Polygon', ns)
        if polygon is not None:
            coords_elem = polygon.find('.//kml:coordinates', ns)
            if coords_elem is not None:
                coordinates = coords_elem.text
                break
    
    # If no Polygon found, try LineString
    if coordinates is None:
        for placemark in root.findall('.//kml:Placemark', ns):
            linestring = placemark.find('.//kml:LineString', ns)
            if linestring is not None:
                coords_elem = linestring.find('.//kml:coordinates', ns)
                if coords_elem is not None:
                    coordinates = coords_elem.text
                    break
    
    if coordinates is None:
        raise ValueError("No coordinates found in KML file")
    
    # Parse coordinates (format: lon,lat,alt or lon,lat)
    points = []
    for line in coordinates.strip().split():
        if line.strip():
            parts = line.strip().split(',')
            if len(parts) >= 2:
                lon = float(parts[0])
                lat = float(parts[1])
                points.append((lat, lon))
    
    print(f"[OK] Parsed {len(points)} points from KML file")
    return points

def get_bounding_box(points):
    """Get the bounding box of a set of points."""
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    
    return min_lat, max_lat, min_lon, max_lon

def generate_lawnmower_pattern(min_lat, max_lat, min_lon, max_lon, spacing_meters, altitude, direction='NS'):
    """
    Generate lawnmower pattern waypoints.
    
    Args:
        min_lat, max_lat, min_lon, max_lon: Bounding box
        spacing_meters: Distance between parallel lines in meters
        altitude: Flight altitude in meters
        direction: 'NS' for North-South lines, 'EW' for East-West lines
    
    Returns:
        List of (lat, lon, alt) tuples
    """
    waypoints = []
    
    # Approximate degrees per meter (varies by latitude)
    avg_lat = (min_lat + max_lat) / 2
    meters_per_degree_lat = 111320  # roughly constant
    meters_per_degree_lon = 111320 * math.cos(math.radians(avg_lat))
    
    if direction == 'NS':
        # North-South passes (lines run vertically)
        lon_spacing = spacing_meters / meters_per_degree_lon
        
        current_lon = min_lon
        going_north = True
        
        while current_lon <= max_lon:
            if going_north:
                # Go from south to north
                waypoints.append((min_lat, current_lon, altitude))
                waypoints.append((max_lat, current_lon, altitude))
            else:
                # Go from north to south
                waypoints.append((max_lat, current_lon, altitude))
                waypoints.append((min_lat, current_lon, altitude))
            
            current_lon += lon_spacing
            going_north = not going_north
    
    else:  # direction == 'EW'
        # East-West passes (lines run horizontally)
        lat_spacing = spacing_meters / meters_per_degree_lat
        
        current_lat = min_lat
        going_east = True
        
        while current_lat <= max_lat:
            if going_east:
                # Go from west to east
                waypoints.append((current_lat, min_lon, altitude))
                waypoints.append((current_lat, max_lon, altitude))
            else:
                # Go from east to west
                waypoints.append((current_lat, max_lon, altitude))
                waypoints.append((current_lat, min_lon, altitude))
            
            current_lat += lat_spacing
            going_east = not going_east
    
    print(f"[INFO] Generated {len(waypoints)} waypoints for lawnmower pattern")
    return waypoints

def fly_lawnmower_pattern(master, waypoints, acceptance_radius=2.0):
    """Fly through all waypoints in the lawnmower pattern."""
    total_waypoints = len(waypoints)
    
    for i, (lat, lon, alt) in enumerate(waypoints, 1):
        print(f"\n[INFO] ===== Waypoint {i}/{total_waypoints} =====")
        goto_waypoint(master, lat, lon, alt, acceptance_radius)
        time.sleep(0.5)  # Brief pause at each waypoint
    
    print("\n[OK] Lawnmower pattern completed!")

def get_user_coordinates():
    """Get area coordinates from user input."""
    print("\n[INFO] Enter the bounding box for the survey area:")
    
    min_lat = float(input("Enter minimum latitude (South): "))
    max_lat = float(input("Enter maximum latitude (North): "))
    min_lon = float(input("Enter minimum longitude (West): "))
    max_lon = float(input("Enter maximum longitude (East): "))
    
    return min_lat, max_lat, min_lon, max_lon

def main():
    parser = argparse.ArgumentParser(description='Fly drone in lawnmower pattern')
    parser.add_argument('--connection', default='udp:127.0.0.1:14550',
                        help='MAVLink connection string (default: udp:127.0.0.1:14550)')
    parser.add_argument('--kml', type=str, help='Path to KML file with area polygon')
    parser.add_argument('--altitude', type=float, default=20,
                        help='Flight altitude in meters (default: 20)')
    parser.add_argument('--spacing', type=float, default=10,
                        help='Spacing between passes in meters (default: 10)')
    parser.add_argument('--direction', choices=['NS', 'EW'], default='NS',
                        help='Pattern direction: NS (North-South) or EW (East-West)')
    parser.add_argument('--radius', type=float, default=2.0,
                        help='Waypoint acceptance radius in meters (default: 2.0)')
    
    args = parser.parse_args()
    
    print("[INFO] Connecting to drone...")
    master = mavutil.mavlink_connection(args.connection)
    master.wait_heartbeat()
    print("[OK] Heartbeat received.")
    
    # Get home position
    print("[INFO] Getting home position...")
    home_lat, home_lon, home_alt = get_current_position(master)
    print(f"[INFO] Home position: Lat={home_lat:.7f}, Lon={home_lon:.7f}")
    
    # Get area coordinates
    if args.kml:
        # Parse KML file
        points = parse_kml_file(args.kml)
        min_lat, max_lat, min_lon, max_lon = get_bounding_box(points)
    else:
        # Get coordinates from user
        min_lat, max_lat, min_lon, max_lon = get_user_coordinates()
    
    print(f"\n[INFO] Survey area:")
    print(f"  Latitude range: {min_lat:.7f} to {max_lat:.7f}")
    print(f"  Longitude range: {min_lon:.7f} to {max_lon:.7f}")
    
    # Calculate area dimensions
    height_m = calculate_distance(min_lat, min_lon, max_lat, min_lon)
    width_m = calculate_distance(min_lat, min_lon, min_lat, max_lon)
    print(f"  Approximate dimensions: {width_m:.1f}m x {height_m:.1f}m")
    
    # Generate lawnmower pattern
    waypoints = generate_lawnmower_pattern(
        min_lat, max_lat, min_lon, max_lon,
        args.spacing, args.altitude, args.direction
    )
    
    # Confirm before takeoff
    print(f"\n[INFO] Ready to fly lawnmower pattern:")
    print(f"  - {len(waypoints)} waypoints")
    print(f"  - Altitude: {args.altitude}m")
    print(f"  - Spacing: {args.spacing}m")
    print(f"  - Direction: {args.direction}")
    response = input("\nProceed with flight? (yes/no): ")
    
    if response.lower() != 'yes':
        print("[INFO] Flight cancelled.")
        return
    
    # Arm and takeoff
    master.set_mode_apm("GUIDED")
    arm_and_takeoff(master, args.altitude)
    
    # Fly the pattern
    fly_lawnmower_pattern(master, waypoints, args.radius)
    
    # Return to home
    print("\n[INFO] ===== Returning to Home =====")
    goto_waypoint(master, home_lat, home_lon, args.altitude, args.radius)
    
    # Land
    print("\n[INFO] Landing...")
    master.set_mode_apm("LAND")
    wait_for_mode(master, "LAND")
    master.motors_disarmed_wait()
    print("[OK] Landed and disarmed.")

if __name__ == "__main__":
    main()