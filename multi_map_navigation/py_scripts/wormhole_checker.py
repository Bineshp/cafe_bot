#!/usr/bin/env python3

import sqlite3

def get_target_map(current_map, x, y, threshold=5):
    conn = sqlite3.connect("wormholes.db")
    cursor = conn.cursor()

    # Query for wormholes near the robot's position
    cursor.execute("SELECT * FROM wormholes WHERE map_name=? AND ABS(wormhole_x - ?) < ? AND ABS(wormhole_y - ?) < ?", 
                   (current_map, x, threshold, y, threshold))
    
    result = cursor.fetchone()
    conn.close()

    if result:
        return result[4], result[5], result[6]  # Target map and new coordinates
    return None

# Example usage
current_map = "map"
robot_x, robot_y = 250, 180
target = get_target_map(current_map, robot_x, robot_y)

if target:
    print(f"Switch to {target[0]} at ({target[1]}, {target[2]})")
else:
    print("No wormhole detected.")
