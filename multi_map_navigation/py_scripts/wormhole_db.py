#!/usr/bin/env python3

import sqlite3

# Connect to SQLite database (or create it if it doesn't exist)
conn = sqlite3.connect("wormholes.db")
cursor = conn.cursor()

# Create a table for wormhole data
cursor.execute('''CREATE TABLE IF NOT EXISTS wormholes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT,
                    wormhole_x REAL,
                    wormhole_y REAL,
                    target_map TEXT,
                    target_x REAL,
                    target_y REAL)''')

# Corrected wormhole data (with 6 values per entry)
wormholes = [
    ("map", 41, 40, "mapp", 38, 41),
    ("mapp", 15, 79, "mappp", 20, 59),
]

cursor.executemany("INSERT INTO wormholes (map_name, wormhole_x, wormhole_y, target_map, target_x, target_y) VALUES (?, ?, ?, ?, ?, ?)", wormholes)

# Commit and close
conn.commit()
conn.close()

print("Wormhole data saved to database!")
