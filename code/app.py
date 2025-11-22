import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# ==============================
# User Input
# ==============================
while True:
    CSV_PATH = input("Enter CSV filename (e.g., car_log.csv): ").strip()
    if os.path.isfile(CSV_PATH):
        break
    print(f"File '{CSV_PATH}' not found. Try again.\n")

print(f"\nLoading {CSV_PATH} ...\n")

# ==============================
# Load CSV (no header)
# ==============================
df = pd.read_csv(CSV_PATH, header=None)

df.columns = [
    "time_ms",  # 0
    "ax",       # 1
    "ay",       # 2
    "az",       # 3
    "roll",     # 4
    "pitch",    # 5
    "yaw",      # 6
    "lat",      # 7
    "lon",      # 8
    "spd_mph"   # 9
]

# Remove invalid GPS
df = df[(df["lat"] != 0.0) & (df["lon"] != 0.0)].copy()
if df.empty:
    raise RuntimeError("ERROR: No usable GPS points found.")

# Time in seconds
df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0


# ==============================
# Helpers
# ==============================
def latlon_to_local_xy(lat, lon):
    R = 6371000.0
    lat = np.radians(lat)
    lon = np.radians(lon)

    lat0 = lat.iloc[0]
    lon0 = lon.iloc[0]

    x = (lon - lon0) * np.cos(lat0) * R
    y = (lat - lat0) * R
    return x, y

def wrap_angle_deg(angle):
    a = np.mod(angle, 360.0)
    a[a < 0] += 360.0
    return a

def angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


# ==============================
# Compute XY track
# ==============================
df["x_m"], df["y_m"] = latlon_to_local_xy(df["lat"], df["lon"])

# Smooth GPS track (rolling average)
df["x_smooth"] = df["x_m"].rolling(5, center=True).mean()
df["y_smooth"] = df["y_m"].rolling(5, center=True).mean()

# Fill endpoints that rolling window cannot compute
df["x_smooth"].fillna(df["x_m"], inplace=True)
df["y_smooth"].fillna(df["y_m"], inplace=True)

# Compute smoothed GPS heading
dx = df["x_smooth"].diff()
dy = df["y_smooth"].diff()
df["heading_deg"] = wrap_angle_deg(np.degrees(np.arctan2(dx, dy)))

# Normalize IMU yaw
df["yaw_norm"] = wrap_angle_deg(df["yaw"])

# Slip angle ≈ yaw error
df["yaw_error"] = angle_diff_deg(df["yaw_norm"], df["heading_deg"])


# ==============================
# FILTER OUT LOW SPEEDS (<2 mph)
# ==============================
df_moving = df[df["spd_mph"] > 2.0].copy()
print(f"Filtered dataset: {len(df_moving)} moving samples (>{2} mph)\n")

if df_moving.empty:
    raise RuntimeError("All samples below speed threshold! Try lowering the threshold.")


# ==============================
# PLOTS
# ==============================

# Track map (colored by speed)
plt.figure(figsize=(8, 6))
sc = plt.scatter(df_moving["x_m"], df_moving["y_m"], c=df_moving["spd_mph"],
                 s=10, cmap="plasma")
plt.colorbar(sc, label="Speed (mph)")
plt.xlabel("X (m, east)")
plt.ylabel("Y (m, north)")
plt.title("Car Track (speed colored)")
plt.axis("equal")
plt.grid(True)

# IMU yaw vs GPS heading
plt.figure(figsize=(10, 6))
plt.plot(df_moving["time_s"], df_moving["yaw_norm"], label="IMU Yaw")
plt.plot(df_moving["time_s"], df_moving["heading_deg"], label="GPS Heading", alpha=0.7)
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("IMU Yaw vs GPS Motion Heading (filtered >2 mph)")
plt.legend()
plt.grid(True)

# Slip angle (yaw error)
plt.figure(figsize=(10, 4))
plt.plot(df_moving["time_s"], df_moving["yaw_error"])
plt.xlabel("Time (s)")
plt.ylabel("Yaw Error (deg)")
plt.title("Slip Angle (Yaw Error) — only when moving >2 mph")
plt.grid(True)

plt.tight_layout()
plt.show()
