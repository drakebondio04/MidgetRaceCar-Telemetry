import base64
import io
import math

import numpy as np
import pandas as pd

from dash import Dash, dcc, html, Input, Output, State
from dash.exceptions import PreventUpdate
import plotly.graph_objects as go

# ============================================================
# USER SETTINGS
# ============================================================ 
START_LAT = 33.825590689244244
START_LON = -118.28829968858749
RADIUS_M = 3          # meters, gate radius
MIN_LAP_TIME_S = 5    # ignore laps shorter than this

# For track use, bump these up
HEADING_SPEED_THRESH_MPH = 10.0   # only trust GPS heading above this
SLIP_SPEED_THRESH_MPH    = 25.0   # only show slip above this
# GPS heading lag so it lines up with IMU yaw
GPS_HEADING_LAG_S = 0.4

# Smoothing constants
HEADING_SMOOTH_ALPHA   = 0.10
IMU_YAW_SMOOTH_ALPHA   = 0.05
SLIP_SMOOTH_ALPHA      = 0.15   # slip smoothing
IMU_ACCEL_SMOOTH_ALPHA = 0.20   # accel smoothing

# RPM calibration: from your data, ~128 pulses per revolution at ~1800 rpm idle
PULSES_PER_REV = 128.0
RPM_SMOOTH_ALPHA = 0.20
# ============================================================


# ------------------------------------------------------------
# Angle helpers
# ------------------------------------------------------------
def wrap180(a):
    return ((a + 180.0) % 360.0) - 180.0


def unwrap_deg(angle_deg):
    """Unwrap degrees to make them continuous (remove 360° jumps)."""
    a = np.unwrap(np.radians(angle_deg))
    return np.degrees(a)


def angle_diff_deg(a, b):
    """Minimal signed difference (a - b) in degrees, in [-180, 180]."""
    return wrap180(np.asarray(a, float) - np.asarray(b, float))


def ema_1d(x, alpha):
    """Simple 1D exponential moving average."""
    x = np.asarray(x, float)
    if len(x) == 0:
        return x
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


# ------------------------------------------------------------
# Time-alignment helper (for GPS heading lag)
# ------------------------------------------------------------
def shift_back_in_time(sig, t, lag_s):
    """
    Shift `sig` backwards by `lag_s` seconds relative to time array `t`.
    For each time t[i], we take the value that originally lived at t[i] + lag_s.
    Edges are held constant.
    """
    sig = np.asarray(sig, float)
    t = np.asarray(t, float)

    if len(sig) == 0:
        return sig

    t_src = t + lag_s
    shifted = np.interp(t_src, t, sig, left=sig[0], right=sig[-1])
    return shifted


# ------------------------------------------------------------
# Geo helpers
# ------------------------------------------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ------------------------------------------------------------
# CSV loader for ESP32 log format (16 columns)
# ------------------------------------------------------------
# ------------------------------------------------------------
# CSV loader for ESP32 log format (14, 16, or 17 columns)
# ------------------------------------------------------------
def load_csv_arduino(file_like):
    """
    Load ESP32 log CSV with NO header.

    Supported formats:

    14 columns (old log, no tach, no throttle):
      time_ms, ax, ay, az, roll, pitch,
      yaw_fused, yaw_gyro, yaw_mag, yaw_gps,
      lat, lon, spd_mph, yaw_mode

    16 columns (tach added, no throttle):
      time_ms, ax, ay, az, roll, pitch,
      yaw_fused, yaw_gyro, yaw_mag, yaw_gps,
      lat, lon, spd_mph, yaw_mode,
      tach_pulses, tach_min_dt_us

    17 columns (tach + throttle_pct):
      time_ms, ax, ay, az, roll, pitch,
      yaw_fused, yaw_gyro, yaw_mag, yaw_gps,
      lat, lon, spd_mph, yaw_mode,
      tach_pulses, tach_min_dt_us, throttle_pct
    """
    df = pd.read_csv(file_like, header=None)
    n_cols = df.shape[1]

    if n_cols == 14:
        # Original format: no tach, no throttle
        df.columns = [
            "timestamp_ms",
            "accel_x_g",
            "accel_y_g",
            "accel_z_g",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",          # fused from Arduino (body yaw)
            "yaw_gyro_deg",
            "yaw_mag_deg",
            "yaw_gps_deg",
            "lat",
            "lon",
            "speed_mph",
            "yaw_mode",
        ]

        # Create dummy tach + throttle columns so downstream code still works
        df["tach_pulses"] = 0.0
        df["tach_min_dt_us"] = np.nan
        df["throttle_pct"] = np.nan

    elif n_cols == 16:
        # Log with tach, but no throttle
        df.columns = [
            "timestamp_ms",
            "accel_x_g",
            "accel_y_g",
            "accel_z_g",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "yaw_gyro_deg",
            "yaw_mag_deg",
            "yaw_gps_deg",
            "lat",
            "lon",
            "speed_mph",
            "yaw_mode",
            "tach_pulses",
            "tach_min_dt_us",
        ]

        # No throttle in this format
        df["throttle_pct"] = np.nan

    elif n_cols == 17:
        # Newest log: tach + throttle_pct
        df.columns = [
            "timestamp_ms",
            "accel_x_g",
            "accel_y_g",
            "accel_z_g",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "yaw_gyro_deg",
            "yaw_mag_deg",
            "yaw_gps_deg",
            "lat",
            "lon",
            "speed_mph",
            "yaw_mode",
            "tach_pulses",
            "tach_min_dt_us",
            "throttle_pct",
        ]
    else:
        raise ValueError(
            f"Expected 14, 16, or 17 columns, got {n_cols}. "
            "Make sure CSV matches one of the supported ESP32 logging formats."
        )

    # Common time column
    df["timestamp_ms"] = df["timestamp_ms"].astype(float)
    df["time_s"] = df["timestamp_ms"] / 1000.0
    return df



# ------------------------------------------------------------
# Yaw alignment helper (works on UNWRAPPED angles)
# ------------------------------------------------------------
def align_yaw_to_heading(yaw_unwrapped, heading_unwrapped, speed):
    """
    Fix mirror/opposite effect between IMU yaw and GPS heading.

    All inputs are assumed to be *unwrapped* (continuous deg).

    Returns:
      yaw_aligned_unwrapped : continuous yaw aligned to GPS frame
      best_sign             : +1 or -1
      best_offset_deg       : constant offset applied (deg)
    """
    yaw_unwrapped = np.asarray(yaw_unwrapped, float)
    heading_unwrapped = np.asarray(heading_unwrapped, float)
    speed = np.asarray(speed, float)

    # ✅ Use HEADING_SPEED_THRESH_MPH, not SLIP_SPEED_THRESH_MPH
    mask = (speed >= HEADING_SPEED_THRESH_MPH) & np.isfinite(heading_unwrapped)
    if not mask.any():
        return yaw_unwrapped, 1.0, 0.0

    best_err = None
    best_sign = 1.0
    best_offset_deg = 0.0
    best_yaw_aligned_unwrapped = None

    for sign in (+1.0, -1.0):
        yaw_signed = sign * yaw_unwrapped

        diff = wrap180(yaw_signed[mask] - heading_unwrapped[mask])
        diff_rad = np.radians(diff)
        mean_s = np.mean(np.sin(diff_rad))
        mean_c = np.mean(np.cos(diff_rad))
        offset_rad = math.atan2(mean_s, mean_c)
        offset_deg = math.degrees(offset_rad)

        yaw_aligned_unwrapped = yaw_signed - offset_deg

        residual = angle_diff_deg(
            yaw_aligned_unwrapped[mask],
            heading_unwrapped[mask],
        )
        err = np.nanstd(residual)

        if (best_err is None) or (err < best_err):
            best_err = err
            best_sign = sign
            best_offset_deg = offset_deg
            best_yaw_aligned_unwrapped = yaw_aligned_unwrapped

    if best_yaw_aligned_unwrapped is None:
        best_yaw_aligned_unwrapped = yaw_unwrapped

    return best_yaw_aligned_unwrapped, best_sign, best_offset_deg

# ------------------------------------------------------------
# Lap detection
# ------------------------------------------------------------
def detect_laps(df):
    lat = df["lat"].values
    lon = df["lon"].values
    t = df["time_s"].values

    dists = [haversine_m(lat[i], lon[i], START_LAT, START_LON)
             for i in range(len(df))]

    crossings = []
    inside_prev = dists[0] <= RADIUS_M

    for i in range(1, len(df)):
        inside = dists[i] <= RADIUS_M
        if inside and not inside_prev:
            dA = dists[i - 1]
            dB = dists[i]
            tA = t[i - 1]
            tB = t[i]

            if dA == dB:
                t_cross = tB
            else:
                ratio = (dA - RADIUS_M) / (dA - dB)
                ratio = max(0.0, min(1.0, ratio))
                t_cross = tA + ratio * (tB - tA)

            crossings.append((i, t_cross))
        inside_prev = inside

    laps = []
    for j in range(1, len(crossings)):
        idxA, tA = crossings[j - 1]
        idxB, tB = crossings[j]
        lap_t = tB - tA
        if lap_t >= MIN_LAP_TIME_S:
            laps.append({
                "lap": len(laps) + 1,
                "start_idx": idxA,
                "end_idx": idxB,
                "start_time": tA,
                "end_time": tB,
                "lap_time_s": lap_t,
            })
    return laps


# ------------------------------------------------------------
# Dash App Setup
# ------------------------------------------------------------
app = Dash(__name__)
server = app.server

app.layout = html.Div(
    style={"backgroundColor": "#050608", "color": "#ECECEC", "minHeight": "100vh", "padding": "20px"},
    children=[
        html.H2("🏎️ Midget Telemetry Viewer (GPS + Yaw + RPM)"),

        dcc.Upload(
            id="upload-data",
            children=html.Div(["📁 Drag & Drop or ", html.A("Select CSV")]),
            style={
                "width": "100%",
                "height": "60px",
                "lineHeight": "60px",
                "borderWidth": "1px",
                "borderStyle": "dashed",
                "borderRadius": "5px",
                "textAlign": "center",
                "marginBottom": "10px",
                "backgroundColor": "#111",
            },
            multiple=False,
        ),

        html.Div(id="file-info", style={"marginBottom": "10px"}),

        dcc.Dropdown(
            id="lap-dropdown",
            style={"marginBottom": "10px"},
            placeholder="Select lap (or full run)",
        ),

        dcc.Dropdown(
            id="plot-type",
            options=[
                {"label": "GPS Track", "value": "GPS Track"},
                {"label": "Yaw vs Heading", "value": "Yaw vs Heading"},
                {"label": "Slip vs Time", "value": "Slip vs Time"},
                {"label": "Speed vs Time", "value": "Speed vs Time"},
                {"label": "Accel (G) vs Time", "value": "Accel vs Time"},
                {"label": "RPM vs Time", "value": "RPM vs Time"},
            ],
            value="Yaw vs Heading",
            style={"marginBottom": "10px"},
        ),

        dcc.Graph(id="telemetry-graph", style={"height": "70vh"}),

        dcc.Store(id="store-data"),
    ],
)


# ------------------------------------------------------------
# Data Ingestion + Processing
# ------------------------------------------------------------
def parse_contents(contents, filename):
    """
    Decode the uploaded CSV and run preprocessing:
    - load data from ESP32 logger (16-column format)
    - time-align + smooth GPS heading
    - auto-align IMU yaw to GPS heading, with extra smoothing
    - smooth accel & compute |a|
    - compute smoothed, gated slip angle
    - compute RPM from tach pulses
    """
    content_type, content_string = contents.split(",")
    decoded = base64.b64decode(content_string)
    df = load_csv_arduino(io.StringIO(decoded.decode("utf-8")))

    t = df["time_s"].values
    speed = df["speed_mph"].astype(float).values
    yaw_mode = df["yaw_mode"].astype(int).values

    # -------- GPS heading from yaw_gps_deg --------
    yaw_gps_raw = df["yaw_gps_deg"].astype(float).values

    heading = np.full_like(yaw_gps_raw, np.nan, dtype=float)
    mask_good = speed >= HEADING_SPEED_THRESH_MPH
    heading[mask_good] = yaw_gps_raw[mask_good]

    # forward-fill from first valid
    valid_idx = np.where(np.isfinite(heading))[0]
    if len(valid_idx) > 0:
        first = valid_idx[0]
        heading[:first] = heading[first]
        for i in range(first + 1, len(heading)):
            if not np.isfinite(heading[i]):
                heading[i] = heading[i - 1]

    # Time-shift GPS heading backwards to compensate for lag
    heading = shift_back_in_time(heading, t, GPS_HEADING_LAG_S)

    # Unwrap & smooth heading
    heading_unwrapped = unwrap_deg(heading)
    heading_unwrapped = ema_1d(heading_unwrapped, HEADING_SMOOTH_ALPHA)
    heading_deg = wrap180(heading_unwrapped)

    df["heading_raw"] = heading
    df["heading_unwrapped"] = heading_unwrapped
    df["heading_deg"] = heading_deg

    # -------- Body yaw from IMU fusion on-board --------
    yaw_body = df["yaw_deg"].astype(float).values
    yaw_unwrapped = unwrap_deg(yaw_body)

    yaw_aligned_unwrapped_raw, yaw_sign, yaw_offset_deg = align_yaw_to_heading(
        yaw_unwrapped,
        heading_unwrapped,
        speed,
    )

    # Extra smoothing on aligned yaw
    yaw_aligned_unwrapped = ema_1d(yaw_aligned_unwrapped_raw, IMU_YAW_SMOOTH_ALPHA)

    df["yaw_unwrapped"] = yaw_unwrapped
    df["yaw_aligned_unwrapped_raw"] = yaw_aligned_unwrapped_raw
    df["yaw_aligned_unwrapped"] = yaw_aligned_unwrapped
    df["yaw_aligned_deg"] = wrap180(yaw_aligned_unwrapped)

    # -------- Accel smoothing and magnitude --------
    ax = df["accel_x_g"].astype(float).values
    ay = df["accel_y_g"].astype(float).values
    az = df["accel_z_g"].astype(float).values

    ax_f = ema_1d(ax, IMU_ACCEL_SMOOTH_ALPHA)
    ay_f = ema_1d(ay, IMU_ACCEL_SMOOTH_ALPHA)
    az_f = ema_1d(az, IMU_ACCEL_SMOOTH_ALPHA)

    df["accel_x_g_filt"] = ax_f
    df["accel_y_g_filt"] = ay_f
    df["accel_z_g_filt"] = az_f

    acc_mag = np.sqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f)
    df["accel_mag_g"] = acc_mag

    # -------- Slip angle: yaw_aligned - heading --------
    slip_unwrapped = yaw_aligned_unwrapped - heading_unwrapped
    slip_deg = wrap180(slip_unwrapped)

    # Gate slip: only when moving and yaw is GPS-corrected
    good_slip = (speed >= SLIP_SPEED_THRESH_MPH) & (yaw_mode == 1)
    slip_deg = np.where(good_slip, slip_deg, np.nan)

    # Smooth slip where valid
    finite = np.isfinite(slip_deg)
    if finite.any():
        slip_ff = slip_deg.copy()
        # forward-fill for smoothing
        last = slip_ff[finite][0]
        for i in range(len(slip_ff)):
            if np.isfinite(slip_ff[i]):
                last = slip_ff[i]
            else:
                slip_ff[i] = last
        slip_smooth = ema_1d(slip_ff, SLIP_SMOOTH_ALPHA)
        slip_smooth[~finite] = np.nan
    else:
        slip_smooth = slip_deg

    df["slip_deg"] = slip_smooth

    # -------- RPM from tach pulses --------
    df["tach_pulses"] = df["tach_pulses"].astype(float)
    df["dt_s"] = df["time_s"].diff()
    if len(df) > 1:
        # first sample: copy second dt
        if pd.isna(df["dt_s"].iloc[0]):
            df.loc[df.index[0], "dt_s"] = df["dt_s"].iloc[1]
    df["dt_s"].replace(0.0, np.nan, inplace=True)

    df["pps"] = df["tach_pulses"] / df["dt_s"]  # pulses per second
    df["rpm_raw"] = df["pps"] * 60.0 / PULSES_PER_REV
    df.loc[df["tach_pulses"] <= 0, "rpm_raw"] = np.nan

    # Smooth rpm
    rpm_ff = df["rpm_raw"].copy()
    if rpm_ff.notna().any():
        first_valid = rpm_ff.first_valid_index()
        rpm_ff.iloc[:first_valid] = rpm_ff.loc[first_valid]
        rpm_smooth = ema_1d(rpm_ff.values, RPM_SMOOTH_ALPHA)
        rpm_smooth = pd.Series(rpm_smooth, index=df.index)
        rpm_smooth[df["rpm_raw"].isna()] = np.nan
        df["rpm_smooth"] = rpm_smooth
    else:
        df["rpm_smooth"] = df["rpm_raw"]

    # Lap detection
    laps = detect_laps(df)

    payload = {
        "df_json": df.to_json(date_format="iso", orient="split"),
        "laps": laps,
        "filename": filename,
        "yaw_sign": float(yaw_sign),
        "yaw_offset_deg": float(yaw_offset_deg),
    }
    return payload


# ------------------------------------------------------------
# Dash Callbacks
# ------------------------------------------------------------
@app.callback(
    Output("store-data", "data"),
    Output("file-info", "children"),
    Output("lap-dropdown", "options"),
    Output("lap-dropdown", "value"),
    Input("upload-data", "contents"),
    State("upload-data", "filename"),
)
def handle_upload(contents, filename):
    if contents is None:
        raise PreventUpdate

    try:
        result = parse_contents(contents, filename)
    except Exception as e:
        return None, f"Error: {e}", [], None

    laps = result["laps"]
    lap_options = [{"label": "Full Run", "value": "full"}] + [
        {"label": f"Lap {lap['lap']} ({lap['lap_time_s']:.1f}s)", "value": f"lap_{lap['lap']}"}
        for lap in laps
    ]

    info = (
        f"Loaded file: {result['filename']} · {len(laps)} laps · "
        f"Yaw sign: {result['yaw_sign']:+.0f} · "
        f"Yaw offset: {result['yaw_offset_deg']:+.1f}°"
    )

    return result, info, lap_options, "full"


@app.callback(
    Output("telemetry-graph", "figure"),
    Input("store-data", "data"),
    Input("lap-dropdown", "value"),
    Input("plot-type", "value"),
)
def update_graph(data, lap_value, plot_type):
    if data is None:
        raise PreventUpdate

    df = pd.read_json(data["df_json"], orient="split")
    laps = data["laps"]

    if lap_value == "full" or not laps:
        view = df.copy()
    else:
        lap_idx = int(lap_value.split("_")[1])
        lap = next((l for l in laps if l["lap"] == lap_idx), None)
        if lap is None:
            view = df.copy()
        else:
            view = df[(df["time_s"] >= lap["start_time"]) & (df["time_s"] <= lap["end_time"])]

    view = view.copy()
    view["t_rel"] = view["time_s"] - view["time_s"].iloc[0]

    fig = go.Figure()

    if plot_type == "GPS Track":
        fig.add_trace(go.Scatter(
            x=view["lon"],
            y=view["lat"],
            mode="lines",
            name="Track",
        ))
        fig.update_layout(
            title="GPS Track",
            xaxis_title="Longitude",
            yaxis_title="Latitude",
            yaxis=dict(scaleanchor="x", scaleratio=1),
        )

    elif plot_type == "Yaw vs Heading":
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["yaw_aligned_unwrapped"],
            mode="lines",
            name="Yaw (IMU fused, aligned) [deg]",
        ))
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["heading_unwrapped"],
            mode="lines",
            name="GPS heading (unwrapped) [deg]",
        ))
        fig.update_layout(
            title="Yaw vs GPS Heading (unwrapped)",
            xaxis_title="Time [s]",
            yaxis_title="Angle [deg]",
        )

    elif plot_type == "Slip vs Time":
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["slip_deg"],
            mode="lines",
            name=f"Slip [deg] (>= {SLIP_SPEED_THRESH_MPH} mph, GPS-corrected)",
        ))
        fig.update_layout(
            title="Slip Angle vs Time",
            xaxis_title="Time [s]",
            yaxis_title="Slip [deg]",
        )

    elif plot_type == "Speed vs Time":
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["speed_mph"],
            mode="lines",
            name="Speed [mph]",
        ))
        fig.update_layout(
            title="Speed vs Time",
            xaxis_title="Time [s]",
            yaxis_title="Speed [mph]",
        )

    elif plot_type == "Accel vs Time":
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["accel_x_g_filt"],
            mode="lines",
            name="Accel X [g]",
        ))
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["accel_y_g_filt"],
            mode="lines",
            name="Accel Y [g]",
        ))
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["accel_z_g_filt"],
            mode="lines",
            name="Accel Z [g]",
        ))
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["accel_mag_g"],
            mode="lines",
            name="|Accel| [g]",
        ))
        fig.update_layout(
            title="Accel (G) vs Time",
            xaxis_title="Time [s]",
            yaxis_title="Acceleration [g]",
        )

    elif plot_type == "RPM vs Time":
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["rpm_smooth"],
            mode="lines",
            name="RPM (smoothed)",
        ))
        fig.add_trace(go.Scatter(
            x=view["t_rel"],
            y=view["rpm_raw"],
            mode="lines",
            name="RPM (raw)",
            opacity=0.3,
        ))
        fig.update_layout(
            title="Engine RPM vs Time",
            xaxis_title="Time [s]",
            yaxis_title="RPM",
        )

    fig.update_layout(
        template="plotly_dark",
        margin=dict(l=60, r=30, t=60, b=50),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )
    return fig


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
if __name__ == "__main__":
    print("Running Midget Telemetry Viewer on http://127.0.0.1:8050")
    app.run(debug=True)
