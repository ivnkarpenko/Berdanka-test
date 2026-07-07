import argparse
import csv
import datetime as dt
import time

try:
    import serial
except Exception as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

try:
    import matplotlib.pyplot as plt
except Exception as exc:
    raise SystemExit("matplotlib is required: pip install matplotlib") from exc


FIELDS = [
    "pc_time_s",
    "ms",
    "state",
    "zero",
    "raw_roll",
    "raw_pitch",
    "raw_yaw",
    "pitch",
    "yaw",
    "pQ",
    "yQ",
    "zero_samples",
    "zero_rej",
    "zero_ready",
    "zero_stable_ms",
    "yaw_auto_zero",
    "yaw_still_ms",
    "yaw_auto_zero_deg",
    "yaw_auto_zero_updates",
    "yaw_bias_dps",
    "quat_acc",
]


def parse_imu_line(line):
    line = line.strip()
    if not line.startswith("IMU,"):
        return None

    data = {}
    for part in line.split(",")[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key == "state":
            data[key] = value
            continue
        try:
            data[key] = float(value)
        except ValueError:
            data[key] = value
    return data


def get_series(rows, key):
    xs = []
    ys = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)):
            xs.append(row["pc_time_s"] - rows[0]["pc_time_s"])
            ys.append(value)
    return xs, ys


def get_series_scaled(rows, key, scale):
    xs, ys = get_series(rows, key)
    return xs, [value * scale for value in ys]


def redraw(fig, axes, rows, window_s):
    if not rows:
        return

    now_rel = rows[-1]["pc_time_s"] - rows[0]["pc_time_s"]
    min_rel = max(0.0, now_rel - window_s)

    for ax in axes:
        ax.clear()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(min_rel, max(window_s, now_rel))

    ax_yaw, ax_tilt, ax_zero = axes

    for key, label in (("raw_yaw", "raw_yaw"), ("yaw", "yaw corrected"), ("yQ", "yQ")):
        xs, ys = get_series(rows, key)
        if xs:
            ax_yaw.plot(xs, ys, label=label)
    ax_yaw.set_ylabel("Yaw deg")
    ax_yaw.legend(loc="upper left")

    for key, label in (("raw_roll", "raw_roll"), ("raw_pitch", "raw_pitch"), ("pitch", "pitch corrected")):
        xs, ys = get_series(rows, key)
        if xs:
            ax_tilt.plot(xs, ys, label=label)
    ax_tilt.set_ylabel("Roll / pitch deg")
    ax_tilt.legend(loc="upper left")

    for key, label in (("zero_samples", "zero_samples"), ("zero_rej", "zero_rej"),
                       ("zero_ready", "zero_ready")):
        xs, ys = get_series(rows, key)
        if xs:
            ax_zero.plot(xs, ys, label=label)
    xs, ys = get_series_scaled(rows, "zero_stable_ms", 0.001)
    if xs:
        ax_zero.plot(xs, ys, label="zero_stable_s")
    xs, ys = get_series(rows, "yaw_auto_zero")
    if xs:
        ax_zero.plot(xs, ys, label="yaw_auto_zero")
    xs, ys = get_series_scaled(rows, "yaw_still_ms", 0.001)
    if xs:
        ax_zero.plot(xs, ys, label="yaw_still_s")
    xs, ys = get_series(rows, "yaw_auto_zero_deg")
    if xs:
        ax_zero.plot(xs, ys, label="yaw_auto_zero_deg")
    xs, ys = get_series(rows, "yaw_bias_dps")
    if xs:
        ax_zero.plot(xs, ys, label="yaw_bias_dps")
    ax_zero.set_ylabel("Zero / bias")
    ax_zero.set_xlabel("Time, s")
    ax_zero.legend(loc="upper left")

    fig.canvas.draw()
    fig.canvas.flush_events()


def main():
    parser = argparse.ArgumentParser(description="Plot Berdanka IMU serial logs from Arduino.")
    parser.add_argument("--port", default="COM4", help="Serial port, default: COM4")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud, default: 115200")
    parser.add_argument("--csv", default=None, help="CSV output path")
    parser.add_argument("--window", type=float, default=120.0, help="Visible plot window in seconds")
    parser.add_argument("--max-rows", type=int, default=20000, help="Rows kept in memory for plotting")
    args = parser.parse_args()

    csv_path = args.csv
    if csv_path is None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = f"imu_log_{stamp}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    print(f"Reading {args.port} at {args.baud}. Saving CSV to {csv_path}")
    print("Waiting for IMU lines...")

    plt.ion()
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(f"Berdanka IMU live plot: {args.port}")

    rows = []
    last_draw = 0.0
    start_pc = time.time()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()

        while plt.fignum_exists(fig.number):
            raw = ser.readline()
            if not raw:
                plt.pause(0.01)
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            data = parse_imu_line(line)
            if data is None:
                print(line)
                continue

            data["pc_time_s"] = time.time()
            rows.append(data)
            if len(rows) > args.max_rows:
                rows = rows[-args.max_rows:]

            writer.writerow({key: data.get(key, "") for key in FIELDS})
            f.flush()
            print(line)

            if time.time() - last_draw >= 0.5:
                redraw(fig, axes, rows, args.window)
                last_draw = time.time()

    ser.close()
    print(f"Stopped. CSV saved to {csv_path}. Runtime {time.time() - start_pc:.1f}s")


if __name__ == "__main__":
    main()
