from __future__ import annotations

import math
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, State, ctx, dcc, html, no_update


DEFAULT_ARDUINO_IP = "192.168.4.1"
DEFAULT_ARDUINO_PORT = 3333
DEFAULT_TARGET_RANGE_M = 300.0
DEFAULT_SCREEN_W = 480
DEFAULT_SCREEN_H = 320
DEFAULT_FOV_X_DEG = 60.0
DEFAULT_FOV_Y_DEG = 80.0


@dataclass
class Telemetry:
    connected: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    target_pitch: float = 0.0
    target_yaw: float = 0.0
    pitch_rel: float = 0.0
    yaw_rel: float = 0.0
    fov_x: float = DEFAULT_FOV_X_DEG
    fov_y: float = DEFAULT_FOV_Y_DEG
    on_target: bool = False
    last_seen_s: float = 0.0
    last_line: str = "-"


class ArduinoTcpClient:
    def __init__(self) -> None:
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._telemetry = Telemetry()
        self._logs: queue.Queue[str] = queue.Queue(maxsize=300)

    def connect(self, host: str, port: int) -> str:
        self.disconnect()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((host, port))
            sock.settimeout(0.2)
        except Exception as exc:
            return f"CONNECT ERROR: {exc}"

        with self._lock:
            self._sock = sock
            self._telemetry.connected = True
            self._telemetry.last_seen_s = time.time()

        self._stop.clear()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

        self.send_line("CFG:TELEMETRY:1")
        self.send_line("PING")
        self._log(f"Connected to {host}:{port}")
        return "CONNECTED"

    def disconnect(self) -> None:
        self._stop.set()
        with self._lock:
            sock = self._sock
            self._sock = None
            self._telemetry.connected = False
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

    def send_line(self, line: str) -> bool:
        if not line.endswith("\n"):
            line += "\n"
        with self._lock:
            sock = self._sock
        if sock is None:
            self._log("TX skipped: not connected")
            return False
        try:
            sock.sendall(line.encode("utf-8"))
            self._log(f"TX {line.strip()}")
            return True
        except Exception as exc:
            self._log(f"TX ERROR: {exc}")
            self.disconnect()
            return False

    def snapshot(self) -> Telemetry:
        with self._lock:
            return Telemetry(**self._telemetry.__dict__)

    def drain_logs(self, limit: int = 40) -> list[str]:
        out: list[str] = []
        for _ in range(limit):
            try:
                out.append(self._logs.get_nowait())
            except queue.Empty:
                break
        return out

    def _log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        item = f"{stamp} {message}"
        try:
            self._logs.put_nowait(item)
        except queue.Full:
            try:
                self._logs.get_nowait()
                self._logs.put_nowait(item)
            except queue.Empty:
                pass

    def _rx_loop(self) -> None:
        buffer = b""
        while not self._stop.is_set():
            with self._lock:
                sock = self._sock
            if sock is None:
                break
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk
            except socket.timeout:
                continue
            except Exception as exc:
                self._log(f"RX ERROR: {exc}")
                break

            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.replace(b"\r", b"").decode("utf-8", errors="replace").strip()
                if line:
                    self._handle_line(line)

        with self._lock:
            self._sock = None
            self._telemetry.connected = False
        self._log("Disconnected")

    def _handle_line(self, line: str) -> None:
        if line.startswith("TEL;"):
            parsed = parse_telemetry(line)
            with self._lock:
                current = self._telemetry
                current.connected = True
                current.last_seen_s = time.time()
                current.last_line = line
                for key, value in parsed.items():
                    setattr(current, key, value)
            return
        self._log(f"RX {line}")


def parse_telemetry(line: str) -> dict[str, Any]:
    aliases = {
        "ROLL": "roll",
        "PITCH": "pitch",
        "YAW": "yaw",
        "TARGET_PITCH": "target_pitch",
        "TARGET_YAW": "target_yaw",
        "PITCH_REL": "pitch_rel",
        "YAW_REL": "yaw_rel",
        "FOV_X": "fov_x",
        "FOV_Y": "fov_y",
        "ON_TARGET": "on_target",
    }
    values: dict[str, Any] = {}
    for part in line.split(";")[1:]:
        if ":" not in part:
            continue
        key, raw = part.split(":", 1)
        attr = aliases.get(key.strip().upper())
        if attr is None:
            continue
        try:
            if attr == "on_target":
                values[attr] = raw.strip() == "1"
            else:
                values[attr] = float(raw)
        except ValueError:
            continue
    return values


def vec3(x: float, y: float, z: float = 0.0) -> np.ndarray:
    return np.array([float(x), float(y), float(z)], dtype=float)


def normalize(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm <= 1e-12:
        return np.zeros(3, dtype=float)
    return v / norm


def wrap180(angle: float) -> float:
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def direction_from_az_el(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    return vec3(math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el))


def target_from_angles(azimuth_deg: float, elevation_deg: float, range_m: float) -> np.ndarray:
    return direction_from_az_el(azimuth_deg, elevation_deg) * float(range_m)


def angles_from_target(target: np.ndarray) -> tuple[float, float, float]:
    distance = float(np.linalg.norm(target))
    direction = normalize(target)
    az = math.degrees(math.atan2(direction[0], direction[1]))
    el = math.degrees(math.atan2(direction[2], math.hypot(direction[0], direction[1])))
    return az, el, distance


def device_forward(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    return direction_from_az_el(yaw_deg, pitch_deg)


def screen_point_firmware(
    target_azimuth_deg: float,
    target_elevation_deg: float,
    device_yaw_deg: float,
    device_pitch_deg: float,
    screen_w: int,
    screen_h: int,
    fov_x_deg: float,
    fov_y_deg: float,
) -> tuple[float, float, float, float, bool]:
    yaw_offset = wrap180(device_yaw_deg - target_azimuth_deg)
    elevation_offset = target_elevation_deg - device_pitch_deg
    x = screen_w / 2.0 + yaw_offset * screen_w / max(1e-6, fov_x_deg)
    y = screen_h / 2.0 - elevation_offset * screen_h / max(1e-6, fov_y_deg)
    outside = x < 0 or x > screen_w or y < 0 or y > screen_h
    return x, y, yaw_offset, elevation_offset, outside


def line3(name: str, a: np.ndarray, b: np.ndarray, color: str, width: int = 4, dash: str = "solid") -> go.Scatter3d:
    return go.Scatter3d(
        x=[a[0], b[0]],
        y=[a[1], b[1]],
        z=[a[2], b[2]],
        mode="lines",
        name=name,
        line={"color": color, "width": width, "dash": dash},
    )


def marker3(name: str, p: np.ndarray, color: str, size: int = 7, symbol: str = "circle") -> go.Scatter3d:
    return go.Scatter3d(
        x=[p[0]],
        y=[p[1]],
        z=[p[2]],
        mode="markers",
        name=name,
        marker={"color": color, "size": size, "symbol": symbol},
        hovertemplate=f"{name}<br>X=%{{x:.2f}} m<br>Y=%{{y:.2f}} m<br>Z=%{{z:.2f}} m<extra></extra>",
    )


def make_scene(target: np.ndarray, device: np.ndarray, yaw: float, pitch: float) -> go.Figure:
    static = vec3(0, 0, 0)
    forward_end = device + device_forward(yaw, pitch) * 80.0
    max_xy = max(40.0, abs(target[0]) * 1.08, abs(target[1]) * 1.08, abs(device[0]) + 30, abs(device[1]) + 30)
    max_z = max(20.0, abs(target[2]) * 1.25 + 10, abs(device[2]) + 20)
    min_y = min(-20.0, float(device[1]) - 30.0)

    grid: list[go.Scatter3d] = []
    step = 25.0 if max_xy > 120 else 10.0
    for x in np.arange(-max_xy, max_xy + step, step):
        grid.append(go.Scatter3d(x=[x, x], y=[min_y, max_xy], z=[0, 0], mode="lines", showlegend=False, line={"color": "#e2e8f0", "width": 1}, hoverinfo="skip"))
    for y in np.arange(min_y, max_xy + step, step):
        grid.append(go.Scatter3d(x=[-max_xy, max_xy], y=[y, y], z=[0, 0], mode="lines", showlegend=False, line={"color": "#e2e8f0", "width": 1}, hoverinfo="skip"))

    traces = [
        *grid,
        line3("X east", static, vec3(25, 0, 0), "#ef4444", 5),
        line3("Y north", static, vec3(0, 25, 0), "#22c55e", 5),
        line3("Z up", static, vec3(0, 0, 15), "#3b82f6", 5),
        marker3("static", static, "#111827", 8, "diamond"),
        marker3("device", device, "#0f766e", 8, "circle"),
        marker3("target", target, "#b91c1c", 7, "diamond"),
        line3("static target ray", static, target, "#6b7280", 3, "dash"),
        line3("device to target", device, target, "#16a34a", 5),
        line3("device forward", device, forward_end, "#2563eb", 5),
        line3("target drop", target, vec3(target[0], target[1], 0), "#991b1b", 2, "dot"),
        line3("device drop", device, vec3(device[0], device[1], 0), "#0f766e", 2, "dot"),
    ]
    fig = go.Figure(traces)
    fig.update_layout(
        margin={"l": 0, "r": 0, "t": 5, "b": 0},
        paper_bgcolor="#ffffff",
        legend={"orientation": "h", "y": 0.01, "x": 0.02, "bgcolor": "rgba(255,255,255,0.85)"},
        scene={
            "xaxis": {"title": "X east, m", "range": [-max_xy, max_xy], "backgroundcolor": "#ffffff"},
            "yaxis": {"title": "Y north, m", "range": [min_y, max_xy], "backgroundcolor": "#ffffff"},
            "zaxis": {"title": "Z up, m", "range": [0, max_z], "backgroundcolor": "#ffffff"},
            "aspectmode": "manual",
            "aspectratio": {"x": 1, "y": 1, "z": 0.45},
        },
    )
    return fig


def make_screen(
    target_azimuth: float,
    target_elevation: float,
    yaw: float,
    pitch: float,
    screen_w: int,
    screen_h: int,
    fov_x: float,
    fov_y: float,
) -> go.Figure:
    raw_x, raw_y, yaw_offset, elevation_offset, outside = screen_point_firmware(
        target_azimuth,
        target_elevation,
        yaw,
        pitch,
        screen_w,
        screen_h,
        fov_x,
        fov_y,
    )
    marker_x = min(max(raw_x, 0.0), float(screen_w))
    marker_y = min(max(raw_y, 0.0), float(screen_h))
    color = "#dc2626" if outside else "#16a34a"

    fig = go.Figure()
    fig.add_shape(type="rect", x0=0, y0=0, x1=screen_w, y1=screen_h, line={"color": "#111827", "width": 2}, fillcolor="#020617")
    fig.add_shape(type="line", x0=screen_w / 2, y0=0, x1=screen_w / 2, y1=screen_h, line={"color": "#475569", "width": 1})
    fig.add_shape(type="line", x0=0, y0=screen_h / 2, x1=screen_w, y1=screen_h / 2, line={"color": "#475569", "width": 1})
    fig.add_trace(go.Scatter(x=[marker_x], y=[marker_y], mode="markers", marker={"size": 22, "color": color, "symbol": "square"}, name="target marker"))
    fig.add_annotation(x=8, y=18, text=f"yawRel={yaw_offset:+.1f} deg  elRel={elevation_offset:+.1f} deg", showarrow=False, font={"color": "#f8fafc"}, xanchor="left")
    fig.update_layout(
        margin={"l": 10, "r": 10, "t": 10, "b": 10},
        paper_bgcolor="#ffffff",
        plot_bgcolor="#020617",
        showlegend=False,
        xaxis={"range": [0, screen_w], "showgrid": False, "zeroline": False, "visible": False},
        yaxis={"range": [screen_h, 0], "showgrid": False, "zeroline": False, "visible": False, "scaleanchor": "x", "scaleratio": 1},
    )
    return fig


client = ArduinoTcpClient()
app = Dash(__name__)


def number_input(id_: str, value: float, step: float = 1.0) -> dcc.Input:
    return dcc.Input(id=id_, type="number", value=value, step=step, debounce=False)


def field(label: str, control: Any) -> html.Div:
    return html.Div([html.Label(label), control], className="field")


app.layout = html.Div(
    [
        html.Div(
            [
                html.H1("Berdanka Jetson Wi-Fi Visualizer"),
                html.Div(
                    [
                        html.H2("Connection"),
                        field("Arduino IP", dcc.Input(id="ip", value=DEFAULT_ARDUINO_IP, type="text")),
                        field("Port", dcc.Input(id="port", value=DEFAULT_ARDUINO_PORT, type="number")),
                        html.Div([html.Button("Connect", id="connect"), html.Button("Disconnect", id="disconnect")], className="button-row"),
                        html.Div(id="connection-status", className="status"),
                    ],
                    className="panel",
                ),
                html.Div(
                    [
                        html.H2("Target"),
                        dcc.RadioItems(
                            id="target-mode",
                            options=[{"label": "Angles", "value": "angles"}, {"label": "Coordinates", "value": "coords"}],
                            value="angles",
                            inline=True,
                        ),
                        field("Azimuth, deg", dcc.Slider(id="target-az", min=-180, max=180, step=1, value=0, marks={-180: "-180", 0: "0", 180: "180"})),
                        field("Elevation, deg", dcc.Slider(id="target-el", min=-45, max=45, step=1, value=0, marks={-45: "-45", 0: "0", 45: "45"})),
                        field("Assumed range, m", dcc.Slider(id="target-range", min=50, max=1000, step=10, value=DEFAULT_TARGET_RANGE_M, marks={100: "100", 300: "300", 600: "600", 1000: "1000"})),
                        html.Div(
                            [
                                field("Target X, m", number_input("target-x", 0, 1)),
                                field("Target Y, m", number_input("target-y", DEFAULT_TARGET_RANGE_M, 1)),
                                field("Target Z, m", number_input("target-z", 0, 1)),
                            ],
                            className="grid3",
                        ),
                        html.Button("Send Target To Arduino", id="send-target"),
                    ],
                    className="panel",
                ),
                html.Div(
                    [
                        html.H2("Device"),
                        dcc.Checklist(id="device-flags", options=[{"label": "Use Wi-Fi telemetry orientation", "value": "wifi"}], value=["wifi"]),
                        html.Div(
                            [
                                field("X, m", number_input("device-x", 0, 0.5)),
                                field("Y, m", number_input("device-y", 0, 0.5)),
                                field("Z, m", number_input("device-z", 0, 0.5)),
                            ],
                            className="grid3",
                        ),
                        html.Div(
                            [
                                field("Yaw, deg", number_input("manual-yaw", 0, 1)),
                                field("Pitch, deg", number_input("manual-pitch", 0, 1)),
                                field("Roll, deg", number_input("manual-roll", 0, 1)),
                            ],
                            className="grid3",
                        ),
                    ],
                    className="panel",
                ),
                html.Div(
                    [
                        html.H2("Screen"),
                        html.Div(
                            [
                                field("Width", number_input("screen-w", DEFAULT_SCREEN_W, 1)),
                                field("Height", number_input("screen-h", DEFAULT_SCREEN_H, 1)),
                                field("FOV X", number_input("fov-x", DEFAULT_FOV_X_DEG, 1)),
                                field("FOV Y", number_input("fov-y", DEFAULT_FOV_Y_DEG, 1)),
                            ],
                            className="grid4",
                        ),
                        html.Button("Apply FOV To Arduino", id="send-fov"),
                    ],
                    className="panel",
                ),
                html.Pre(id="action-log", className="log"),
            ],
            className="sidebar",
        ),
        html.Div(
            [
                html.Div([html.H2("3D Scene"), dcc.Graph(id="scene", className="graph")], className="card"),
                html.Div([html.H2("Device Screen"), dcc.Graph(id="screen", className="screen")], className="card"),
                html.Div([html.H2("Telemetry"), html.Div(id="metrics", className="metrics")], className="card"),
            ],
            className="main",
        ),
        dcc.Interval(id="tick", interval=250, n_intervals=0),
    ],
    className="app",
)


app.index_string = """
<!DOCTYPE html>
<html>
  <head>
    {%metas%}
    <title>Berdanka Jetson Visualizer</title>
    {%favicon%}
    {%css%}
    <style>
      body { margin: 0; font-family: Arial, sans-serif; background: #f1f5f9; color: #0f172a; }
      .app { display: grid; grid-template-columns: 390px 1fr; min-height: 100vh; }
      .sidebar { padding: 14px; overflow: auto; background: #ffffff; border-right: 1px solid #cbd5e1; }
      h1 { font-size: 22px; margin: 0 0 14px; }
      h2 { font-size: 15px; margin: 0 0 10px; }
      .panel, .card { border: 1px solid #cbd5e1; border-radius: 6px; background: #ffffff; padding: 12px; margin-bottom: 12px; }
      .main { padding: 14px; display: grid; grid-template-rows: minmax(420px, 1fr) 360px auto; gap: 12px; }
      .field { margin-bottom: 10px; }
      .field label { display: block; font-size: 12px; color: #475569; margin-bottom: 4px; }
      input { width: 100%; box-sizing: border-box; padding: 6px 8px; border: 1px solid #cbd5e1; border-radius: 4px; }
      button { padding: 7px 10px; border: 1px solid #334155; border-radius: 4px; background: #0f172a; color: #fff; cursor: pointer; }
      .button-row { display: flex; gap: 8px; }
      .grid3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
      .grid4 { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; }
      .status { margin-top: 10px; font-weight: bold; }
      .graph { height: 100%; min-height: 390px; }
      .screen { height: 320px; }
      .metrics { display: grid; grid-template-columns: repeat(4, minmax(140px, 1fr)); gap: 8px; }
      .metric { border: 1px solid #e2e8f0; border-radius: 6px; padding: 8px; background: #f8fafc; }
      .metric b { display: block; color: #475569; font-size: 12px; margin-bottom: 4px; }
      .log { min-height: 110px; white-space: pre-wrap; font-size: 12px; background: #020617; color: #dbeafe; padding: 8px; border-radius: 6px; }
    </style>
  </head>
  <body>
    {%app_entry%}
    <footer>{%config%}{%scripts%}{%renderer%}</footer>
  </body>
</html>
"""


def current_target(mode: str, az: float, el: float, range_m: float, x: float, y: float, z: float) -> tuple[np.ndarray, float, float, float]:
    if mode == "coords":
        target = vec3(x, y, z)
        target_az, target_el, target_range = angles_from_target(target)
        return target, target_az, target_el, target_range
    target = target_from_angles(az, el, range_m)
    return target, az, el, range_m


@app.callback(
    Output("action-log", "children"),
    Input("connect", "n_clicks"),
    Input("disconnect", "n_clicks"),
    Input("send-target", "n_clicks"),
    Input("send-fov", "n_clicks"),
    State("ip", "value"),
    State("port", "value"),
    State("target-mode", "value"),
    State("target-az", "value"),
    State("target-el", "value"),
    State("target-range", "value"),
    State("target-x", "value"),
    State("target-y", "value"),
    State("target-z", "value"),
    State("fov-x", "value"),
    State("fov-y", "value"),
    prevent_initial_call=True,
)
def actions(
    connect_clicks: int | None,
    disconnect_clicks: int | None,
    send_target_clicks: int | None,
    send_fov_clicks: int | None,
    ip: str,
    port: int,
    mode: str,
    az: float,
    el: float,
    range_m: float,
    target_x: float,
    target_y: float,
    target_z: float,
    fov_x: float,
    fov_y: float,
) -> str:
    trigger = ctx.triggered_id
    if trigger == "connect":
        result = client.connect(str(ip or DEFAULT_ARDUINO_IP), int(port or DEFAULT_ARDUINO_PORT))
        return result
    if trigger == "disconnect":
        client.send_line("CFG:TELEMETRY:0")
        client.disconnect()
        return "DISCONNECTED"
    if trigger == "send-target":
        _, target_az, target_el, _ = current_target(
            mode,
            float(az or 0),
            float(el or 0),
            float(range_m or DEFAULT_TARGET_RANGE_M),
            float(target_x or 0),
            float(target_y or 0),
            float(target_z or 0),
        )
        ok = client.send_line(f"MSG:JETSON;X:{target_el:.2f};Y:{target_az:.2f}")
        return f"TARGET SENT az={target_az:.2f} el={target_el:.2f}" if ok else "TARGET SEND FAILED"
    if trigger == "send-fov":
        ok1 = client.send_line(f"CFG:FOV_X:{float(fov_x or DEFAULT_FOV_X_DEG):.2f}")
        ok2 = client.send_line(f"CFG:FOV_Y:{float(fov_y or DEFAULT_FOV_Y_DEG):.2f}")
        return "FOV SENT" if ok1 and ok2 else "FOV SEND FAILED"
    return no_update


@app.callback(
    Output("scene", "figure"),
    Output("screen", "figure"),
    Output("metrics", "children"),
    Output("connection-status", "children"),
    Input("tick", "n_intervals"),
    State("target-mode", "value"),
    State("target-az", "value"),
    State("target-el", "value"),
    State("target-range", "value"),
    State("target-x", "value"),
    State("target-y", "value"),
    State("target-z", "value"),
    State("device-x", "value"),
    State("device-y", "value"),
    State("device-z", "value"),
    State("manual-yaw", "value"),
    State("manual-pitch", "value"),
    State("manual-roll", "value"),
    State("device-flags", "value"),
    State("screen-w", "value"),
    State("screen-h", "value"),
    State("fov-x", "value"),
    State("fov-y", "value"),
)
def refresh(
    _n: int,
    mode: str,
    az: float,
    el: float,
    range_m: float,
    target_x: float,
    target_y: float,
    target_z: float,
    device_x: float,
    device_y: float,
    device_z: float,
    manual_yaw: float,
    manual_pitch: float,
    manual_roll: float,
    flags: list[str],
    screen_w: int,
    screen_h: int,
    fov_x: float,
    fov_y: float,
) -> tuple[go.Figure, go.Figure, list[html.Div], str]:
    telemetry = client.snapshot()
    target, target_az, target_el, target_distance = current_target(
        mode,
        float(az or 0),
        float(el or 0),
        float(range_m or DEFAULT_TARGET_RANGE_M),
        float(target_x or 0),
        float(target_y or DEFAULT_TARGET_RANGE_M),
        float(target_z or 0),
    )
    device = vec3(float(device_x or 0), float(device_y or 0), float(device_z or 0))
    telemetry_fresh = telemetry.connected and (time.time() - telemetry.last_seen_s <= 3.0)
    use_wifi = "wifi" in (flags or []) and telemetry_fresh
    yaw = telemetry.yaw if use_wifi else float(manual_yaw or 0)
    pitch = telemetry.pitch if use_wifi else float(manual_pitch or 0)
    roll = telemetry.roll if use_wifi else float(manual_roll or 0)

    screen_w_i = max(120, int(screen_w or DEFAULT_SCREEN_W))
    screen_h_i = max(120, int(screen_h or DEFAULT_SCREEN_H))
    fov_x_f = max(5.0, float(fov_x or DEFAULT_FOV_X_DEG))
    fov_y_f = max(5.0, float(fov_y or DEFAULT_FOV_Y_DEG))

    scene = make_scene(target, device, yaw, pitch)
    screen = make_screen(target_az, target_el, yaw, pitch, screen_w_i, screen_h_i, fov_x_f, fov_y_f)
    raw_x, raw_y, yaw_rel, el_rel, outside = screen_point_firmware(target_az, target_el, yaw, pitch, screen_w_i, screen_h_i, fov_x_f, fov_y_f)

    distance_from_device = float(np.linalg.norm(target - device))
    status = "CONNECTED" if telemetry.connected else "DISCONNECTED"
    if telemetry.connected:
        age = time.time() - telemetry.last_seen_s if telemetry.last_seen_s else 999.0
        status += f" telemetry_age={age:.1f}s"

    def metric(name: str, value: str) -> html.Div:
        return html.Div([html.B(name), html.Span(value)], className="metric")

    metrics = [
        metric("target az/el", f"{target_az:+.1f} / {target_el:+.1f} deg"),
        metric("assumed range", f"{target_distance:.1f} m"),
        metric("target xyz", f"{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}"),
        metric("device yaw/pitch/roll", f"{yaw:+.1f}, {pitch:+.1f}, {roll:+.1f}"),
        metric("firmware yawRel/elRel", f"{yaw_rel:+.1f}, {el_rel:+.1f} deg"),
        metric("screen xy", f"{raw_x:.1f}, {raw_y:.1f} px"),
        metric("target from device", f"{distance_from_device:.1f} m"),
        metric("outside screen", "yes" if outside else "no"),
        metric("wifi telemetry", "used" if use_wifi else "manual"),
        metric("last TEL", telemetry.last_line[:120]),
    ]
    return scene, screen, metrics, status


def main() -> None:
    app.run(host="0.0.0.0", port=8050, debug=False)


if __name__ == "__main__":
    main()
