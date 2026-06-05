from __future__ import annotations

import math
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

from dash import Dash, Input, Output, State, ctx, dcc, html, no_update


DEFAULT_ARDUINO_IP = "192.168.4.1"
DEFAULT_ARDUINO_PORT = 3333
DEFAULT_TARGET_RANGE_M = 300.0
DEFAULT_SCREEN_W = 480
DEFAULT_SCREEN_H = 320
DEFAULT_FOV_X_DEG = 60.0
DEFAULT_FOV_Y_DEG = 80.0
DEFAULT_BOX_SIZE_PX = 50
DEFAULT_BOX_REFRESH_MS = 33


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
        self.send_line("MSGONLY:CONNECTED")
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

    def logs(self, limit: int = 80) -> list[str]:
        items = list(self._logs.queue)
        return items[-limit:]

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
            values[attr] = raw.strip() == "1" if attr == "on_target" else float(raw)
        except ValueError:
            continue
    return values


def wrap180(angle: float) -> float:
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def target_xyz(azimuth_deg: float, elevation_deg: float, range_m: float) -> tuple[float, float, float]:
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    return (
        math.cos(el) * math.sin(az) * range_m,
        math.cos(el) * math.cos(az) * range_m,
        math.sin(el) * range_m,
    )


def screen_projection(
    target_yaw: float,
    target_pitch: float,
    device_yaw: float,
    device_pitch: float,
    screen_w: int,
    screen_h: int,
    fov_x: float,
    fov_y: float,
) -> tuple[float, float, float, float]:
    yaw_rel = wrap180(device_yaw - target_yaw)
    pitch_rel = target_pitch - device_pitch
    x = screen_w / 2.0 + yaw_rel * screen_w / max(1e-6, fov_x)
    y = screen_h / 2.0 - pitch_rel * screen_h / max(1e-6, fov_y)
    return x, y, yaw_rel, pitch_rel


client = ArduinoTcpClient()
app = Dash(__name__)


def number_input(id_: str, value: float, step: float = 1.0, min_: float | None = None, max_: float | None = None) -> dcc.Input:
    return dcc.Input(id=id_, type="number", value=value, step=step, min=min_, max=max_, debounce=False)


def field(label: str, control: Any) -> html.Div:
    return html.Div([html.Label(label), control], className="field")


app.layout = html.Div(
    [
        html.Div(
            [
                html.H1("Berdanka Jetson Control"),
                html.Div(
                    [
                        html.H2("Arduino Wi-Fi"),
                        html.Div(
                            [
                                field("IP", dcc.Input(id="ip", value=DEFAULT_ARDUINO_IP, type="text")),
                                field("Port", number_input("port", DEFAULT_ARDUINO_PORT, 1)),
                            ],
                            className="grid2",
                        ),
                        html.Div([html.Button("Connect", id="connect"), html.Button("Disconnect", id="disconnect")], className="button-row"),
                        html.Div(id="connection-status", className="status"),
                    ],
                    className="panel",
                ),
                html.Div(
                    [
                        html.H2("Target"),
                        field("Msg on Arduino", dcc.Input(id="target-msg", value="JETSON", type="text", maxLength=30)),
                        field("Azimuth, deg", dcc.Slider(id="target-az", min=-180, max=180, step=1, value=0, marks={-180: "-180", 0: "0", 180: "180"})),
                        field("Elevation, deg", dcc.Slider(id="target-el", min=-45, max=45, step=1, value=0, marks={-45: "-45", 0: "0", 45: "45"})),
                        field("Assumed range, m", dcc.Slider(id="target-range", min=50, max=1000, step=10, value=DEFAULT_TARGET_RANGE_M, marks={100: "100", 300: "300", 600: "600", 1000: "1000"})),
                        html.Div(
                            [
                                html.Button("Send Target", id="send-target"),
                                html.Button("Send Msg Only", id="send-msg"),
                                html.Button("Center Arduino", id="center"),
                            ],
                            className="button-row",
                        ),
                    ],
                    className="panel",
                ),
                html.Div(
                    [
                        html.H2("Arduino Display"),
                        html.Div(
                            [
                                field("Screen W", number_input("screen-w", DEFAULT_SCREEN_W, 1, 120, 1920)),
                                field("Screen H", number_input("screen-h", DEFAULT_SCREEN_H, 1, 120, 1920)),
                                field("FOV X", number_input("fov-x", DEFAULT_FOV_X_DEG, 1, 5, 180)),
                                field("FOV Y", number_input("fov-y", DEFAULT_FOV_Y_DEG, 1, 5, 180)),
                            ],
                            className="grid4",
                        ),
                        html.Div(
                            [
                                field("Box size", number_input("box-size", DEFAULT_BOX_SIZE_PX, 1, 8, 200)),
                                field("Box refresh ms", number_input("box-refresh", DEFAULT_BOX_REFRESH_MS, 1, 10, 1000)),
                                field("TCP poll ms", number_input("tcp-poll", 80, 1, 20, 2000)),
                            ],
                            className="grid3",
                        ),
                        dcc.Checklist(
                            id="render-flags",
                            options=[
                                {"label": "Draw box", "value": "box"},
                                {"label": "Delta fill", "value": "delta"},
                                {"label": "Telemetry", "value": "tel"},
                            ],
                            value=["box", "delta", "tel"],
                            inline=True,
                        ),
                        html.Div(
                            [
                                html.Button("Apply Display", id="send-display"),
                                html.Button("Apply Box/Net", id="send-box-net"),
                            ],
                            className="button-row",
                        ),
                    ],
                    className="panel",
                ),
            ],
            className="left",
        ),
        html.Div(
            [
                html.Div([html.H2("Current Values"), html.Div(id="metrics", className="metrics")], className="panel"),
                html.Div([html.H2("Log"), html.Pre(id="log", className="log")], className="panel"),
            ],
            className="right",
        ),
        dcc.Interval(id="tick", interval=500, n_intervals=0),
    ],
    className="app",
)


app.index_string = """
<!DOCTYPE html>
<html>
  <head>
    {%metas%}
    <title>Berdanka Jetson Control</title>
    {%favicon%}
    {%css%}
    <style>
      body { margin: 0; font-family: Arial, sans-serif; background: #f1f5f9; color: #0f172a; }
      .app { display: grid; grid-template-columns: minmax(420px, 520px) 1fr; gap: 14px; min-height: 100vh; padding: 14px; box-sizing: border-box; }
      h1 { margin: 0 0 12px; font-size: 24px; }
      h2 { margin: 0 0 10px; font-size: 16px; }
      .panel { background: #fff; border: 1px solid #cbd5e1; border-radius: 6px; padding: 12px; margin-bottom: 12px; }
      .field { margin-bottom: 10px; }
      .field label { display: block; font-size: 12px; color: #475569; margin-bottom: 4px; }
      input { width: 100%; box-sizing: border-box; padding: 7px 8px; border: 1px solid #cbd5e1; border-radius: 4px; }
      button { padding: 8px 11px; border: 1px solid #334155; border-radius: 4px; background: #0f172a; color: #fff; cursor: pointer; }
      .button-row { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 8px; }
      .grid2 { display: grid; grid-template-columns: 2fr 1fr; gap: 10px; }
      .grid3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
      .grid4 { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }
      .status { margin-top: 10px; font-weight: bold; }
      .metrics { display: grid; grid-template-columns: repeat(3, minmax(150px, 1fr)); gap: 10px; }
      .metric { border: 1px solid #e2e8f0; border-radius: 6px; padding: 9px; background: #f8fafc; min-height: 48px; }
      .metric b { display: block; color: #475569; font-size: 12px; margin-bottom: 5px; }
      .log { min-height: 360px; white-space: pre-wrap; font-size: 12px; background: #020617; color: #dbeafe; padding: 10px; border-radius: 6px; }
    </style>
  </head>
  <body>
    {%app_entry%}
    <footer>{%config%}{%scripts%}{%renderer%}</footer>
  </body>
</html>
"""


@app.callback(
    Output("log", "children", allow_duplicate=True),
    Input("connect", "n_clicks"),
    Input("disconnect", "n_clicks"),
    Input("send-target", "n_clicks"),
    Input("send-msg", "n_clicks"),
    Input("center", "n_clicks"),
    Input("send-display", "n_clicks"),
    Input("send-box-net", "n_clicks"),
    State("ip", "value"),
    State("port", "value"),
    State("target-msg", "value"),
    State("target-az", "value"),
    State("target-el", "value"),
    State("fov-x", "value"),
    State("fov-y", "value"),
    State("box-size", "value"),
    State("box-refresh", "value"),
    State("tcp-poll", "value"),
    State("render-flags", "value"),
    prevent_initial_call=True,
)
def actions(
    _connect: int | None,
    _disconnect: int | None,
    _send_target: int | None,
    _send_msg: int | None,
    _center: int | None,
    _send_display: int | None,
    _send_box_net: int | None,
    ip: str,
    port: int,
    target_msg: str,
    target_az: float,
    target_el: float,
    fov_x: float,
    fov_y: float,
    box_size: int,
    box_refresh: int,
    tcp_poll: int,
    render_flags: list[str],
) -> str:
    trigger = ctx.triggered_id
    msg = str(target_msg or "JETSON").strip()[:30] or "JETSON"
    flags = set(render_flags or [])

    if trigger == "connect":
        client.connect(str(ip or DEFAULT_ARDUINO_IP), int(port or DEFAULT_ARDUINO_PORT))
    elif trigger == "disconnect":
        client.send_line("CFG:TELEMETRY:0")
        client.disconnect()
    elif trigger == "send-target":
        client.send_line(f"MSG:{msg};X:{float(target_el or 0):.2f};Y:{float(target_az or 0):.2f}")
    elif trigger == "send-msg":
        client.send_line(f"MSGONLY:{msg}")
    elif trigger == "center":
        client.send_line("CMD:CENTER")
    elif trigger == "send-display":
        client.send_line(f"CFG:FOV_X:{float(fov_x or DEFAULT_FOV_X_DEG):.2f}")
        client.send_line(f"CFG:FOV_Y:{float(fov_y or DEFAULT_FOV_Y_DEG):.2f}")
    elif trigger == "send-box-net":
        client.send_line(f"CFG:BOX_SIZE:{int(box_size or DEFAULT_BOX_SIZE_PX)}")
        client.send_line(f"CFG:BOX_REFRESH_MS:{int(box_refresh or DEFAULT_BOX_REFRESH_MS)}")
        client.send_line(f"CFG:TCP_POLL_MS:{int(tcp_poll or 80)}")
        client.send_line(f"CFG:BOX_RENDER:{1 if 'box' in flags else 0}")
        client.send_line(f"CFG:BOX_DELTA_RENDER:{1 if 'delta' in flags else 0}")
        client.send_line(f"CFG:TELEMETRY:{1 if 'tel' in flags else 0}")
    else:
        return no_update

    return "\n".join(client.logs())


@app.callback(
    Output("metrics", "children"),
    Output("connection-status", "children"),
    Output("log", "children"),
    Input("tick", "n_intervals"),
    State("target-az", "value"),
    State("target-el", "value"),
    State("target-range", "value"),
    State("screen-w", "value"),
    State("screen-h", "value"),
    State("fov-x", "value"),
    State("fov-y", "value"),
)
def refresh(
    _n: int,
    target_az: float,
    target_el: float,
    target_range: float,
    screen_w: int,
    screen_h: int,
    fov_x: float,
    fov_y: float,
) -> tuple[list[html.Div], str, str]:
    telemetry = client.snapshot()
    range_m = float(target_range or DEFAULT_TARGET_RANGE_M)
    az = float(target_az or 0)
    el = float(target_el or 0)
    x, y, z = target_xyz(az, el, range_m)

    sw = max(120, int(screen_w or DEFAULT_SCREEN_W))
    sh = max(120, int(screen_h or DEFAULT_SCREEN_H))
    fx = max(5.0, float(fov_x or DEFAULT_FOV_X_DEG))
    fy = max(5.0, float(fov_y or DEFAULT_FOV_Y_DEG))
    screen_x, screen_y, yaw_rel, pitch_rel = screen_projection(az, el, telemetry.yaw, telemetry.pitch, sw, sh, fx, fy)
    outside = screen_x < 0 or screen_x > sw or screen_y < 0 or screen_y > sh

    age = time.time() - telemetry.last_seen_s if telemetry.last_seen_s else 999.0
    status = "CONNECTED" if telemetry.connected else "DISCONNECTED"
    if telemetry.connected:
        status += f" telemetry_age={age:.1f}s"

    def metric(name: str, value: str) -> html.Div:
        return html.Div([html.B(name), html.Span(value)], className="metric")

    metrics = [
        metric("target azimuth", f"{az:+.1f} deg"),
        metric("target elevation", f"{el:+.1f} deg"),
        metric("assumed range", f"{range_m:.1f} m"),
        metric("target XYZ", f"{x:.1f}, {y:.1f}, {z:.1f} m"),
        metric("device yaw/pitch/roll", f"{telemetry.yaw:+.1f}, {telemetry.pitch:+.1f}, {telemetry.roll:+.1f} deg"),
        metric("Arduino target", f"yaw={telemetry.target_yaw:+.1f}, pitch={telemetry.target_pitch:+.1f}"),
        metric("local yawRel/pitchRel", f"{yaw_rel:+.1f}, {pitch_rel:+.1f} deg"),
        metric("Arduino yawRel/pitchRel", f"{telemetry.yaw_rel:+.1f}, {telemetry.pitch_rel:+.1f} deg"),
        metric("screen marker", f"{screen_x:.1f}, {screen_y:.1f} px"),
        metric("outside screen", "yes" if outside else "no"),
        metric("on target", "yes" if telemetry.on_target else "no"),
        metric("last TEL", telemetry.last_line[:120]),
    ]
    return metrics, status, "\n".join(client.logs())


def main() -> None:
    app.run(host="0.0.0.0", port=8050, debug=False)


if __name__ == "__main__":
    main()
