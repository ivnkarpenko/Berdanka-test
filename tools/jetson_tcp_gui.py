import socket
import threading
import subprocess
import queue
import tkinter as tk
import os
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText
from tkinter import ttk
import time
import base64

os.environ.setdefault("YOLO_AUTOINSTALL", "false")

try:
    import cv2
except Exception:
    cv2 = None

try:
    import numpy as np
except Exception:
    np = None

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None

PORT_DEFAULT = 3333
WIFI_SSID = "cisco"
WIFI_PASS = "cisco1234"
HEARTBEAT_INTERVAL_S = 1.0
DEFAULT_YOLO_MODEL = "yolo11n.pt"
QUADRO_YOLO_MODEL = "quadron_1280.onnx"
QUADRO_YOLO_IMGSZ = 1280
MODEL_PRESETS = {
    "YOLO11n (.pt)": DEFAULT_YOLO_MODEL,
    "Quadron 1280 ONNX (.onnx)": QUADRO_YOLO_MODEL,
}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Jetson <-> Arduino UNO R4 WiFi")
        self.root.geometry("920x640")
        self.root.minsize(720, 500)

        self.sock = None
        self.rx_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()

        # vision state
        self.cap = None
        self.camera_index = tk.IntVar(value=0)
        self.camera_running = False
        self.yolo_enabled = tk.BooleanVar(value=False)
        self.send_enabled = tk.BooleanVar(value=False)
        self.show_center_cross = tk.BooleanVar(value=True)
        self.invert_x = tk.BooleanVar(value=False)
        self.invert_y = tk.BooleanVar(value=False)
        self.rate_hz = tk.IntVar(value=5)
        self.hfov = tk.DoubleVar(value=90.0)
        self.vfov = tk.DoubleVar(value=30.0)
        self.pitch_trim_deg = tk.DoubleVar(value=0.0)
        self.yaw_trim_deg = tk.DoubleVar(value=0.0)
        self.display_fov_x_deg = tk.DoubleVar(value=60.0)
        self.display_fov_y_deg = tk.DoubleVar(value=80.0)
        self.tcp_poll_ms = tk.IntVar(value=250)
        self.box_size_px = tk.IntVar(value=50)
        self.box_refresh_ms = tk.IntVar(value=33)
        self.box_render_enabled = tk.BooleanVar(value=True)
        self.box_delta_render_enabled = tk.BooleanVar(value=True)
        self.manual_target_enabled = tk.BooleanVar(value=True)
        self.manual_target_auto_send = tk.BooleanVar(value=False)
        self.manual_target_yaw_deg = tk.StringVar(value="0.00")
        self.manual_target_pitch_deg = tk.StringVar(value="0.00")
        self.manual_target_square_px = tk.StringVar(value="18")
        self.manual_target_msg = tk.StringVar(value="JETSON")
        self.yolo_model = None
        self.yolo_backend = None
        self.yolo_model_path = tk.StringVar(value=DEFAULT_YOLO_MODEL)
        self.yolo_model_preset = tk.StringVar(value="YOLO11n (.pt)")
        self.yolo_conf = tk.DoubleVar(value=0.10)
        self.yolo_imgsz = tk.IntVar(value=640)
        self.yolo_min_box_px = tk.IntVar(value=20)
        self.yolo_predict_imgsz = 640
        self.yolo_status = "YOLO: idle"
        self.last_yolo_log_ts = 0.0
        self.last_frame = None
        self.last_det = None
        self.last_det_center = None
        self.calibrated_center_norm = (0.5, 0.5)
        self.last_det_ts = 0.0
        self.last_send_ts = 0.0
        self.last_heartbeat_ts = 0.0
        self.single_request = False
        self.hold_until = 0.0
        self.hold_frame = None
        self.last_manual_send_ts = 0.0
        self.camera_view = None

        self.build_ui()
        self.root.after(50, self.process_queue)
        self.root.after(30, self.update_camera)
        self.log("[APP] Ready. 1) Connect Wi-Fi 2) Connect TCP 3) Send.")

    def build_ui(self):
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        vcmd = (self.root.register(self.only_digits), "%P")

        paned = tk.PanedWindow(self.root, orient="horizontal", sashwidth=7, sashrelief="raised")
        paned.grid(row=0, column=0, sticky="nsew")

        left = tk.Frame(paned, width=330)
        right = tk.Frame(paned)
        paned.add(left, minsize=285, width=335)
        paned.add(right, minsize=360)

        left.grid_rowconfigure(0, weight=1)
        left.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        right.grid_columnconfigure(0, weight=1)

        notebook = ttk.Notebook(left)
        notebook.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        tab_link = tk.Frame(notebook, padx=8, pady=8)
        tab_target = tk.Frame(notebook, padx=8, pady=8)
        tab_vision = tk.Frame(notebook, padx=8, pady=8)
        tab_cfg = tk.Frame(notebook, padx=8, pady=8)
        tab_log = tk.Frame(notebook, padx=6, pady=6)
        for tab, title in (
            (tab_link, "Link"),
            (tab_target, "Target"),
            (tab_vision, "Vision"),
            (tab_cfg, "Config"),
            (tab_log, "Log"),
        ):
            tab.grid_columnconfigure(1, weight=1)
            notebook.add(tab, text=title)

        # Link tab
        tk.Label(tab_link, text="SSID").grid(row=0, column=0, sticky="e", pady=3)
        self.ed_ssid = tk.Entry(tab_link, width=20)
        self.ed_ssid.insert(0, WIFI_SSID)
        self.ed_ssid.grid(row=0, column=1, sticky="ew", padx=6, pady=3)

        tk.Label(tab_link, text="Password").grid(row=1, column=0, sticky="e", pady=3)
        self.ed_pass = tk.Entry(tab_link, width=20, show="*")
        self.ed_pass.insert(0, WIFI_PASS)
        self.ed_pass.grid(row=1, column=1, sticky="ew", padx=6, pady=3)

        self.bt_wifi = tk.Button(tab_link, text="Connect Wi-Fi", command=self.connect_wifi_linux)
        self.bt_wifi.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 12))

        tk.Label(tab_link, text="Arduino IP").grid(row=3, column=0, sticky="e", pady=3)
        self.ed_ip = tk.Entry(tab_link, width=18)
        self.ed_ip.insert(0, "192.168.4.1")
        self.ed_ip.grid(row=3, column=1, sticky="ew", padx=6, pady=3)

        tk.Label(tab_link, text="Port").grid(row=4, column=0, sticky="e", pady=3)
        self.ed_port = tk.Entry(tab_link, width=8)
        self.ed_port.insert(0, str(PORT_DEFAULT))
        self.ed_port.grid(row=4, column=1, sticky="w", padx=6, pady=3)

        link_buttons = tk.Frame(tab_link)
        link_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=6)
        link_buttons.grid_columnconfigure((0, 1), weight=1)
        self.bt_connect = tk.Button(link_buttons, text="Connect", command=self.connect_arduino)
        self.bt_connect.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.bt_disconnect = tk.Button(link_buttons, text="Disconnect", command=self.disconnect_arduino, state="disabled")
        self.bt_disconnect.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        # Target tab
        tk.Checkbutton(tab_target, text="Show camera target square", variable=self.manual_target_enabled).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )

        tk.Label(tab_target, text="Tag").grid(row=1, column=0, sticky="e", pady=3)
        self.ed_msg = tk.Entry(tab_target, textvariable=self.manual_target_msg, width=18)
        self.ed_msg.grid(row=1, column=1, sticky="ew", padx=6, pady=3)

        tk.Label(tab_target, text="Elev X deg").grid(row=2, column=0, sticky="e", pady=3)
        self.ed_x = tk.Entry(tab_target, textvariable=self.manual_target_pitch_deg, width=10,
                             validate="key", validatecommand=vcmd)
        self.ed_x.grid(row=2, column=1, sticky="w", padx=6, pady=3)

        tk.Label(tab_target, text="Az Y deg").grid(row=3, column=0, sticky="e", pady=3)
        self.ed_y = tk.Entry(tab_target, textvariable=self.manual_target_yaw_deg, width=10,
                             validate="key", validatecommand=vcmd)
        self.ed_y.grid(row=3, column=1, sticky="w", padx=6, pady=3)

        tk.Label(tab_target, text="Camera square px").grid(row=4, column=0, sticky="e", pady=3)
        self.ed_manual_square = tk.Entry(tab_target, textvariable=self.manual_target_square_px, width=8)
        self.ed_manual_square.grid(row=4, column=1, sticky="w", padx=6, pady=3)

        target_buttons = tk.Frame(tab_target)
        target_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=8)
        target_buttons.grid_columnconfigure((0, 1), weight=1)
        self.bt_send = tk.Button(target_buttons, text="Send Target", command=self.send_packet)
        self.bt_send.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.bt_center = tk.Button(target_buttons, text="Center 0/0", command=self.send_center_command)
        self.bt_center.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        tk.Checkbutton(tab_target, text="Auto-send target", variable=self.manual_target_auto_send).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=3
        )
        tk.Label(
            tab_target,
            text="Click inside the camera image to place the target from camera FOV.",
            wraplength=280,
            justify="left",
            fg="#555",
        ).grid(row=7, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        # Vision tab
        tk.Checkbutton(tab_vision, text="YOLO detection", variable=self.yolo_enabled).grid(row=0, column=0, columnspan=2, sticky="w")
        tk.Checkbutton(tab_vision, text="Send YOLO center", variable=self.send_enabled).grid(row=1, column=0, columnspan=2, sticky="w")
        tk.Checkbutton(tab_vision, text="Show center cross", variable=self.show_center_cross).grid(row=2, column=0, columnspan=2, sticky="w")

        tk.Label(tab_vision, text="Model preset").grid(row=3, column=0, sticky="e", pady=(10, 3))
        self.model_menu = tk.OptionMenu(
            tab_vision,
            self.yolo_model_preset,
            *MODEL_PRESETS.keys(),
            command=self.on_model_preset_changed,
        )
        self.model_menu.grid(row=3, column=1, sticky="ew", padx=6, pady=(10, 3))

        tk.Label(tab_vision, text="Model path").grid(row=4, column=0, sticky="e", pady=3)
        self.ed_model = tk.Entry(tab_vision, width=22, textvariable=self.yolo_model_path)
        self.ed_model.grid(row=4, column=1, sticky="ew", padx=6, pady=3)

        model_buttons = tk.Frame(tab_vision)
        model_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=6)
        model_buttons.grid_columnconfigure((0, 1), weight=1)
        self.bt_reload = tk.Button(model_buttons, text="Load Model", command=self.load_model)
        self.bt_reload.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.bt_single = tk.Button(model_buttons, text="Single Detect", command=self.single_detect)
        self.bt_single.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        for row, label, var, width in (
            (6, "Conf", self.yolo_conf, 6),
            (7, "ImgSz", self.yolo_imgsz, 6),
            (8, "Min box px", self.yolo_min_box_px, 6),
            (9, "HFOV deg", self.hfov, 6),
            (10, "VFOV deg", self.vfov, 6),
            (11, "Pitch trim", self.pitch_trim_deg, 6),
            (12, "Yaw trim", self.yaw_trim_deg, 6),
        ):
            tk.Label(tab_vision, text=label).grid(row=row, column=0, sticky="e", pady=3)
            tk.Entry(tab_vision, width=width, textvariable=var).grid(row=row, column=1, sticky="w", padx=6, pady=3)

        tk.Checkbutton(tab_vision, text="Invert X", variable=self.invert_x).grid(row=13, column=0, sticky="w", pady=(8, 0))
        tk.Checkbutton(tab_vision, text="Invert Y", variable=self.invert_y).grid(row=13, column=1, sticky="w", pady=(8, 0))

        # Config tab
        for row, label, var in (
            (0, "Screen FOV X", self.display_fov_x_deg),
            (1, "Screen FOV Y", self.display_fov_y_deg),
            (3, "TCP poll ms", self.tcp_poll_ms),
            (5, "Arduino box px", self.box_size_px),
            (6, "Box refresh ms", self.box_refresh_ms),
        ):
            tk.Label(tab_cfg, text=label).grid(row=row, column=0, sticky="e", pady=3)
            tk.Entry(tab_cfg, width=8, textvariable=var).grid(row=row, column=1, sticky="w", padx=6, pady=3)

        self.bt_apply_display = tk.Button(tab_cfg, text="Apply Display FOV", command=self.send_display_config)
        self.bt_apply_display.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 12))

        self.bt_apply_net = tk.Button(tab_cfg, text="Apply Net", command=self.send_net_config)
        self.bt_apply_net.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(4, 12))

        tk.Checkbutton(tab_cfg, text="Draw Arduino box", variable=self.box_render_enabled).grid(row=7, column=0, columnspan=2, sticky="w")
        tk.Checkbutton(tab_cfg, text="Delta fill", variable=self.box_delta_render_enabled).grid(row=8, column=0, columnspan=2, sticky="w")
        self.bt_apply_box = tk.Button(tab_cfg, text="Apply Box", command=self.send_box_config)
        self.bt_apply_box.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        # Log tab
        tab_log.grid_rowconfigure(0, weight=1)
        tab_log.grid_columnconfigure(0, weight=1)
        self.log_view = ScrolledText(tab_log, height=12, width=34, state="normal")
        self.log_view.bind("<Key>", lambda e: "break")
        self.log_view.grid(row=0, column=0, sticky="nsew")

        # Camera panel
        cam_bar = tk.Frame(right)
        cam_bar.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 2))
        cam_bar.grid_columnconfigure(6, weight=1)

        tk.Label(cam_bar, text="Camera").grid(row=0, column=0, sticky="w")
        self.ed_cam = tk.Entry(cam_bar, width=5)
        self.ed_cam.insert(0, "0")
        self.ed_cam.grid(row=0, column=1, padx=4)
        self.bt_cam_start = tk.Button(cam_bar, text="Start", command=self.start_camera)
        self.bt_cam_start.grid(row=0, column=2, padx=3)
        self.bt_cam_stop = tk.Button(cam_bar, text="Stop", command=self.stop_camera, state="disabled")
        self.bt_cam_stop.grid(row=0, column=3, padx=3)
        tk.Label(cam_bar, text="Rate").grid(row=0, column=4, padx=(12, 2))
        self.sc_rate = tk.Scale(cam_bar, from_=1, to=30, orient="horizontal", variable=self.rate_hz, length=95)
        self.sc_rate.grid(row=0, column=5, sticky="w")
        self.lb_res = tk.Label(cam_bar, text="Res: n/a")
        self.lb_res.grid(row=0, column=6, padx=10, sticky="w")
        self.lb_target_status = tk.Label(cam_bar, text="Target X=0.0 Y=0.0")
        self.lb_target_status.grid(row=1, column=0, columnspan=7, padx=0, pady=(2, 0), sticky="w")

        self.canvas = tk.Canvas(right, bg="black", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew", padx=6, pady=(2, 6))
        self.canvas.bind("<Button-1>", self.on_canvas_click)

    def only_digits(self, new_value: str) -> bool:
        if new_value == "":
            return True
        try:
            float(new_value)
            return True
        except ValueError:
            return new_value in ("-", "+", ".", "-.", "+.")

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def read_number_var(var, default=0.0):
        try:
            return float(str(var.get()).strip())
        except Exception:
            return default

    def read_manual_target(self, warn_invalid=False):
        msg = self.manual_target_msg.get().strip() or "JETSON"
        try:
            pitch = float(str(self.manual_target_pitch_deg.get()).strip() or "0")
            yaw = float(str(self.manual_target_yaw_deg.get()).strip() or "0")
            square = int(float(str(self.manual_target_square_px.get()).strip() or "18"))
        except Exception:
            if warn_invalid:
                messagebox.showwarning("Target", "Target X/Y and square size must be numbers.")
            return None

        square = int(self.clamp(square, 4, 240))
        return msg, pitch, yaw, square

    def send_manual_target(self, warn_title="Target", log_tx=True, warn_invalid=True):
        target = self.read_manual_target(warn_invalid=warn_invalid)
        if target is None:
            return False

        msg, pitch, yaw, _ = target
        line = f"MSG:{msg};X:{pitch:.2f};Y:{yaw:.2f}\n"
        return self.send_line(line, warn_title=warn_title, log_tx=log_tx)

    def target_to_frame_point(self, frame_w, frame_h):
        target = self.read_manual_target(warn_invalid=False)
        if target is None:
            return None

        _, pitch, yaw, square = target
        hfov = max(0.001, self.read_number_var(self.hfov, 90.0))
        vfov = max(0.001, self.read_number_var(self.vfov, 30.0))
        ref_x = self.calibrated_center_norm[0] * frame_w
        ref_y = self.calibrated_center_norm[1] * frame_h

        raw_yaw = yaw - self.read_number_var(self.yaw_trim_deg, 0.0)
        raw_pitch = pitch - self.read_number_var(self.pitch_trim_deg, 0.0)
        if self.invert_x.get():
            raw_yaw = -raw_yaw
        if self.invert_y.get():
            raw_pitch = -raw_pitch

        frame_x = ref_x + (raw_yaw / hfov) * frame_w
        frame_y = ref_y - (raw_pitch / vfov) * frame_h
        return frame_x, frame_y, pitch, yaw, square

    def frame_point_to_target(self, frame_x, frame_y, frame_w, frame_h):
        hfov = max(0.001, self.read_number_var(self.hfov, 90.0))
        vfov = max(0.001, self.read_number_var(self.vfov, 30.0))
        ref_x = self.calibrated_center_norm[0] * frame_w
        ref_y = self.calibrated_center_norm[1] * frame_h

        yaw = ((frame_x - ref_x) / max(1, frame_w)) * hfov
        pitch = ((ref_y - frame_y) / max(1, frame_h)) * vfov
        if self.invert_x.get():
            yaw = -yaw
        if self.invert_y.get():
            pitch = -pitch

        yaw += self.read_number_var(self.yaw_trim_deg, 0.0)
        pitch += self.read_number_var(self.pitch_trim_deg, 0.0)
        return pitch, yaw

    def draw_manual_target_on_canvas(self, x0, y0, view_w, view_h, frame_w, frame_h):
        target = self.target_to_frame_point(frame_w, frame_h)
        if target is None:
            self.lb_target_status.configure(text="Target invalid")
            return

        frame_x, frame_y, pitch, yaw, square = target
        self.lb_target_status.configure(text=f"Target X={pitch:.2f} Y={yaw:.2f}")
        if not self.manual_target_enabled.get():
            return

        canvas_x = x0 + int(round((frame_x / max(1, frame_w)) * view_w))
        canvas_y = y0 + int(round((frame_y / max(1, frame_h)) * view_h))
        half = max(2, square // 2)

        if (canvas_x + half < x0 or canvas_x - half > x0 + view_w or
                canvas_y + half < y0 or canvas_y - half > y0 + view_h):
            self.lb_target_status.configure(text=f"Target X={pitch:.2f} Y={yaw:.2f} off image")
            return

        left = self.clamp(canvas_x - half, x0, x0 + view_w)
        top = self.clamp(canvas_y - half, y0, y0 + view_h)
        right = self.clamp(canvas_x + half, x0, x0 + view_w)
        bottom = self.clamp(canvas_y + half, y0, y0 + view_h)

        self.canvas.create_rectangle(left, top, right, bottom, outline="#ffd32a", width=2)
        self.canvas.create_line(canvas_x - 5, canvas_y, canvas_x + 5, canvas_y,
                                fill="#ffe680", width=2)
        self.canvas.create_line(canvas_x, canvas_y - 5, canvas_x, canvas_y + 5,
                                fill="#ffe680", width=2)

    def on_canvas_click(self, event):
        if not self.camera_view:
            return

        view = self.camera_view
        x0 = view["x0"]
        y0 = view["y0"]
        view_w = view["w"]
        view_h = view["h"]
        if event.x < x0 or event.x > x0 + view_w or event.y < y0 or event.y > y0 + view_h:
            return

        frame_w = view["frame_w"]
        frame_h = view["frame_h"]
        frame_x = ((event.x - x0) / max(1, view_w)) * frame_w
        frame_y = ((event.y - y0) / max(1, view_h)) * frame_h
        pitch, yaw = self.frame_point_to_target(frame_x, frame_y, frame_w, frame_h)

        self.manual_target_pitch_deg.set(f"{pitch:.2f}")
        self.manual_target_yaw_deg.set(f"{yaw:.2f}")
        self.manual_target_enabled.set(True)
        self.log(f"[TARGET] Camera click -> X={pitch:.2f} Y={yaw:.2f}")

        if self.manual_target_auto_send.get() and self.sock:
            if self.send_manual_target(log_tx=True, warn_invalid=False):
                self.last_manual_send_ts = time.time()

    def log(self, s: str):
        self.log_view.insert("end", s + "\n")
        self.log_view.see("end")

    def connect_wifi_linux(self):
        ssid = self.ed_ssid.get().strip()
        pw = self.ed_pass.get()

        if not ssid:
            messagebox.showwarning("Wi-Fi", "SSID is empty.")
            return

        cmd = ["nmcli", "dev", "wifi", "connect", ssid]
        if pw:
            cmd += ["password", pw]

        self.log(f"[WIFI] {' '.join(cmd)}")
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
            if p.returncode == 0:
                self.log("[WIFI] Connected OK.")
                if p.stdout.strip():
                    self.log(p.stdout.strip())
            else:
                self.log("[WIFI] ERROR")
                if p.stdout.strip():
                    self.log(p.stdout.strip())
                if p.stderr.strip():
                    self.log(p.stderr.strip())
                messagebox.showwarning("Wi-Fi", "nmcli failed. Try connecting via system UI.")
        except FileNotFoundError:
            messagebox.showwarning("Wi-Fi", "nmcli not found. Install network-manager.")
        except subprocess.TimeoutExpired:
            messagebox.showwarning("Wi-Fi", "nmcli timeout.")

    def connect_arduino(self):
        ip = self.ed_ip.get().strip()
        port_s = self.ed_port.get().strip()

        if not ip or not port_s.isdigit():
            messagebox.showwarning("TCP", "IP is empty or Port is invalid.")
            return

        port = int(port_s)
        self.disconnect_arduino()

        self.log(f"[NET] Connecting to {ip}:{port} ...")
        self.log(f"[NET] Hostname: {socket.gethostname()}")
        try:
            host_info = socket.gethostbyname_ex(socket.gethostname())
            self.log(f"[NET] Local IPs: {host_info[2]}")
            if not any(addr.startswith("192.168.4.") for addr in host_info[2]):
                self.log("[NET] WARN: no 192.168.4.x address. Check AP connection.")
        except Exception as e:
            self.log(f"[NET] Local IPs error: {e}")

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(5.0)
            s.connect((ip, port))
            s.settimeout(None)
            self.sock = s
        except Exception as e:
            self.sock = None
            self.log(f"[NET] Connect error: {repr(e)}")
            messagebox.showwarning("TCP", f"Connect failed: {e}")
            return

        self.stop_event.clear()
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.bt_connect.configure(state="disabled")
        self.bt_disconnect.configure(state="normal")
        self.last_heartbeat_ts = 0.0
        self.log("[NET] Connected.")

    def disconnect_arduino(self):
        self.stop_event.set()

        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

        self.bt_connect.configure(state="normal")
        self.bt_disconnect.configure(state="disabled")
        self.last_heartbeat_ts = 0.0

    def send_packet(self):
        if self.send_manual_target(warn_title="Send"):
            self.last_manual_send_ts = time.time()

    def send_center_command(self):
        if self.last_frame is not None and self.last_det_center is not None:
            fh, fw = self.last_frame.shape[:2]
            cx, cy = self.last_det_center
            self.calibrated_center_norm = (
                cx / max(1, fw),
                cy / max(1, fh),
            )
            self.log(f"[CAL] Vision center set from detection: {cx},{cy}")
        else:
            self.calibrated_center_norm = (0.5, 0.5)
            self.log("[CAL] Vision center set to geometric frame center.")

        self.manual_target_pitch_deg.set("0.00")
        self.manual_target_yaw_deg.set("0.00")
        if self.send_manual_target(warn_title="Centering"):
            self.last_manual_send_ts = time.time()

    def send_display_config(self):
        try:
            fov_x = float(self.display_fov_x_deg.get())
            fov_y = float(self.display_fov_y_deg.get())
        except Exception:
            messagebox.showwarning("Display", "Screen FOV values must be numbers.")
            return

        if fov_x < 5.0 or fov_x > 180.0 or fov_y < 5.0 or fov_y > 180.0:
            messagebox.showwarning("Display", "Screen FOV must be between 5 and 180 degrees.")
            return

        self.send_line(f"CFG:FOV_X:{fov_x:.2f}\n", warn_title="Display")
        self.send_line(f"CFG:FOV_Y:{fov_y:.2f}\n", warn_title="Display")

    def send_net_config(self):
        try:
            tcp_poll_ms = int(self.tcp_poll_ms.get())
        except Exception:
            messagebox.showwarning("Network", "TCP Poll ms must be an integer.")
            return

        if tcp_poll_ms < 20 or tcp_poll_ms > 2000:
            messagebox.showwarning("Network", "TCP Poll ms must be between 20 and 2000.")
            return

        self.send_line(f"CFG:TCP_POLL_MS:{tcp_poll_ms}\n", warn_title="Network")

    def send_box_config(self):
        try:
            box_size_px = int(self.box_size_px.get())
            box_refresh_ms = int(self.box_refresh_ms.get())
        except Exception:
            messagebox.showwarning("Box", "Box settings must be integers.")
            return

        if box_size_px < 8 or box_size_px > 200:
            messagebox.showwarning("Box", "Box Size must be between 8 and 200.")
            return

        if box_refresh_ms < 10 or box_refresh_ms > 1000:
            messagebox.showwarning("Box", "Box ms must be between 10 and 1000.")
            return

        ok = self.send_line(f"CFG:BOX_SIZE:{box_size_px}\n", warn_title="Box")
        ok = self.send_line(f"CFG:BOX_REFRESH_MS:{box_refresh_ms}\n", warn_title="Box") and ok
        render_value = 1 if self.box_render_enabled.get() else 0
        ok = self.send_line(f"CFG:BOX_RENDER:{render_value}\n", warn_title="Box") and ok
        delta_value = 1 if self.box_delta_render_enabled.get() else 0
        self.send_line(f"CFG:BOX_DELTA_RENDER:{delta_value}\n", warn_title="Box")

    def send_line(self, line: str, warn_title: str = "Send", log_tx: bool = True):
        if not self.sock:
            messagebox.showwarning(warn_title, "Not connected to Arduino.")
            return False

        try:
            self.sock.sendall(line.encode("utf-8"))
            if log_tx:
                self.log(f"[TX] {line.strip()}")
            return True
        except Exception as e:
            self.log(f"[NET] Send error: {e}")
            self.disconnect_arduino()
            return False

    def rx_loop(self):
        buf = b""
        try:
            while not self.stop_event.is_set() and self.sock:
                data = self.sock.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.replace(b"\r", b"").decode("utf-8", errors="replace").strip()
                    if line:
                        self.q.put(f"[RX] {line}")
        except Exception as e:
            self.q.put(f"[NET] RX error: {e}")
        finally:
            self.q.put("[NET] Disconnected.")
            self.q.put("__DISCONNECT__")

    def process_queue(self):
        try:
            while True:
                item = self.q.get_nowait()
                if item == "__DISCONNECT__":
                    self.disconnect_arduino()
                    continue
                self.log(item)
        except queue.Empty:
            pass

        now = time.time()
        if self.sock and (now - self.last_heartbeat_ts) >= HEARTBEAT_INTERVAL_S:
            if self.send_line("PING\n", warn_title="Heartbeat", log_tx=False):
                self.last_heartbeat_ts = now

        self.root.after(50, self.process_queue)

    # ===== Vision =====
    def on_model_preset_changed(self, preset):
        preset = str(preset).strip()
        model_path = MODEL_PRESETS.get(preset, preset)
        if model_path:
            self.yolo_model_path.set(model_path)
            if model_path.lower().endswith(".onnx"):
                self.yolo_imgsz.set(QUADRO_YOLO_IMGSZ)
            else:
                self.yolo_imgsz.set(640)
        self.update_yolo_model_config()

    def update_yolo_model_config(self):
        model_path = self.yolo_model_path.get().strip().lower()
        if model_path.endswith(".onnx"):
            self.yolo_predict_imgsz = QUADRO_YOLO_IMGSZ
            self.yolo_imgsz.set(QUADRO_YOLO_IMGSZ)
            return

        try:
            imgsz = int(self.yolo_imgsz.get())
        except Exception:
            imgsz = 640
        self.yolo_predict_imgsz = max(320, min(1280, imgsz))

    def get_yolo_predict_classes(self):
        model_path = self.yolo_model_path.get().strip().lower()
        if model_path.endswith(".onnx"):
            return None
        return [67]

    def get_yolo_conf(self):
        try:
            conf = float(self.yolo_conf.get())
        except Exception:
            conf = 0.10
        return max(0.01, min(0.99, conf))

    def get_yolo_min_box_px(self):
        try:
            min_box_px = int(self.yolo_min_box_px.get())
        except Exception:
            min_box_px = 20
        return max(1, min(500, min_box_px))

    def load_model(self):
        model_path_raw = self.yolo_model_path.get().strip()
        if not model_path_raw:
            messagebox.showwarning("YOLO", "Model path is empty.")
            return

        # Load only local weights on Jetson to avoid online download failures.
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = []
        if os.path.isabs(model_path_raw):
            candidates.append(model_path_raw)
        else:
            candidates.append(model_path_raw)
            candidates.append(os.path.join(script_dir, model_path_raw))
            candidates.append(os.path.join(script_dir, "models", model_path_raw))

        model_path = None
        for c in candidates:
            p = os.path.abspath(os.path.expanduser(os.path.expandvars(c)))
            if os.path.isfile(p):
                model_path = p
                break

        if model_path is None:
            msg = (
                "Local model file not found.\n\n"
                "Place weights next to this script, for example:\n"
                "  tools/yolo11n.pt\n"
                "  tools/quadron_1280.onnx\n\n"
                "Then select a preset or set Model path to that file."
            )
            self.yolo_model = None
            self.log(f"[YOLO] Model not found locally: {model_path_raw}")
            messagebox.showwarning("YOLO", msg)
            return

        try:
            self.update_yolo_model_config()
            if model_path.lower().endswith(".onnx"):
                if cv2 is None or np is None:
                    messagebox.showwarning("YOLO", "OpenCV and numpy are required for ONNX DNN inference.")
                    return
                net = cv2.dnn.readNet(model_path)
                try:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
                    self.log("[YOLO] OpenCV DNN backend: CUDA FP16")
                except Exception as e:
                    self.log(f"[YOLO] CUDA DNN backend unavailable, using default backend: {e}")
                self.yolo_model = net
                self.yolo_backend = "opencv_onnx"
                self.log(f"[YOLO] Loaded ONNX via OpenCV DNN: {model_path} imgsz={self.yolo_predict_imgsz}")
            else:
                if YOLO is None:
                    messagebox.showwarning("YOLO", "ultralytics not installed. Install: pip install ultralytics")
                    return
                self.yolo_model = YOLO(model_path, task="detect")
                self.yolo_backend = "ultralytics"
                self.log(f"[YOLO] Loaded model: {model_path} imgsz={self.yolo_predict_imgsz}")
        except Exception as e:
            self.yolo_model = None
            self.yolo_backend = None
            self.log(f"[YOLO] Failed to load local model: {model_path}; error: {e}")
            messagebox.showwarning("YOLO", f"Failed to load local model:\n{model_path}\n\n{e}")

    def start_camera(self):
        if cv2 is None:
            messagebox.showwarning("Camera", "opencv-python not installed. Install: pip install opencv-python")
            return
        try:
            idx = int(self.ed_cam.get().strip())
        except Exception:
            messagebox.showwarning("Camera", "Camera index must be a number.")
            return

        self.stop_camera()
        self.cap = cv2.VideoCapture(idx)
        if not self.cap.isOpened():
            self.cap = None
            messagebox.showwarning("Camera", "Cannot open camera.")
            return
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.camera_running = True
        self.bt_cam_start.configure(state="disabled")
        self.bt_cam_stop.configure(state="normal")
        self.log(f"[CAM] Started camera {idx} at {actual_w}x{actual_h}")

    def stop_camera(self):
        self.camera_running = False
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.camera_view = None
        self.bt_cam_start.configure(state="normal")
        self.bt_cam_stop.configure(state="disabled")

    def single_detect(self):
        self.single_request = True

    def draw_yolo_overlay(self, frame, box, conf, box_count, kept_count):
        x1, y1, x2, y2 = box
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        self.last_det = (x1, y1, x2, y2, conf)
        self.last_det_center = (cx, cy)
        self.yolo_status = f"YOLO kept={kept_count}/{box_count} conf={conf:.2f} center={cx},{cy}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
        cv2.rectangle(frame, (8, 8), (min(frame.shape[1] - 1, 410), 42), (0, 0, 0), -1)
        cv2.putText(
            frame,
            self.yolo_status,
            (14, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"target {conf:.2f}",
            (max(0, x1), max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            3,
        )

    def run_opencv_onnx(self, frame, single=False):
        conf_threshold = self.get_yolo_conf()
        nms_threshold = 0.5
        imgsz = QUADRO_YOLO_IMGSZ
        fh, fw = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame,
            1.0 / 122.0,
            (imgsz, imgsz),
            (120, 120, 120),
            True,
            False,
            cv2.CV_32F,
        )
        self.yolo_model.setInput(blob)
        try:
            outs = self.yolo_model.forward(self.yolo_model.getUnconnectedOutLayersNames())
        except Exception as e:
            self.log(f"[YOLO-DNN] Forward failed with current backend, retrying CPU: {e}")
            self.yolo_model.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.yolo_model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.yolo_model.setInput(blob)
            outs = self.yolo_model.forward(self.yolo_model.getUnconnectedOutLayersNames())

        boxes_1280 = []
        confidences = []
        output_shapes = []
        for preds in outs:
            arr = np.asarray(preds)
            output_shapes.append(tuple(arr.shape))
            if arr.ndim > 2:
                arr = arr.reshape(arr.shape[-2], -1)
            elif arr.ndim == 2 and arr.shape[0] == 1:
                arr = arr.reshape(arr.shape[1], -1)

            for det in arr:
                if det.shape[0] < 6:
                    continue
                obj_conf = float(det[4])
                if obj_conf < conf_threshold:
                    continue
                class_scores = det[5:]
                best_class_score = float(np.max(class_scores))
                conf = obj_conf * best_class_score
                if conf < conf_threshold:
                    continue
                cx, cy, bw, bh = map(float, det[:4])
                x = cx - 0.5 * bw
                y = cy - 0.5 * bh
                boxes_1280.append([int(round(x)), int(round(y)), int(round(bw)), int(round(bh))])
                confidences.append(conf)

        keep = cv2.dnn.NMSBoxes(boxes_1280, confidences, conf_threshold, nms_threshold)
        keep_indices = []
        if len(keep) > 0:
            keep_indices = np.asarray(keep).reshape(-1).astype(int).tolist()

        sx = fw / float(imgsz)
        sy = fh / float(imgsz)
        candidates = []
        min_box_px = self.get_yolo_min_box_px()
        for idx in keep_indices:
            x, y, bw, bh = boxes_1280[idx]
            x1 = max(0, min(fw - 1, int(round(x * sx))))
            y1 = max(0, min(fh - 1, int(round(y * sy))))
            x2 = max(0, min(fw - 1, int(round((x + bw) * sx))))
            y2 = max(0, min(fh - 1, int(round((y + bh) * sy))))
            sw = max(0, x2 - x1)
            sh = max(0, y2 - y1)
            area = sw * sh
            if sw < min_box_px or sh < min_box_px or area <= 0:
                continue
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            score = confidences[idx] * 1000.0 + (area ** 0.5) * 0.7
            if self.last_det_center is not None:
                dx = center[0] - self.last_det_center[0]
                dy = center[1] - self.last_det_center[1]
                dist = (dx * dx + dy * dy) ** 0.5
                score -= min(dist, 1000.0) * 0.6
                if dist <= 260.0:
                    score += 180.0
            candidates.append((score, (x1, y1, x2, y2), confidences[idx], area))

        if single:
            best = max(candidates, default=None, key=lambda c: c[0])
            self.log(
                f"[YOLO-DNN] shapes={output_shapes}, raw={len(boxes_1280)}, "
                f"kept={len(keep_indices)}, valid={len(candidates)}, "
                f"best={None if best is None else (best[1], round(best[2], 3), best[3])}"
            )
        elif time.time() - self.last_yolo_log_ts >= 2.0:
            best = max(candidates, default=None, key=lambda c: c[0])
            self.log(
                f"[YOLO-DNN] raw={len(boxes_1280)}, kept={len(keep_indices)}, "
                f"valid={len(candidates)}, best={None if best is None else (best[1], round(best[2], 3), best[3])}"
            )
            self.last_yolo_log_ts = time.time()

        if not candidates:
            self.last_det = None
            self.last_det_center = None
            self.yolo_status = f"YOLO kept=0/{len(boxes_1280)}"
            return frame

        candidates.sort(key=lambda c: c[0], reverse=True)
        _, selected_box, selected_conf, _ = candidates[0]
        for _, box, conf, _ in candidates[1:]:
            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 1)
        self.draw_yolo_overlay(frame, selected_box, selected_conf, len(boxes_1280), len(candidates))
        return frame

    def run_yolo(self, frame, single=False):
        if self.yolo_model is None:
            if single:
                messagebox.showwarning("YOLO", "Model not loaded.")
            return frame

        try:
            if self.yolo_backend == "opencv_onnx":
                return self.run_opencv_onnx(frame, single=single)

            if YOLO is None:
                if single:
                    messagebox.showwarning("YOLO", "ultralytics not installed.")
                return frame

            self.update_yolo_model_config()
            results = self.yolo_model.predict(
                frame,
                verbose=False,
                conf=self.get_yolo_conf(),
                classes=self.get_yolo_predict_classes(),
                imgsz=self.yolo_predict_imgsz,
            )
            if len(results) == 0:
                self.last_det = None
                self.last_det_center = None
                self.yolo_status = "YOLO: no result"
                if single:
                    self.log("[YOLO] Detect: no result objects returned.")
                return frame
            r = results[0]

            best = None
            best_xyxy = None
            best_conf = 0.0
            best_area = 0
            best_score = -1.0
            box_count = 0
            valid_count = 0
            fh, fw = frame.shape[:2]
            min_box_px = self.get_yolo_min_box_px()
            for b in r.boxes:
                box_count += 1
                conf = float(b.conf.item())
                x1f, y1f, x2f, y2f = map(float, b.xyxy[0].tolist())
                x1 = max(0, min(fw - 1, int(round(x1f))))
                y1 = max(0, min(fh - 1, int(round(y1f))))
                x2 = max(0, min(fw - 1, int(round(x2f))))
                y2 = max(0, min(fh - 1, int(round(y2f))))
                bw = max(0, x2 - x1)
                bh = max(0, y2 - y1)
                area = bw * bh
                if bw < min_box_px or bh < min_box_px:
                    continue
                valid_count += 1
                score = conf * area
                if score > best_score:
                    best_score = score
                    best_conf = conf
                    best = b
                    best_xyxy = (x1, y1, x2, y2)
                    best_area = area

            if single:
                self.log(
                    f"[YOLO] Detect: boxes={box_count}, valid={valid_count}, "
                    f"best_conf={best_conf:.3f}, xyxy={best_xyxy}, area={best_area}"
                )
            elif time.time() - self.last_yolo_log_ts >= 2.0:
                self.log(
                    f"[YOLO] Detect: boxes={box_count}, valid={valid_count}, "
                    f"best_conf={best_conf:.3f}, xyxy={best_xyxy}, area={best_area}"
                )
                self.last_yolo_log_ts = time.time()

            if best is not None and best_xyxy is not None:
                x1, y1, x2, y2 = best_xyxy
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                self.last_det = (x1, y1, x2, y2, best_conf)
                self.last_det_center = (cx, cy)
                self.yolo_status = f"YOLO valid={valid_count}/{box_count} conf={best_conf:.2f} center={cx},{cy}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
                cv2.rectangle(frame, (8, 8), (min(frame.shape[1] - 1, 390), 42), (0, 0, 0), -1)
                cv2.putText(
                    frame,
                    self.yolo_status,
                    (14, 33),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    f"target {best_conf:.2f}",
                    (max(0, x1), max(24, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (255, 255, 255),
                    3,
                )
            else:
                self.last_det = None
                self.last_det_center = None
                self.yolo_status = f"YOLO boxes=0 conf={best_conf:.2f}"
                if single:
                    self.log("[YOLO] Detect: no boxes after postprocess.")
            return frame
        except Exception as e:
            self.last_det = None
            self.last_det_center = None
            if single:
                messagebox.showwarning("YOLO", f"Detect error: {e}")
            return frame

    def update_camera(self):
        if self.camera_running and self.cap:
            ret, frame = self.cap.read()
            if ret:
                fh, fw = frame.shape[:2]
                self.lb_res.configure(text=f"Res: {fw}x{fh}")
                self.last_frame = frame
                now = time.time()
                interval = 1.0 / max(1, int(self.rate_hz.get()))

                if self.single_request:
                    frame = self.run_yolo(frame, single=True)
                    self.hold_frame = frame.copy()
                    self.hold_until = now + 5.0
                    self.single_request = False
                elif self.yolo_enabled.get() and (now - self.last_det_ts) >= interval:
                    frame = self.run_yolo(frame)
                    self.last_det_ts = now

                if self.hold_frame is not None and now < self.hold_until:
                    frame = self.hold_frame.copy()
                elif self.hold_frame is not None and now >= self.hold_until:
                    self.hold_frame = None

                if self.manual_target_auto_send.get() and self.sock:
                    if (now - self.last_manual_send_ts) >= interval:
                        if self.send_manual_target(log_tx=False, warn_invalid=False):
                            self.last_manual_send_ts = now
                elif self.send_enabled.get() and self.sock and self.last_det_center is not None:
                    if (now - self.last_send_ts) >= interval:
                        cx, cy = self.last_det_center
                        hfov = max(0.001, self.read_number_var(self.hfov, 90.0))
                        vfov = max(0.001, self.read_number_var(self.vfov, 30.0))
                        ref_x = self.calibrated_center_norm[0] * fw
                        ref_y = self.calibrated_center_norm[1] * fh
                        angle_x = ((cx - ref_x) / max(1, fw)) * hfov
                        angle_y = ((ref_y - cy) / max(1, fh)) * vfov
                        if self.invert_x.get():
                            angle_x = -angle_x
                        if self.invert_y.get():
                            angle_y = -angle_y
                        angle_x += self.read_number_var(self.yaw_trim_deg, 0.0)
                        angle_y += self.read_number_var(self.pitch_trim_deg, 0.0)
                        line = f"MSG:PHONE;X:{angle_y:.2f};Y:{angle_x:.2f}\n"
                        try:
                            self.sock.sendall(line.encode("utf-8"))
                            self.last_send_ts = now
                        except Exception as e:
                            self.log(f"[NET] Send error: {e}")
                            self.disconnect_arduino()

                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, _ = frame_rgb.shape
                canvas_w = max(1, self.canvas.winfo_width())
                canvas_h = max(1, self.canvas.winfo_height())

                scale = min(canvas_w / w, canvas_h / h)
                new_w = max(1, int(w * scale))
                new_h = max(1, int(h * scale))
                resized = cv2.resize(frame_rgb, (new_w, new_h))

                if ImageTk is not None:
                    img = ImageTk.PhotoImage(Image.fromarray(resized))
                else:
                    png_bytes = cv2.imencode(".png", resized)[1].tobytes()
                    img = tk.PhotoImage(master=self.canvas, data=base64.b64encode(png_bytes))

                self.canvas.delete("all")
                x0 = (canvas_w - new_w) // 2
                y0 = (canvas_h - new_h) // 2
                self.camera_view = {
                    "x0": x0,
                    "y0": y0,
                    "w": new_w,
                    "h": new_h,
                    "frame_w": w,
                    "frame_h": h,
                }
                self.canvas.image = img
                self.canvas.create_image(x0, y0, image=img, anchor="nw")
                if self.show_center_cross.get():
                    cross_x = x0 + int(self.calibrated_center_norm[0] * new_w)
                    cross_y = y0 + int(self.calibrated_center_norm[1] * new_h)
                    self.canvas.create_line(cross_x - 16, cross_y, cross_x + 16, cross_y, fill="#ff4040", width=2)
                    self.canvas.create_line(cross_x, cross_y - 16, cross_x, cross_y + 16, fill="#ff4040", width=2)
                    self.canvas.create_oval(cross_x - 3, cross_y - 3, cross_x + 3, cross_y + 3,
                                            outline="#ff4040", width=2)
                self.draw_manual_target_on_canvas(x0, y0, new_w, new_h, w, h)

        self.root.after(30, self.update_camera)


def main():
    root = tk.Tk()
    app = App(root)

    def on_close():
        app.disconnect_arduino()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
