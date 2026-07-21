import socket
import threading
import subprocess
import queue
import tkinter as tk
import os
import tempfile
import glob
import re
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText
from tkinter import ttk
import time
import math
import base64
from xml.sax.saxutils import escape as xml_escape

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

try:
    from tensorrt_backend import TensorRTEngine, TensorRTInferenceWorker
except Exception:
    TensorRTEngine = None
    TensorRTInferenceWorker = None

PORT_DEFAULT = 3333
WIFI_SSID = "cisco"
WIFI_PASS = "cisco1234"
HEARTBEAT_INTERVAL_S = 1.0
DEFAULT_CAMERA_FRAME_W = 640
DEFAULT_CAMERA_FRAME_H = 480
DEFAULT_CAMERA_HFOV_DEG = 43.60
DEFAULT_CAMERA_VFOV_DEG = 33.40
# Full camera frame is normalized onto the full 480x320 Arduino display:
# display_x = camera_x * 480/640, display_y = camera_y * 320/480.
DEFAULT_DISPLAY_FOV_X_DEG = DEFAULT_CAMERA_HFOV_DEG
DEFAULT_DISPLAY_FOV_Y_DEG = DEFAULT_CAMERA_VFOV_DEG
DEFAULT_YOLO_MODEL = "yolo11n.pt"
QUADRO_YOLO_MODEL = "quadron_1280.onnx"
QUADRO_TENSORRT_MODEL = "quadron_1280_fp16.engine"
QUADRO_YOLO_IMGSZ = 1280
MODEL_PRESETS = {
    "Quadron 1280 TensorRT FP16 (.engine)": QUADRO_TENSORRT_MODEL,
    "YOLO11n (.pt)": DEFAULT_YOLO_MODEL,
    "Quadron 1280 ONNX (.onnx)": QUADRO_YOLO_MODEL,
}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.is_windows = os.name == "nt"
        host_title = "Windows" if self.is_windows else "Jetson"
        self.root.title(f"{host_title} <-> Arduino UNO R4 WiFi")
        self.root.geometry("920x640")
        self.root.minsize(720, 500)

        self.sock = None
        self.rx_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()

        # vision state
        self.cap = None
        self.camera_source = tk.StringVar(value="0")
        self.camera_resolution = tk.StringVar(value="Auto (camera default)")
        self.camera_resolution_modes = {}
        self.last_camera_devices = []
        self.camera_running = False
        self.yolo_enabled = tk.BooleanVar(value=not self.is_windows)
        self.send_enabled = tk.BooleanVar(value=False)
        self.show_center_cross = tk.BooleanVar(value=True)
        self.invert_x = tk.BooleanVar(value=False)
        self.invert_y = tk.BooleanVar(value=False)
        self.rate_hz = tk.IntVar(value=30)
        self.hfov = tk.DoubleVar(value=DEFAULT_CAMERA_HFOV_DEG)
        self.vfov = tk.DoubleVar(value=DEFAULT_CAMERA_VFOV_DEG)
        self.pitch_trim_deg = tk.DoubleVar(value=0.0)
        self.yaw_trim_deg = tk.DoubleVar(value=0.0)
        self.display_fov_x_deg = tk.DoubleVar(value=DEFAULT_DISPLAY_FOV_X_DEG)
        self.display_fov_y_deg = tk.DoubleVar(value=DEFAULT_DISPLAY_FOV_Y_DEG)
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
        self.circle_target_enabled = tk.BooleanVar(value=False)
        self.circle_radius_yaw_deg = tk.StringVar(value="10.00")
        self.circle_radius_pitch_deg = tk.StringVar(value="10.00")
        self.circle_period_s = tk.StringVar(value="8.00")
        self.circle_send_hz = tk.StringVar(value="10")
        self.yolo_model = None
        self.yolo_backend = None
        initial_model = QUADRO_TENSORRT_MODEL if not self.is_windows else DEFAULT_YOLO_MODEL
        initial_preset = "Quadron 1280 TensorRT FP16 (.engine)" if not self.is_windows else "YOLO11n (.pt)"
        self.yolo_model_path = tk.StringVar(value=initial_model)
        self.yolo_model_preset = tk.StringVar(value=initial_preset)
        self.yolo_conf = tk.DoubleVar(value=0.10)
        self.yolo_imgsz = tk.IntVar(value=640)
        self.yolo_min_box_px = tk.IntVar(value=20)
        self.yolo_predict_imgsz = 640
        self.yolo_status = "YOLO: idle"
        self.last_yolo_log_ts = 0.0
        self.tensorrt_worker = None
        self.last_tensorrt_result = None
        self.contrast_enabled = tk.BooleanVar(value=False)
        self.contrast_send_enabled = tk.BooleanVar(value=False)
        self.contrast_threshold = tk.IntVar(value=35)
        self.contrast_blur_px = tk.IntVar(value=3)
        self.contrast_dilate_px = tk.IntVar(value=5)
        self.contrast_min_area_px = tk.IntVar(value=80)
        self.contrast_max_area_px = tk.IntVar(value=40000)
        self.contrast_min_box_px = tk.IntVar(value=6)
        self.contrast_lock_radius_px = tk.IntVar(value=80)
        self.contrast_max_jump_px = tk.IntVar(value=160)
        self.contrast_switch_margin = tk.DoubleVar(value=1.35)
        self.contrast_smoothing = tk.DoubleVar(value=0.25)
        self.contrast_hold_ms = tk.IntVar(value=250)
        self.contrast_status = "Contrast: idle"
        self.last_contrast_log_ts = 0.0
        self.orb_enabled = tk.BooleanVar(value=False)
        self.orb_send_enabled = tk.BooleanVar(value=False)
        self.orb_nfeatures = tk.IntVar(value=500)
        self.orb_fast_threshold = tk.IntVar(value=20)
        self.orb_edge_threshold = tk.IntVar(value=31)
        self.orb_min_response = tk.DoubleVar(value=0.0005)
        self.orb_lock_radius_px = tk.IntVar(value=80)
        self.orb_max_jump_px = tk.IntVar(value=160)
        self.orb_switch_margin = tk.DoubleVar(value=1.40)
        self.orb_smoothing = tk.DoubleVar(value=0.35)
        self.orb_hold_ms = tk.IntVar(value=300)
        self.orb_status = "ORB: idle"
        self.last_orb_log_ts = 0.0
        self.last_frame = None
        self.last_det = None
        self.last_det_center = None
        self.last_contrast_det = None
        self.last_contrast_center = None
        self.last_contrast_seen_ts = 0.0
        self.contrast_smoothed_center = None
        self.contrast_locked_score = 0.0
        self.last_orb_point = None
        self.last_orb_seen_ts = 0.0
        self.orb_smoothed_point = None
        self.orb_locked_score = 0.0
        self.orb_locked_response = 0.0
        self.calibrated_center_norm = (0.5, 0.5)
        self.last_det_ts = 0.0
        self.last_contrast_ts = 0.0
        self.last_orb_ts = 0.0
        self.last_send_ts = 0.0
        self.last_heartbeat_ts = 0.0
        self.single_request = False
        self.hold_until = 0.0
        self.hold_frame = None
        self.last_manual_send_ts = 0.0
        self.circle_start_ts = time.time()
        self.last_circle_send_ts = 0.0
        self.camera_view = None

        self.build_ui()
        self.root.after(50, self.process_queue)
        self.root.after(30, self.update_camera)
        self.root.after(250, self.poll_camera_devices)
        if not self.is_windows:
            self.root.after(500, self.load_model)
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
        tab_contrast = tk.Frame(notebook, padx=8, pady=8)
        tab_orb = tk.Frame(notebook, padx=8, pady=8)
        tab_cfg = tk.Frame(notebook, padx=8, pady=8)
        tab_log = tk.Frame(notebook, padx=6, pady=6)
        for tab, title in (
            (tab_link, "Link"),
            (tab_target, "Target"),
            (tab_vision, "Vision"),
            (tab_contrast, "Contrast"),
            (tab_orb, "ORB"),
            (tab_cfg, "Config"),
            (tab_log, "Log"),
        ):
            tab.grid_columnconfigure(1, weight=1)
            tab.grid_columnconfigure(2, weight=1)
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

        wifi_text = "Connect Wi-Fi (netsh)" if self.is_windows else "Connect Wi-Fi (nmcli)"
        self.bt_wifi = tk.Button(tab_link, text=wifi_text, command=self.connect_wifi)
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

        tk.Checkbutton(tab_target, text="Circle test around 0/0", variable=self.circle_target_enabled,
                       command=self.reset_circle_test).grid(
            row=7, column=0, columnspan=2, sticky="w", pady=(10, 3)
        )

        for row, label, var in (
            (8, "Circle yaw deg", self.circle_radius_yaw_deg),
            (9, "Circle pitch deg", self.circle_radius_pitch_deg),
            (10, "Period sec", self.circle_period_s),
            (11, "Send Hz", self.circle_send_hz),
        ):
            tk.Label(tab_target, text=label).grid(row=row, column=0, sticky="e", pady=3)
            tk.Entry(tab_target, textvariable=var, width=8, validate="key",
                     validatecommand=vcmd).grid(row=row, column=1, sticky="w", padx=6, pady=3)

        tk.Label(
            tab_target,
            text="Click inside the camera image to place the target from camera FOV.",
            wraplength=280,
            justify="left",
            fg="#555",
        ).grid(row=12, column=0, columnspan=2, sticky="ew", pady=(10, 0))

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

        # Contrast tab
        tk.Checkbutton(tab_contrast, text="Включить контрастный захват", variable=self.contrast_enabled).grid(
            row=0, column=0, columnspan=3, sticky="w"
        )
        tk.Checkbutton(tab_contrast, text="Отправлять точку на Arduino", variable=self.contrast_send_enabled).grid(
            row=1, column=0, columnspan=3, sticky="w"
        )
        self.lb_contrast_status = tk.Label(tab_contrast, text=self.contrast_status, anchor="w", justify="left", fg="#225588")
        self.lb_contrast_status.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 8))

        for row, label, var, width, help_text in (
            (3, "Порог", self.contrast_threshold, 6, "Насколько сильным должен быть перепад яркости."),
            (4, "Размытие px", self.contrast_blur_px, 6, "Снижает шум; нечетное число, 1 почти без фильтра."),
            (5, "Склейка px", self.contrast_dilate_px, 6, "Объединяет близкие контрастные пиксели в одну область."),
            (6, "Мин. площадь", self.contrast_min_area_px, 7, "Отсекает мелкие шумовые области."),
            (7, "Макс. площадь", self.contrast_max_area_px, 7, "Отсекает слишком большие фоновые области."),
            (8, "Мин. размер", self.contrast_min_box_px, 6, "Минимальная ширина и высота области в пикселях."),
            (9, "Радиус lock", self.contrast_lock_radius_px, 6, "В этом радиусе предпочитается старая область."),
            (10, "Макс. скачок", self.contrast_max_jump_px, 6, "Дальше этого новая область игнорируется."),
            (11, "Перекл. x", self.contrast_switch_margin, 6, "Новая дальняя область должна быть сильнее во столько раз."),
            (12, "Сглаживание", self.contrast_smoothing, 6, "0 - быстро, 0.95 - очень плавно, но с задержкой."),
            (13, "Удержание ms", self.contrast_hold_ms, 6, "Резервный параметр; метка держится до Reset."),
        ):
            tk.Label(tab_contrast, text=label).grid(row=row, column=0, sticky="e", pady=3)
            tk.Entry(tab_contrast, width=width, textvariable=var).grid(row=row, column=1, sticky="w", padx=6, pady=3)
            tk.Label(tab_contrast, text=help_text, wraplength=155, justify="left", fg="#555").grid(
                row=row, column=2, sticky="w", pady=3
            )

        tk.Button(tab_contrast, text="Reset contrast track", command=self.reset_contrast_track).grid(
            row=14, column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )

        # ORB tab
        tk.Checkbutton(tab_orb, text="Включить ORB захват", variable=self.orb_enabled).grid(
            row=0, column=0, columnspan=3, sticky="w"
        )
        tk.Checkbutton(tab_orb, text="Отправлять точку на Arduino", variable=self.orb_send_enabled).grid(
            row=1, column=0, columnspan=3, sticky="w"
        )
        self.lb_orb_status = tk.Label(tab_orb, text=self.orb_status, anchor="w", justify="left", fg="#225588")
        self.lb_orb_status.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 8))

        for row, label, var, width, help_text in (
            (3, "Точек ORB", self.orb_nfeatures, 6, "Сколько keypoint искать до выбора одной лучшей."),
            (4, "FAST порог", self.orb_fast_threshold, 6, "Выше - меньше точек, но они контрастнее."),
            (5, "Край px", self.orb_edge_threshold, 6, "Не брать точки слишком близко к краю кадра."),
            (6, "Мин. response", self.orb_min_response, 7, "Минимальная сила ORB-точки."),
            (7, "Радиус lock", self.orb_lock_radius_px, 6, "В этом радиусе предпочитается старая цель."),
            (8, "Макс. скачок", self.orb_max_jump_px, 6, "Дальше этого новая точка игнорируется."),
            (9, "Перекл. x", self.orb_switch_margin, 6, "Новая дальняя точка должна быть сильнее во столько раз."),
            (10, "Сглаживание", self.orb_smoothing, 6, "0 - быстро, 0.95 - плавно, но с задержкой."),
            (11, "Удержание ms", self.orb_hold_ms, 6, "Резервный параметр; метка держится до Reset."),
        ):
            tk.Label(tab_orb, text=label).grid(row=row, column=0, sticky="e", pady=3)
            tk.Entry(tab_orb, width=width, textvariable=var).grid(row=row, column=1, sticky="w", padx=6, pady=3)
            tk.Label(tab_orb, text=help_text, wraplength=155, justify="left", fg="#555").grid(
                row=row, column=2, sticky="w", pady=3
            )

        tk.Button(tab_orb, text="Reset ORB track", command=self.reset_orb_track).grid(
            row=12, column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )

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
        cam_bar.grid_columnconfigure(7, weight=1)

        tk.Label(cam_bar, text="Camera").grid(row=0, column=0, sticky="w")
        self.camera_menu = ttk.Combobox(
            cam_bar,
            width=18,
            textvariable=self.camera_source,
            state="readonly",
            values=("0",),
        )
        self.camera_menu.grid(row=0, column=1, padx=4, sticky="w")
        self.camera_menu.bind("<<ComboboxSelected>>", self.on_camera_selected)
        self.bt_cam_refresh = tk.Button(cam_bar, text="Refresh", command=lambda: self.refresh_cameras(log_changes=True))
        self.bt_cam_refresh.grid(row=0, column=2, padx=3)
        self.bt_cam_start = tk.Button(cam_bar, text="Start", command=self.start_camera)
        self.bt_cam_start.grid(row=0, column=3, padx=3)
        self.bt_cam_stop = tk.Button(cam_bar, text="Stop", command=self.stop_camera, state="disabled")
        self.bt_cam_stop.grid(row=0, column=4, padx=3)
        tk.Label(cam_bar, text="Rate").grid(row=0, column=5, padx=(12, 2))
        self.sc_rate = tk.Scale(cam_bar, from_=1, to=60, orient="horizontal", variable=self.rate_hz, length=95)
        self.sc_rate.grid(row=0, column=6, sticky="w")
        self.lb_res = tk.Label(cam_bar, text="Res: n/a")
        self.lb_res.grid(row=0, column=7, padx=10, sticky="w")

        tk.Label(cam_bar, text="Mode").grid(row=1, column=0, sticky="w")
        self.camera_resolution_menu = ttk.Combobox(
            cam_bar,
            width=31,
            textvariable=self.camera_resolution,
            state="readonly",
            values=("Auto (camera default)",),
        )
        self.camera_resolution_menu.grid(row=1, column=1, columnspan=3, padx=4, pady=(2, 0), sticky="w")
        self.lb_target_status = tk.Label(cam_bar, text="Target X=0.0 Y=0.0")
        self.lb_target_status.grid(row=1, column=4, columnspan=4, padx=6, pady=(2, 0), sticky="w")

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

    def reset_circle_test(self):
        self.circle_start_ts = time.time()
        self.last_circle_send_ts = 0.0

    def read_circle_config(self):
        try:
            yaw_radius = abs(float(str(self.circle_radius_yaw_deg.get()).strip() or "0"))
            pitch_radius = abs(float(str(self.circle_radius_pitch_deg.get()).strip() or "0"))
            period_s = float(str(self.circle_period_s.get()).strip() or "8")
            send_hz = float(str(self.circle_send_hz.get()).strip() or "10")
        except Exception:
            return None

        yaw_radius = self.clamp(yaw_radius, 0.0, 180.0)
        pitch_radius = self.clamp(pitch_radius, 0.0, 90.0)
        period_s = self.clamp(period_s, 0.2, 120.0)
        send_hz = self.clamp(send_hz, 1.0, 30.0)
        return yaw_radius, pitch_radius, period_s, send_hz

    def update_circle_target(self, now):
        if not self.circle_target_enabled.get():
            return False

        cfg = self.read_circle_config()
        if cfg is None:
            self.lb_target_status.configure(text="Circle invalid")
            return True

        yaw_radius, pitch_radius, period_s, send_hz = cfg
        theta = ((now - self.circle_start_ts) / period_s) * 2.0 * math.pi
        yaw = yaw_radius * math.cos(theta)
        pitch = pitch_radius * math.sin(theta)
        self.manual_target_yaw_deg.set(f"{yaw:.2f}")
        self.manual_target_pitch_deg.set(f"{pitch:.2f}")

        interval = 1.0 / send_hz
        if self.sock and (now - self.last_circle_send_ts) >= interval:
            if self.send_manual_target(log_tx=False, warn_invalid=False):
                self.last_circle_send_ts = now
        return True

    def target_to_frame_point(self, frame_w, frame_h):
        target = self.read_manual_target(warn_invalid=False)
        if target is None:
            return None

        _, pitch, yaw, square = target
        hfov = max(0.001, self.read_number_var(self.hfov, DEFAULT_CAMERA_HFOV_DEG))
        vfov = max(0.001, self.read_number_var(self.vfov, DEFAULT_CAMERA_VFOV_DEG))
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
        hfov = max(0.001, self.read_number_var(self.hfov, DEFAULT_CAMERA_HFOV_DEG))
        vfov = max(0.001, self.read_number_var(self.vfov, DEFAULT_CAMERA_VFOV_DEG))
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

    def connect_wifi(self):
        if self.is_windows:
            self.connect_wifi_windows()
        else:
            self.connect_wifi_linux()

    def connect_wifi_windows(self):
        ssid = self.ed_ssid.get().strip()
        pw = self.ed_pass.get()

        if not ssid:
            messagebox.showwarning("Wi-Fi", "SSID is empty.")
            return

        ssid_xml = xml_escape(ssid)
        pw_xml = xml_escape(pw)
        if pw:
            security_xml = f"""
        <security>
            <authEncryption>
                <authentication>WPA2PSK</authentication>
                <encryption>AES</encryption>
                <useOneX>false</useOneX>
            </authEncryption>
            <sharedKey>
                <keyType>passPhrase</keyType>
                <protected>false</protected>
                <keyMaterial>{pw_xml}</keyMaterial>
            </sharedKey>
        </security>"""
        else:
            security_xml = """
        <security>
            <authEncryption>
                <authentication>open</authentication>
                <encryption>none</encryption>
                <useOneX>false</useOneX>
            </authEncryption>
        </security>"""

        profile_xml = f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>{ssid_xml}</name>
    <SSIDConfig>
        <SSID>
            <name>{ssid_xml}</name>
        </SSID>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <MSM>{security_xml}
    </MSM>
</WLANProfile>"""

        profile_path = None
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False, encoding="utf-8") as f:
                profile_path = f.name
                f.write(profile_xml)

            self.log("[WIFI] netsh wlan add profile")
            add = subprocess.run(
                ["netsh", "wlan", "add", "profile", f"filename={profile_path}"],
                capture_output=True,
                text=True,
                timeout=25,
            )
            if add.stdout.strip():
                self.log(add.stdout.strip())
            if add.stderr.strip():
                self.log(add.stderr.strip())
            if add.returncode != 0:
                messagebox.showwarning("Wi-Fi", "netsh add profile failed. Try connecting via Windows UI.")
                return

            self.log(f"[WIFI] netsh wlan connect name={ssid}")
            connect = subprocess.run(
                ["netsh", "wlan", "connect", f"name={ssid}"],
                capture_output=True,
                text=True,
                timeout=25,
            )
            if connect.stdout.strip():
                self.log(connect.stdout.strip())
            if connect.stderr.strip():
                self.log(connect.stderr.strip())
            if connect.returncode == 0:
                self.log("[WIFI] Connect command sent OK.")
            else:
                messagebox.showwarning("Wi-Fi", "netsh connect failed. Try connecting via Windows UI.")
        except FileNotFoundError:
            messagebox.showwarning("Wi-Fi", "netsh not found. Run this GUI from Windows, or connect via system UI.")
        except subprocess.TimeoutExpired:
            messagebox.showwarning("Wi-Fi", "netsh timeout.")
        finally:
            if profile_path:
                try:
                    os.unlink(profile_path)
                except Exception:
                    pass

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

    def get_active_detection_center(self):
        if self.contrast_enabled.get() and self.last_contrast_center is not None:
            return "contrast", self.last_contrast_center
        if self.orb_enabled.get() and self.last_orb_point is not None:
            return "ORB", self.last_orb_point
        if self.yolo_enabled.get() and self.last_det_center is not None:
            return "YOLO", self.last_det_center
        return None, None

    def send_center_command(self):
        source, center = self.get_active_detection_center()
        if self.last_frame is not None and center is not None:
            fh, fw = self.last_frame.shape[:2]
            cx, cy = center
            self.calibrated_center_norm = (
                cx / max(1, fw),
                cy / max(1, fh),
            )
            self.log(f"[CAL] Vision center set from {source}: {int(round(cx))},{int(round(cy))}")
        else:
            self.calibrated_center_norm = (0.5, 0.5)
            self.log("[CAL] Vision center set to geometric frame center.")

        self.manual_target_pitch_deg.set("0.00")
        self.manual_target_yaw_deg.set("0.00")
        self.reset_circle_test()
        if self.send_line("CMD:CENTER\n", warn_title="Centering"):
            self.last_manual_send_ts = time.time()
            self.log("[CAL] Arduino target basis recentered.")

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
            if model_path.lower().endswith((".onnx", ".engine")):
                self.yolo_imgsz.set(QUADRO_YOLO_IMGSZ)
            else:
                self.yolo_imgsz.set(640)
        self.update_yolo_model_config()

    def update_yolo_model_config(self):
        model_path = self.yolo_model_path.get().strip().lower()
        if model_path.endswith((".onnx", ".engine")):
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
        if model_path.endswith((".onnx", ".engine")):
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

    def close_yolo_backend(self):
        worker = self.tensorrt_worker
        self.tensorrt_worker = None
        if worker is not None:
            try:
                worker.close()
            except Exception as e:
                self.log(f"[TensorRT] Close warning: {e}")
        self.last_tensorrt_result = None
        self.yolo_model = None
        self.yolo_backend = None

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
                "  tools/quadron_1280_fp16.engine\n\n"
                "Then select a preset or set Model path to that file."
            )
            self.yolo_model = None
            self.log(f"[YOLO] Model not found locally: {model_path_raw}")
            messagebox.showwarning("YOLO", msg)
            return

        try:
            self.close_yolo_backend()
            self.update_yolo_model_config()
            if model_path.lower().endswith(".engine"):
                if TensorRTEngine is None or TensorRTInferenceWorker is None:
                    messagebox.showwarning(
                        "YOLO",
                        "TensorRT backend is unavailable. Run this GUI on Jetson with python3-libnvinfer installed.",
                    )
                    return
                engine = TensorRTEngine(model_path)
                if engine.input_shape != (1, 3, QUADRO_YOLO_IMGSZ, QUADRO_YOLO_IMGSZ):
                    engine.close()
                    raise RuntimeError(f"Unexpected TensorRT input shape: {engine.input_shape}")
                self.tensorrt_worker = TensorRTInferenceWorker(engine, QUADRO_YOLO_IMGSZ)
                self.yolo_model = engine
                self.yolo_backend = "tensorrt_engine"
                self.log(
                    f"[TensorRT] Loaded engine: {model_path}; "
                    f"input={engine.input_shape}, outputs={engine.output_shapes}"
                )
            elif model_path.lower().endswith(".onnx"):
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
            self.close_yolo_backend()
            self.log(f"[YOLO] Failed to load local model: {model_path}; error: {e}")
            messagebox.showwarning("YOLO", f"Failed to load local model:\n{model_path}\n\n{e}")

    @staticmethod
    def camera_source_value(display_value):
        value = str(display_value).strip()
        if " — " in value:
            value = value.split(" — ", 1)[0].strip()
        if value.startswith("/dev/video"):
            return value
        try:
            return int(value)
        except Exception:
            return value

    def discover_camera_devices(self):
        if os.name == "nt":
            return [str(index) for index in range(6)]

        devices = []
        for path in sorted(glob.glob("/dev/video*")):
            label = path
            try:
                result = subprocess.run(
                    ["v4l2-ctl", "--device", path, "--info"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    check=False,
                )
                device_caps = result.stdout.split("Device Caps", 1)[-1]
                if "Video Capture" not in device_caps:
                    continue
                match = re.search(r"Card type\s*:\s*(.+)", result.stdout)
                if match:
                    label = f"{path} — {match.group(1).strip()}"
            except Exception:
                pass
            devices.append(label)
        return devices

    def refresh_cameras(self, log_changes=False):
        devices = self.discover_camera_devices()
        if not devices:
            devices = ["0"]
        previous_source = self.camera_source_value(self.camera_source.get())
        previous_devices = self.last_camera_devices
        self.last_camera_devices = devices
        self.camera_menu.configure(values=devices)

        selected = None
        for value in devices:
            if self.camera_source_value(value) == previous_source:
                selected = value
                break
        if selected is None:
            selected = devices[0]
        changed = selected != self.camera_source.get() or devices != previous_devices
        self.camera_source.set(selected)
        if changed:
            self.refresh_camera_modes()
        if log_changes:
            self.log(f"[CAM] Devices: {', '.join(devices)}")

    def poll_camera_devices(self):
        try:
            if not self.camera_running:
                self.refresh_cameras(log_changes=False)
        finally:
            self.root.after(2000, self.poll_camera_devices)

    def on_camera_selected(self, _event=None):
        if not self.camera_running:
            self.refresh_camera_modes()

    def query_camera_modes(self, source):
        if os.name == "nt" or not str(source).startswith("/dev/video"):
            return []
        try:
            result = subprocess.run(
                ["v4l2-ctl", "--device", str(source), "--list-formats-ext"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
        except Exception as e:
            self.log(f"[CAM] Cannot query modes for {source}: {e}")
            return []

        modes = {}
        current_fourcc = ""
        current_size = None
        for line in result.stdout.splitlines():
            format_match = re.search(r"\[\d+\]:\s+'([^']+)'", line)
            if format_match:
                current_fourcc = format_match.group(1).strip()
                current_size = None
                continue
            size_match = re.search(r"Size:\s+Discrete\s+(\d+)x(\d+)", line)
            if size_match:
                current_size = (int(size_match.group(1)), int(size_match.group(2)))
                modes.setdefault((current_fourcc, *current_size), 0.0)
                continue
            fps_match = re.search(r"\(([0-9.]+)\s+fps\)", line)
            if fps_match and current_size is not None:
                key = (current_fourcc, *current_size)
                modes[key] = max(modes.get(key, 0.0), float(fps_match.group(1)))

        return sorted(
            [(width, height, fps, fourcc) for (fourcc, width, height), fps in modes.items()],
            key=lambda mode: (mode[0] * mode[1], mode[2], mode[3]),
            reverse=True,
        )

    def refresh_camera_modes(self):
        source = self.camera_source_value(self.camera_source.get())
        modes = self.query_camera_modes(source)
        mode_map = {"Auto (camera default)": None}
        for width, height, fps, fourcc in modes:
            fps_text = f"{fps:g}" if fps > 0 else "default"
            label = f"{width}x{height} @ {fps_text} fps"
            if fourcc:
                label += f" ({fourcc})"
            mode_map[label] = (width, height, fps, fourcc)
        self.camera_resolution_modes = mode_map
        labels = tuple(mode_map.keys())
        self.camera_resolution_menu.configure(values=labels)
        current_mode = self.camera_resolution.get()
        if modes and (current_mode not in mode_map or current_mode == "Auto (camera default)"):
            preferred = min(
                modes,
                key=lambda mode: (
                    abs(mode[0] - QUADRO_YOLO_IMGSZ) + abs(mode[1] - QUADRO_YOLO_IMGSZ),
                    -mode[2],
                ),
            )
            preferred_label = next(
                (label for label, value in mode_map.items() if value == preferred),
                labels[0],
            )
            self.camera_resolution.set(preferred_label)
        elif current_mode not in mode_map:
            self.camera_resolution.set(labels[0])
        if modes:
            self.log(
                f"[CAM] Found {len(modes)} mode(s) for {source}; "
                f"selected {self.camera_resolution.get()}"
            )

    def start_camera(self):
        if cv2 is None:
            messagebox.showwarning("Camera", "opencv-python not installed. Install: pip install opencv-python")
            return

        source = self.camera_source_value(self.camera_source.get())

        self.stop_camera()
        if os.name != "nt" and str(source).startswith("/dev/video"):
            self.cap = cv2.VideoCapture(str(source), cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            self.cap = None
            messagebox.showwarning("Camera", f"Cannot open camera: {source}")
            return

        selected_mode = self.camera_resolution_modes.get(self.camera_resolution.get())
        try:
            if selected_mode is not None:
                width, height, fps, fourcc = selected_mode
                if fourcc and len(fourcc) == 4:
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                if fps > 0:
                    self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))

        self.camera_running = True
        self.bt_cam_start.configure(state="disabled")
        self.bt_cam_stop.configure(state="normal")
        self.camera_menu.configure(state="disabled")
        self.camera_resolution_menu.configure(state="disabled")
        self.bt_cam_refresh.configure(state="disabled")
        self.log(f"[CAM] Started camera {source} at {actual_w}x{actual_h} @ {actual_fps:g} fps")

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
        self.camera_menu.configure(state="readonly")
        self.camera_resolution_menu.configure(state="readonly")
        self.bt_cam_refresh.configure(state="normal")

    def single_detect(self):
        self.single_request = True

    def set_contrast_status(self, text):
        self.contrast_status = text
        if hasattr(self, "lb_contrast_status"):
            self.lb_contrast_status.configure(text=text)

    def set_orb_status(self, text):
        self.orb_status = text
        if hasattr(self, "lb_orb_status"):
            self.lb_orb_status.configure(text=text)

    def reset_contrast_track(self):
        self.last_contrast_det = None
        self.last_contrast_center = None
        self.last_contrast_seen_ts = 0.0
        self.contrast_smoothed_center = None
        self.contrast_locked_score = 0.0
        self.set_contrast_status("Contrast: reset")
        self.log("[CONTRAST] Track reset.")

    def reset_orb_track(self):
        self.last_orb_point = None
        self.last_orb_seen_ts = 0.0
        self.orb_smoothed_point = None
        self.orb_locked_score = 0.0
        self.orb_locked_response = 0.0
        self.set_orb_status("ORB: reset")
        self.log("[ORB] Track reset.")

    def get_contrast_config(self):
        try:
            threshold = int(float(self.contrast_threshold.get()))
        except Exception:
            threshold = 35
        try:
            blur_px = int(float(self.contrast_blur_px.get()))
        except Exception:
            blur_px = 3
        try:
            dilate_px = int(float(self.contrast_dilate_px.get()))
        except Exception:
            dilate_px = 5
        try:
            min_area = int(float(self.contrast_min_area_px.get()))
        except Exception:
            min_area = 80
        try:
            max_area = int(float(self.contrast_max_area_px.get()))
        except Exception:
            max_area = 40000
        try:
            min_box_px = int(float(self.contrast_min_box_px.get()))
        except Exception:
            min_box_px = 6
        try:
            lock_radius_px = int(float(self.contrast_lock_radius_px.get()))
        except Exception:
            lock_radius_px = 80
        try:
            max_jump_px = int(float(self.contrast_max_jump_px.get()))
        except Exception:
            max_jump_px = 160
        try:
            switch_margin = float(self.contrast_switch_margin.get())
        except Exception:
            switch_margin = 1.35
        try:
            smoothing = float(self.contrast_smoothing.get())
        except Exception:
            smoothing = 0.25
        try:
            hold_ms = int(float(self.contrast_hold_ms.get()))
        except Exception:
            hold_ms = 250

        threshold = int(self.clamp(threshold, 1, 255))
        blur_px = int(self.clamp(blur_px, 1, 51))
        if blur_px > 1 and blur_px % 2 == 0:
            blur_px += 1
        dilate_px = int(self.clamp(dilate_px, 0, 51))
        if dilate_px > 1 and dilate_px % 2 == 0:
            dilate_px += 1
        min_area = int(self.clamp(min_area, 1, 1000000))
        max_area = int(self.clamp(max_area, min_area, 2000000))
        min_box_px = int(self.clamp(min_box_px, 1, 1000))
        lock_radius_px = int(self.clamp(lock_radius_px, 0, 2000))
        max_jump_px = int(self.clamp(max_jump_px, 0, 4000))
        switch_margin = self.clamp(switch_margin, 1.0, 20.0)
        smoothing = self.clamp(smoothing, 0.0, 0.95)
        hold_ms = int(self.clamp(hold_ms, 0, 5000))
        return (
            threshold,
            blur_px,
            dilate_px,
            min_area,
            max_area,
            min_box_px,
            lock_radius_px,
            max_jump_px,
            switch_margin,
            smoothing,
            hold_ms,
        )

    def get_orb_config(self):
        try:
            nfeatures = int(float(self.orb_nfeatures.get()))
        except Exception:
            nfeatures = 500
        try:
            fast_threshold = int(float(self.orb_fast_threshold.get()))
        except Exception:
            fast_threshold = 20
        try:
            edge_threshold = int(float(self.orb_edge_threshold.get()))
        except Exception:
            edge_threshold = 31
        try:
            min_response = float(self.orb_min_response.get())
        except Exception:
            min_response = 0.0005
        try:
            lock_radius_px = int(float(self.orb_lock_radius_px.get()))
        except Exception:
            lock_radius_px = 80
        try:
            max_jump_px = int(float(self.orb_max_jump_px.get()))
        except Exception:
            max_jump_px = 160
        try:
            switch_margin = float(self.orb_switch_margin.get())
        except Exception:
            switch_margin = 1.40
        try:
            smoothing = float(self.orb_smoothing.get())
        except Exception:
            smoothing = 0.35
        try:
            hold_ms = int(float(self.orb_hold_ms.get()))
        except Exception:
            hold_ms = 300

        nfeatures = int(self.clamp(nfeatures, 1, 5000))
        fast_threshold = int(self.clamp(fast_threshold, 1, 100))
        edge_threshold = int(self.clamp(edge_threshold, 0, 200))
        min_response = self.clamp(min_response, 0.0, 1.0)
        lock_radius_px = int(self.clamp(lock_radius_px, 0, 2000))
        max_jump_px = int(self.clamp(max_jump_px, 0, 4000))
        switch_margin = self.clamp(switch_margin, 1.0, 20.0)
        smoothing = self.clamp(smoothing, 0.0, 0.95)
        hold_ms = int(self.clamp(hold_ms, 0, 5000))
        return (
            nfeatures,
            fast_threshold,
            edge_threshold,
            min_response,
            lock_radius_px,
            max_jump_px,
            switch_margin,
            smoothing,
            hold_ms,
        )

    @staticmethod
    def point_distance(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return (dx * dx + dy * dy) ** 0.5

    def choose_stable_candidate(self, candidates, previous_point, previous_score, lock_radius_px, max_jump_px, switch_margin):
        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0], reverse=True)
        best = candidates[0]
        if previous_point is None:
            return best

        scored = []
        for cand in candidates:
            dist = self.point_distance(cand[2], previous_point)
            scored.append((dist, cand))

        nearby = [item for item in scored if lock_radius_px > 0 and item[0] <= lock_radius_px]
        if nearby:
            nearby.sort(key=lambda item: item[1][0] - item[0] * 0.02, reverse=True)
            local = nearby[0][1]
            local_dist = nearby[0][0]
            best_dist = self.point_distance(best[2], previous_point)
            if best is local or best_dist <= lock_radius_px:
                return local
            if best[0] >= max(previous_score, local[0]) * switch_margin:
                if max_jump_px <= 0 or best_dist <= max_jump_px:
                    return best
            return local

        best_dist = self.point_distance(best[2], previous_point)
        if max_jump_px > 0 and best_dist > max_jump_px:
            return None
        if previous_score > 0.0 and best[0] < previous_score * switch_margin:
            return None
        return best

    def smooth_point(self, previous, raw, smoothing):
        if previous is None or smoothing <= 0.0:
            return raw
        return (
            previous[0] * smoothing + raw[0] * (1.0 - smoothing),
            previous[1] * smoothing + raw[1] * (1.0 - smoothing),
        )

    def draw_tracking_point(self, frame, center, color, held=False):
        cx, cy = int(round(center[0])), int(round(center[1]))
        radius = 8 if not held else 10
        cv2.circle(frame, (cx, cy), radius, color, 2)
        cv2.circle(frame, (cx, cy), 2, color, -1)
        cv2.line(frame, (cx - 12, cy), (cx - 4, cy), color, 2)
        cv2.line(frame, (cx + 4, cy), (cx + 12, cy), color, 2)
        cv2.line(frame, (cx, cy - 12), (cx, cy - 4), color, 2)
        cv2.line(frame, (cx, cy + 4), (cx, cy + 12), color, 2)

    def draw_persistent_track_markers(self, frame):
        if self.contrast_enabled.get() and self.last_contrast_center is not None:
            self.draw_tracking_point(frame, self.last_contrast_center, (255, 120, 0), held=True)
        if self.orb_enabled.get() and self.last_orb_point is not None:
            self.draw_tracking_point(frame, self.last_orb_point, (0, 150, 255), held=True)

    def draw_contrast_overlay(self, frame, box, center, score, raw_count, valid_count, held=False):
        cx, cy = int(round(center[0])), int(round(center[1]))
        color = (255, 200, 0) if not held else (255, 120, 0)
        label = "Contrast hold" if held else "Contrast"
        self.set_contrast_status(f"{label}: valid={valid_count}/{raw_count}, score={score:.1f}, center={cx},{cy}")
        self.draw_tracking_point(frame, center, color, held=held)

    def run_contrast_tracker(self, frame, single=False):
        if cv2 is None:
            self.set_contrast_status("Contrast: OpenCV не установлен")
            return frame

        (
            threshold,
            blur_px,
            dilate_px,
            min_area,
            max_area,
            min_box_px,
            lock_radius_px,
            max_jump_px,
            switch_margin,
            smoothing,
            hold_ms,
        ) = self.get_contrast_config()
        now = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if blur_px > 1:
            gray = cv2.GaussianBlur(gray, (blur_px, blur_px), 0)

        lap = cv2.Laplacian(gray, cv2.CV_16S, ksize=3)
        contrast = cv2.convertScaleAbs(lap)
        _, mask = cv2.threshold(contrast, threshold, 255, cv2.THRESH_BINARY)

        if dilate_px > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px, dilate_px))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.dilate(mask, kernel, iterations=1)

        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[-2]
        candidates = []
        fh, fw = frame.shape[:2]
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < min_area or area > max_area:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw < min_box_px or bh < min_box_px:
                continue
            if x <= 0 and y <= 0 and x + bw >= fw - 1 and y + bh >= fh - 1:
                continue

            component_mask = gray.copy()
            component_mask[:, :] = 0
            cv2.drawContours(component_mask, [cnt], -1, 255, -1)
            mean_contrast = float(cv2.mean(contrast, mask=component_mask)[0])
            score = mean_contrast * (area ** 0.5)
            cx = x + bw * 0.5
            cy = y + bh * 0.5
            candidates.append((score, (x, y, x + bw, y + bh), (cx, cy), area, mean_contrast))

        if not candidates:
            if self.last_contrast_center is not None and self.last_contrast_det is not None:
                x1, y1, x2, y2, score = self.last_contrast_det
                self.draw_contrast_overlay(
                    frame,
                    (x1, y1, x2, y2),
                    self.last_contrast_center,
                    score,
                    len(contours),
                    0,
                    held=True,
                )
                return frame

            self.set_contrast_status(f"Contrast: valid=0/{len(contours)}")
            if single or time.time() - self.last_contrast_log_ts >= 2.0:
                self.log(f"[CONTRAST] contours={len(contours)}, valid=0")
                self.last_contrast_log_ts = time.time()
            return frame

        selected = self.choose_stable_candidate(
            candidates,
            self.last_contrast_center,
            self.contrast_locked_score,
            lock_radius_px,
            max_jump_px,
            switch_margin,
        )
        if selected is None:
            if self.last_contrast_center is not None:
                self.draw_contrast_overlay(
                    frame,
                    self.last_contrast_det[:4] if self.last_contrast_det else (0, 0, 0, 0),
                    self.last_contrast_center,
                    self.contrast_locked_score,
                    len(contours),
                    len(candidates),
                    held=True,
                )
                return frame
            self.set_contrast_status(f"Contrast: jump rejected, valid={len(candidates)}/{len(contours)}")
            return frame

        selected_score, selected_box, raw_center, selected_area, selected_mean = selected
        center = self.smooth_point(self.contrast_smoothed_center, raw_center, smoothing)
        self.contrast_smoothed_center = center
        self.last_contrast_center = center
        x1, y1, x2, y2 = selected_box
        self.last_contrast_det = (x1, y1, x2, y2, selected_score)
        self.contrast_locked_score = selected_score
        self.last_contrast_seen_ts = now

        self.draw_contrast_overlay(frame, selected_box, self.last_contrast_center, selected_score, len(contours), len(candidates))

        if single or time.time() - self.last_contrast_log_ts >= 2.0:
            self.log(
                f"[CONTRAST] contours={len(contours)}, valid={len(candidates)}, "
                f"score={selected_score:.1f}, mean={selected_mean:.1f}, area={selected_area:.0f}, box={selected_box}"
            )
            self.last_contrast_log_ts = time.time()
        return frame

    def draw_orb_overlay(self, frame, center, score, raw_count, valid_count, held=False):
        cx, cy = int(round(center[0])), int(round(center[1]))
        label = "ORB hold" if held else "ORB"
        self.set_orb_status(f"{label}: valid={valid_count}/{raw_count}, response={score:.5f}, center={cx},{cy}")
        color = (0, 220, 255) if not held else (0, 150, 255)
        self.draw_tracking_point(frame, center, color, held=held)

    def run_orb_tracker(self, frame, single=False):
        if cv2 is None:
            self.set_orb_status("ORB: OpenCV не установлен")
            return frame

        (
            nfeatures,
            fast_threshold,
            edge_threshold,
            min_response,
            lock_radius_px,
            max_jump_px,
            switch_margin,
            smoothing,
            hold_ms,
        ) = self.get_orb_config()
        now = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create(
            nfeatures=nfeatures,
            edgeThreshold=edge_threshold,
            fastThreshold=fast_threshold,
        )
        keypoints = orb.detect(gray, None)
        candidates = []
        fh, fw = frame.shape[:2]
        for kp in keypoints:
            response = float(kp.response)
            if response < min_response:
                continue
            x, y = kp.pt
            if edge_threshold > 0 and (
                x < edge_threshold or
                y < edge_threshold or
                x > fw - edge_threshold or
                y > fh - edge_threshold
            ):
                continue
            size_bonus = max(1.0, float(kp.size)) ** 0.5
            score = response * size_bonus
            candidates.append((score, None, (float(x), float(y)), response, kp.size))

        if not candidates:
            if self.last_orb_point is not None:
                self.draw_orb_overlay(frame, self.last_orb_point, self.orb_locked_response, len(keypoints), 0, held=True)
                return frame

            self.set_orb_status(f"ORB: valid=0/{len(keypoints)}")
            if single or time.time() - self.last_orb_log_ts >= 2.0:
                self.log(f"[ORB] keypoints={len(keypoints)}, valid=0")
                self.last_orb_log_ts = time.time()
            return frame

        selected = self.choose_stable_candidate(
            candidates,
            self.last_orb_point,
            self.orb_locked_score,
            lock_radius_px,
            max_jump_px,
            switch_margin,
        )
        if selected is None:
            if self.last_orb_point is not None:
                self.draw_orb_overlay(frame, self.last_orb_point, self.orb_locked_response, len(keypoints), len(candidates), held=True)
                return frame
            self.set_orb_status(f"ORB: jump rejected, valid={len(candidates)}/{len(keypoints)}")
            return frame

        selected_score, _, raw_center, selected_response, selected_size = selected
        center = self.smooth_point(self.orb_smoothed_point, raw_center, smoothing)
        self.orb_smoothed_point = center
        self.last_orb_point = center
        self.orb_locked_score = selected_score
        self.orb_locked_response = selected_response
        self.last_orb_seen_ts = now
        self.draw_orb_overlay(frame, self.last_orb_point, selected_response, len(keypoints), len(candidates))

        if single or time.time() - self.last_orb_log_ts >= 2.0:
            self.log(
                f"[ORB] keypoints={len(keypoints)}, valid={len(candidates)}, "
                f"response={selected_response:.5f}, size={selected_size:.1f}, center={raw_center}"
            )
            self.last_orb_log_ts = time.time()
        return frame

    def send_center_target(self, center, frame_w, frame_h, tag, now, interval):
        if not self.sock or center is None:
            return False
        if (now - self.last_send_ts) < interval:
            return False

        cx, cy = center
        hfov = max(0.001, self.read_number_var(self.hfov, DEFAULT_CAMERA_HFOV_DEG))
        vfov = max(0.001, self.read_number_var(self.vfov, DEFAULT_CAMERA_VFOV_DEG))
        ref_x = self.calibrated_center_norm[0] * frame_w
        ref_y = self.calibrated_center_norm[1] * frame_h
        angle_x = ((cx - ref_x) / max(1, frame_w)) * hfov
        angle_y = ((ref_y - cy) / max(1, frame_h)) * vfov
        if self.invert_x.get():
            angle_x = -angle_x
        if self.invert_y.get():
            angle_y = -angle_y
        angle_x += self.read_number_var(self.yaw_trim_deg, 0.0)
        angle_y += self.read_number_var(self.pitch_trim_deg, 0.0)
        line = f"MSG:{tag};X:{angle_y:.2f};Y:{angle_x:.2f}\n"
        if self.send_line(line, warn_title=tag, log_tx=False):
            self.last_send_ts = now
            return True
        return False

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

    def consume_tensorrt_result(self):
        if self.tensorrt_worker is None:
            return
        result = self.tensorrt_worker.latest_result()
        if result is None:
            return
        if result.get("error"):
            self.last_tensorrt_result = None
            self.last_det = None
            self.last_det_center = None
            self.yolo_status = f"TensorRT error: {result['error']}"
            self.log(f"[TensorRT] Inference error: {result['error']}")
            return

        self.last_tensorrt_result = result
        selected = result.get("selected")
        if selected is None:
            self.last_det = None
            self.last_det_center = None
            self.yolo_status = (
                f"TRT kept=0/{result.get('raw_count', 0)} "
                f"total={result.get('total_ms', 0.0):.1f}ms"
            )
        else:
            x1, y1, x2, y2 = selected["box"]
            confidence = selected["confidence"]
            self.last_det = (x1, y1, x2, y2, confidence)
            self.last_det_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            self.yolo_status = (
                f"TRT {confidence:.2f} center={self.last_det_center[0]},{self.last_det_center[1]} "
                f"infer={result.get('inference_ms', 0.0):.1f}ms "
                f"total={result.get('total_ms', 0.0):.1f}ms"
            )

        if result.get("single"):
            self.hold_until = time.time() + 5.0
            self.log(
                f"[TensorRT] Single detect: raw={result.get('raw_count', 0)}, "
                f"kept={result.get('kept_count', 0)}, valid={len(result.get('candidates', []))}, "
                f"selected={selected}, total={result.get('total_ms', 0.0):.2f} ms"
            )
        elif time.time() - self.last_yolo_log_ts >= 2.0:
            self.log(f"[TensorRT] {self.yolo_status}")
            self.last_yolo_log_ts = time.time()

    def submit_tensorrt(self, frame, single=False):
        if self.tensorrt_worker is None:
            if single:
                messagebox.showwarning("YOLO", "TensorRT engine is not loaded.")
            return False
        self.tensorrt_worker.submit(
            frame,
            self.get_yolo_conf(),
            self.get_yolo_min_box_px(),
            single=single,
        )
        return True

    def draw_tensorrt_overlay(self, frame):
        result = self.last_tensorrt_result
        if not result or result.get("error"):
            return frame
        if result.get("frame_size") != (frame.shape[1], frame.shape[0]):
            return frame
        selected = result.get("selected")
        if selected is None:
            return frame

        selected_box = selected["box"]
        for candidate in result.get("candidates", []):
            box = candidate["box"]
            if box == selected_box:
                continue
            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 1)

        x1, y1, x2, y2 = selected_box
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
        cv2.rectangle(frame, (8, 8), (min(frame.shape[1] - 1, 600), 42), (0, 0, 0), -1)
        cv2.putText(
            frame,
            self.yolo_status,
            (14, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"target {selected['confidence']:.2f}",
            (max(0, x1), max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            3,
        )
        return frame

    def run_yolo(self, frame, single=False):
        if self.yolo_model is None:
            if single:
                messagebox.showwarning("YOLO", "Model not loaded.")
            return frame

        try:
            if self.yolo_backend == "tensorrt_engine":
                return frame
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
        now = time.time()
        circle_active = self.update_circle_target(now)

        if self.camera_running and self.cap:
            ret, frame = self.cap.read()
            if ret:
                fh, fw = frame.shape[:2]
                self.lb_res.configure(text=f"Res: {fw}x{fh}")
                self.last_frame = frame
                interval = 1.0 / max(1, int(self.rate_hz.get()))

                if self.yolo_backend == "tensorrt_engine":
                    self.consume_tensorrt_result()
                    if self.single_request:
                        self.submit_tensorrt(frame, single=True)
                        self.single_request = False
                    elif self.yolo_enabled.get() and (now - self.last_det_ts) >= interval:
                        self.submit_tensorrt(frame)
                        self.last_det_ts = now
                    if self.yolo_enabled.get() or now < self.hold_until:
                        frame = self.draw_tensorrt_overlay(frame)
                else:
                    if self.single_request:
                        frame = self.run_yolo(frame, single=True)
                        self.hold_frame = frame.copy()
                        self.hold_until = now + 5.0
                        self.single_request = False
                    elif self.yolo_enabled.get() and (now - self.last_det_ts) >= interval:
                        frame = self.run_yolo(frame)
                        self.last_det_ts = now

                if self.contrast_enabled.get() and (now - self.last_contrast_ts) >= interval:
                    frame = self.run_contrast_tracker(frame)
                    self.last_contrast_ts = now

                if self.orb_enabled.get() and (now - self.last_orb_ts) >= interval:
                    frame = self.run_orb_tracker(frame)
                    self.last_orb_ts = now

                if self.hold_frame is not None and now < self.hold_until:
                    frame = self.hold_frame.copy()
                elif self.hold_frame is not None and now >= self.hold_until:
                    self.hold_frame = None

                if circle_active:
                    pass
                elif self.manual_target_auto_send.get() and self.sock:
                    if (now - self.last_manual_send_ts) >= interval:
                        if self.send_manual_target(log_tx=False, warn_invalid=False):
                            self.last_manual_send_ts = now
                elif self.contrast_enabled.get() and self.contrast_send_enabled.get() and self.last_contrast_center is not None:
                    self.send_center_target(self.last_contrast_center, fw, fh, "CONTRAST", now, interval)
                elif self.orb_enabled.get() and self.orb_send_enabled.get() and self.last_orb_point is not None:
                    self.send_center_target(self.last_orb_point, fw, fh, "ORB", now, interval)
                elif self.send_enabled.get() and self.sock and self.last_det_center is not None:
                    self.send_center_target(self.last_det_center, fw, fh, "PHONE", now, interval)

                self.draw_persistent_track_markers(frame)

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
        app.stop_camera()
        app.close_yolo_backend()
        app.disconnect_arduino()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
