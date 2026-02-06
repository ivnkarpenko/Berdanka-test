import socket
import threading
import subprocess
import queue
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText
import time
import base64

try:
    import cv2
except Exception:
    cv2 = None

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
WIFI_PASS = "cisco"

class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Windows <-> Arduino UNO R4 WiFi")
        self.root.geometry("1100x700")
        self.root.minsize(900, 600)

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
        self.rate_hz = tk.IntVar(value=5)
        self.hfov = tk.DoubleVar(value=90.0)
        self.vfov = tk.DoubleVar(value=30.0)
        self.yolo_model = None
        self.yolo_model_path = tk.StringVar(value="yolo11n.pt")
        self.last_frame = None
        self.last_det = None
        self.last_det_center = None
        self.last_det_ts = 0.0
        self.last_send_ts = 0.0
        self.single_request = False
        self.hold_until = 0.0
        self.hold_frame = None

        self.build_ui()
        self.root.after(50, self.process_queue)
        self.root.after(30, self.update_camera)
        self.log("[APP] Ready. 1) Connect Wi‑Fi 2) Connect TCP 3) Send.")

    def build_ui(self):
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        main = tk.Frame(self.root)
        main.grid(row=0, column=0, sticky="nsew")
        main.grid_rowconfigure(0, weight=1)
        main.grid_columnconfigure(0, weight=0)
        main.grid_columnconfigure(1, weight=1)

        left = tk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        left.grid_rowconfigure(3, weight=1)
        left.grid_columnconfigure(0, weight=1)

        right = tk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        right.grid_rowconfigure(2, weight=1)
        right.grid_columnconfigure(0, weight=1)

        wifi = tk.LabelFrame(left, text="Wi‑Fi (Windows)", padx=10, pady=10)
        wifi.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        tk.Label(wifi, text="SSID:").grid(row=0, column=0, sticky="e")
        self.ed_ssid = tk.Entry(wifi, width=28)
        self.ed_ssid.insert(0, WIFI_SSID)
        self.ed_ssid.grid(row=0, column=1, padx=6)

        tk.Label(wifi, text="Password:").grid(row=1, column=0, sticky="e")
        self.ed_pass = tk.Entry(wifi, width=28, show="*")
        self.ed_pass.insert(0, WIFI_PASS)
        self.ed_pass.grid(row=1, column=1, padx=6)

        self.bt_wifi = tk.Button(wifi, text="Connect Wi‑Fi (netsh)", command=self.connect_wifi_windows)
        self.bt_wifi.grid(row=0, column=2, rowspan=2, padx=10, ipadx=10)

        ard = tk.LabelFrame(left, text="Arduino TCP", padx=10, pady=10)
        ard.grid(row=1, column=0, sticky="ew", pady=(0, 8))

        tk.Label(ard, text="IP:").grid(row=0, column=0, sticky="e")
        self.ed_ip = tk.Entry(ard, width=20)
        self.ed_ip.insert(0, "192.168.4.1")
        self.ed_ip.grid(row=0, column=1, padx=6)

        tk.Label(ard, text="Port:").grid(row=0, column=2, sticky="e")
        self.ed_port = tk.Entry(ard, width=8)
        self.ed_port.insert(0, str(PORT_DEFAULT))
        self.ed_port.grid(row=0, column=3, padx=6)

        self.bt_connect = tk.Button(ard, text="Connect", command=self.connect_arduino)
        self.bt_connect.grid(row=1, column=1, pady=6, sticky="w")

        self.bt_disconnect = tk.Button(ard, text="Disconnect", command=self.disconnect_arduino, state="disabled")
        self.bt_disconnect.grid(row=1, column=3, pady=6, sticky="e")

        send = tk.LabelFrame(left, text="Send", padx=10, pady=10)
        send.grid(row=2, column=0, sticky="ew", pady=(0, 8))

        tk.Label(send, text="Message:").grid(row=0, column=0, sticky="e")
        self.ed_msg = tk.Entry(send, width=60)
        self.ed_msg.grid(row=0, column=1, columnspan=5, padx=6, sticky="we")

        vcmd = (self.root.register(self.only_digits), "%P")

        tk.Label(send, text="X:").grid(row=1, column=0, sticky="e")
        self.ed_x = tk.Entry(send, width=10, validate="key", validatecommand=vcmd)
        self.ed_x.insert(0, "0")
        self.ed_x.grid(row=1, column=1, padx=6, sticky="w")

        tk.Label(send, text="Y:").grid(row=1, column=2, sticky="e")
        self.ed_y = tk.Entry(send, width=10, validate="key", validatecommand=vcmd)
        self.ed_y.insert(0, "0")
        self.ed_y.grid(row=1, column=3, padx=6, sticky="w")

        self.bt_send = tk.Button(send, text="Send", command=self.send_packet)
        self.bt_send.grid(row=1, column=5, padx=10, ipadx=10)

        # ---- Vision group
        vision = tk.LabelFrame(right, text="Vision (YOLO11)", padx=10, pady=10)
        vision.grid(row=0, column=0, sticky="nsew")
        vision.grid_rowconfigure(2, weight=1)
        vision.grid_columnconfigure(0, weight=1)

        top_row = tk.Frame(vision)
        top_row.grid(row=0, column=0, sticky="ew")

        tk.Label(top_row, text="Camera:").pack(side="left")
        self.ed_cam = tk.Entry(top_row, width=6)
        self.ed_cam.insert(0, "0")
        self.ed_cam.pack(side="left", padx=6)

        self.bt_cam_start = tk.Button(top_row, text="Start Camera", command=self.start_camera)
        self.bt_cam_start.pack(side="left", padx=6)

        self.bt_cam_stop = tk.Button(top_row, text="Stop Camera", command=self.stop_camera, state="disabled")
        self.bt_cam_stop.pack(side="left", padx=6)

        tk.Label(top_row, text="Model:").pack(side="left", padx=6)
        self.ed_model = tk.Entry(top_row, width=20, textvariable=self.yolo_model_path)
        self.ed_model.pack(side="left")

        self.cb_yolo = tk.Checkbutton(top_row, text="YOLO On", variable=self.yolo_enabled)
        self.cb_yolo.pack(side="left", padx=6)

        self.bt_single = tk.Button(top_row, text="Single Detect", command=self.single_detect)
        self.bt_single.pack(side="left", padx=6)

        self.bt_reload = tk.Button(top_row, text="Load Model", command=self.load_model)
        self.bt_reload.pack(side="left", padx=6)

        tk.Label(top_row, text="Rate Hz:").pack(side="left", padx=6)
        self.sc_rate = tk.Scale(top_row, from_=1, to=30, orient="horizontal", variable=self.rate_hz, length=120)
        self.sc_rate.pack(side="left")

        self.cb_send = tk.Checkbutton(top_row, text="Send Center", variable=self.send_enabled)
        self.cb_send.pack(side="left", padx=6)

        fov_row = tk.Frame(vision)
        fov_row.grid(row=1, column=0, sticky="ew", pady=(6, 0))

        tk.Label(fov_row, text="HFOV°:").pack(side="left")
        self.ed_hfov = tk.Entry(fov_row, width=6, textvariable=self.hfov)
        self.ed_hfov.pack(side="left", padx=6)

        tk.Label(fov_row, text="VFOV°:").pack(side="left")
        self.ed_vfov = tk.Entry(fov_row, width=6, textvariable=self.vfov)
        self.ed_vfov.pack(side="left", padx=6)

        self.lb_res = tk.Label(fov_row, text="Res: n/a")
        self.lb_res.pack(side="left", padx=12)

        self.canvas = tk.Canvas(vision, bg="black", highlightthickness=0)
        self.canvas.grid(row=2, column=0, sticky="nsew", pady=6)

        logf = tk.LabelFrame(left, text="Log (from Arduino)", padx=10, pady=10)
        logf.grid(row=3, column=0, sticky="nsew")
        logf.grid_rowconfigure(0, weight=1)
        logf.grid_columnconfigure(0, weight=1)

        self.log_view = ScrolledText(logf, height=12, state="normal")
        self.log_view.bind("<Key>", lambda e: "break")
        self.log_view.grid(row=0, column=0, sticky="nsew")

    def only_digits(self, new_value: str) -> bool:
        return new_value == "" or new_value.isdigit()

    def log(self, s: str):
        self.log_view.insert("end", s + "\n")
        self.log_view.see("end")

    def connect_wifi_windows(self):
        ssid = self.ed_ssid.get().strip()
        pw = self.ed_pass.get()
        if not ssid:
            messagebox.showwarning("Wi‑Fi", "SSID is empty.")
            return

        # Build a temporary WLAN profile via netsh
        profile_xml = f"""<?xml version=\"1.0\"?>
<WLANProfile xmlns=\"http://www.microsoft.com/networking/WLAN/profile/v1\">
    <name>{ssid}</name>
    <SSIDConfig>
        <SSID>
            <name>{ssid}</name>
        </SSID>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <MSM>
        <security>
            <authEncryption>
                <authentication>WPA2PSK</authentication>
                <encryption>AES</encryption>
                <useOneX>false</useOneX>
            </authEncryption>
            <sharedKey>
                <keyType>passPhrase</keyType>
                <protected>false</protected>
                <keyMaterial>{pw}</keyMaterial>
            </sharedKey>
        </security>
    </MSM>
</WLANProfile>"""

        try:
            # Use PowerShell + netsh for Windows. Works from native Python or WSL.
            cmd_add = [
                "powershell.exe",
                "-NoProfile",
                "-Command",
                f"$p=\"$env:TEMP\\wifi_profile.xml\";"
                f"Set-Content -Path $p -Value @'\n{profile_xml}\n'@;"
                f"netsh wlan add profile filename=$p;"
                f"netsh wlan connect name=\"{ssid}\""
            ]
            self.log("[WIFI] netsh add profile + connect")
            p = subprocess.run(cmd_add, capture_output=True, text=True, timeout=25)
            if p.returncode == 0:
                self.log("[WIFI] Connected OK.")
            else:
                self.log("[WIFI] ERROR")
                if p.stdout.strip():
                    self.log(p.stdout.strip())
                if p.stderr.strip():
                    self.log(p.stderr.strip())
                messagebox.showwarning("Wi‑Fi", "netsh failed. Try connecting via Windows UI.")
        except FileNotFoundError:
            messagebox.showwarning("Wi‑Fi", "powershell.exe not found. Connect via Windows UI.")
        except subprocess.TimeoutExpired:
            messagebox.showwarning("Wi‑Fi", "netsh timeout.")

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
        except Exception as e:
            self.log(f"[NET] Local IPs error: {e}")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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

    def send_packet(self):
        if not self.sock:
            messagebox.showwarning("Send", "Not connected to Arduino.")
            return

        msg = self.ed_msg.get().strip()
        x = self.ed_x.get().strip() or "0"
        y = self.ed_y.get().strip() or "0"

        line = f"MSG:{msg};X:{x};Y:{y}\n"
        try:
            self.sock.sendall(line.encode("utf-8"))
            self.log(f"[TX] {line.strip()}")
        except Exception as e:
            self.log(f"[NET] Send error: {e}")
            self.disconnect_arduino()

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
                        self.q.put(f"[RвевX] {line}")
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
        self.root.after(50, self.process_queue)

    # ===== Vision =====
    def load_model(self):
        if YOLO is None:
            messagebox.showwarning("YOLO", "ultralytics not installed. Install: pip install ultralytics")
            return
        model_path = self.yolo_model_path.get().strip()
        if not model_path:
            messagebox.showwarning("YOLO", "Model path is empty.")
            return
        try:
            self.yolo_model = YOLO(model_path)
            self.log(f"[YOLO] Loaded model: {model_path}")
        except Exception as e:
            self.yolo_model = None
            messagebox.showwarning("YOLO", f"Failed to load model: {e}")

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
        self.camera_running = True
        self.bt_cam_start.configure(state="disabled")
        self.bt_cam_stop.configure(state="normal")
        self.log(f"[CAM] Started camera {idx}")

    def stop_camera(self):
        self.camera_running = False
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.bt_cam_start.configure(state="normal")
        self.bt_cam_stop.configure(state="disabled")

    def single_detect(self):
        self.single_request = True

    def run_yolo(self, frame, single=False):
        if YOLO is None or self.yolo_model is None:
            if single:
                messagebox.showwarning("YOLO", "Model not loaded.")
            return frame
        try:
            results = self.yolo_model.predict(frame, verbose=False, conf=0.25, classes=[67])
            if len(results) == 0:
                return frame
            r = results[0]
            best = None
            best_conf = 0.0
            for b in r.boxes:
                conf = float(b.conf.item())
                if conf > best_conf:
                    best_conf = conf
                    best = b
            if best is not None:
                x1, y1, x2, y2 = map(int, best.xyxy[0].tolist())
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                self.last_det = (x1, y1, x2, y2, best_conf)
                self.last_det_center = (cx, cy)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
                cv2.putText(frame, f"phone {best_conf:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            return frame
        except Exception as e:
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

                # If single detect is holding, show that frame
                if self.hold_frame is not None and now < self.hold_until:
                    frame = self.hold_frame.copy()
                elif self.hold_frame is not None and now >= self.hold_until:
                    self.hold_frame = None

                # Send center via TCP (throttled)
                if self.send_enabled.get() and self.sock and self.last_det_center is not None:
                    if (now - self.last_send_ts) >= interval:
                        cx, cy = self.last_det_center
                        hfov = float(self.hfov.get())
                        vfov = float(self.vfov.get())
                        angle_x = (cx / max(1, fw) - 0.5) * hfov
                        angle_y = (0.5 - cy / max(1, fh)) * vfov
                        # swap axes to match Processing expectations
                        line = f"MSG:PHONE;X:{angle_y:.2f};Y:{angle_x:.2f}\n"
                        try:
                            self.sock.sendall(line.encode("utf-8"))
                            self.last_send_ts = now
                        except Exception as e:
                            self.log(f"[NET] Send error: {e}")
                            self.disconnect_arduino()

                # convert BGR -> RGB for Tk
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, _ = frame_rgb.shape
                canvas_w = max(1, self.canvas.winfo_width())
                canvas_h = max(1, self.canvas.winfo_height())

                # preserve aspect ratio (letterbox)
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
                self.canvas.image = img
                self.canvas.create_image(x0, y0, image=img, anchor="nw")
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
