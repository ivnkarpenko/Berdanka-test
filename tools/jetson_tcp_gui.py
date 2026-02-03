import socket
import threading
import subprocess
import queue
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

PORT_DEFAULT = 3333

class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Jetson ⇄ Arduino UNO R4 WiFi (TCP) — Tkinter")
        self.root.geometry("900x650")

        # networking state
        self.sock = None
        self.rx_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()

        # ===== UI =====
        self.build_ui()

        # periodic GUI updater (from queue)
        self.root.after(50, self.process_queue)
        self.log("[APP] Ready. 1) (optional) Connect Wi-Fi 2) Connect TCP 3) Send.")

    def build_ui(self):
        # ---- WiFi group
        wifi = tk.LabelFrame(self.root, text="Wi-Fi (Jetson)", padx=10, pady=10)
        wifi.pack(fill="x", padx=10, pady=8)

        tk.Label(wifi, text="SSID:").grid(row=0, column=0, sticky="e")
        self.ed_ssid = tk.Entry(wifi, width=28)
        self.ed_ssid.grid(row=0, column=1, padx=6)

        tk.Label(wifi, text="Password:").grid(row=1, column=0, sticky="e")
        self.ed_pass = tk.Entry(wifi, width=28, show="*")
        self.ed_pass.grid(row=1, column=1, padx=6)

        self.bt_wifi = tk.Button(wifi, text="Connect Wi-Fi (nmcli)", command=self.connect_wifi_nmcli)
        self.bt_wifi.grid(row=0, column=2, rowspan=2, padx=10, ipadx=10)

        # ---- Arduino group
        ard = tk.LabelFrame(self.root, text="Arduino TCP", padx=10, pady=10)
        ard.pack(fill="x", padx=10, pady=8)

        tk.Label(ard, text="IP:").grid(row=0, column=0, sticky="e")
        self.ed_ip = tk.Entry(ard, width=20)
        self.ed_ip.insert(0, "10.42.0.2")  # change to Arduino IP
        self.ed_ip.grid(row=0, column=1, padx=6)

        tk.Label(ard, text="Port:").grid(row=0, column=2, sticky="e")
        self.ed_port = tk.Entry(ard, width=8)
        self.ed_port.insert(0, str(PORT_DEFAULT))
        self.ed_port.grid(row=0, column=3, padx=6)

        self.bt_connect = tk.Button(ard, text="Connect", command=self.connect_arduino)
        self.bt_connect.grid(row=1, column=1, pady=6, sticky="w")

        self.bt_disconnect = tk.Button(ard, text="Disconnect", command=self.disconnect_arduino, state="disabled")
        self.bt_disconnect.grid(row=1, column=3, pady=6, sticky="e")

        # ---- Send group
        send = tk.LabelFrame(self.root, text="Send", padx=10, pady=10)
        send.pack(fill="x", padx=10, pady=8)

        tk.Label(send, text="Message:").grid(row=0, column=0, sticky="e")
        self.ed_msg = tk.Entry(send, width=60)
        self.ed_msg.grid(row=0, column=1, columnspan=5, padx=6, sticky="we")

        # only digits validation
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

        # ---- Log window
        logf = tk.LabelFrame(self.root, text="Serial / Log (from Arduino)", padx=10, pady=10)
        logf.pack(fill="both", expand=True, padx=10, pady=8)

        self.log_view = ScrolledText(logf, height=20, state="disabled")
        self.log_view.pack(fill="both", expand=True)

    def only_digits(self, new_value: str) -> bool:
        return new_value == "" or new_value.isdigit()

    def log(self, s: str):
        self.log_view.configure(state="normal")
        self.log_view.insert("end", s + "\n")
        self.log_view.see("end")
        self.log_view.configure(state="disabled")

    def connect_wifi_nmcli(self):
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
                messagebox.showwarning("Wi-Fi", "nmcli failed. Try connecting via system UI or check permissions.")
        except FileNotFoundError:
            messagebox.showwarning("Wi-Fi", "nmcli not found. Install network-manager or connect via system UI.")
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
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((ip, port))
            s.settimeout(None)
            self.sock = s
        except Exception as e:
            self.sock = None
            self.log(f"[NET] Connect error: {e}")
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
        self.root.after(50, self.process_queue)


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
