import socket
import threading
import subprocess
import queue
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

PORT_DEFAULT = 3333
WIFI_SSID = "GABELLA"
WIFI_PASS = "J8f2829a"

class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Windows ⇄ Arduino UNO R4 WiFi (TCP) — Tkinter")
        self.root.geometry("900x650")

        self.sock = None
        self.rx_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()

        self.build_ui()
        self.root.after(50, self.process_queue)
        self.log("[APP] Ready. 1) Connect Wi‑Fi 2) Connect TCP 3) Send.")

    def build_ui(self):
        wifi = tk.LabelFrame(self.root, text="Wi‑Fi (Windows)", padx=10, pady=10)
        wifi.pack(fill="x", padx=10, pady=8)

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

        ard = tk.LabelFrame(self.root, text="Arduino TCP", padx=10, pady=10)
        ard.pack(fill="x", padx=10, pady=8)

        tk.Label(ard, text="IP:").grid(row=0, column=0, sticky="e")
        self.ed_ip = tk.Entry(ard, width=20)
        self.ed_ip.insert(0, "10.42.0.2")
        self.ed_ip.grid(row=0, column=1, padx=6)

        tk.Label(ard, text="Port:").grid(row=0, column=2, sticky="e")
        self.ed_port = tk.Entry(ard, width=8)
        self.ed_port.insert(0, str(PORT_DEFAULT))
        self.ed_port.grid(row=0, column=3, padx=6)

        tk.Label(ard, text="Bind IP:").grid(row=0, column=4, sticky="e")
        self.ed_bind = tk.Entry(ard, width=15)
        self.ed_bind.insert(0, "")
        self.ed_bind.grid(row=0, column=5, padx=6)

        self.bt_connect = tk.Button(ard, text="Connect", command=self.connect_arduino)
        self.bt_connect.grid(row=1, column=1, pady=6, sticky="w")

        self.bt_disconnect = tk.Button(ard, text="Disconnect", command=self.disconnect_arduino, state="disabled")
        self.bt_disconnect.grid(row=1, column=5, pady=6, sticky="e")

        send = tk.LabelFrame(self.root, text="Send", padx=10, pady=10)
        send.pack(fill="x", padx=10, pady=8)

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

        logf = tk.LabelFrame(self.root, text="Log (from Arduino)", padx=10, pady=10)
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
        bind_ip = self.ed_bind.get().strip()

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
            if bind_ip:
                s.bind((bind_ip, 0))
                self.log(f"[NET] Bound to {bind_ip}")
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
