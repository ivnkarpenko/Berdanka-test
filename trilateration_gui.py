import math
import tkinter as tk
from tkinter import ttk


DEFAULT_ERROR_METERS = 0.5
ANCHOR_RADIUS = 9
TARGET_RADIUS = 8
GOAL_SIZE = 12


class TrilaterationGui:
    def __init__(self, root):
        self.root = root
        self.root.title("DWM3000 trilateration error viewer")
        self.root.geometry("1240x760")
        self.root.minsize(980, 560)

        self.scale = tk.DoubleVar(value=1.6)
        self.scale_label = tk.StringVar()
        self.error_meters = tk.DoubleVar(value=DEFAULT_ERROR_METERS)
        self.error_label = tk.StringVar()
        self.show_target_circles = tk.BooleanVar(value=False)
        self.show_second_sensor = tk.BooleanVar(value=True)
        self.show_grid = tk.BooleanVar(value=True)

        self.points = {
            "A": [-2.5, 0.0],
            "B": [2.5, 0.0],
            "T": [0.7, 2.8],
            "G": [300.0, 250.0],
        }
        self.dragging = None
        self.hover_point = None

        self._build_ui()
        self._bind_events()
        self.draw()

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(self.root, bg="#f8fafc", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.sidebar = ttk.Frame(self.root, padding=(14, 12), width=245)
        self.sidebar.grid(row=0, column=1, rowspan=2, sticky="ns")
        self.sidebar.grid_propagate(False)
        self.sidebar.columnconfigure(0, weight=1)
        self._build_sidebar()

        panel = ttk.Frame(self.root, padding=(10, 8))
        panel.grid(row=1, column=0, sticky="ew")
        panel.columnconfigure(1, weight=1)

        ttk.Label(panel, text="Масштаб").grid(row=0, column=0, padx=(0, 8))
        scale = ttk.Scale(
            panel,
            from_=0.3,
            to=140,
            orient="horizontal",
            variable=self.scale,
            command=lambda _value: self.on_scale_change(),
        )
        scale.grid(row=0, column=1, sticky="ew")

        ttk.Label(panel, textvariable=self.scale_label, width=10).grid(row=0, column=2, padx=(8, 0))
        ttk.Button(panel, text="Показать всё", command=self.fit_all).grid(row=0, column=3, padx=(12, 0))
        ttk.Button(panel, text="Сброс", command=self.reset).grid(row=0, column=4, padx=(8, 0))

    def _build_sidebar(self):
        ttk.Label(
            self.sidebar,
            text="Точность датчиков",
            font=("Segoe UI", 12, "bold"),
        ).grid(row=0, column=0, sticky="w")

        ttk.Label(
            self.sidebar,
            textvariable=self.error_label,
            font=("Segoe UI", 18, "bold"),
        ).grid(row=1, column=0, sticky="w", pady=(8, 4))

        ttk.Label(
            self.sidebar,
            text="Погрешность дальности для окружностей и угловых лучей.",
            wraplength=210,
        ).grid(row=2, column=0, sticky="w")

        error_slider = ttk.Scale(
            self.sidebar,
            from_=0.05,
            to=2.0,
            orient="horizontal",
            variable=self.error_meters,
            command=lambda _value: self.on_error_change(),
        )
        error_slider.grid(row=3, column=0, sticky="ew", pady=(18, 8))

        ticks = ttk.Frame(self.sidebar)
        ticks.grid(row=4, column=0, sticky="ew")
        ticks.columnconfigure((0, 1, 2), weight=1)
        ttk.Label(ticks, text="5 см").grid(row=0, column=0, sticky="w")
        ttk.Label(ticks, text="100 см").grid(row=0, column=1)
        ttk.Label(ticks, text="200 см").grid(row=0, column=2, sticky="e")

        quick = ttk.Frame(self.sidebar)
        quick.grid(row=5, column=0, sticky="ew", pady=(16, 0))
        quick.columnconfigure((0, 1, 2), weight=1)
        ttk.Button(quick, text="25", command=lambda: self.set_error_cm(25)).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(quick, text="50", command=lambda: self.set_error_cm(50)).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(quick, text="100", command=lambda: self.set_error_cm(100)).grid(row=0, column=2, sticky="ew", padx=(4, 0))

        ttk.Label(quick, text="см").grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 0))

        ttk.Separator(self.sidebar).grid(row=6, column=0, sticky="ew", pady=18)

        ttk.Label(
            self.sidebar,
            text="Визуализация",
            font=("Segoe UI", 11, "bold"),
        ).grid(row=7, column=0, sticky="w")

        ttk.Checkbutton(
            self.sidebar,
            text="Сетка",
            variable=self.show_grid,
            command=self.draw,
        ).grid(row=8, column=0, sticky="w", pady=(8, 0))

        ttk.Checkbutton(
            self.sidebar,
            text="Окружности от 10DOF",
            variable=self.show_target_circles,
            command=self.draw,
        ).grid(row=9, column=0, sticky="w", pady=(4, 0))

        ttk.Checkbutton(
            self.sidebar,
            text="Второй 10DOF",
            variable=self.show_second_sensor,
            command=self.draw,
        ).grid(row=10, column=0, sticky="w", pady=(4, 0))

        ttk.Separator(self.sidebar).grid(row=11, column=0, sticky="ew", pady=18)
        ttk.Label(
            self.sidebar,
            text="A и B - якори. 10DOF сенсор наводится на квадрат ЦЕЛЬ. Все точки можно перетаскивать мышью.",
            wraplength=210,
        ).grid(row=12, column=0, sticky="w")

        self.on_error_change()
        self.on_scale_change()

    def on_error_change(self):
        cm = self.error_meters.get() * 100
        self.error_label.set(f"±{cm:.0f} см")
        self.draw()

    def on_scale_change(self):
        self.scale_label.set(f"{self.scale.get():.1f} px/м")
        self.draw()

    def set_error_cm(self, centimeters):
        self.error_meters.set(centimeters / 100)
        self.on_error_change()

    def _bind_events(self):
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_motion)
        self.root.bind("<Escape>", lambda _event: self.reset())

    def reset(self):
        self.points = {
            "A": [-2.5, 0.0],
            "B": [2.5, 0.0],
            "T": [0.7, 2.8],
            "G": [300.0, 250.0],
        }
        self.draw()

    def fit_all(self):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        max_abs_x = max(abs(point[0]) for point in self.points.values())
        max_abs_y = max(abs(point[1]) for point in self.points.values())
        second = self.second_sensor_position()
        if second and self.show_second_sensor.get():
            max_abs_x = max(max_abs_x, abs(second[0]))
            max_abs_y = max(max_abs_y, abs(second[1]))
        scale_x = (width / 2 - 80) / max(max_abs_x, 1.0)
        scale_y = (height / 2 - 120) / max(max_abs_y, 1.0)
        self.scale.set(max(0.3, min(140, scale_x, scale_y)))
        self.on_scale_change()

    def world_to_screen(self, x, y):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        origin_x = width / 2
        origin_y = height / 2 + 80
        return origin_x + x * self.scale.get(), origin_y - y * self.scale.get()

    def screen_to_world(self, x, y):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        origin_x = width / 2
        origin_y = height / 2 + 80
        return (x - origin_x) / self.scale.get(), (origin_y - y) / self.scale.get()

    def on_press(self, event):
        self.dragging = self.nearest_point(event.x, event.y)

    def on_drag(self, event):
        if self.dragging is None:
            return
        x, y = self.screen_to_world(event.x, event.y)
        self.points[self.dragging] = [x, y]
        self.draw()

    def on_release(self, _event):
        self.dragging = None

    def on_motion(self, event):
        point = self.nearest_point(event.x, event.y, max_distance=18)
        if point != self.hover_point:
            self.hover_point = point
            self.canvas.configure(cursor="fleur" if point else "")
            self.draw()

    def nearest_point(self, sx, sy, max_distance=24):
        nearest = None
        best = max_distance
        for name, (x, y) in self.points.items():
            px, py = self.world_to_screen(x, y)
            distance = math.hypot(sx - px, sy - py)
            if distance < best:
                best = distance
                nearest = name
        return nearest

    def draw(self):
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_distances()
        self.draw_error_circles()
        self.draw_goal_guides()
        self.draw_second_sensor()
        self.draw_angle_guides()
        self.draw_points()
        self.draw_info_box()

    def draw_grid(self):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        scale = self.scale.get()
        ox, oy = self.world_to_screen(0, 0)

        if self.show_grid.get():
            step = self.grid_step()
            first_x = math.floor(self.screen_to_world(0, 0)[0] / step) * step
            last_x = math.ceil(self.screen_to_world(width, 0)[0] / step) * step
            first_y = math.floor(self.screen_to_world(0, height)[1] / step) * step
            last_y = math.ceil(self.screen_to_world(0, 0)[1] / step) * step

            x = first_x
            while x <= last_x:
                sx, _ = self.world_to_screen(x, 0)
                color = "#cbd5e1" if abs(x) < 1e-9 else "#e2e8f0"
                self.canvas.create_line(sx, 0, sx, height, fill=color)
                if abs(x) > 1e-9:
                    self.canvas.create_text(sx + 3, oy + 14, text=f"{x:g}", fill="#64748b", anchor="nw", font=("Segoe UI", 8))
                x += step

            y = first_y
            while y <= last_y:
                _, sy = self.world_to_screen(0, y)
                color = "#cbd5e1" if abs(y) < 1e-9 else "#e2e8f0"
                self.canvas.create_line(0, sy, width, sy, fill=color)
                if abs(y) > 1e-9:
                    self.canvas.create_text(ox + 6, sy - 3, text=f"{y:g}", fill="#64748b", anchor="w", font=("Segoe UI", 8))
                y += step

        self.canvas.create_line(0, oy, width, oy, fill="#94a3b8", width=2)
        self.canvas.create_line(ox, 0, ox, height, fill="#94a3b8", width=2)
        self.canvas.create_text(width - 12, oy - 10, text="X, м", fill="#475569", anchor="e", font=("Segoe UI", 9, "bold"))
        self.canvas.create_text(ox + 10, 12, text="Y, м", fill="#475569", anchor="w", font=("Segoe UI", 9, "bold"))

    def grid_step(self):
        scale = self.scale.get()
        if scale >= 100:
            return 0.5
        if scale >= 55:
            return 1.0
        if scale >= 18:
            return 5.0
        if scale >= 7:
            return 10.0
        if scale >= 2.5:
            return 25.0
        if scale >= 1.0:
            return 50.0
        return 100.0

    def draw_distances(self):
        a = self.points["A"]
        b = self.points["B"]
        t = self.points["T"]
        self.draw_segment("A", "B", "#334155", 3, offset=-16)
        self.draw_segment("A", "T", "#2563eb", 2, offset=14)
        self.draw_segment("B", "T", "#16a34a", 2, offset=14)

        ax, ay = self.world_to_screen(*a)
        bx, by = self.world_to_screen(*b)
        tx, ty = self.world_to_screen(*t)
        self.canvas.create_line(ax, ay, tx, ty, fill="#2563eb", width=2, dash=(5, 4))
        self.canvas.create_line(bx, by, tx, ty, fill="#16a34a", width=2, dash=(5, 4))

    def draw_goal_guides(self):
        target = self.points["T"]
        goal = self.points["G"]
        tx, ty = self.world_to_screen(*target)
        gx, gy = self.world_to_screen(*goal)

        for anchor_name in ("A", "B"):
            ax, ay = self.world_to_screen(*self.points[anchor_name])
            self.canvas.create_line(ax, ay, gx, gy, fill="#94a3b8", width=1, dash=(2, 6))

        self.canvas.create_line(tx, ty, gx, gy, fill="#ea580c", width=3, arrow=tk.LAST)
        self.draw_segment("T", "G", "#ea580c", 2, offset=-18)

        bearing = self.angle(target, goal)
        self.draw_ray(target, bearing, "#ea580c", dash=(8, 6), width=2)

        arc_radius = 66
        base_angle = self.angle(self.points["A"], self.points["B"])
        delta = self.signed_delta(base_angle, bearing)
        self.draw_arc((tx, ty), base_angle, delta, arc_radius, "#ea580c", width=3)
        label_angle = base_angle + delta / 2
        self.canvas.create_text(
            tx + math.cos(math.radians(label_angle)) * (arc_radius + 22),
            ty - math.sin(math.radians(label_angle)) * (arc_radius + 22),
            text=f"курс {bearing:.1f}°",
            fill="#ea580c",
            font=("Segoe UI", 10, "bold"),
        )

    def draw_second_sensor(self):
        if not self.show_second_sensor.get():
            return

        second = self.second_sensor_position()
        if second is None:
            return

        goal = self.points["G"]
        sx, sy = self.world_to_screen(*second)
        gx, gy = self.world_to_screen(*goal)
        bearing = self.angle(second, goal)
        distance = self.distance(second, goal)

        self.canvas.create_line(sx, sy, gx, gy, fill="#7c3aed", width=3, dash=(7, 5), arrow=tk.LAST)
        self.draw_ray(second, bearing, "#7c3aed", dash=(3, 7), width=2)

        radius = TARGET_RADIUS + 2
        self.canvas.create_oval(
            sx - radius,
            sy - radius,
            sx + radius,
            sy + radius,
            fill="#a855f7",
            outline="white",
            width=3,
        )
        self.canvas.create_text(
            sx + 14,
            sy + 18,
            text=f"10DOF 2\n({second[0]:.2f}; {second[1]:.2f})",
            anchor="nw",
            fill="#581c87",
            font=("Segoe UI", 10, "bold"),
        )

        mid_x = (sx + gx) / 2
        mid_y = (sy + gy) / 2
        self.canvas.create_text(
            mid_x,
            mid_y + 18,
            text=f"10DOF2-ЦЕЛЬ: {distance:.1f} м",
            fill="#7c3aed",
            font=("Segoe UI", 10, "bold"),
        )

        self.canvas.create_text(
            sx + 18,
            sy - 18,
            text=f"курс {bearing:.1f}°",
            anchor="sw",
            fill="#7c3aed",
            font=("Segoe UI", 10, "bold"),
        )

    def draw_segment(self, start, end, color, width, offset=0):
        p1 = self.points[start]
        p2 = self.points[end]
        x1, y1 = self.world_to_screen(*p1)
        x2, y2 = self.world_to_screen(*p2)
        self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width)

        distance = self.distance(p1, p2)
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2
        nx, ny = self.normal(x1, y1, x2, y2)
        label = f"{start}-{end}: {distance:.2f} м"
        self.canvas.create_text(
            mx + nx * offset,
            my + ny * offset,
            text=label,
            fill=color,
            font=("Segoe UI", 10, "bold"),
        )

    def draw_error_circles(self):
        self.draw_measurement_circle("A", "T", "#2563eb")
        self.draw_measurement_circle("B", "T", "#16a34a")
        if self.show_target_circles.get():
            self.draw_measurement_circle("T", "A", "#0f766e")
            self.draw_measurement_circle("T", "B", "#ca8a04")

    def draw_measurement_circle(self, center_name, target_name, color):
        center = self.points[center_name]
        target = self.points[target_name]
        radius = self.distance(center, target)
        error = self.error_meters.get()

        self.draw_world_circle(center, max(radius - error, 0.0), color, dash=(2, 4), width=1)
        self.draw_world_circle(center, radius, color, dash=None, width=2)
        self.draw_world_circle(center, radius + error, color, dash=(2, 4), width=1)
        self.draw_error_band(center, radius, color)

    def draw_world_circle(self, center, radius, color, dash=None, width=1):
        cx, cy = self.world_to_screen(*center)
        sr = radius * self.scale.get()
        self.canvas.create_oval(
            cx - sr,
            cy - sr,
            cx + sr,
            cy + sr,
            outline=color,
            width=width,
            dash=dash,
        )

    def draw_error_band(self, center, radius, color):
        if radius <= 0:
            return
        error = self.error_meters.get()
        outer = (radius + error) * self.scale.get()
        inner = max(radius - error, 0.0) * self.scale.get()
        cx, cy = self.world_to_screen(*center)
        # Полупрозрачный цвет в tkinter без alpha недоступен, поэтому кольцо отмечается частыми точками.
        for angle in range(0, 360, 6):
            radians = math.radians(angle)
            x1 = cx + math.cos(radians) * inner
            y1 = cy - math.sin(radians) * inner
            x2 = cx + math.cos(radians) * outer
            y2 = cy - math.sin(radians) * outer
            self.canvas.create_line(x1, y1, x2, y2, fill=color, stipple="gray75")

    def draw_angle_guides(self):
        self.draw_anchor_angle("A", "B", "T", "#1d4ed8")
        self.draw_anchor_angle("B", "A", "T", "#15803d")
        self.draw_target_angle("#9333ea")

    def draw_anchor_angle(self, anchor_name, base_name, target_name, color):
        anchor = self.points[anchor_name]
        base = self.points[base_name]
        target = self.points[target_name]
        anchor_screen = self.world_to_screen(*anchor)

        base_angle = self.angle(anchor, base)
        target_angle = self.angle(anchor, target)
        arc_delta = self.signed_delta(base_angle, target_angle)
        distance = self.distance(anchor, target)
        deviation = self.angular_deviation(distance)

        self.draw_arc(anchor_screen, base_angle, arc_delta, 54, color, width=3)
        self.draw_ray(anchor, target_angle - deviation, color, dash=(3, 5))
        self.draw_ray(anchor, target_angle + deviation, color, dash=(3, 5))
        self.draw_ray(anchor, target_angle, color, width=3)

        label_angle = base_angle + arc_delta / 2
        lx = anchor_screen[0] + math.cos(math.radians(label_angle)) * 74
        ly = anchor_screen[1] - math.sin(math.radians(label_angle)) * 74
        self.canvas.create_text(
            lx,
            ly,
            text=f"{abs(arc_delta):.1f}°",
            fill=color,
            font=("Segoe UI", 10, "bold"),
        )

    def draw_target_angle(self, color):
        target = self.points["T"]
        angle_to_a = self.angle(target, self.points["A"])
        angle_to_b = self.angle(target, self.points["B"])
        delta = self.signed_delta(angle_to_a, angle_to_b)
        self.draw_arc(self.world_to_screen(*target), angle_to_a, delta, 42, color, width=3)

        mid = angle_to_a + delta / 2
        tx, ty = self.world_to_screen(*target)
        self.canvas.create_text(
            tx + math.cos(math.radians(mid)) * 60,
            ty - math.sin(math.radians(mid)) * 60,
            text=f"{abs(delta):.1f}°",
            fill=color,
            font=("Segoe UI", 10, "bold"),
        )

    def draw_arc(self, center, start_angle, delta, radius, color, width=2):
        x, y = center
        extent = delta
        self.canvas.create_arc(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            start=start_angle,
            extent=extent,
            style=tk.ARC,
            outline=color,
            width=width,
        )

    def draw_ray(self, start, angle_degrees, color, dash=None, width=1):
        sx, sy = self.world_to_screen(*start)
        length = max(self.canvas.winfo_width(), self.canvas.winfo_height())
        radians = math.radians(angle_degrees)
        ex = sx + math.cos(radians) * length
        ey = sy - math.sin(radians) * length
        self.canvas.create_line(sx, sy, ex, ey, fill=color, width=width, dash=dash)

    def draw_points(self):
        styles = {
            "A": ("#1d4ed8", "Якорь A"),
            "B": ("#15803d", "Якорь B"),
            "T": ("#dc2626", "10DOF сенсор"),
            "G": ("#f97316", "ЦЕЛЬ"),
        }
        for name, (x, y) in self.points.items():
            sx, sy = self.world_to_screen(x, y)
            color, label = styles[name]
            outline = "#0f172a" if name == self.hover_point or name == self.dragging else "white"
            if name == "G":
                half = GOAL_SIZE
                self.canvas.create_rectangle(
                    sx - half,
                    sy - half,
                    sx + half,
                    sy + half,
                    fill=color,
                    outline=outline,
                    width=3,
                )
            else:
                radius = TARGET_RADIUS if name == "T" else ANCHOR_RADIUS
                self.canvas.create_oval(
                    sx - radius,
                    sy - radius,
                    sx + radius,
                    sy + radius,
                    fill=color,
                    outline=outline,
                    width=3,
                )
            self.canvas.create_text(
                sx + 14,
                sy - 14,
                text=f"{label}\n({x:.2f}; {y:.2f})",
                anchor="sw",
                fill="#0f172a",
                font=("Segoe UI", 10, "bold"),
            )

    def draw_info_box(self):
        width = max(self.canvas.winfo_width(), 1)
        x0 = width - 306
        y0 = 14
        x1 = width - 14
        y1 = 384 if self.show_second_sensor.get() and self.second_sensor_position() else 318
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="#ffffff", outline="#cbd5e1", width=1)

        a = self.points["A"]
        b = self.points["B"]
        t = self.points["T"]
        g = self.points["G"]
        ab = self.distance(a, b)
        at = self.distance(a, t)
        bt = self.distance(b, t)
        tg = self.distance(t, g)
        ag = self.distance(a, g)
        bg = self.distance(b, g)
        angle_a = abs(self.signed_delta(self.angle(a, b), self.angle(a, t)))
        angle_b = abs(self.signed_delta(self.angle(b, a), self.angle(b, t)))
        angle_t = abs(self.signed_delta(self.angle(t, a), self.angle(t, b)))
        target_bearing = self.angle(t, g)
        baseline_bearing = self.angle(a, b)
        correction = self.signed_delta(baseline_bearing, target_bearing)
        dev_a = self.angular_deviation(at)
        dev_b = self.angular_deviation(bt)
        error_cm = self.error_meters.get() * 100
        second = self.second_sensor_position()

        lines = [
            f"DWM3000 / погрешность ±{error_cm:.0f} см",
            f"A-B: {ab:.2f} м",
            f"A-10DOF: {at:.2f} м, угол A: {angle_a:.1f}°",
            f"B-10DOF: {bt:.2f} м, угол B: {angle_b:.1f}°",
            f"Угол при 10DOF: {angle_t:.1f}°",
            f"Отклонение от A: ±{dev_a:.1f}°",
            f"Отклонение от B: ±{dev_b:.1f}°",
            "",
            f"ЦЕЛЬ от 10DOF: {tg:.1f} м",
            f"ЦЕЛЬ от A/B: {ag:.1f} / {bg:.1f} м",
            f"Курс на ЦЕЛЬ: {target_bearing:.1f}°",
            f"Поправка от A-B: {correction:+.1f}°",
        ]

        if self.show_second_sensor.get() and second:
            second_goal_distance = self.distance(second, g)
            second_bearing = self.angle(second, g)
            second_correction = self.signed_delta(baseline_bearing, second_bearing)
            lines.extend([
                "",
                f"10DOF 2: ({second[0]:.2f}; {second[1]:.2f})",
                f"10DOF 2-ЦЕЛЬ: {second_goal_distance:.1f} м",
                f"Курс 10DOF 2: {second_bearing:.1f}°",
                f"Поправка 10DOF 2: {second_correction:+.1f}°",
            ])

        lines.extend([
            "",
            "Перетаскивайте A, B, 10DOF и ЦЕЛЬ",
        ])

        y = y0 + 16
        for index, line in enumerate(lines):
            font = ("Segoe UI", 10, "bold") if index == 0 else ("Segoe UI", 10)
            self.canvas.create_text(x0 + 12, y, text=line, anchor="nw", fill="#0f172a", font=font)
            y += 22

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    @staticmethod
    def angle(p1, p2):
        return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

    @staticmethod
    def signed_delta(start, end):
        return (end - start + 180) % 360 - 180

    def angular_deviation(self, distance):
        error = self.error_meters.get()
        if distance <= error:
            return 90.0
        return math.degrees(math.asin(error / distance))

    def second_sensor_position(self):
        a = self.points["A"]
        b = self.points["B"]
        t = self.points["T"]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            return None

        projection = ((t[0] - a[0]) * dx + (t[1] - a[1]) * dy) / length_squared
        foot_x = a[0] + projection * dx
        foot_y = a[1] + projection * dy
        return [2 * foot_x - t[0], 2 * foot_y - t[1]]

    @staticmethod
    def normal(x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length == 0:
            return 0, -1
        return -dy / length, dx / length


def main():
    root = tk.Tk()
    ttk.Style().theme_use("clam")
    TrilaterationGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
