import threading
import time
import math
import random

import cv2
import numpy as np
from pypylon import pylon
import serial
import serial.tools.list_ports  # <-- eklendi

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from datetime import datetime

from port_listener import list_serial_ports  # <-- eklendi

# ======================================================
#  CONFIG / CONSTANTS
# ======================================================

# --- Logo path for GUI header (update to your real path) ---
LOGO_PATH = "logo.png"

# --- Telemetry scale factors (counts → mm) ---
X_MM_PER_COUNT = 1.0
Y_MM_PER_COUNT = 1.0
AZ_MM_PER_COUNT = 1.0

# --- Kamera ve optik parametreler ---
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

DISPLAY_W = 512
DISPLAY_H = 512

# --- Gimbal / seri port parametreleri ---
BAUD = 250000
FEEDRATE = 450
SER_ENABLED = True

K_AZ_MM_PER_DEG = 0.1
K_EL_MM_PER_DEG = 0.1

AZ_DEADBAND_DEG = 0.02
EL_DEADBAND_DEG = 0.02

MAX_STEP_MM = 0.5

# Yazılımsal limitler
X_MIN_MM = -100.0
X_MAX_MM = 100.0
Y_MIN_MM = -40.0
Y_MAX_MM = 40.0

current_x_mm = 0.0
current_y_mm = 0.0

ser = None           # MKS seri handle
SER_MKS_PORT = None  # <-- eklendi

# RASTGELE HEDEF
HOLD_TIME_SEC = 5.0
CENTER_TOL_PX = 3
TARGET_LOST_MAX_DIST_PX = 60

# Global GUI ref
gui = None


# ======================================================
#  GUI CLASS
# ======================================================

class TelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Magnetic Ruler + Camera Dashboard")
        self.root.geometry("1600x900")
        self.root.configure(bg="#e5e5e5")

        self.node_data = {
            1: {'x': 0, 'y': 0, 'az': 0, 'last_update': None, 'active': False},
            2: {'x': 0, 'y': 0, 'az': 0, 'last_update': None, 'active': False}
        }

        self.camera_frame = None
        self.camera_active = False
        self.camera_lock = threading.Lock()

        self.create_widgets()
        self.update_display()

    def create_widgets(self):
        header_frame = tk.Frame(self.root, bg="#2b2b2b")
        header_frame.pack(fill=tk.X, pady=10)

        logo_frame = tk.Frame(header_frame, bg="#2b2b2b")
        logo_frame.pack(side=tk.LEFT, padx=20)

        try:
            img = Image.open(LOGO_PATH)
            img = img.resize((720, 251), Image.Resampling.LANCZOS)
            self.logo_photo = ImageTk.PhotoImage(img)
            logo_label = tk.Label(logo_frame, image=self.logo_photo, bg="#2b2b2b")
            logo_label.pack()
        except Exception as e:
            print(f"Could not load logo: {e}")

        title_frame = tk.Frame(header_frame, bg="#2b2b2b")
        title_frame.pack(side=tk.LEFT, padx=450)

        title = tk.Label(
            title_frame,
            text="Dashboard",
            font=("Arial", 25, "bold"),
            fg="#ffffff",
            bg="#2b2b2b"
        )
        title.pack(anchor=tk.W)

        subtitle = tk.Label(
            title_frame,
            text="Real-time sensor and camera monitoring system",
            font=("Arial", 10),
            fg="#9ca3af",
            bg="#2b2b2b"
        )
        subtitle.pack(anchor=tk.W)

        main_container = tk.Frame(self.root, bg="#2b2b2b")
        main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        telemetry_container = tk.Frame(main_container, bg="#2b2b2b")
        telemetry_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        nodes_frame = tk.Frame(telemetry_container, bg="#2b2b2b")
        nodes_frame.pack(fill=tk.BOTH, expand=True)

        self.node1_frame = self.create_node_card(nodes_frame, 1)
        self.node1_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

        self.node2_frame = self.create_node_card(nodes_frame, 2)
        self.node2_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        nodes_frame.grid_rowconfigure(0, weight=1)
        nodes_frame.grid_rowconfigure(1, weight=1)
        nodes_frame.grid_columnconfigure(0, weight=1)

        camera_container = tk.Frame(
            main_container,
            bg="#3a3a3a",
            relief=tk.RAISED,
            borderwidth=2
        )
        camera_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))

        cam_header = tk.Frame(camera_container, bg="#3a3a3a")
        cam_header.pack(fill=tk.X, padx=15, pady=15)

        cam_title = tk.Label(
            cam_header,
            text="Camera Feed",
            font=("Arial", 20, "bold"),
            fg="#ffffff",
            bg="#3a3a3a"
        )
        cam_title.pack(side=tk.LEFT)

        self.cam_status = tk.Label(
            cam_header,
            text="● Disconnected",
            font=("Arial", 11),
            fg="#ef4444",
            bg="#3a3a3a"
        )
        self.cam_status.pack(side=tk.RIGHT)

        ttk.Separator(camera_container, orient='horizontal').pack(fill=tk.X, padx=15)

        self.camera_label = tk.Label(
            camera_container,
            bg="#2b2b2b",
            text="No Camera Feed",
            font=("Arial", 14),
            fg="#6b7280"
        )
        self.camera_label.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)

    def create_node_card(self, parent, node_id):
        card = tk.Frame(parent, bg="#3a3a3a", relief=tk.RAISED, borderwidth=2)

        header = tk.Frame(card, bg="#3a3a3a")
        header.pack(fill=tk.X, padx=15, pady=15)

        if node_id == 1:
            title_text = "Gimbal Camera"
        elif node_id == 2:
            title_text = "Gimbal Laser"
        else:
            title_text = f"Node {node_id}"

        title = tk.Label(
            header,
            text=title_text,
            font=("Arial", 20, "bold"),
            fg="#ffffff",
            bg="#3a3a3a"
        )
        title.pack(side=tk.LEFT)

        status = tk.Label(
            header,
            text="● Disconnected",
            font=("Arial", 11),
            fg="#ef4444",
            bg="#3a3a3a"
        )
        status.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_status', status)

        ttk.Separator(card, orient='horizontal').pack(fill=tk.X, padx=15)

        axes_frame = tk.Frame(card, bg="#3a3a3a")
        axes_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)

        self.create_axis_display(axes_frame, node_id, 'X', '#3b82f6', 0)
        self.create_axis_display(axes_frame, node_id, 'Y', '#10b981', 1)
        self.create_axis_display(axes_frame, node_id, 'AZ', '#f59e0b', 2)

        update_label = tk.Label(
            card,
            text="Last update: --:--:--",
            font=("Arial", 9),
            fg="#6b7280",
            bg="#3a3a3a"
        )
        update_label.pack(pady=10)
        setattr(self, f'node{node_id}_update', update_label)

        return card

    def create_axis_display(self, parent, node_id, axis, color, row):
        frame = tk.Frame(parent, bg="#3a3a3a")
        frame.pack(fill=tk.X, pady=8)

        label_frame = tk.Frame(frame, bg="#3a3a3a")
        label_frame.pack(fill=tk.X)

        label = tk.Label(
            label_frame,
            text=f"{axis}-Axis",
            font=("Arial", 11, "bold"),
            fg="#d1d5db",
            bg="#3a3a3a"
        )
        label.pack(side=tk.LEFT)

        count_label = tk.Label(
            label_frame,
            text="0",
            font=("Arial", 14, "bold"),
            fg="#ffffff",
            bg="#3a3a3a"
        )
        count_label.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_{axis.lower()}_count', count_label)

        unit_label = tk.Label(
            label_frame,
            text="counts",
            font=("Arial", 9),
            fg="#9ca3af",
            bg="#3a3a3a"
        )
        unit_label.pack(side=tk.RIGHT, padx=(5, 10))

        progress_frame = tk.Frame(frame, bg="#505050", height=12)
        progress_frame.pack(fill=tk.X, pady=5)
        progress_frame.pack_propagate(False)

        progress_bar = tk.Frame(progress_frame, bg=color, width=0, height=12)
        progress_bar.place(relx=0.5, rely=0, anchor='n')
        setattr(self, f'node{node_id}_{axis.lower()}_bar', progress_bar)

        mm_label = tk.Label(
            frame,
            text="0.000 mm",
            font=("Arial", 12, "bold"),
            fg=color,
            bg="#3a3a3a"
        )
        mm_label.pack(anchor=tk.E)
        setattr(self, f'node{node_id}_{axis.lower()}_mm', mm_label)

    def update_display(self):
        for node_id in [1, 2]:
            data = self.node_data[node_id]

            status_label = getattr(self, f'node{node_id}_status')
            if data['active'] and data['last_update']:
                time_diff = time.time() - data['last_update']
                if time_diff < 1.0:
                    status_label.config(text="● Connected", fg="#10b981")
                else:
                    status_label.config(text="● Disconnected", fg="#ef4444")
            else:
                status_label.config(text="● Disconnected", fg="#ef4444")

            for axis, mm_per_count in [('x', X_MM_PER_COUNT),
                                       ('y', Y_MM_PER_COUNT),
                                       ('az', AZ_MM_PER_COUNT)]:
                count = data[axis]
                mm = count * mm_per_count

                count_label = getattr(self, f'node{node_id}_{axis}_count')
                count_label.config(text=f"{count:,}")

                mm_label = getattr(self, f'node{node_id}_{axis}_mm')
                mm_label.config(text=f"{mm:.3f} mm")

                bar = getattr(self, f'node{node_id}_{axis}_bar')
                max_count = 10000
                percentage = min(abs(count) / max_count, 1.0)
                bar_width = int(percentage * 250)
                bar.config(width=bar_width)

            if data['last_update']:
                timestamp = datetime.fromtimestamp(
                    data['last_update']
                ).strftime('%H:%M:%S')
                update_label = getattr(self, f'node{node_id}_update')
                update_label.config(text=f"Last update: {timestamp}")

        with self.camera_lock:
            if self.camera_frame is not None:
                try:
                    frame_rgb = cv2.cvtColor(self.camera_frame, cv2.COLOR_BGR2RGB)

                    container = self.camera_label
                    container_w = container.winfo_width()
                    container_h = container.winfo_height()
                    if container_w < 50 or container_h < 50:
                        container_w, container_h = 800, 600

                    h, w = frame_rgb.shape[:2]
                    scale = min(container_w / w, container_h / h)
                    new_w, new_h = int(w * scale), int(h * scale)

                    frame_resized = cv2.resize(
                        frame_rgb, (new_w, new_h), interpolation=cv2.INTER_CUBIC
                    )

                    img = Image.fromarray(frame_resized)
                    imgtk = ImageTk.PhotoImage(image=img)

                    self.camera_label.config(image=imgtk, text="")
                    self.camera_label.image = imgtk

                    if self.camera_active:
                        self.cam_status.config(text="● Active", fg="#10b981")
                except Exception as e:
                    print(f"Camera display error: {e}")
            elif not self.camera_active:
                self.cam_status.config(text="● Disconnected", fg="#ef4444")

        self.root.after(50, self.update_display)

    def update_node_data(self, node_id, x, y, az):
        self.node_data[node_id]['x'] = x
        self.node_data[node_id]['y'] = y
        self.node_data[node_id]['az'] = az
        self.node_data[node_id]['last_update'] = time.time()
        self.node_data[node_id]['active'] = True

    def update_camera_frame(self, frame):
        with self.camera_lock:
            self.camera_frame = frame.copy()
            self.camera_active = True


# ======================================================
#  TRACKING HELPERS
# ======================================================

def pixels_to_angle(delta_x, delta_y):
    dx_mm = delta_x * PIXEL_SIZE_MM
    dy_mm = delta_y * PIXEL_SIZE_MM
    theta_x = math.degrees(math.atan(dx_mm / FOCAL_LENGTH_MM))
    theta_y = math.degrees(math.atan(dy_mm / FOCAL_LENGTH_MM))
    return theta_x, theta_y


def set_exposure_basler(cam, exp_us):
    try:
        if cam.ExposureAuto.IsWritable():
            cam.ExposureAuto.SetValue("Off")
            print("ExposureAuto → Off")
    except Exception as e:
        print("ExposureAuto kapatılamadı:", e)

    try:
        node = cam.ExposureTime
    except Exception:
        print("ExposureTime noduna erişilemedi.")
        return None

    min_exp = node.Min
    max_exp = node.Max
    print(f"Exposure range: {min_exp:.1f} – {max_exp:.1f} us")

    exp_value = max(min(exp_us, max_exp), min_exp)
    node.SetValue(exp_value)
    val = node.GetValue()
    print("Exposure set to:", val, "us")
    return val


def snap_to_inc(val, inc):
    try:
        inc = int(inc)
    except Exception:
        inc = 1
    return val if inc <= 1 else (val // inc) * inc


def set_safe(node, value):
    try:
        vmin = node.GetMin()
        vmax = node.GetMax()
        if isinstance(value, (int, float)):
            value = max(vmin, min(vmax, value))
    except Exception:
        pass
    node.SetValue(value)


def set_roi_basler(cam, roi_w, roi_h, offx, offy):
    try:
        try:
            set_safe(cam.OffsetX, 0)
        except Exception:
            pass
        try:
            set_safe(cam.OffsetY, 0)
        except Exception:
            pass

        try:
            w_inc = cam.Width.GetInc()
        except Exception:
            w_inc = 1

        try:
            h_inc = cam.Height.GetInc()
        except Exception:
            h_inc = 1

        try:
            ox_inc = cam.OffsetX.GetInc()
        except Exception:
            ox_inc = 1

        try:
            oy_inc = cam.OffsetY.GetInc()
        except Exception:
            oy_inc = 1

        W = snap_to_inc(roi_w, w_inc)
        H = snap_to_inc(roi_h, h_inc)
        set_safe(cam.Width, W)
        set_safe(cam.Height, H)

        OX = snap_to_inc(offx, ox_inc)
        OY = snap_to_inc(offy, oy_inc)
        set_safe(cam.OffsetX, OX)
        set_safe(cam.OffsetY, OY)

        print(f"ROI ayarlandı: Width={W}, Height={H}, OffsetX={OX}, OffsetY={OY}")

    except Exception as e:
        print("ROI ayarlanamadı:", e)


def open_basler_camera():
    tl_factory = pylon.TlFactory.GetInstance()

    gige_tl = None
    for tl_info in tl_factory.EnumerateTls():
        if "GigE" in tl_info.GetDeviceClass():
            gige_tl = tl_factory.CreateTl(tl_info)
            break

    devices = []

    if gige_tl is not None:
        try:
            devices = gige_tl.EnumerateAllDevices()
            print(f"GigE üzerinden bulunan cihaz sayısı: {len(devices)}")
        except Exception as e:
            print("GigE EnumerateAllDevices hata verdi:", e)

    if not devices:
        print("GigE enumerate sonuçsuz, TlFactory.EnumerateDevices() ile tekrar deneniyor...")
        devices = tl_factory.EnumerateDevices()

    if not devices:
        raise RuntimeError("Hiç Basler kamera bulunamadı (GigE + USB).")

    cam = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
    cam.Open()

    di = cam.GetDeviceInfo()
    try:
        serial_no = di.GetSerialNumber()
    except Exception:
        serial_no = "N/A"

    print(f"✅ Bağlı Basler kamera: {di.GetModelName()} [{serial_no}]")

    try:
        if hasattr(cam, "GevSCPSPacketSize") and cam.GevSCPSPacketSize.IsWritable():
            cam.GevSCPSPacketSize.SetValue(cam.GevSCPSPacketSize.Max)
            print("GevSCPSPacketSize max'a ayarlandı.")
    except Exception as e:
        print("Packet size ayarlanamadı:", e)

    return cam


# ======================================================
#  SERIAL / GIMBAL
# ======================================================

def init_serial():
    """MKS / Marlin için port auto-detect + open."""
    global ser, SER_MKS_PORT

    if not SER_ENABLED:
        print("Seri port devre dışı (SER_ENABLED = False).")
        ser = None
        return

    if SER_MKS_PORT is None:
        print("⚠ MKS port bulunamadı, DRY-RUN modunda devam ediyorum.")
        ser = None
        return

    try:
        ser = serial.Serial(SER_MKS_PORT, BAUD, timeout=0.01)
        print(f"Seri port açıldı: {SER_MKS_PORT} @ {BAUD}")
        time.sleep(2.0)
        send_gcode("G91")  # Göreceli mod
    except Exception as e:
        print("Seri port açılamadı, sadece görüntü takibi yapılacak:", e)
        ser = None


def send_gcode(cmd: str):
    global ser
    if ser is None:
        return
    try:
        line = (cmd.strip() + "\n").encode()
        ser.write(line)
        print("→ GCODE:", cmd)
        time.sleep(0.001)
        while ser.in_waiting:
            resp = ser.readline().decode(errors="ignore").strip()
            if resp:
                print("<", resp)
    except Exception as e:
        print("G-code gönderilemedi:", e)


def send_to_gimbal(az_deg, el_deg):
    global ser, current_x_mm, current_y_mm, gui

    if ser is None:
        print(f"(DRY-RUN) az={az_deg:.3f} el={el_deg:.3f}  "
              f"[X={current_x_mm:.2f}mm Y={current_y_mm:.2f}mm]")
        if gui is not None:
            gui.update_node_data(1, int(current_x_mm), int(current_y_mm), 0)
        return

    if abs(az_deg) < AZ_DEADBAND_DEG and abs(el_deg) < EL_DEADBAND_DEG:
        return

    step_x = -K_AZ_MM_PER_DEG * az_deg
    step_y = K_EL_MM_PER_DEG * el_deg

    step_x = max(min(step_x, MAX_STEP_MM), -MAX_STEP_MM)
    step_y = max(min(step_y, MAX_STEP_MM), -MAX_STEP_MM)

    target_x = current_x_mm + step_x
    target_y = current_y_mm + step_y

    if target_x > X_MAX_MM:
        step_x = X_MAX_MM - current_x_mm
        target_x = X_MAX_MM
        print("⚠ X yazılımsal endstop (üst limit)!")
    elif target_x < X_MIN_MM:
        step_x = X_MIN_MM - current_x_mm
        target_x = X_MIN_MM
        print("⚠ X yazılımsal endstop (alt limit)!")

    if target_y > Y_MAX_MM:
        step_y = Y_MAX_MM - current_y_mm
        target_y = Y_MAX_MM
        print("⚠ Y yazılımsal endstop (üst limit)!")
    elif target_y < Y_MIN_MM:
        step_y = Y_MIN_MM - current_y_mm
        target_y = Y_MIN_MM
        print("⚠ Y yazılımsal endstop (alt limit)!")

    if abs(step_x) < 1e-3:
        step_x = 0.0
    if abs(step_y) < 1e-3:
        step_y = 0.0

    if step_x == 0.0 and step_y == 0.0:
        return

    cmd_parts = []
    if step_x != 0.0:
        cmd_parts.append(f"X{step_x:.3f}")
    if step_y != 0.0:
        cmd_parts.append(f"Y{step_y:.3f}")
    cmd = "G1 " + " ".join(cmd_parts) + f" F{FEEDRATE}"
    send_gcode(cmd)

    current_x_mm += step_x
    current_y_mm += step_y

    print(f"[POS] X={current_x_mm:.2f}mm  Y={current_y_mm:.2f}mm")

    if gui is not None:
        gui.update_node_data(1, int(current_x_mm), int(current_y_mm), 0)


# ======================================================
#  MULTI-POINT RANDOM TARGET TRACKING
# ======================================================

def find_white_points(img_bgr, min_area=30, thresh_val=220):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    _, mask = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    points = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        radius = int(math.sqrt(area / math.pi))
        points.append((cx, cy, radius, area))

    return points, mask


def find_bright_points_mono(img, min_area=20):
    if len(img.shape) == 3:
        gray = img[:, :, 0]
    else:
        gray = img

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    points = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        radius = int(math.sqrt(area / math.pi))
        points.append((cx, cy, radius, area))
    return points, gray


def track_star(stop_event):
    global gui

    cam = open_basler_camera()
    init_serial()

    pixel_format = "Unknown"
    is_bayer = False
    is_true_mono = False

    current_target = None
    target_center_px = None
    target_centered_t = None
    rng = random.Random()

    try:
        try:
            if cam.PixelFormat.IsWritable():
                enum_entries = cam.PixelFormat.GetSymbolics()
                print("Mevcut PixelFormat seçenekleri:", enum_entries)

                if "BGR8" in enum_entries:
                    cam.PixelFormat.SetValue("BGR8")
                    print("🎨 PixelFormat → BGR8 (renkli)")
                elif "RGB8Packed" in enum_entries:
                    cam.PixelFormat.SetValue("RGB8Packed")
                    print("🎨 PixelFormat → RGB8Packed (renkli)")
                else:
                    print("⚠ BGR8/RGB8Packed yok, mevcut formatla devam ediliyor.")
            else:
                print("PixelFormat yazılabilir değil.")
        except Exception as e:
            print("PixelFormat ayarlanırken hata:", e)

        try:
            pixel_format = cam.PixelFormat.GetValue()
            print("Aktif PixelFormat:", pixel_format)
            if pixel_format.startswith("Bayer"):
                is_bayer = True
            if pixel_format.startswith("Mono"):
                is_true_mono = True
        except Exception as e:
            print("Aktif PixelFormat okunamadı:", e)

        set_roi_basler(cam, 2748, 2800, 828, 230)
        current_exp = set_exposure_basler(cam, 22600.0)

        try:
            if cam.AcquisitionMode.IsWritable():
                cam.AcquisitionMode.SetValue("Continuous")
                print("AcquisitionMode → Continuous")
        except Exception as e:
            print("AcquisitionMode ayarlanamadı:", e)

        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        print("Kamera grabbing başladı.")

        while cam.IsGrabbing() and not stop_event.is_set():
            try:
                grabResult = cam.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
            except Exception as e:
                print("Frame alınırken timeout/hata:", e)
                continue

            if not grabResult.GrabSucceeded():
                print("Frame grab başarısız:", grabResult.ErrorCode, grabResult.ErrorDescription)
                grabResult.Release()
                continue

            image = grabResult.Array
            grabResult.Release()

            detection_mode = "green"

            if len(image.shape) == 2:
                if is_bayer:
                    if pixel_format == "BayerRG8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    elif pixel_format == "BayerBG8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_BG2BGR)
                    elif pixel_format == "BayerGR8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GR2BGR)
                    elif pixel_format == "BayerGB8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GB2RGB)
                    else:
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    detection_mode = "green"
                else:
                    print("⚠ Görüntü gerçek tek kanal (mono), parlak nokta moduna geçiyorum.")
                    gray_img = image
                    color_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
                    detection_mode = "mono"
            else:
                color_img = image
                detection_mode = "green"

            h, w = color_img.shape[:2]
            center = (w // 2, h // 2)

            if detection_mode == "green":
                points, aux_img = find_white_points(color_img)
            else:
                points, aux_img = find_bright_points_mono(color_img)

            if not points:
                current_target = None
                target_center_px = None
                target_centered_t = None

                annotated = color_img.copy()
                cv2.drawMarker(
                    annotated, center, (0, 255, 0),
                    cv2.MARKER_CROSS, 20, 2
                )
                if gui is not None:
                    gui.update_camera_frame(annotated)
                continue

            if current_target is None:
                current_target = rng.choice(points)
                target_center_px = (current_target[0], current_target[1])
                target_centered_t = None
                print(f"🎯 Yeni rastgele hedef: {target_center_px}")
            else:
                tx_prev, ty_prev = target_center_px
                best = None
                best_d = None
                for (cx, cy, radius, area) in points:
                    d = math.hypot(cx - tx_prev, cy - ty_prev)
                    if best_d is None or d < best_d:
                        best_d = d
                        best = (cx, cy, radius, area)

                if best is None or best_d > TARGET_LOST_MAX_DIST_PX:
                    current_target = rng.choice(points)
                    target_center_px = (current_target[0], current_target[1])
                    target_centered_t = None
                    print(f"🎯 Hedef kayboldu, yeni rastgele hedef: {target_center_px}")
                else:
                    current_target = best
                    target_center_px = (current_target[0], current_target[1])

            cx, cy, radius, area = current_target
            dx = cx - center[0]
            dy = center[1] - cy

            az_deg, el_deg = pixels_to_angle(dx, dy)

            err_pix = math.hypot(dx, dy)
            mode_icon = "🟢" if detection_mode == "green" else "⚪"
            print(
                f"{mode_icon} mode={detection_mode} target={target_center_px}  "
                f"Δx={dx:4d} Δy={dy:4d}  "
                f"az={az_deg:7.3f}° el={el_deg:7.3f}°  |err|={err_pix:.1f}"
            )

            send_to_gimbal(az_deg, el_deg)

            now = time.time()
            if err_pix <= CENTER_TOL_PX:
                if target_centered_t is None:
                    target_centered_t = now
                    print("✅ Hedef merkezde, timer başlatıldı.")
                else:
                    if (now - target_centered_t) >= HOLD_TIME_SEC:
                        print(f"⏱ Hedefte {HOLD_TIME_SEC} sn bekledim, yeni hedefe geçiyorum.")
                        current_target = None
                        target_center_px = None
                        target_centered_t = None
            else:
                target_centered_t = None

            annotated = color_img.copy()
            cv2.drawMarker(
                annotated, center, (0, 255, 0),
                cv2.MARKER_CROSS, 20, 2
            )

            for (px, py, r, a) in points:
                cv2.circle(annotated, (px, py), max(r, 3), (255, 0, 0), 1)

            cv2.circle(annotated, (cx, cy), max(radius, 5), (0, 0, 255), 2)
            cv2.circle(annotated, (cx, cy), 3, (0, 255, 255), -1)

            if current_exp is None:
                exp_text = "Exp: N/A"
            else:
                exp_text = f"Exp={current_exp:.0f} us"

            cv2.putText(
                annotated, f"dx={dx}px dy={dy}px",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2
            )
            cv2.putText(
                annotated, f"az={az_deg:.2f} el={el_deg:.2f}",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2
            )
            cv2.putText(
                annotated, f"{exp_text}  PF={pixel_format}",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (200, 200, 200), 1
            )

            if gui is not None:
                gui.update_camera_frame(annotated)

        print("Track loop exiting...")

    finally:
        try:
            if cam.IsGrabbing():
                cam.StopGrabbing()
        except Exception:
            pass
        cam.Close()
        print("Kamera kapatıldı.")

        global ser
        if ser is not None:
            try:
                ser.close()
                print("Seri port kapatıldı.")
            except Exception:
                pass

        if gui is not None:
            gui.camera_active = False


# ======================================================
#  ENTRY POINT – PORT DISCOVERY + GUI
# ======================================================

def main():
    global gui, SER_MKS_PORT

    # Port discovery – port_listener'daki implementasyonunu kullanıyoruz
    mks_port, ard_ports = list_serial_ports()
    SER_MKS_PORT = mks_port
    print("MKS port:", SER_MKS_PORT, "Arduino ports:", ard_ports)

    root = tk.Tk()
    gui = TelemetryGUI(root)

    stop_event = threading.Event()
    t = threading.Thread(target=track_star, args=(stop_event,), daemon=True)
    t.start()

    def on_close():
        print("GUI closing, stopping tracking thread...")
        stop_event.set()
        root.after(200, root.destroy)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
