from pypylon import pylon
import cv2
import numpy as np
import math
import time
import serial
import serial.tools.list_ports
import config as cfg
import threading
import os
import tkinter as tk
from tkinter import ttk
from datetime import datetime
from PIL import Image, ImageTk

from port_listener import list_serial_ports

# -------------------------------------------------
#  Ortak sabitler
# -------------------------------------------------

# --- Lazer tespiti için HSV aralığı (mavi lazer) ---
LASER_BLUE_LOWER = (90, 80, 120)
LASER_BLUE_UPPER = (140, 255, 255)
MIN_AREA_LASER = 5

# --- Lazer gimbali için kontrol gain'leri ---
K_AZ_MM_PER_DEG_LASER = 0.1
K_EL_MM_PER_DEG_LASER = 0.1

LASER_DEADBAND_DEG = 0.02
MAX_STEP_MM_LASER = 2.0

# --- Kamera ve optik parametreler ---
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

# --- Seri haberleşme / manyetik cetvel protokolü (GUI tarafı) ---
BAUD_ARDUINO = 115200
START_BYTE = 0x7E
END_BYTE = 0x7F
MSG_SENSOR = 0x10
FRAME_SIZE = 16

# --- Manyetik cetvel kalibrasyonları (GUI göstergesi için) ---
X_MM_PER_COUNT = 10.0 / 7459.0
Y_MM_PER_COUNT = 10.0 / 6875.0
AZ_MM_PER_COUNT = 0.001

# Logo path (GUI)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOGO_PATH = os.path.join(BASE_DIR, "UZAY-Yatay-Beyaz.png")

# --- Gimbal / seri port parametreleri ---
SER_ENABLED = True  # Seri port kapatırsan sadece takip yapılır

current_x_mm = 0.0
current_y_mm = 0.0

SER_MKS_PORT = None
SER_ARD1_PORT = None
SER_ARD2_PORT = None

ser = None  # MKS seri handle


# -------------------------------------------------
#  Yardımcı fonksiyonlar (kamera + lazer)
# -------------------------------------------------

def pixels_to_angle(delta_x, delta_y):
    """Piksel ofsetini (dx, dy) açıya çevir (derece) → (azimuth, elevation)."""
    dx_mm = delta_x * PIXEL_SIZE_MM
    dy_mm = delta_y * PIXEL_SIZE_MM
    theta_x = math.degrees(math.atan(dx_mm / FOCAL_LENGTH_MM))  # azimut
    theta_y = math.degrees(math.atan(dy_mm / FOCAL_LENGTH_MM))  # elevasyon
    return theta_x, theta_y


def detect_bright_circle_center_mono(img):
    """Mono8 görüntüde en büyük parlak yuvarlak cismin merkezini bulur."""
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

    if not contours:
        raise ValueError("Hiç kontur bulunamadı (parlak cisim algılanamadı).")

    big_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(big_cnt)
    if area < 20:
        raise ValueError("Algılanan kontur çok küçük (muhtemelen gürültü).")

    M = cv2.moments(big_cnt)
    if M["m00"] == 0:
        raise ValueError("Moment hatası (m00=0).")

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    radius = int(math.sqrt(area / math.pi))
    maxVal = float(gray[cy, cx])

    return (cx, cy), radius, maxVal


def detect_green_circle_center(img):
    """Renkli görüntüde yeşil dairenin merkezini bul."""
    if len(img.shape) == 2 or img.shape[2] == 1:
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img_bgr = img

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    lower_green = np.array([35, 60, 40], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        raise ValueError("Yeşil bölge bulunamadı (kontur yok).")

    big_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(big_cnt)
    if area < 50:
        raise ValueError("Algılanan yeşil kontur çok küçük (gürültü).")

    M = cv2.moments(big_cnt)
    if M["m00"] == 0:
        raise ValueError("Moment hatası (m00=0).")

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    radius = int(math.sqrt(area / math.pi))
    maxVal = float(mask[cy, cx])

    return (cx, cy), radius, maxVal


def detect_laser_circle_center(color_img):
    """Mavi lazer spotunu HSV uzayında bulur."""
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LASER_BLUE_LOWER, LASER_BLUE_UPPER)
    mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        raise RuntimeError("Mavi lazer spotu bulunamadı (kontur yok).")

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA_LASER:
        raise RuntimeError(f"Lazer spot alanı çok küçük: {area:.1f}")

    (x, y), radius = cv2.minEnclosingCircle(c)
    x, y = int(x), int(y)
    radius = int(radius)

    laser_mask_roi = mask[max(0, y-5):y+5, max(0, x-5):x+5]
    maxVal = float(laser_mask_roi.max()) if laser_mask_roi.size > 0 else 0.0

    return (x, y), radius, maxVal


# -------------------------------------------------
#  Basler kamera yardımcıları
# -------------------------------------------------

def set_exposure_basler(cam, exp_us):
    """Basler kamerada manuel exposure ayarla (µs)."""
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
    """Basler kamerada ROI ayarı (Width, Height, OffsetX, OffsetY)."""
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
    """İlk bulunan Basler kamerayı açar (önce GigE deniyor)."""
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
        print("GigE enumerate sonuçsuz, TlFactory.EnumerateDevices() deneniyor...")
        devices = tl_factory.EnumerateDevices()

    if not devices:
        raise RuntimeError("Hiç Basler kamera bulunamadı.")

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


# -------------------------------------------------
#  Seri port / gimbal fonksiyonları
# -------------------------------------------------

def init_serial():
    """Marlin kart ile seri haberleşmeyi başlat."""
    global ser
    if not SER_ENABLED:
        print("Seri port devre dışı (SER_ENABLED = False).")
        ser = None
        return

    try:
        #ser = serial.Serial(SER_MKS_PORT, cfg.BAUD_MKS, timeout=0.01)
        print(f"Seri port açıldı: {SER_MKS_PORT} @ {cfg.BAUD_MKS}")
        time.sleep(2.0)
        send_gcode("G91")  # Göreceli mod
    except Exception as e:
        print("Seri port açılamadı, sadece görüntü takibi:", e)
        ser = None


def send_gcode(cmd: str):
    """Tek satır G-code gönder, cevapları non-blocking oku."""
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


def move_laser_gimbal_relative(dx_mm, dy_mm):
    """Lazer gimbalını relatif modda hareket ettirir."""
    if abs(dx_mm) < 1e-6 and abs(dy_mm) < 1e-6:
        return

    cmd = f"G1 Z{dx_mm:.3f} A{dy_mm:.3f} F{cfg.FEEDRATE}\n"
    send_gcode(cmd)


def send_to_gimbal(az_deg, el_deg):
    """Kamera gimbali için açısal hatayı G-code'a çevirir."""
    global ser, current_x_mm, current_y_mm

    if ser is None:
        print(f"(DRY-RUN) az={az_deg:.3f} el={el_deg:.3f}  "
              f"[X={current_x_mm:.2f}mm Y={current_y_mm:.2f}mm]")
        return

    if abs(az_deg) < cfg.AZ_DEADBAND_DEG and abs(el_deg) < cfg.EL_DEADBAND_DEG:
        return

    step_x = -cfg.K_AZ_MM_PER_DEG * az_deg
    step_y =  cfg.K_EL_MM_PER_DEG * el_deg

    step_x = max(min(step_x, cfg.MAX_STEP_MM), -cfg.MAX_STEP_MM)
    step_y = max(min(step_y, cfg.MAX_STEP_MM), -cfg.MAX_STEP_MM)

    target_x = current_x_mm + step_x
    target_y = current_y_mm + step_y

    if target_x > cfg.X_MAX_MM:
        step_x = cfg.X_MAX_MM - current_x_mm
        target_x = cfg.X_MAX_MM
        print("⚠ X yazılımsal endstop (üst limit)!")
    elif target_x < cfg.X_MIN_MM:
        step_x = cfg.X_MIN_MM - current_x_mm
        target_x = cfg.X_MIN_MM
        print("⚠ X yazılımsal endstop (alt limit)!")

    if target_y > cfg.Y_MAX_MM:
        step_y = cfg.Y_MAX_MM - current_y_mm
        target_y = cfg.Y_MAX_MM
        print("⚠ Y yazılımsal endstop (üst limit)!")
    elif target_y < cfg.Y_MIN_MM:
        step_y = cfg.Y_MIN_MM - current_y_mm
        target_y = cfg.Y_MIN_MM
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
    cmd = "G1 " + " ".join(cmd_parts) + f" F{cfg.FEEDRATE}"
    send_gcode(cmd)

    current_x_mm += step_x
    current_y_mm += step_y

    print(f"[POS] X={current_x_mm:.2f}mm  Y={current_y_mm:.2f}mm")


# -------------------------------------------------
#  GUI: Telemetry + kamera gösterimi
# -------------------------------------------------



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
            img = img.resize((864, 301), Image.Resampling.LANCZOS)#1400x502px original resolution.
            self.logo_photo = ImageTk.PhotoImage(img)
            logo_label = tk.Label(logo_frame, image=self.logo_photo, bg="#2b2b2b")
            logo_label.pack()
        except Exception as e:
            print(f"Could not load logo: {e}")

        title_frame = tk.Frame(header_frame, bg="#2b2b2b")
        title_frame.pack(side=tk.LEFT, padx=450)

        title = tk.Label(
            title_frame,
            text="Gimbal Automation ",
            font=("Gotham", 36, "bold"),
            fg="#ffffff",
            bg="#2b2b2b"
        )
        title.pack(anchor=tk.W)

        subtitle = tk.Label(
            title_frame,
            text="Real-time camera and laser tracking, magnetic reader feedback system.",
            font=("Gotham", 10),
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
            font=("Gotham", 20, "bold"),
            fg="#ffffff",
            bg="#3a3a3a"
        )
        cam_title.pack(side=tk.LEFT)

        self.cam_status = tk.Label(
            cam_header,
            text="● Disconnected",
            font=("Gotham", 11),
            fg="#ef4444",
            bg="#3a3a3a"
        )
        self.cam_status.pack(side=tk.RIGHT)

        ttk.Separator(camera_container, orient='horizontal').pack(fill=tk.X, padx=15)

        self.camera_label = tk.Label(
            camera_container,
            bg="#2b2b2b",
            text="No Camera Feed",
            font=("Gotham", 14),
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
            font=("Gotham", 20, "bold"),
            fg="#ffffff",
            bg="#3a3a3a"
        )
        title.pack(side=tk.LEFT)

        status = tk.Label(
            header,
            text="● Disconnected",
            font=("Gotham", 11),
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
            font=("Gotham", 9),
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
            font=("Gotham", 11, "bold"),
            fg="#d1d5db",
            bg="#3a3a3a"
        )
        label.pack(side=tk.LEFT)

        count_label = tk.Label(
            label_frame,
            text="0",
            font=("Gotham", 14, "bold"),
            fg="#ffffff",
            bg="#3a3a3a"
        )
        count_label.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_{axis.lower()}_count', count_label)

        unit_label = tk.Label(
            label_frame,
            text="counts",
            font=("Gotham", 9),
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
            font=("Gotham", 12, "bold"),
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

        # Kamera görüntüsünü GUI'ye bas
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


# -------------------------------------------------
#  Arduino → GUI worker (manyetik cetvel)
# -------------------------------------------------

def parse_frame(frame: bytes):
    if len(frame) != FRAME_SIZE:
        return None
    if frame[0] != START_BYTE or frame[-1] != END_BYTE:
        return None

    node_id = frame[1]
    msg_type = frame[2]
    if msg_type != MSG_SENSOR:
        return None

    x = int.from_bytes(frame[3:7], "big", signed=True)
    y = int.from_bytes(frame[7:11], "big", signed=True)
    az = int.from_bytes(frame[11:15], "big", signed=True)
    return node_id, x, y, az


def arduino_worker(port: str, gui: TelemetryGUI):
    try:
        ser_a = serial.Serial(port=port, baudrate=BAUD_ARDUINO, timeout=0.1)
    except Exception as e:
        print(f"{port} error: {e}")
        return

    print(f"[{port}] Listening for sensor data...")
    buf = bytearray()

    while True:
        data = ser_a.read(64)
        if not data:
            continue
        buf.extend(data)

        while True:
            if len(buf) < FRAME_SIZE:
                break
            try:
                start_idx = buf.index(START_BYTE)
            except ValueError:
                buf.clear()
                break
            if start_idx > 0:
                del buf[:start_idx]
            if len(buf) < FRAME_SIZE:
                break

            frame = bytes(buf[:FRAME_SIZE])
            res = parse_frame(frame)
            if res is None:
                del buf[0]
                continue
            del buf[:FRAME_SIZE]

            node_id, x, y, az = res
            gui.update_node_data(node_id, x, y, az)


# -------------------------------------------------
#  Ana takip döngüsü (GUI’ye entegre)
# -------------------------------------------------

def track_star(gui: TelemetryGUI):
    cam = open_basler_camera()
    init_serial()

    pixel_format = "Unknown"
    is_bayer = False
    is_true_mono = False

    try:
        try:
            if cam.PixelFormat.IsWritable():
                enum_entries = cam.PixelFormat.GetSymbolics()
                print("Mevcut PixelFormat seçenekleri:", enum_entries)

                if "BGR8" in enum_entries:
                    cam.PixelFormat.SetValue("BGR8")
                    print("🎨 PixelFormat → BGR8")
                elif "RGB8Packed" in enum_entries:
                    cam.PixelFormat.SetValue("RGB8Packed")
                    print("🎨 PixelFormat → RGB8Packed")
                else:
                    print("⚠ BGR8/RGB8Packed yok, mevcut formatla devam.")
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

        set_roi_basler(cam, cfg.ROI_W, cfg.ROI_H, cfg.ROI_OFFX, cfg.ROI_OFFY)
        current_exp = set_exposure_basler(cam, 22600.0)

        try:
            if cam.AcquisitionMode.IsWritable():
                cam.AcquisitionMode.SetValue("Continuous")
                print("AcquisitionMode → Continuous")
        except Exception as e:
            print("AcquisitionMode ayarlanamadı:", e)

        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        print("Kamera grabbing başladı.")

        while cam.IsGrabbing():
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
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GB2BGR)
                    else:
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    detection_mode = "green"
                else:
                    print("⚠ Gerçek mono, renk tespiti yok; parlak daire moduna düşüyorum.")
                    gray_img = image
                    color_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
                    detection_mode = "mono"
            else:
                color_img = image
                detection_mode = "green"

            h, w = color_img.shape[:2]
            center = (w // 2, h // 2)

            try:
                if detection_mode == "green":
                    circle_center, radius_star, maxVal = detect_green_circle_center(color_img)
                else:
                    circle_center, radius_star, maxVal = detect_bright_circle_center_mono(color_img)
            except Exception as e:
                print(f"Nesne bulunamadı ({detection_mode}):", e)
                annotated = color_img.copy()
                cv2.drawMarker(
                    annotated, center, (0, 255, 0),
                    cv2.MARKER_CROSS, 20, 2
                )
                gui.update_camera_frame(annotated)
                continue

            cx, cy = circle_center
            dx = cx - center[0]
            dy = center[1] - cy

            az_deg, el_deg = pixels_to_angle(dx, dy)

            mode_icon = "🟢" if detection_mode == "green" else "⚪"
            print(
                f"{mode_icon} STAR mode={detection_mode} center={circle_center}  "
                f"Δx={dx:4d} Δy={dy:4d}  "
                f"az={az_deg:7.3f}° el={el_deg:7.3f}°  I={maxVal:.1f}"
            )

            send_to_gimbal(az_deg, el_deg)

            annotated = color_img.copy()
            cv2.drawMarker(
                annotated, center, (0, 255, 0),
                cv2.MARKER_CROSS, 20, 2
            )
            cv2.circle(annotated, (cx, cy), radius_star, (255, 0, 0), 2)
            cv2.circle(annotated, (cx, cy), 3, (0, 0, 255), -1)

            # Lazer takibi
            try:
                laser_center, radius_laser, laser_I = detect_laser_circle_center(color_img)
                lx, ly = laser_center

                dx_laser_px = cx - lx
                dy_laser_px = ly - cy

                az_laser_deg, el_laser_deg = pixels_to_angle(dx_laser_px, dy_laser_px)

                if abs(az_laser_deg) < LASER_DEADBAND_DEG:
                    az_laser_deg = 0.0
                if abs(el_laser_deg) < LASER_DEADBAND_DEG:
                    el_laser_deg = 0.0

                try:
                    move_laser_gimbal_relative(az_laser_deg, el_laser_deg)
                except NameError:
                    pass

                print(
                    f"🔴 LASER center={laser_center} "
                    f"Δx_laser={dx_laser_px:4d} Δy_laser={dy_laser_px:4d} "
                    f"az_L={az_laser_deg:7.3f}° el_L={el_laser_deg:7.3f}° I_L={laser_I:.1f}"
                )

                cv2.circle(annotated, (lx, ly), radius_laser, (0, 255, 255), 2)
                cv2.circle(annotated, (lx, ly), 3, (0, 255, 255), -1)

            except Exception:
                pass

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

            # GUI'ye frame gönder
            gui.update_camera_frame(annotated)

        print("Track loop bitti.")

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


# -------------------------------------------------
#  main
# -------------------------------------------------

def main():
    global SER_MKS_PORT
    global SER_ARD1_PORT
    global SER_ARD2_PORT

    # Tkinter GUI
    root = tk.Tk()
    gui = TelemetryGUI(root)

    # Seri portları tespit et
    mks_port, ard_ports = list_serial_ports()
    SER_MKS_PORT = mks_port
    if ard_ports:
        SER_ARD1_PORT = ard_ports[0]
    print("MKS port:", SER_MKS_PORT, "Arduino ports:", ard_ports)

    # Manyetik cetvel / Arduino thread’leri GUI’ye bağla
    for port in ard_ports:
        t = threading.Thread(target=arduino_worker, args=(port, gui), daemon=True)
        t.start()

    # Basler + gimbal tracking thread’i
    track_thread = threading.Thread(target=track_star, args=(gui,), daemon=True)
    track_thread.start()

    # GUI main loop
    root.mainloop()


if __name__ == "__main__":
    main()
