import os
import tkinter as tk
from tkinter import ttk
import threading
import time
import serial
import serial.tools.list_ports
from datetime import datetime
import cv2
from PIL import Image, ImageTk
import numpy as np
import math

try:
    from pypylon import pylon
    PYLON_AVAILABLE = True
except ImportError:
    PYLON_AVAILABLE = False
    print("⚠ pypylon not available - camera features disabled")

# --- Calibration Constants ---
X_MM_PER_COUNT = 10.0 / 7459.0
Y_MM_PER_COUNT = 10.0 / 6875.0
AZ_MM_PER_COUNT = 0.001

# --- Protocol Constants ---
BAUD_ARDUINO = 115200
START_BYTE = 0x7E
END_BYTE = 0x7F
MSG_SENSOR = 0x10
FRAME_SIZE = 16

# --- Camera Constants ---
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

# --- Laser Detection HSV Range (Blue) ---
LASER_BLUE_LOWER = (90, 80, 120)
LASER_BLUE_UPPER = (140, 255, 255)
MIN_AREA_LASER = 5

# Logo path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOGO_PATH = os.path.join(BASE_DIR, "UZAY-Yatay-Beyaz.png")


# --- Detection Functions ---

def pixels_to_angle(delta_x, delta_y):
    """Convert pixel offset (dx, dy) to angle (degrees) → (azimuth, elevation)."""
    dx_mm = delta_x * PIXEL_SIZE_MM
    dy_mm = delta_y * PIXEL_SIZE_MM
    theta_x = math.degrees(math.atan(dx_mm / FOCAL_LENGTH_MM))
    theta_y = math.degrees(math.atan(dy_mm / FOCAL_LENGTH_MM))
    return theta_x, theta_y


def find_white_points(img_bgr, min_area=3, high_thr=200):
    """
    Sadece beyaz / nötr parlak spotları bul.
    Mavi lazeri (LASER_BLUE_LOWER/UPPER aralığı) ve mavi-dominant pikselleri eler.
    """
    # 1) Griye çevir
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    # 2) Yüksek threshold: sadece en parlak pikseller
    _, bright_mask = cv2.threshold(gray, high_thr, 255, cv2.THRESH_BINARY)

    # 3) HSV uzayında mavi (lazer) maskesi oluştur
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(
        hsv,
        np.array(LASER_BLUE_LOWER, dtype=np.uint8),
        np.array(LASER_BLUE_UPPER, dtype=np.uint8)
    )
    # Lazer noktasının etrafını da kapatsın diye hafif genişlet
    kernel_blue = np.ones((3, 3), np.uint8)
    blue_mask = cv2.dilate(blue_mask, kernel_blue, iterations=1)

    # 4) Parlak ama mavi olmayan bölgeleri bırak
    mask = cv2.bitwise_and(bright_mask, cv2.bitwise_not(blue_mask))

    # 5) Küçük gürültüleri temizle
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # 6) Kontur bul
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    points = []
    h, w = img_bgr.shape[:2]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        if not (0 <= cx < w and 0 <= cy < h):
            continue

        # 7) Emin olmak için BGR bazlı mavi-dominant kontrolü
        b, g, r = img_bgr[cy, cx]
        if b > r + 15 and b > g + 15:
            # Bu nokta mavi ağırlıklı → lazer / mavi parıltı → at
            continue

        radius = int(math.sqrt(area / math.pi))
        points.append((cx, cy, radius, area))

    return points, mask





def detect_laser_circle_center(color_img):
    """Detect blue laser spot in HSV space."""
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LASER_BLUE_LOWER, LASER_BLUE_UPPER)
    mask = cv2.medianBlur(mask, 5)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("Blue laser spot not found")
    
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA_LASER:
        raise RuntimeError(f"Laser spot area too small: {area:.1f}")
    
    (x, y), radius = cv2.minEnclosingCircle(c)
    return (int(x), int(y)), int(radius)


class TelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Magnetic Ruler + Camera Dashboard")
        self.root.geometry("1600x900")
        self.root.configure(bg="#e5e5e5")
        
        # Data storage
        self.node_data = {
            1: {'x': 0, 'y': 0, 'az': 0, 'last_update': None, 'active': False},
            2: {'x': 0, 'y': 0, 'az': 0, 'last_update': None, 'active': False}
        }
        
        # Camera data
        self.camera_frame = None
        self.camera_active = False
        self.camera_lock = threading.Lock()
        
        self.create_widgets()
        self.update_display()
        
    def create_widgets(self):
        # Header with logo and title
        header_frame = tk.Frame(self.root, bg="#2b2b2b")
        header_frame.pack(fill=tk.X, pady=10)
        
        # Logo on the left
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
        
        # Title on the right of logo
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
        
        # Main container
        main_container = tk.Frame(self.root, bg="#2b2b2b")
        main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Left side: Telemetry nodes
        telemetry_container = tk.Frame(main_container, bg="#2b2b2b")
        telemetry_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Node cards in a grid
        nodes_frame = tk.Frame(telemetry_container, bg="#2b2b2b")
        nodes_frame.pack(fill=tk.BOTH, expand=True)
        
        self.node1_frame = self.create_node_card(nodes_frame, 1)
        self.node1_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.node2_frame = self.create_node_card(nodes_frame, 2)
        self.node2_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        
        nodes_frame.grid_rowconfigure(0, weight=1)
        nodes_frame.grid_rowconfigure(1, weight=1)
        nodes_frame.grid_columnconfigure(0, weight=1)
        
        # Right side: Camera feed
        camera_container = tk.Frame(main_container, bg="#3a3a3a", 
                                    relief=tk.RAISED, borderwidth=2)
        camera_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # Camera header
        cam_header = tk.Frame(camera_container, bg="#3a3a3a")
        cam_header.pack(fill=tk.X, padx=15, pady=15)
        
        cam_title = tk.Label(cam_header, text="Camera Feed", 
                            font=("Arial", 20, "bold"), fg="#ffffff", bg="#3a3a3a")
        cam_title.pack(side=tk.LEFT)
        
        self.cam_status = tk.Label(cam_header, text="● Disconnected", 
                                   font=("Arial", 11), fg="#ef4444", bg="#3a3a3a")
        self.cam_status.pack(side=tk.RIGHT)
        
        ttk.Separator(camera_container, orient='horizontal').pack(fill=tk.X, padx=15)
        
        # Camera display
        self.camera_label = tk.Label(
            camera_container,
            bg="#2b2b2b", 
            text="No Camera Feed", 
            font=("Arial", 14),
            fg="#6b7280"
        )
        self.camera_label.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
    
    def create_node_card(self, parent, node_id):
        # Main card frame
        card = tk.Frame(parent, bg="#3a3a3a", relief=tk.RAISED, borderwidth=2)
        
        # Header
        header = tk.Frame(card, bg="#3a3a3a")
        header.pack(fill=tk.X, padx=15, pady=15)

        # Node labels
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
        
        # Separator
        ttk.Separator(card, orient='horizontal').pack(fill=tk.X, padx=15)
        
        # Axes data
        axes_frame = tk.Frame(card, bg="#3a3a3a")
        axes_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        # X, Y, AZ Axes
        self.create_axis_display(axes_frame, node_id, 'X', '#3b82f6', 0)
        self.create_axis_display(axes_frame, node_id, 'Y', '#10b981', 1)
        self.create_axis_display(axes_frame, node_id, 'AZ', '#f59e0b', 2)
        
        # Last update
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
        
        # Label and value
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
        
        # Progress bar
        progress_frame = tk.Frame(frame, bg="#505050", height=12)
        progress_frame.pack(fill=tk.X, pady=5)
        progress_frame.pack_propagate(False)
        
        progress_bar = tk.Frame(progress_frame, bg=color, width=0, height=12)
        progress_bar.place(relx=0.5, rely=0, anchor='n')
        setattr(self, f'node{node_id}_{axis.lower()}_bar', progress_bar)
        
        # MM value
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
        # Update telemetry nodes
        for node_id in [1, 2]:
            data = self.node_data[node_id]
            
            # Update status
            status_label = getattr(self, f'node{node_id}_status')
            if data['active'] and data['last_update']:
                time_diff = time.time() - data['last_update']
                if time_diff < 1.0:
                    status_label.config(text="● Connected", fg="#10b981")
                else:
                    status_label.config(text="● Disconnected", fg="#ef4444")
            else:
                status_label.config(text="● Disconnected", fg="#ef4444")
            
            # Update axes
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
            
            # Update timestamp
            if data['last_update']:
                timestamp = datetime.fromtimestamp(data['last_update']).strftime('%H:%M:%S')
                update_label = getattr(self, f'node{node_id}_update')
                update_label.config(text=f"Last update: {timestamp}")
        
        # Update camera feed
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


# --- Arduino Communication ---

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
        ser = serial.Serial(port=port, baudrate=BAUD_ARDUINO, timeout=0.1)
    except Exception as e:
        print(f"{port} error: {e}")
        return
    
    print(f"[{port}] Listening for sensor data...")
    buf = bytearray()
    
    while True:
        data = ser.read(64)
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


# --- Camera Worker with Tracking ---

def camera_worker(gui: TelemetryGUI):
    if not PYLON_AVAILABLE:
        print("⚠ Camera worker skipped - pypylon not available")
        return
    
    try:
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        
        if not devices:
            print("⚠ No Basler camera found")
            return
        
        cam = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
        cam.Open()
        
        print(f"✅ Camera opened: {cam.GetDeviceInfo().GetModelName()}")
        
        # Detect pixel format
        pixel_format = "Unknown"
        is_bayer = False
        is_true_mono = False
        
        try:
            pixel_format = cam.PixelFormat.GetValue()
            print("Active PixelFormat:", pixel_format)
            if pixel_format.startswith("Bayer"):
                is_bayer = True
            if pixel_format.startswith("Mono"):
                is_true_mono = True
        except Exception as e:
            print("Could not read PixelFormat:", e)
        
        # Set to continuous mode
        cam.AcquisitionMode.SetValue("Continuous")
        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        
        while cam.IsGrabbing():
            try:
                grabResult = cam.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
                
                if grabResult.GrabSucceeded():
                    image = grabResult.Array

                    # --- 1) Her durumda BGR frame üret ---
                    if len(image.shape) == 2:
                        if is_bayer:
                            if pixel_format == "BayerRG8":
                                frame = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                            elif pixel_format == "BayerBG8":
                                frame = cv2.cvtColor(image, cv2.COLOR_BAYER_BG2BGR)
                            elif pixel_format == "BayerGR8":
                                frame = cv2.cvtColor(image, cv2.COLOR_BAYER_GR2BGR)
                            elif pixel_format == "BayerGB8":
                                frame = cv2.cvtColor(image, cv2.COLOR_BAYER_GB2BGR)
                            else:
                                frame = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                        else:
                            # gerçek mono -> BGR'ye kopyala
                            frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    else:
                        # Zaten BGR / RGB benzeri
                        frame = image

                    h, w = frame.shape[:2]
                    center = (w // 2, h // 2)

                    # Merkez crosshair
                    cv2.drawMarker(frame, center, (0, 255, 0),
                                   cv2.MARKER_CROSS, 20, 2)

                    # --- 2) Beyaz spot tespiti (TEK fonksiyon) ---
                    points, debug_mask = find_white_points(frame, min_area=3, high_thr=200)
                    # Debug için istersen aç:
                    # cv2.imshow("mask", debug_mask); cv2.waitKey(1)

                    if points:
                        # Draw all detected points
                        for (px, py, r, a) in points:
                            cv2.circle(frame, (px, py), max(r, 3), (255, 0, 0), 1)
                        
                        # Find closest point to center as target
                        closest = min(points, key=lambda p: math.hypot(p[0]-center[0], p[1]-center[1]))
                        cx, cy, radius, area = closest
                        
                        # Highlight the tracked target
                        cv2.circle(frame, (cx, cy), max(radius, 5), (0, 0, 255), 2)
                        cv2.circle(frame, (cx, cy), 3, (0, 255, 255), -1)
                        
                        # Calculate offset
                        dx = cx - center[0]
                        dy = center[1] - cy
                        az_deg, el_deg = pixels_to_angle(dx, dy)
                        err_pix = math.hypot(dx, dy)
                        
                        # Display tracking info
                        cv2.putText(frame, f"dx={dx}px dy={dy}px",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.7, (255, 255, 255), 2)
                        cv2.putText(frame, f"az={az_deg:.2f} el={el_deg:.2f}",
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0, 255, 255), 2)
                        cv2.putText(frame, f"Points: {len(points)} | err={err_pix:.1f}px",
                                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.5, (200, 200, 200), 1)
                    else:
                        cv2.putText(frame, "No targets detected",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0, 0, 255), 2)
                    
                    gui.update_camera_frame(frame)
                
                grabResult.Release()
                
            except Exception as e:
                print(f"Camera grab error: {e}")
                time.sleep(0.1)
        
        cam.StopGrabbing()
        cam.Close()
        
    except Exception as e:
        print(f"Camera worker error: {e}")


def list_candidate_ports():
    ports = []
    for p in serial.tools.list_ports.comports():
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            ports.append(p.device)
    return ports


def main():
    root = tk.Tk()
    gui = TelemetryGUI(root)
    
    # Start Arduino communication threads
    arduino_ports = list_candidate_ports()
    print("Arduino ports:", arduino_ports)
    
    for port in arduino_ports:
        t = threading.Thread(target=arduino_worker, args=(port, gui), daemon=True)
        t.start()
    
    # Start camera thread
    camera_thread = threading.Thread(target=camera_worker, args=(gui,), daemon=True)
    camera_thread.start()
    
    root.mainloop()


if __name__ == "__main__":
    main()