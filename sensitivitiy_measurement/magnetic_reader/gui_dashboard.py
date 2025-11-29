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


class TelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Magnetic Ruler + Camera Dashboard")
        self.root.geometry("1400x900")
        self.root.configure(bg="#1a1a2e")
        
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
        header_frame = tk.Frame(self.root, bg="#1a1a2e")
        header_frame.pack(fill=tk.X, pady=10)
        
        # Logo on the left
        logo_frame = tk.Frame(header_frame, bg="#1a1a2e")
        logo_frame.pack(side=tk.LEFT, padx=20)
        
        try:
            from PIL import Image, ImageTk
            
            # Load local PNG logo
            img = Image.open("logo.png")
            img = img.resize((100, 100), Image.Resampling.LANCZOS)
            
            # Create a background with the same color as the frame
            background = Image.new('RGB', (100, 100), '#1a1a2e')
            
            # Composite the logo onto the background (handles transparency)
            if img.mode == 'RGBA':
                background.paste(img, (0, 0), img)
            else:
                background.paste(img, (0, 0))
            
            self.logo_photo = ImageTk.PhotoImage(background)
            
            logo_label = tk.Label(logo_frame, image=self.logo_photo, bg="#1a1a2e")
            logo_label.pack()
        except Exception as e:
            print(f"Could not load logo: {e}")
        
        # Title on the right of logo
        title_frame = tk.Frame(header_frame, bg="#1a1a2e")
        title_frame.pack(side=tk.LEFT, padx=10)
        
        title = tk.Label(title_frame, text="Magnetic Ruler + Camera Dashboard", 
                        font=("Arial", 22, "bold"), fg="#ffffff", bg="#1a1a2e")
        title.pack(anchor=tk.W)
        
        subtitle = tk.Label(title_frame, text="Real-time sensor and camera monitoring system",
                          font=("Arial", 11), fg="#9ca3af", bg="#1a1a2e")
        subtitle.pack(anchor=tk.W)
        
        # Main container
        main_container = tk.Frame(self.root, bg="#1a1a2e")
        main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Left side: Telemetry nodes
        telemetry_container = tk.Frame(main_container, bg="#1a1a2e")
        telemetry_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Node cards in a grid
        nodes_frame = tk.Frame(telemetry_container, bg="#1a1a2e")
        nodes_frame.pack(fill=tk.BOTH, expand=True)
        
        self.node1_frame = self.create_node_card(nodes_frame, 1)
        self.node1_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.node2_frame = self.create_node_card(nodes_frame, 2)
        self.node2_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        
        nodes_frame.grid_rowconfigure(0, weight=1)
        nodes_frame.grid_rowconfigure(1, weight=1)
        nodes_frame.grid_columnconfigure(0, weight=1)
        
        # Right side: Camera feed
        camera_container = tk.Frame(main_container, bg="#2d2d44", 
                                    relief=tk.RAISED, borderwidth=2)
        camera_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # Camera header
        cam_header = tk.Frame(camera_container, bg="#2d2d44")
        cam_header.pack(fill=tk.X, padx=15, pady=15)
        
        cam_title = tk.Label(cam_header, text="Camera Feed", 
                            font=("Arial", 18, "bold"), fg="#ffffff", bg="#2d2d44")
        cam_title.pack(side=tk.LEFT)
        
        self.cam_status = tk.Label(cam_header, text="● Disconnected", 
                                   font=("Arial", 11), fg="#ef4444", bg="#2d2d44")
        self.cam_status.pack(side=tk.RIGHT)
        
        ttk.Separator(camera_container, orient='horizontal').pack(fill=tk.X, padx=15)
        
        # Camera display
        self.camera_label = tk.Label(camera_container, bg="#1a1a2e", 
                                     text="No Camera Feed", 
                                     font=("Arial", 14), fg="#6b7280")
        self.camera_label.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
    
    def create_node_card(self, parent, node_id):
        # Main card frame - more compact
        card = tk.Frame(parent, bg="#2d2d44", relief=tk.RAISED, borderwidth=2)
        
        # Header
        header = tk.Frame(card, bg="#2d2d44")
        header.pack(fill=tk.X, padx=10, pady=10)
        
        title = tk.Label(header, text=f"Node {node_id}", 
                        font=("Arial", 16, "bold"), fg="#ffffff", bg="#2d2d44")
        title.pack(side=tk.LEFT)
        
        status = tk.Label(header, text="● Disconnected", 
                         font=("Arial", 10), fg="#ef4444", bg="#2d2d44")
        status.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_status', status)
        
        # Separator
        ttk.Separator(card, orient='horizontal').pack(fill=tk.X, padx=10)
        
        # Axes data - more compact
        axes_frame = tk.Frame(card, bg="#2d2d44")
        axes_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # X Axis
        self.create_axis_display(axes_frame, node_id, 'X', '#3b82f6', 0)
        # Y Axis
        self.create_axis_display(axes_frame, node_id, 'Y', '#10b981', 1)
        # AZ Axis
        self.create_axis_display(axes_frame, node_id, 'AZ', '#f59e0b', 2)
        
        # Last update
        update_label = tk.Label(card, text="Last update: --:--:--", 
                               font=("Arial", 8), fg="#6b7280", bg="#2d2d44")
        update_label.pack(pady=8)
        setattr(self, f'node{node_id}_update', update_label)
        
        return card
    
    def create_axis_display(self, parent, node_id, axis, color, row):
        frame = tk.Frame(parent, bg="#2d2d44")
        frame.pack(fill=tk.X, pady=5)
        
        # Label and value
        label_frame = tk.Frame(frame, bg="#2d2d44")
        label_frame.pack(fill=tk.X)
        
        label = tk.Label(label_frame, text=f"{axis}-Axis", 
                        font=("Arial", 10, "bold"), fg="#d1d5db", bg="#2d2d44")
        label.pack(side=tk.LEFT)
        
        count_label = tk.Label(label_frame, text="0", 
                              font=("Arial", 12, "bold"), fg="#ffffff", bg="#2d2d44")
        count_label.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_{axis.lower()}_count', count_label)
        
        unit_label = tk.Label(label_frame, text="counts", 
                             font=("Arial", 8), fg="#9ca3af", bg="#2d2d44")
        unit_label.pack(side=tk.RIGHT, padx=(5, 10))
        
        # Progress bar
        progress_frame = tk.Frame(frame, bg="#374151", height=10)
        progress_frame.pack(fill=tk.X, pady=3)
        progress_frame.pack_propagate(False)
        
        progress_bar = tk.Frame(progress_frame, bg=color, width=0, height=10)
        progress_bar.place(relx=0.5, rely=0, anchor='n')
        setattr(self, f'node{node_id}_{axis.lower()}_bar', progress_bar)
        
        # MM value
        mm_label = tk.Label(frame, text="0.000 mm", 
                           font=("Arial", 11, "bold"), fg=color, bg="#2d2d44")
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
                
                # Update count
                count_label = getattr(self, f'node{node_id}_{axis}_count')
                count_label.config(text=f"{count:,}")
                
                # Update mm
                mm_label = getattr(self, f'node{node_id}_{axis}_mm')
                mm_label.config(text=f"{mm:.3f} mm")
                
                # Update progress bar
                bar = getattr(self, f'node{node_id}_{axis}_bar')
                max_count = 10000
                percentage = min(abs(count) / max_count, 1.0)
                bar_width = int(percentage * 200)
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
                    # Convert BGR to RGB
                    frame_rgb = cv2.cvtColor(self.camera_frame, cv2.COLOR_BGR2RGB)
                    # Resize to fit display
                    h, w = frame_rgb.shape[:2]
                    max_h, max_w = 600, 600
                    scale = min(max_w/w, max_h/h)
                    new_w, new_h = int(w*scale), int(h*scale)
                    frame_resized = cv2.resize(frame_rgb, (new_w, new_h))
                    
                    # Convert to ImageTk
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


# --- Camera Worker ---

def camera_worker(gui: TelemetryGUI):
    if not PYLON_AVAILABLE:
        print("⚠ Camera worker skipped - pypylon not available")
        return
    
    try:
        # Open Basler camera
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        
        if not devices:
            print("⚠ No Basler camera found")
            return
        
        cam = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
        cam.Open()
        
        print(f"✅ Camera opened: {cam.GetDeviceInfo().GetModelName()}")
        
        # Set to continuous mode
        cam.AcquisitionMode.SetValue("Continuous")
        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        
        while cam.IsGrabbing():
            try:
                grabResult = cam.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
                
                if grabResult.GrabSucceeded():
                    image = grabResult.Array
                    
                    # Convert to BGR if needed
                    if len(image.shape) == 2:
                        frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    else:
                        frame = image
                    
                    # Draw crosshair at center
                    h, w = frame.shape[:2]
                    center = (w // 2, h // 2)
                    cv2.drawMarker(frame, center, (0, 255, 0), 
                                  cv2.MARKER_CROSS, 20, 2)
                    
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