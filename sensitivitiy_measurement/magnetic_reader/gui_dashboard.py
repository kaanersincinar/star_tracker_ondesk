import tkinter as tk
from tkinter import ttk
import threading
import time
import serial
import serial.tools.list_ports
from datetime import datetime

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


class TelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Magnetic Ruler Dashboard")
        self.root.geometry("1200x700")
        self.root.configure(bg="#1a1a2e")
        
        # Data storage
        self.node_data = {
            1: {'x': 0, 'y': 0, 'az': 0, 'last_update': None, 'active': False},
            2: {'x': 0, 'y': 0, 'az': 0, 'last_update': None, 'active': False}
        }
        
        self.create_widgets()
        self.update_display()
        
    def create_widgets(self):
        # Title
        title_frame = tk.Frame(self.root, bg="#1a1a2e")
        title_frame.pack(pady=20)
        
        title = tk.Label(title_frame, text="Magnetic Ruler Dashboard", 
                        font=("Arial", 28, "bold"), fg="#ffffff", bg="#1a1a2e")
        title.pack()
        
        subtitle = tk.Label(title_frame, text="Real-time sensor monitoring system",
                          font=("Arial", 12), fg="#9ca3af", bg="#1a1a2e")
        subtitle.pack()
        
        # Container for both nodes
        container = tk.Frame(self.root, bg="#1a1a2e")
        container.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Create node cards
        self.node1_frame = self.create_node_card(container, 1)
        self.node1_frame.grid(row=0, column=0, padx=10, sticky="nsew")
        
        self.node2_frame = self.create_node_card(container, 2)
        self.node2_frame.grid(row=0, column=1, padx=10, sticky="nsew")
        
        container.grid_columnconfigure(0, weight=1)
        container.grid_columnconfigure(1, weight=1)
        container.grid_rowconfigure(0, weight=1)
    
    def create_node_card(self, parent, node_id):
        # Main card frame
        card = tk.Frame(parent, bg="#2d2d44", relief=tk.RAISED, borderwidth=2)
        
        # Header
        header = tk.Frame(card, bg="#2d2d44")
        header.pack(fill=tk.X, padx=15, pady=15)
        
        title = tk.Label(header, text=f"Node {node_id}", 
                        font=("Arial", 20, "bold"), fg="#ffffff", bg="#2d2d44")
        title.pack(side=tk.LEFT)
        
        status = tk.Label(header, text="● Disconnected", 
                         font=("Arial", 11), fg="#ef4444", bg="#2d2d44")
        status.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_status', status)
        
        # Separator
        ttk.Separator(card, orient='horizontal').pack(fill=tk.X, padx=15)
        
        # Axes data
        axes_frame = tk.Frame(card, bg="#2d2d44")
        axes_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        # X Axis
        self.create_axis_display(axes_frame, node_id, 'X', '#3b82f6', 0)
        # Y Axis
        self.create_axis_display(axes_frame, node_id, 'Y', '#10b981', 1)
        # AZ Axis
        self.create_axis_display(axes_frame, node_id, 'AZ', '#f59e0b', 2)
        
        # Last update
        update_label = tk.Label(card, text="Last update: --:--:--", 
                               font=("Arial", 9), fg="#6b7280", bg="#2d2d44")
        update_label.pack(pady=10)
        setattr(self, f'node{node_id}_update', update_label)
        
        return card
    
    def create_axis_display(self, parent, node_id, axis, color, row):
        frame = tk.Frame(parent, bg="#2d2d44")
        frame.pack(fill=tk.X, pady=8)
        
        # Label and value
        label_frame = tk.Frame(frame, bg="#2d2d44")
        label_frame.pack(fill=tk.X)
        
        label = tk.Label(label_frame, text=f"{axis}-Axis", 
                        font=("Arial", 11, "bold"), fg="#d1d5db", bg="#2d2d44")
        label.pack(side=tk.LEFT)
        
        count_label = tk.Label(label_frame, text="0", 
                              font=("Arial", 14, "bold"), fg="#ffffff", bg="#2d2d44")
        count_label.pack(side=tk.RIGHT)
        setattr(self, f'node{node_id}_{axis.lower()}_count', count_label)
        
        unit_label = tk.Label(label_frame, text="counts", 
                             font=("Arial", 9), fg="#9ca3af", bg="#2d2d44")
        unit_label.pack(side=tk.RIGHT, padx=(5, 10))
        
        # Progress bar
        progress_frame = tk.Frame(frame, bg="#374151", height=12)
        progress_frame.pack(fill=tk.X, pady=5)
        progress_frame.pack_propagate(False)
        
        progress_bar = tk.Frame(progress_frame, bg=color, width=0, height=12)
        progress_bar.place(relx=0.5, rely=0, anchor='n')
        setattr(self, f'node{node_id}_{axis.lower()}_bar', progress_bar)
        
        # MM value
        mm_label = tk.Label(frame, text="0.000 mm", 
                           font=("Arial", 12, "bold"), fg=color, bg="#2d2d44")
        mm_label.pack(anchor=tk.E)
        setattr(self, f'node{node_id}_{axis.lower()}_mm', mm_label)
    
    def update_display(self):
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
                bar_width = int(percentage * 250)
                bar.config(width=bar_width)
            
            # Update timestamp
            if data['last_update']:
                timestamp = datetime.fromtimestamp(data['last_update']).strftime('%H:%M:%S')
                update_label = getattr(self, f'node{node_id}_update')
                update_label.config(text=f"Last update: {timestamp}")
        
        self.root.after(100, self.update_display)
    
    def update_node_data(self, node_id, x, y, az):
        self.node_data[node_id]['x'] = x
        self.node_data[node_id]['y'] = y
        self.node_data[node_id]['az'] = az
        self.node_data[node_id]['last_update'] = time.time()
        self.node_data[node_id]['active'] = True


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
    
    root.mainloop()


if __name__ == "__main__":
    main()