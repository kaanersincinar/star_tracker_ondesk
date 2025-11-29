import time
import threading
import serial
import serial.tools.list_ports
import re
import config as cfg
from port_listener import find_mks_port,list_candidate_ports
#-----------------------------------------------------------
#  ARDUINO FRAME PARSE
# ----------------------------------------------------------

def parse_frame(frame: bytes):
    if len(frame) != cfg.FRAME_SIZE:
        return None
    if frame[0] != cfg.START_BYTE or frame[-1] != cfg.END_BYTE:
        return None

    node_id  = frame[1]
    msg_type = frame[2]
    if msg_type != cfg.MSG_SENSOR:
        return None

    x  = int.from_bytes(frame[3:7],   "big", signed=True)
    y  = int.from_bytes(frame[7:11],  "big", signed=True)
    az = int.from_bytes(frame[11:15], "big", signed=True)
    return node_id, x, y, az

def arduino_worker(port: str):
    try:
        ser = serial.Serial(port=port, baudrate=cfg.BAUD_ARDUINO, timeout=0.1)
    except Exception as e:
        print(f"{port} açılırken hata: {e}")
        return

    print(f"[{port}] Arduino sensör verisi dinleniyor...")
    buf = bytearray()

    while True:
        data = ser.read(64)
        if not data:
            continue
        buf.extend(data)

        while True:
            if len(buf) < cfg.FRAME_SIZE:
                break
            try:
                start_idx = buf.index(cfg.START_BYTE)
            except ValueError:
                buf.clear()
                break
            if start_idx > 0:
                del buf[:start_idx]
            if len(buf) < cfg.FRAME_SIZE:
                break

            frame = bytes(buf[:cfg.FRAME_SIZE])
            res = parse_frame(frame)
            if res is None:
                del buf[0]
                continue
            del buf[:cfg.FRAME_SIZE]

            node_id, x_cnt, y_cnt, az_cnt = res

            # Node / eksen bazlı mm dönüşümü
            if node_id == 1:
                x_mm = cfg.X1_K * x_cnt + cfg.X1_B
                y_mm = cfg.Y1_K * y_cnt + cfg.Y1_B
            elif node_id == 2:
                x_mm = cfg.X2_K * x_cnt + cfg.X2_B
                y_mm = cfg.Y2_K * y_cnt + cfg.Y2_B
            else:
                x_mm = float("nan")
                y_mm = float("nan")

            az_mm = cfg.AZ_K * az_cnt + cfg.AZ_B

            print(
                f"[{port}] NODE {node_id} | "
                f"X={x_cnt:7d} ({x_mm:8.3f} mm) | "
                f"Y={y_cnt:7d} ({y_mm:8.3f} mm) | "
                f"AZ={az_cnt:7d} ({az_mm:8.3f} mm)"
            )



