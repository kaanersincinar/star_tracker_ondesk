import time
import threading
import serial
import serial.tools.list_ports

# --- Sabitler ---

BAUD_MKS     = 250000
BAUD_ARDUINO = 115200

START_BYTE = 0x7E
END_BYTE   = 0x7F
MSG_SENSOR = 0x10
FRAME_SIZE = 16

# =====================================================
#  EKSEN BAZLI KALIBRASYON (senin ölçtüğün değerlere göre)
#  10 mm hareket -> X: 7459 count, Y: 6875 count
# =====================================================
X_MM_PER_COUNT  = 10.0 / 7459.0      # ≈ 0.0013407 mm/count
Y_MM_PER_COUNT  = 10.0 / 6875.0      # ≈ 0.0014545 mm/count
AZ_MM_PER_COUNT = 0.001              # şimdilik placeholder, ölçünce güncelle


# ----------------------------------------------------------
#  PORT ENUMERATION
# ----------------------------------------------------------

def list_candidate_ports():
    ports = []
    for p in serial.tools.list_ports.comports():
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            ports.append(p.device)
    return ports


# ----------------------------------------------------------
#  MKS TESPIT (250000 BAUD) + DEBUG
# ----------------------------------------------------------

def is_mks_port(port: str) -> bool:
    print(f"[DEBUG] {port}: MKS testi basladi (baud={BAUD_MKS})")
    try:
        with serial.Serial(port=port, baudrate=BAUD_MKS, timeout=0.5) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            try:
                ser.dtr = False
                time.sleep(0.05)
                ser.dtr = True
            except Exception:
                pass

            # 1) Boot mesajını dinle
            t0 = time.time()
            while time.time() - t0 < 2.0:
                line = ser.readline()
                if not line:
                    continue
                s = line.decode(errors="ignore").strip()
                if s:
                    print(f"[DEBUG] {port} RX(boot): {repr(s)}")
                low = s.lower()
                if ("marlin" in low or "mks" in low or
                    "firmware_name" in low or "start" in low):
                    print(f"[DEBUG] {port}: boot mesaji MKS gibi görünüyor.")
                    return True

            # 2) M115 gönder, cevap dinle
            cmd = b"M115\n"
            print(f"[DEBUG] {port} TX: {cmd!r}")
            ser.write(cmd)
            ser.flush()
            t1 = time.time()
            while time.time() - t1 < 2.0:
                line = ser.readline()
                if not line:
                    continue
                s = line.decode(errors="ignore").strip()
                if s:
                    print(f"[DEBUG] {port} RX(M115): {repr(s)}")
                low = s.lower()
                if ("marlin" in low or "mks" in low or
                    "firmware_name" in low or "ok" in low):
                    print(f"[DEBUG] {port}: M115 cevabi MKS gibi görünüyor.")
                    return True
    except Exception as e:
        print(f"[DEBUG] {port}: MKS testi hata: {e}")

    print(f"[DEBUG] {port}: MKS degil.")
    return False


def find_mks_port():
    candidates = list_candidate_ports()
    print("Aday portlar:", candidates)
    for port in candidates:
        print(f"{port} MKS kontrolü...")
        if is_mks_port(port):
            print(f"--> {port} MKS olarak tespit edildi.")
            return port
    print("MKS bulunamadı.")
    return None


# ----------------------------------------------------------
#  ARDUINO FRAME PARSE
# ----------------------------------------------------------

def parse_frame(frame: bytes):
    if len(frame) != FRAME_SIZE:
        return None
    if frame[0] != START_BYTE or frame[-1] != END_BYTE:
        return None

    node_id  = frame[1]
    msg_type = frame[2]
    if msg_type != MSG_SENSOR:
        return None

    x  = int.from_bytes(frame[3:7],   "big", signed=True)
    y  = int.from_bytes(frame[7:11],  "big", signed=True)
    az = int.from_bytes(frame[11:15], "big", signed=True)
    return node_id, x, y, az


def arduino_worker(port: str):
    try:
        ser = serial.Serial(port=port, baudrate=BAUD_ARDUINO, timeout=0.1)
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

            node_id, x_cnt, y_cnt, az_cnt = res

            # Eksene göre mm'ye çevir
            x_mm  = x_cnt  * X_MM_PER_COUNT
            y_mm  = y_cnt  * Y_MM_PER_COUNT
            az_mm = az_cnt * AZ_MM_PER_COUNT

            print(
                f"[{port}] NODE {node_id} | "
                f"X={x_cnt:7d} ({x_mm:8.3f} mm) | "
                f"Y={y_cnt:7d} ({y_mm:8.3f} mm) | "
                f"AZ={az_cnt:7d} ({az_mm:8.3f} mm)"
            )


# ----------------------------------------------------------
#  MAIN
# ----------------------------------------------------------

if __name__ == "__main__":
    mks_port = find_mks_port()
    if mks_port:
        print(f"\nMKS portu: {mks_port}")
    else:
        print("\nMKS bulunamadı, yalnızca Arduino'ları dinliyorum.")

    all_ports = list_candidate_ports()
    arduino_ports = [p for p in all_ports if p != mks_port]
    print("Arduino portları (115200 baud):", arduino_ports)

    threads = []
    for port in arduino_ports:
        t = threading.Thread(target=arduino_worker, args=(port,), daemon=True)
        t.start()
        threads.append(t)

    print("\nSensör verisi akışı başladı. Ctrl+C ile durdur.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nDurdu.")
