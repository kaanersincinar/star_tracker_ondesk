import time
import threading
import serial
import serial.tools.list_ports

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
#  MAIN
# ----------------------------------------------------------

def list_serial_ports():
    mks_port = find_mks_port()
    if mks_port:
        print(f"\nMKS portu: {mks_port}")
    else:
        print("\nMKS bulunamadı, yalnızca Arduino'ları dinliyorum.")

    all_ports = list_candidate_ports()
    arduino_ports = [p for p in all_ports if p != mks_port]
    print("Arduino portları (115200 baud):", arduino_ports)
    return mks_port, arduino_ports
