import serial
import time

PORT = "COM3"      # Linux: "/dev/ttyUSB0"
BAUD = 250000      # Configuration.h'daki BAUDRATE ile aynı olmalı

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Kart resetten kalksın

def send(cmd: str):
    print(">>", cmd)
    ser.write((cmd + "\n").encode("ascii"))
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            break
        print("<<", line)

# 1) FW ile handshake
send("M115")        # Firmware bilgisi gelsin

# 2) Extruder (E0) korumasını bypass et
# Config'te PREVENT_COLD_EXTRUSION açık ve EXTRUDE_MINTEMP 170°C. :contentReference[oaicite:2]{index=2}
# E eksenini sıcaklık olmadan çevirmek için:
send("M302 S0")     # Her sıcaklıkta E hareketine izin ver

# 3) Konum modlarını set et
send("G90")         # X,Y,Z için absolute mode
send("M82")         # E için absolute mode (opsiyonel, istersen M83 ile relative de kullanabilirsin)

# 4) Basit hareket testleri
send("G1 X10 F3000")   # X ekseni 10mm'ye git
send("G1 Y20 F3000")   # Y ekseni 20mm'ye git
send("G1 Z5  F600")    # Z ekseni 5mm'ye git
send("G1 E10 F1200")   # E0 motorunu 10 "mm" extrude et (sen bunu 4. eksen gibi kullanıyorsun)

# 5) Relative mode ile jog yapmak istersen:
send("G91")            # Relative
send("G1 X5 F3000")    # X'i +5mm
send("G1 E-5 F1200")   # E'yi -5 "mm"
