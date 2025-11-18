import serial
import time

# --- Seri port ayarları ---
PORT = '/dev/ttyUSB2'        # Doğru port
BAUD = 250000        # Marlin çoğunlukla 250000, sende 115200 ise onu yaz

# Seri portu aç
ser = serial.Serial(PORT, BAUD, timeout=1)

LONG_DELAY = 6
SHORT_DELAY = 3
# Kart reset olup hazır olsun diye 2 sn bekle
time.sleep(1)

# Göreceli konum moduna geç (her komut önceki pozisyona göre)
ser.write(b'G91\n')
for i in range(50):
    # 1. nokta Kamera
    ser.write(b'G1 X27.5 Y-9.4 F450\n')
    time.sleep(SHORT_DELAY)
    # 1. nokta Lazer
    ser.write(b'G1 Z30.5 A9.4 F450\n')
    time.sleep(LONG_DELAY)

    # 2. nokta Kamera
    ser.write(b'G1 X-25.8 Y29.3 F450\n')
    time.sleep(SHORT_DELAY)
    # 2. nokta Lazer
    ser.write(b'G1 Z-29.0 A-30.9 F450\n')
    time.sleep(LONG_DELAY)

    # 3. nokta Kamera
    ser.write(b'G1 X-35.5 Y-9.5 F450\n')
    time.sleep(SHORT_DELAY)
    # 3. nokta Lazer
    ser.write(b'G1 Z-33.5 A12.3 F450\n')
    time.sleep(LONG_DELAY)

    # 4. nokta Kamera
    ser.write(b'G1 X31.5 Y-9.5 F450\n')
    time.sleep(SHORT_DELAY)
    # 4. nokta Lazer
    ser.write(b'G1 Z29.5 A7.8 F450\n')
    time.sleep(LONG_DELAY)

    # 5. nokta Kamera
    ser.write(b'G1 X34.5 Y10.7 F450\n')
    time.sleep(SHORT_DELAY)
    # 5. nokta Lazer
    ser.write(b'G1 Z38.5 A-13.5 F450\n')
    time.sleep(LONG_DELAY)

    # 6. nokta Kamera
    ser.write(b'G1 X-30.5 Y-30.5 F450\n')
    time.sleep(SHORT_DELAY)
    # 6. nokta Lazer
    ser.write(b'G1 Z-34.2 A34 F450\n')
    time.sleep(LONG_DELAY)

    # 7. nokta Kamera
    ser.write(b'G1 X-34.0 Y7.2 F450\n')
    time.sleep(SHORT_DELAY)
    # 7. nokta Lazer
    ser.write(b'G1 Z-33.5 A-7.5 F450\n')
    time.sleep(LONG_DELAY)

    # 8. nokta Kamera Home)
    ser.write(b'G1 X32.38 Y12.05 F450\n')
    time.sleep(SHORT_DELAY)
    # 8. nokta Lazer
    ser.write(b'G1 Z31.7 A-11.5 F450\n')
    time.sleep(LONG_DELAY)

        #30 sn delay yerine 
    for i in range(10):
        ser.write(b'G1 X0.03 Y0.03 F450\n')
        time.sleep(0.5)
        ser.write(b'G1 X-0.03 Y-0.03 F450\n')
        time.sleep(0.5)

    
    

# Biraz zaman tanı (hareket bitmeden portu kapatma)
time.sleep(0.2)

ser.close()