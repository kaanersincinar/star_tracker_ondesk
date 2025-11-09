import serial
import time

# --- Seri port ayarları ---
PORT = '/dev/ttyUSB2'        # Doğru port
BAUD = 250000        # Marlin çoğunlukla 250000, sende 115200 ise onu yaz

# Seri portu aç
ser = serial.Serial(PORT, BAUD, timeout=1)

# Kart reset olup hazır olsun diye 2 sn bekle
time.sleep(1)

# Göreceli konum moduna geç (her komut önceki pozisyona göre)
ser.write(b'G91\n')

# Gimbal Kamera - X eksenini hareket ettir
#ser.write(b'G1 X10 F450\n')

# Gimbal Kamera - Y eksenini hareket ettir
#ser.write(b'G1 Y10 F450\n')

# Gimbal Lazer - X eksenini hareket ettir
#ser.write(b'G1 Z10 F450\n')

# Gimbal Lazer - Y eksenini hareket ettir
#ser.write(b'G1 A10 F450\n')

# Gimbal Kamera - X ve Y eksenini hareket ettir
#ser.write(b'G1 X10 Y10 F450\n')

# Gimbal Lazer - X ve Y eksenini hareket ettir
#ser.write(b'G1 Z10 A10 F450\n')

# Gimbal Kamera ve Lazer - X ve Y eksenini hareket ettir
#ser.write(b'G1 X10 Y10 Z10 A10 F450\n')

#Manyetik cetvel hassasiyet-hata ölçümü için gimbale 20 dakika hareket komutu gönderen döngü

for x in range (120):
    ser.write(b'G1 X50 Y50 F450\n')
    time.sleep(5)
    ser.write(b'G1 X-50 Y-50 F450\n')
    time.sleep(5)


# Biraz zaman tanı (hareket bitmeden portu kapatma)
time.sleep(0.2)

ser.close()