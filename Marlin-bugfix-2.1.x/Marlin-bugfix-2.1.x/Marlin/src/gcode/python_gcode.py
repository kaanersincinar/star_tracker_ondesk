import serial
import time

# --- Seri port ayarları ---
PORT = 'COM7'        # Doğru port
BAUD = 250000        # Marlin çoğunlukla 250000, sende 115200 ise onu yaz

# Seri portu aç
ser = serial.Serial(PORT, BAUD, timeout=1)

# Kart reset olup hazır olsun diye 2 sn bekle
time.sleep(2)

# Göreceli konum moduna geç (her komut önceki pozisyona göre
ser.write(b'G91\n')

# X eksenini +10 mm hareket ettir, F300 = feedrate (mm/dk)
ser.write(b'G1 X10 F300\n')

# İstersen tekrar absolute moda dönebilirsin (şart değil)
# ser.write(b'G90\n')

# Biraz zaman tanı (hareket bitmeden portu kapatma)
time.sleep(3)

ser.close()
