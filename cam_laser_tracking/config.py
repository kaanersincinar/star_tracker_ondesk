# --- BAUD Rate of MKS and Arduino Mega's ---
BAUD_MKS     = 250000
BAUD_ARDUINO = 115200

# --- Kamera ve optik parametreler ---
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

# Ekranda görmek istediğin pencere boyutu
DISPLAY_W = 512
DISPLAY_H = 512

# --- Gimbal hız parametresi ---
FEEDRATE = 2500         # G1 F hızı (mm/dk)


# Açısal hata → mm (veya kartının beklediği birim) çeviren gain
K_AZ_MM_PER_DEG = 0.1   # azimut ekseni için
K_EL_MM_PER_DEG = 0.1   # elevasyon ekseni için

# Çok küçük hatalarda komut göndermemek için deadband
AZ_DEADBAND_DEG = 0.02
EL_DEADBAND_DEG = 0.02

# Tek seferde gönderilecek maksimum adım (mm)
MAX_STEP_MM = 0.5

# Görüntü boyutu (ROI) – Pylon'daki değerler
ROI_W = 4508
ROI_H = 4096
ROI_OFFX = 0
ROI_OFFY = 0

# --- Yazılımsal endstop limitleri (mm) ---
# X ekseni: toplam 20 cm → -10 cm .. +10 cm
# Y ekseni: toplam 8  cm → -4  cm .. +4  cm
X_MIN_MM = -100.0   # -10 cm
X_MAX_MM =  100.0   # +10 cm
Y_MIN_MM = -40.0    # -4 cm
Y_MAX_MM =  40.0    # +4 cm






SER_MKS_PORT =  None
SER_ARD1_PORT = None
SER_ARD2_PORT = None

SER_ENABLED = True


#---------------------Magnetic Reader-----------------------------------------------------

#Mangnetic reader parameters
START_BYTE = 0x7E
END_BYTE   = 0x7F
MSG_SENSOR = 0x10
FRAME_SIZE = 16

# =====================================================
#  NODE / EKSEN BAZLI LINEER KALIBRASYON
#  Gerçek referans: G-code mm (MKS tarafı)
#  Model: mm = K * count + B
# =====================================================

# NODE 1 – X : (10 mm -> 5722), (40 mm -> 23271)
X1_K = 0.0017094991
X1_B = 0.2182461

# NODE 1 – Y : (3 mm -> 1012), (30 mm -> 11324)
Y1_K = 0.0026183088
Y1_B = 0.3502715

# NODE 2 – X : (10 mm -> 4640), (60 mm -> 26247)
X2_K = 0.0023140649
X2_B = -0.7372611

# NODE 2 – Y : şimdilik tek nokta: (10 mm -> 1859)
# Offset belirsiz, sadece scale kullanıyoruz.
Y2_K = 10.0 / 1859.0   # ≈ 0.00537924
Y2_B = 0.0

# Azimuth için placeholder – gerçek ölçüm gelince güncelle
AZ_K = 0.001
AZ_B = 0.0
