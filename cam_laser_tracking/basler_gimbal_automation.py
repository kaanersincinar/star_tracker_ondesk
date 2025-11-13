from pypylon import pylon
import cv2
import numpy as np
import math
import time
import serial
import serial.tools.list_ports
import config as cfg
import threading

from port_listener import list_serial_ports
from magnetic_reader import arduino_worker

# --- Lazer tespiti için HSV aralığı (kırmızı lazer varsayıyorum) ---
# LASER_RED1_LOWER = (0,   120, 200)
# LASER_RED1_UPPER = (10,  255, 255)
# LASER_RED2_LOWER = (170, 120, 200)
# LASER_RED2_UPPER = (180, 255, 255)

# --- Mavi lazer HSV aralığı ---
LASER_BLUE_LOWER = (90,  80, 120)
LASER_BLUE_UPPER = (140, 255, 255)


MIN_AREA_LASER = 5  # lazer noktası için minimum alan (px^2) - gerekirse küçült

# --- Lazer gimbali için kontrol gain'leri ---
K_AZ_MM_PER_DEG_LASER = 0.1   # senin mekaniğine göre tune et
K_EL_MM_PER_DEG_LASER = 0.1

LASER_DEADBAND_DEG = 0.02     # çok küçük hataları ignore et
MAX_STEP_MM_LASER   = 2.0     # tek komutta max adım (mm)


# --- Kamera ve optik parametreler ---
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

# --- Gimbal / seri port parametreleri ---
SER_ENABLED = True      # Seri port açılmazsa sadece takip yapılır

# Script başladığında kabul edilen referans konum (mm)
current_x_mm = 0.0
current_y_mm = 0.0

SER_MKS_PORT = None
SER_ARD1_PORT = None
SER_ARD2_PORT = None


# -------------------------------------------------
#  Yardımcı fonksiyonlar
# -------------------------------------------------

def pixels_to_angle(delta_x, delta_y):
    """Piksel ofsetini (dx, dy) açıya çevir (derece) → (azimuth, elevation)."""
    dx_mm = delta_x * PIXEL_SIZE_MM
    dy_mm = delta_y * PIXEL_SIZE_MM
    theta_x = math.degrees(math.atan(dx_mm / FOCAL_LENGTH_MM))  # azimut
    theta_y = math.degrees(math.atan(dy_mm / FOCAL_LENGTH_MM))  # elevasyon
    return theta_x, theta_y


def detect_bright_circle_center_mono(img):
    """
    Mono8 görüntüde, en büyük parlak yuvarlak cismin merkezini bul.
    Eski beyaz daire takibi fonksiyonun.
    """
    if len(img.shape) == 3:
        gray = img[:, :, 0]
    else:
        gray = img

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        raise ValueError("Hiç kontur bulunamadı (parlak cisim algılanamadı).")

    big_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(big_cnt)
    if area < 20:
        raise ValueError("Algılanan kontur çok küçük (muhtemelen gürültü).")

    M = cv2.moments(big_cnt)
    if M["m00"] == 0:
        raise ValueError("Moment hatası (m00=0).")

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    radius = int(math.sqrt(area / math.pi))
    maxVal = float(gray[cy, cx])

    return (cx, cy), radius, maxVal

def detect_green_circle_center(img):
    """
    Renkli görüntüde yeşil dairenin merkezini bul.
    Adımlar:
      - BGR → HSV
      - Yeşil renk için maskeleme
      - Maske üzerinden contour + centroid
    """
    if len(img.shape) == 2 or img.shape[2] == 1:
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img_bgr = img

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Yeşil renk aralığı (gerekirse ayarlarsın)
    lower_green = np.array([35, 60, 40], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Gürültü azaltma
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        raise ValueError("Yeşil bölge bulunamadı (kontur yok).")

    big_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(big_cnt)
    if area < 50:
        raise ValueError("Algılanan yeşil kontur çok küçük (muhtemelen gürültü).")

    M = cv2.moments(big_cnt)
    if M["m00"] == 0:
        raise ValueError("Moment hatası (m00=0).")

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    radius = int(math.sqrt(area / math.pi))
    maxVal = float(mask[cy, cx])  # maske yoğunluğu

    return (cx, cy), radius, maxVal

def detect_laser_circle_center(color_img):
    """
    Mavi lazer spotunu HSV uzayında bulur.
    Çıkış: (center(x,y), radius, maxVal)
    """
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LASER_BLUE_LOWER, LASER_BLUE_UPPER)
    mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("Mavi lazer spotu bulunamadı (kontur yok).")

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA_LASER:
        raise RuntimeError(f"Lazer spot alanı çok küçük: {area:.1f}")

    (x, y), radius = cv2.minEnclosingCircle(c)
    x, y = int(x), int(y)
    radius = int(radius)

    laser_mask_roi = mask[max(0, y-5):y+5, max(0, x-5):x+5]
    maxVal = float(laser_mask_roi.max()) if laser_mask_roi.size > 0 else 0.0
    print("LASER HSV center pixel:", hsv[y, x])

    return (x, y), radius, maxVal

def set_exposure_basler(cam, exp_us):
    """Basler kamerada manuel exposure ayarla (µs)."""
    try:
        if cam.ExposureAuto.IsWritable():
            cam.ExposureAuto.SetValue("Off")
            print("ExposureAuto → Off")
    except Exception as e:
        print("ExposureAuto kapatılamadı (önemli değil olabilir):", e)

    try:
        node = cam.ExposureTime
    except Exception:
        print("ExposureTime noduna erişilemedi.")
        return None

    min_exp = node.Min
    max_exp = node.Max
    print(f"Exposure range: {min_exp:.1f} – {max_exp:.1f} us")

    exp_value = max(min(exp_us, max_exp), min_exp)
    node.SetValue(exp_value)
    val = node.GetValue()
    print("Exposure set to:", val, "us")
    return val

def snap_to_inc(val, inc):
    """Değerleri node'un increment'ine oturt."""
    try:
        inc = int(inc)
    except Exception:
        inc = 1
    return val if inc <= 1 else (val // inc) * inc

def set_safe(node, value):
    """GenICam node'a güvenli yaz (min/max clamp)."""
    try:
        vmin = node.GetMin()
        vmax = node.GetMax()
        if isinstance(value, (int, float)):
            value = max(vmin, min(vmax, value))
    except Exception:
        pass
    node.SetValue(value)

def set_roi_basler(cam, roi_w, roi_h, offx, offy):
    """Basler kamerada ROI ayarı (Width, Height, OffsetX, OffsetY)."""
    try:
        # 1) Offset'leri sıfıra çek
        try:
            set_safe(cam.OffsetX, 0)
        except Exception:
            pass
        try:
            set_safe(cam.OffsetY, 0)
        except Exception:
            pass

        # 2) Artışları (increment) oku
        try:
            w_inc = cam.Width.GetInc()
        except Exception:
            w_inc = 1

        try:
            h_inc = cam.Height.GetInc()
        except Exception:
            h_inc = 1

        try:
            ox_inc = cam.OffsetX.GetInc()
        except Exception:
            ox_inc = 1

        try:
            oy_inc = cam.OffsetY.GetInc()
        except Exception:
            oy_inc = 1

        # 3) Width/Height
        W = snap_to_inc(roi_w, w_inc)
        H = snap_to_inc(roi_h, h_inc)
        set_safe(cam.Width, W)
        set_safe(cam.Height, H)

        # 4) OffsetX/OffsetY
        OX = snap_to_inc(offx, ox_inc)
        OY = snap_to_inc(offy, oy_inc)
        set_safe(cam.OffsetX, OX)
        set_safe(cam.OffsetY, OY)

        print(f"ROI ayarlandı: Width={W}, Height={H}, OffsetX={OX}, OffsetY={OY}")

    except Exception as e:
        print("ROI ayarlanamadı:", e)

def open_basler_camera():
    """
    İlk bulunan Basler GigE (tercihen) veya başka bir Basler kamerayı açar.
    """
    tl_factory = pylon.TlFactory.GetInstance()

    # Önce GigE TL üzerinden enumerate dene
    gige_tl = None
    for tl_info in tl_factory.EnumerateTls():
        if "GigE" in tl_info.GetDeviceClass():
            gige_tl = tl_factory.CreateTl(tl_info)
            break

    devices = []

    if gige_tl is not None:
        try:
            devices = gige_tl.EnumerateAllDevices()
            print(f"GigE üzerinden bulunan cihaz sayısı: {len(devices)}")
        except Exception as e:
            print("GigE EnumerateAllDevices hata verdi:", e)

    # Fallback
    if not devices:
        print("GigE enumerate sonuçsuz, TlFactory.EnumerateDevices() ile tekrar deneniyor...")
        devices = tl_factory.EnumerateDevices()

    if not devices:
        raise RuntimeError("Hiç Basler kamera bulunamadı (GigE + USB).")

    cam = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
    cam.Open()

    di = cam.GetDeviceInfo()
    try:
        serial_no = di.GetSerialNumber()
    except Exception:
        serial_no = "N/A"

    print(f"✅ Bağlı Basler kamera: {di.GetModelName()} [{serial_no}]")

    # GigE ise paket boyutunu optimize etmeye çalış
    try:
        if hasattr(cam, "GevSCPSPacketSize") and cam.GevSCPSPacketSize.IsWritable():
            cam.GevSCPSPacketSize.SetValue(cam.GevSCPSPacketSize.Max)
            print("GevSCPSPacketSize max'a ayarlandı.")
    except Exception as e:
        print("Packet size ayarlanamadı:", e)

    return cam


# -------------------------------------------------
#  Seri port / gimbal fonksiyonları
# -------------------------------------------------
def init_serial():
    """Marlin kart ile seri haberleşmeyi başlat."""
    global ser
    if not SER_ENABLED:
        print("Seri port devre dışı (SER_ENABLED = False).")
        ser = None
        return

    try:
        ser = serial.Serial(SER_MKS_PORT, cfg.BAUD_MKS, timeout=0.01)
        print(f"Seri port açıldı: {SER_MKS_PORT} @ {cfg.BAUD_MKS}")
        time.sleep(2.0)  # Marlin reset için
        send_gcode("G91")  # Göreceli mod
    except Exception as e:
        print("Seri port açılamadı, sadece görüntü takibi yapılacak:", e)
        ser = None


def send_gcode(cmd: str):
    """Tek satır G-code gönder, cevapları non-blocking oku."""
    global ser
    if ser is None:
        return
    try:
        line = (cmd.strip() + "\n").encode()
        ser.write(line)
        print("→ GCODE:", cmd)
        time.sleep(0.001)
        while ser.in_waiting:
            resp = ser.readline().decode(errors="ignore").strip()
            if resp:
                print("<", resp)
    except Exception as e:
        print("G-code gönderilemedi:", e)

def move_laser_gimbal_relative(dx_mm, dy_mm):
    """
    Lazer gimbalını relatif modda hareket ettirir.
    dx_mm, dy_mm = X/Y ekseninde gidilecek mesafe (mm).
    """

    if abs(dx_mm) < 1e-6 and abs(dy_mm) < 1e-6:
        return

    # G91 relatif, G90 absolute
    cmd = f"G1 Z{dx_mm:.3f} A{dy_mm:.3f} F{cfg.FEEDRATE}\n"
    send_gcode(cmd)

def send_to_gimbal(az_deg, el_deg):
    """
    Kamera merkezine göre bulunan açısal hatayı
    X/Y eksenlerine göre G-code hareketine çevirir.
    """
    global ser, current_x_mm, current_y_mm

    if ser is None:
        print(f"(DRY-RUN) az={az_deg:.3f} el={el_deg:.3f}  "
              f"[X={current_x_mm:.2f}mm Y={current_y_mm:.2f}mm]")
        return

    # Deadband
    if abs(az_deg) < cfg.AZ_DEADBAND_DEG and abs(el_deg) < cfg.EL_DEADBAND_DEG:
        return

    # Negatif feedback
    step_x = -cfg.K_AZ_MM_PER_DEG * az_deg
    step_y =  cfg.K_EL_MM_PER_DEG * el_deg

    # Frame başına limit
    step_x = max(min(step_x, cfg.MAX_STEP_MM), -cfg.MAX_STEP_MM)
    step_y = max(min(step_y, cfg.MAX_STEP_MM), -cfg.MAX_STEP_MM)

    # Yazılımsal endstop
    target_x = current_x_mm + step_x
    target_y = current_y_mm + step_y

    if target_x > cfg.X_MAX_MM:
        step_x = cfg.X_MAX_MM - current_x_mm
        target_x = cfg.X_MAX_MM
        print("⚠ X yazılımsal endstop (üst limit)!")
    elif target_x < cfg.X_MIN_MM:
        step_x = cfg.X_MIN_MM - current_x_mm
        target_x = cfg.X_MIN_MM
        print("⚠ X yazılımsal endstop (alt limit)!")

    if target_y > cfg.Y_MAX_MM:
        step_y = cfg.Y_MAX_MM - current_y_mm
        target_y = cfg.Y_MAX_MM
        print("⚠ Y yazılımsal endstop (üst limit)!")
    elif target_y < cfg.Y_MIN_MM:
        step_y = cfg.Y_MIN_MM - current_y_mm
        target_y = cfg.Y_MIN_MM
        print("⚠ Y yazılımsal endstop (alt limit)!")

    if abs(step_x) < 1e-3:
        step_x = 0.0
    if abs(step_y) < 1e-3:
        step_y = 0.0

    if step_x == 0.0 and step_y == 0.0:
        return

    cmd_parts = []
    if step_x != 0.0:
        cmd_parts.append(f"X{step_x:.3f}")
    if step_y != 0.0:
        cmd_parts.append(f"Y{step_y:.3f}")
    cmd = "G1 " + " ".join(cmd_parts) + f" F{cfg.FEEDRATE}"
    send_gcode(cmd)

    current_x_mm += step_x
    current_y_mm += step_y

    print(f"[POS] X={current_x_mm:.2f}mm  Y={current_y_mm:.2f}mm")


# -------------------------------------------------
#  Ana takip döngüsü
# -------------------------------------------------
def track_star():
    cam = open_basler_camera()
    init_serial()

    # Bu değişkenlerle frame içinde ne yapacağımızı belirleyeceğiz
    pixel_format = "Unknown"
    is_bayer = False
    is_true_mono = False

    try:
        # PixelFormat ayarla / oku
        try:
            if cam.PixelFormat.IsWritable():
                enum_entries = cam.PixelFormat.GetSymbolics()
                print("Mevcut PixelFormat seçenekleri:", enum_entries)

                # Önce doğrudan BGR8/RGB8Packed dene
                if "BGR8" in enum_entries:
                    cam.PixelFormat.SetValue("BGR8")
                    print("🎨 PixelFormat → BGR8 (renkli)")
                elif "RGB8Packed" in enum_entries:
                    cam.PixelFormat.SetValue("RGB8Packed")
                    print("🎨 PixelFormat → RGB8Packed (renkli)")
                else:
                    print("⚠ BGR8/RGB8Packed yok, mevcut formatla devam ediliyor.")
            else:
                print("PixelFormat yazılabilir değil.")
        except Exception as e:
            print("PixelFormat ayarlanırken hata:", e)

        # Gerçek PixelFormat'ı oku
        try:
            pixel_format = cam.PixelFormat.GetValue()
            print("Aktif PixelFormat:", pixel_format)
            if pixel_format.startswith("Bayer"):
                is_bayer = True
            if pixel_format.startswith("Mono"):
                is_true_mono = True
        except Exception as e:
            print("Aktif PixelFormat okunamadı:", e)

        # ROI ve exposure
        set_roi_basler(cam, cfg.ROI_W, cfg.ROI_H, cfg.ROI_OFFX, cfg.ROI_OFFY)
        current_exp = set_exposure_basler(cam, 22600.0)

        # AcquisitionMode → Continuous
        try:
            if cam.AcquisitionMode.IsWritable():
                cam.AcquisitionMode.SetValue("Continuous")
                print("AcquisitionMode → Continuous")
        except Exception as e:
            print("AcquisitionMode ayarlanamadı:", e)

        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        print("Kamera grabbing başladı.")

        cv2.namedWindow("Circle Tracking", cv2.WINDOW_AUTOSIZE)

        while cam.IsGrabbing():
            try:
                grabResult = cam.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
            except Exception as e:
                print("Frame alınırken timeout/hata:", e)
                continue

            if not grabResult.GrabSucceeded():
                print("Frame grab başarısız:", grabResult.ErrorCode, grabResult.ErrorDescription)
                grabResult.Release()
                continue

            image = grabResult.Array  # H x W veya H x W x C
            grabResult.Release()

            detection_mode = "green"  # varsayılan

            if len(image.shape) == 2:
                # Tek kanal veri: Bayer mi, gerçek mono mu?
                if is_bayer:
                    # Bayer → BGR convert
                    if pixel_format == "BayerRG8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    elif pixel_format == "BayerBG8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_BG2BGR)
                    elif pixel_format == "BayerGR8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GR2BGR)
                    elif pixel_format == "BayerGB8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GB2BGR)
                    else:
                        # Bilinmeyen Bayer: yine de bir şey deneyelim
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    detection_mode = "green"
                else:
                    # Gerçek Mono8 vb.
                    print("⚠ Görüntü gerçek tek kanal (mono), renk tespiti olmayacak, parlak daireye düşüyorum.")
                    gray_img = image
                    color_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
                    detection_mode = "mono"
            else:
                # Zaten H x W x 3 → BGR varsay
                color_img = image
                detection_mode = "green"

            h, w = color_img.shape[:2]
            center = (w // 2, h // 2)

            # --- NESNE TESPİTİ: YEŞİL HEDEF ---
            try:
                if detection_mode == "green":
                    circle_center, radius_star, maxVal = detect_green_circle_center(color_img)
                else:
                    # mono parlak daire
                    circle_center, radius_star, maxVal = detect_bright_circle_center_mono(color_img)
            except Exception as e:
                print(f"Nesne bulunamadı ({detection_mode}):", e)
                annotated = color_img.copy()
                cv2.drawMarker(
                    annotated, center, (0, 255, 0),
                    cv2.MARKER_CROSS, 20, 2
                )
                display_img = cv2.resize(
                    annotated, (cfg.DISPLAY_W, cfg.DISPLAY_H),
                    interpolation=cv2.INTER_AREA
                )
                cv2.imshow("Circle Tracking", display_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
                continue

            cx, cy = circle_center
            dx = cx - center[0]
            dy = center[1] - cy  # yukarı pozitif

            az_deg, el_deg = pixels_to_angle(dx, dy)

            mode_icon = "🟢" if detection_mode == "green" else "⚪"
            print(
                f"{mode_icon} STAR mode={detection_mode} center={circle_center}  "
                f"Δx={dx:4d} Δy={dy:4d}  "
                f"az={az_deg:7.3f}° el={el_deg:7.3f}°  I={maxVal:.1f}"
            )

            # --- KAMERA GİMBAL KOMUTU (AYNI KALDI) ---
            send_to_gimbal(az_deg, el_deg)

            # --- OVERLAY BAŞLANGIÇ ---
            annotated = color_img.copy()
            cv2.drawMarker(
                annotated, center, (0, 255, 0),
                cv2.MARKER_CROSS, 20, 2
            )
            cv2.circle(annotated, (cx, cy), radius_star, (255, 0, 0), 2)
            cv2.circle(annotated, (cx, cy), 3, (0, 0, 255), -1)

            # ================= LAZER TAKİP BLOĞU =================
            try:
                laser_center, radius_laser, laser_I = detect_laser_circle_center(color_img)
                lx, ly = laser_center

                # Lazer → YILDIZ hizalama: lazer yeşil noktaya kilitlensin
                dx_laser_px = cx - lx          # hedef: yeşil nokta
                dy_laser_px = ly - cy          # ekran koordinatına göre

                az_laser_deg, el_laser_deg = pixels_to_angle(dx_laser_px, dy_laser_px)

                # Deadband
                if abs(az_laser_deg) < LASER_DEADBAND_DEG:
                    az_laser_deg = 0.0
                if abs(el_laser_deg) < LASER_DEADBAND_DEG:
                    el_laser_deg = 0.0

                # BURAYA KENDİ LAZER GİMBAL KOMUTUNU BAĞLA
                # Örn: send_to_laser_gimbal(az_laser_deg, el_laser_deg)
                try:
                    move_laser_gimbal_relative(az_laser_deg, el_laser_deg)
                except NameError:
                    # Henüz tanımlamadıysan sessizce geç, sadece log yaz
                    pass

                print(
                    f"🔴 LASER center={laser_center} "
                    f"Δx_laser={dx_laser_px:4d} Δy_laser={dy_laser_px:4d} "
                    f"az_L={az_laser_deg:7.3f}° el_L={el_laser_deg:7.3f}° I_L={laser_I:.1f}"
                )

                # Overlay: lazeri de çiz
                cv2.circle(annotated, (lx, ly), radius_laser, (0, 255, 255), 2)
                cv2.circle(annotated, (lx, ly), 3, (0, 255, 255), -1)

            except Exception as e:
                # Lazer görünmüyorsa veya tespit edilemediyse sadece logla, takip döngüsü bozulmasın
                # print(f"Lazer bulunamadı: {e}")
                pass
            # =====================================================

            if current_exp is None:
                exp_text = "Exp: N/A"
            else:
                exp_text = f"Exp={current_exp:.0f} us"

            cv2.putText(
                annotated, f"dx={dx}px dy={dy}px",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2
            )
            cv2.putText(
                annotated, f"az={az_deg:.2f} el={el_deg:.2f}",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2
            )
            cv2.putText(
                annotated, f"{exp_text}  PF={pixel_format}",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (200, 200, 200), 1
            )

            display_img = cv2.resize(
                annotated,
                (cfg.DISPLAY_W, cfg.DISPLAY_H),
                interpolation=cv2.INTER_AREA
            )
            cv2.imshow("Circle Tracking", display_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

        cv2.destroyAllWindows()

    finally:
        try:
            if cam.IsGrabbing():
                cam.StopGrabbing()
        except Exception:
            pass
        cam.Close()
        print("Kamera kapatıldı.")

        global ser
        if ser is not None:
            try:
                ser.close()
                print("Seri port kapatıldı.")
            except Exception:
                pass


def main ():
    global SER_MKS_PORT
    global SER_ARD1_PORT
    global SER_ARD2_PORT

    mks_port, ard_ports = list_serial_ports()
    SER_MKS_PORT = mks_port
    SER_ARD1_PORT = ard_ports[0]
    SER_ARD2_PORT = ard_ports[1]

    threads = []
    for port in ard_ports:
        t = threading.Thread(target=arduino_worker, args=(port,), daemon=True)
        t.start()
        threads.append(t)
    track_star()
    


if __name__ == "__main__":
    main()