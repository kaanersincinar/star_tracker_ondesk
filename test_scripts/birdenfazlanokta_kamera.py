from pypylon import pylon
import cv2
import numpy as np
import math
import time
import serial
import random  # <-- RANDOM TARGET SEÇİMİ İÇİN

# --- Kamera ve optik parametreler ---
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

# Ekranda görmek istediğin pencere boyutu
DISPLAY_W = 512
DISPLAY_H = 512

# --- Gimbal / seri port parametreleri ---
PORT = "/dev/ttyUSB0"   # kendi portun
BAUD = 250000           # Marlin baud
FEEDRATE = 3000         # G1 F hızı (mm/dk ya da senin birimin)
SER_ENABLED = True      # Seri port açılmazsa sadece takip yapılır

# Açısal hata → mm (veya kartının beklediği birim) çeviren gain
K_AZ_MM_PER_DEG = 0.1   # azimut ekseni için
K_EL_MM_PER_DEG = 0.1   # elevasyon ekseni için

# Çok küçük hatalarda komut göndermemek için deadband
AZ_DEADBAND_DEG = 0.02
EL_DEADBAND_DEG = 0.02

# Tek seferde gönderilecek maksimum adım (mm)
MAX_STEP_MM = 0.5

# Görüntü boyutu (ROI) – Pylon'daki değerler
ROI_W = 2748
ROI_H = 2800
ROI_OFFX = 828
ROI_OFFY = 230

# --- Yazılımsal endstop limitleri (mm) ---
# X ekseni: toplam 20 cm → -10 cm .. +10 cm
# Y ekseni: toplam 8  cm → -4  cm .. +4  cm
X_MIN_MM = -100.0   # -10 cm
X_MAX_MM =  100.0   # +10 cm
Y_MIN_MM = -40.0    # -4 cm
Y_MAX_MM =  40.0    # +4 cm

# Script başladığında kabul edilen referans konum (mm)
current_x_mm = 0.0
current_y_mm = 0.0

ser = None

# --- RASTGELE HEDEF AYARLARI ---
HOLD_TIME_SEC = 5.0          # Hedefte kalma süresi
CENTER_TOL_PX = 3            # "Merkezde say" toleransı (piksel)
TARGET_LOST_MAX_DIST_PX = 60 # Hedeften kopma mesafesi

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
    Eski tek hedef fonksiyonun – ARTIK RANDOM MODE İÇİN KULLANMIYORUZ ama dursun.
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
    Eski tek hedef fonksiyonun – ARTIK RANDOM MODE İÇİN KULLANMIYORUZ ama dursun.
    """
    if len(img.shape) == 2 or img.shape[2] == 1:
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img_bgr = img

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    lower_green = np.array([35, 60, 40], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green)

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
    maxVal = float(mask[cy, cx])

    return (cx, cy), radius, maxVal


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
        try:
            set_safe(cam.OffsetX, 0)
        except Exception:
            pass
        try:
            set_safe(cam.OffsetY, 0)
        except Exception:
            pass

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

        W = snap_to_inc(roi_w, w_inc)
        H = snap_to_inc(roi_h, h_inc)
        set_safe(cam.Width, W)
        set_safe(cam.Height, H)

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
        ser = serial.Serial(PORT, BAUD, timeout=0.01)
        print(f"Seri port açıldı: {PORT} @ {BAUD}")
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

    if abs(az_deg) < AZ_DEADBAND_DEG and abs(el_deg) < EL_DEADBAND_DEG:
        return

    step_x = -K_AZ_MM_PER_DEG * az_deg
    step_y =  K_EL_MM_PER_DEG * el_deg

    step_x = max(min(step_x, MAX_STEP_MM), -MAX_STEP_MM)
    step_y = max(min(step_y, MAX_STEP_MM), -MAX_STEP_MM)

    target_x = current_x_mm + step_x
    target_y = current_y_mm + step_y

    if target_x > X_MAX_MM:
        step_x = X_MAX_MM - current_x_mm
        target_x = X_MAX_MM
        print("⚠ X yazılımsal endstop (üst limit)!")
    elif target_x < X_MIN_MM:
        step_x = X_MIN_MM - current_x_mm
        target_x = X_MIN_MM
        print("⚠ X yazılımsal endstop (alt limit)!")

    if target_y > Y_MAX_MM:
        step_y = Y_MAX_MM - current_y_mm
        target_y = Y_MAX_MM
        print("⚠ Y yazılımsal endstop (üst limit)!")
    elif target_y < Y_MIN_MM:
        step_y = Y_MIN_MM - current_y_mm
        target_y = Y_MIN_MM
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
    cmd = "G1 " + " ".join(cmd_parts) + f" F{FEEDRATE}"
    send_gcode(cmd)

    current_x_mm += step_x
    current_y_mm += step_y

    print(f"[POS] X={current_x_mm:.2f}mm  Y={current_y_mm:.2f}mm")


# -------------------------------------------------
#  Birden fazla noktayı bulmak için yeni fonksiyonlar
# -------------------------------------------------
def find_green_points(img_bgr, min_area=30):
    """Yeşil maske üzerinde birden fazla noktayı (cx,cy,area,radius) listesi olarak döndürür."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 60, 40], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    points = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        radius = int(math.sqrt(area / math.pi))
        points.append((cx, cy, radius, area))
    return points, mask


def find_bright_points_mono(img, min_area=20):
    """Mono görüntüde birden fazla parlak noktayı döndürür."""
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

    points = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        radius = int(math.sqrt(area / math.pi))
        points.append((cx, cy, radius, area))
    return points, gray


# -------------------------------------------------
#  Ana takip döngüsü – RANDOM HEDEF + 5 sn bekleme
# -------------------------------------------------
def track_star():
    cam = open_basler_camera()
    init_serial()

    pixel_format = "Unknown"
    is_bayer = False
    is_true_mono = False

    # RASTGELE HEDEF STATE
    current_target = None      # (cx, cy, radius, area)
    target_center_px = None    # (cx, cy)
    target_centered_t = None   # merkezdeyken zaman
    rng = random.Random()      # istersen rng.seed(...) ile deterministik yaparsın

    try:
        try:
            if cam.PixelFormat.IsWritable():
                enum_entries = cam.PixelFormat.GetSymbolics()
                print("Mevcut PixelFormat seçenekleri:", enum_entries)

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

        try:
            pixel_format = cam.PixelFormat.GetValue()
            print("Aktif PixelFormat:", pixel_format)
            if pixel_format.startswith("Bayer"):
                is_bayer = True
            if pixel_format.startswith("Mono"):
                is_true_mono = True
        except Exception as e:
            print("Aktif PixelFormat okunamadı:", e)

        set_roi_basler(cam, ROI_W, ROI_H, ROI_OFFX, ROI_OFFY)
        current_exp = set_exposure_basler(cam, 22600.0)

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

            image = grabResult.Array
            grabResult.Release()

            detection_mode = "green"

            if len(image.shape) == 2:
                if is_bayer:
                    if pixel_format == "BayerRG8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    elif pixel_format == "BayerBG8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_BG2BGR)
                    elif pixel_format == "BayerGR8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GR2BGR)
                    elif pixel_format == "BayerGB8":
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_GB2BGR)
                    else:
                        color_img = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
                    detection_mode = "green"
                else:
                    print("⚠ Görüntü gerçek tek kanal (mono), renk tespiti olmayacak, parlak noktalara geçiyorum.")
                    gray_img = image
                    color_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
                    detection_mode = "mono"
            else:
                color_img = image
                detection_mode = "green"

            h, w = color_img.shape[:2]
            center = (w // 2, h // 2)

            # --- Birden fazla nokta tespiti ---
            if detection_mode == "green":
                points, aux_img = find_green_points(color_img)
            else:
                points, aux_img = find_bright_points_mono(color_img)

            if not points:
                print("Nokta bulunamadı, hedef sıfırlanıyor.")
                current_target = None
                target_center_px = None
                target_centered_t = None

                annotated = color_img.copy()
                cv2.drawMarker(
                    annotated, center, (0, 255, 0),
                    cv2.MARKER_CROSS, 20, 2
                )
                display_img = cv2.resize(
                    annotated, (DISPLAY_W, DISPLAY_H),
                    interpolation=cv2.INTER_AREA
                )
                cv2.imshow("Circle Tracking", display_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
                continue

            # --- Hedef seçimi: random ---
            if current_target is None:
                current_target = rng.choice(points)  # (cx, cy, radius, area)
                target_center_px = (current_target[0], current_target[1])
                target_centered_t = None
                print(f"🎯 Yeni rastgele hedef: {target_center_px}")

            else:
                # Aynı hedefi tutmak için: mevcut noktalardan target_center_px'e en yakın olanı bul
                tx_prev, ty_prev = target_center_px
                best = None
                best_d = None
                for (cx, cy, radius, area) in points:
                    d = math.hypot(cx - tx_prev, cy - ty_prev)
                    if best_d is None or d < best_d:
                        best_d = d
                        best = (cx, cy, radius, area)

                if best is None or best_d > TARGET_LOST_MAX_DIST_PX:
                    # hedef kayboldu → yeni rastgele hedef
                    current_target = rng.choice(points)
                    target_center_px = (current_target[0], current_target[1])
                    target_centered_t = None
                    print(f"🎯 Hedef kayboldu, yeni rastgele hedef: {target_center_px}")
                else:
                    current_target = best
                    target_center_px = (current_target[0], current_target[1])

            cx, cy, radius, area = current_target
            dx = cx - center[0]
            dy = center[1] - cy  # yukarı pozitif

            az_deg, el_deg = pixels_to_angle(dx, dy)

            err_pix = math.hypot(dx, dy)
            mode_icon = "🟢" if detection_mode == "green" else "⚪"
            print(
                f"{mode_icon} mode={detection_mode} target={target_center_px}  "
                f"Δx={dx:4d} Δy={dy:4d}  "
                f"az={az_deg:7.3f}° el={el_deg:7.3f}°  |err|={err_pix:.1f}"
            )

            # --- Gimbale komut (hedefe hizalama) ---
            send_to_gimbal(az_deg, el_deg)

            # --- 5 saniye merkezde kalma mantığı ---
            now = time.time()
            if err_pix <= CENTER_TOL_PX:
                if target_centered_t is None:
                    target_centered_t = now
                    print("✅ Hedef merkezde, timer başlatıldı.")
                else:
                    if (now - target_centered_t) >= HOLD_TIME_SEC:
                        print(f"⏱ Hedefte {HOLD_TIME_SEC} sn bekledim, yeni hedefe geçiyorum.")
                        current_target = None
                        target_center_px = None
                        target_centered_t = None
            else:
                # merkezde değilse timer reset
                target_centered_t = None

            # --- Görsel overlay ---
            annotated = color_img.copy()
            cv2.drawMarker(
                annotated, center, (0, 255, 0),
                cv2.MARKER_CROSS, 20, 2
            )

            # Tüm noktaları çiz
            for (px, py, r, a) in points:
                cv2.circle(annotated, (px, py), max(r, 3), (255, 0, 0), 1)

            # Seçili hedefi farklı renkle vurgula
            cv2.circle(annotated, (cx, cy), max(radius, 5), (0, 0, 255), 2)
            cv2.circle(annotated, (cx, cy), 3, (0, 255, 255), -1)

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
                (DISPLAY_W, DISPLAY_H),
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


if __name__ == "__main__":
    track_star()
