from pypylon import pylon
import cv2
import numpy as np
import math
import time
import serial
import random

# ============================================================
#  MKS / TEK SERİ PORT AYARLARI
# ============================================================
MKS_PORT = "/dev/ttyUSB4"   # TODO: MKS kartının portu
MKS_BAUD = 250000
FEEDRATE = 450              # G1 F değeri

# Kamera gimbali eksen isimleri (MKS firmware'ine göre)
CAM_AXIS_X = "X"
CAM_AXIS_Y = "Y"

# Lazer gimbali eksen isimleri
LAS_AXIS_X = "Z"            # Örn: Z ekseni
LAS_AXIS_Y = "A"            # Örn: E ekseni

# ------------------------------------------------------------
#  KAMERA GİMBALI İÇİN mm/deg GAIN
# ------------------------------------------------------------
K_AZ_MM_PER_DEG = 0.1   # Kamera azimuth (X) için 1° -> kaç mm
K_EL_MM_PER_DEG = 0.1   # Kamera elevation (Y) için 1° -> kaç mm

# ------------------------------------------------------------
#  LAZER GİMBALI İÇİN mm/deg GAIN
# ------------------------------------------------------------
K_LAZ_AZ_MM_PER_DEG = 0.1   # Lazer azimuth (Z)
K_LAZ_EL_MM_PER_DEG = 0.1   # Lazer elevation (E)

# Deadband ve max adımlar
AZ_DEADBAND_DEG = 0.02
EL_DEADBAND_DEG = 0.02
CAM_MAX_STEP_MM = 0.5
LASER_MAX_STEP_MM = 0.5

# Kamera gimbali için yazılımsal limitler
X_MIN_MM = -100.0
X_MAX_MM =  100.0
Y_MIN_MM = -40.0
Y_MAX_MM =  40.0

# Kamera gimbali pozisyonu ve mutlak açıları
current_x_mm    = 0.0
current_y_mm    = 0.0
cam_az_abs_deg  = 0.0
cam_el_abs_deg  = 0.0

# Lazer gimbali pozisyonu ve mutlak açıları
laser_x_mm      = 0.0
laser_y_mm      = 0.0
laser_az_abs_deg = 0.0
laser_el_abs_deg = 0.0

# Ortak seri port
ser = None

# ------------------------------------------------------------
#  KAMERA–LAZER GEOMETRİSİ (YAN YANA, AYNI YÜKSEKLİKTE)
# ------------------------------------------------------------
# Kamera merkezini (0,0,0) alıyoruz, lazer merkezini (BASELINE_X_MM, 0, 0).
BASELINE_X_MM        = 300.0   # TODO: kamera–lazer yatay mesafe (mm)
DIST_TO_HOLOGRAM_MM  = 540.0   # TODO: gimbal pivotu -> hologram düzlemi (mm)

# ------------------------------------------------------------
#  KAMERA / OPTİK PARAMETRELER
# ------------------------------------------------------------
PIXEL_SIZE_UM = 2.5
PIXEL_SIZE_MM = PIXEL_SIZE_UM / 1000.0
FOCAL_LENGTH_MM = 12.39

DISPLAY_W = 512
DISPLAY_H = 512

# Görüntü ROI
ROI_W = 2748
ROI_H = 2800
ROI_OFFX = 828
ROI_OFFY = 230

# Rastgele hedef ayarları
HOLD_TIME_SEC = 5.0
CENTER_TOL_PX = 3
TARGET_LOST_MAX_DIST_PX = 60

# ============================================================
#  SERİ PORT / MKS
# ============================================================
def init_mks_serial():
    global ser
    try:
        ser = serial.Serial(MKS_PORT, MKS_BAUD, timeout=0.01)
        print(f"✅ MKS seri port açıldı: {MKS_PORT} @ {MKS_BAUD}")
        time.sleep(2.0)
        ser.write(b"G91\n")   # Tüm eksenler göreceli mod
    except Exception as e:
        print("❌ MKS seri port AÇILAMADI, DRY-RUN mod:", e)
        ser = None


def send_mks_gcode(cmd: str):
    global ser
    if ser is None:
        print("(DRY-RUN)", cmd)
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


# ============================================================
#  GÖRÜNTÜDEN AÇIYA GEÇİŞ
# ============================================================
def pixels_to_angle(delta_x, delta_y):
    """Piksel ofsetini (dx, dy) açıya çevir (derece) → (azimuth, elevation)."""
    dx_mm = delta_x * PIXEL_SIZE_MM
    dy_mm = delta_y * PIXEL_SIZE_MM
    theta_x = math.degrees(math.atan(dx_mm / FOCAL_LENGTH_MM))  # azimut
    theta_y = math.degrees(math.atan(dy_mm / FOCAL_LENGTH_MM))  # elevasyon
    return theta_x, theta_y


# ============================================================
#  BASLER / PYLON YARDIMCI FONKSİYONLAR
# ============================================================
def set_exposure_basler(cam, exp_us):
    try:
        if cam.ExposureAuto.IsWritable():
            cam.ExposureAuto.SetValue("Off")
            print("ExposureAuto → Off")
    except Exception as e:
        print("ExposureAuto kapatılamadı:", e)

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
    try:
        inc = int(inc)
    except Exception:
        inc = 1
    return val if inc <= 1 else (val // inc) * inc


def set_safe(node, value):
    try:
        vmin = node.GetMin()
        vmax = node.GetMax()
        if isinstance(value, (int, float)):
            value = max(vmin, min(vmax, value))
    except Exception:
        pass
    node.SetValue(value)


def set_roi_basler(cam, roi_w, roi_h, offx, offy):
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


# ============================================================
#  KAMERA GİMBALINI (X/Y) SÜREN FONKSİYON
# ============================================================
def send_to_camera_gimbal(az_err_deg, el_err_deg):
    """
    Kameranın gördüğü hata açısını (az_err_deg, el_err_deg)
    CAM_AXIS_X / CAM_AXIS_Y eksenleriyle kapatır.
    Aynı zamanda kameranın mutlak açısını (cam_az_abs_deg / cam_el_abs_deg) günceller.
    """
    global ser, current_x_mm, current_y_mm
    global cam_az_abs_deg, cam_el_abs_deg

    if ser is None:
        print(f"(DRY CAM) az={az_err_deg:.3f} el={el_err_deg:.3f}")
        return

    if abs(az_err_deg) < AZ_DEADBAND_DEG and abs(el_err_deg) < EL_DEADBAND_DEG:
        return

    step_x = - K_AZ_MM_PER_DEG * az_err_deg
    step_y =   K_EL_MM_PER_DEG * el_err_deg

    step_x = max(min(step_x, CAM_MAX_STEP_MM), -CAM_MAX_STEP_MM)
    step_y = max(min(step_y, CAM_MAX_STEP_MM), -CAM_MAX_STEP_MM)

    target_x = current_x_mm + step_x
    target_y = current_y_mm + step_y

    if target_x > X_MAX_MM:
        step_x = X_MAX_MM - current_x_mm
        target_x = X_MAX_MM
        print("⚠ CAM X yazılımsal endstop üst!")
    elif target_x < X_MIN_MM:
        step_x = X_MIN_MM - current_x_mm
        target_x = X_MIN_MM
        print("⚠ CAM X yazılımsal endstop alt!")

    if target_y > Y_MAX_MM:
        step_y = Y_MAX_MM - current_y_mm
        target_y = Y_MAX_MM
        print("⚠ CAM Y yazılımsal endstop üst!")
    elif target_y < Y_MIN_MM:
        step_y = Y_MIN_MM - current_y_mm
        target_y = Y_MIN_MM
        print("⚠ CAM Y yazılımsal endstop alt!")

    if abs(step_x) < 1e-3:
        step_x = 0.0
    if abs(step_y) < 1e-3:
        step_y = 0.0

    if step_x == 0.0 and step_y == 0.0:
        return

    cmd_parts = []
    if step_x != 0.0:
        cmd_parts.append(f"{CAM_AXIS_X}{step_x:.3f}")
    if step_y != 0.0:
        cmd_parts.append(f"{CAM_AXIS_Y}{step_y:.3f}")
    cmd = "G1 " + " ".join(cmd_parts) + f" F{FEEDRATE}"
    send_mks_gcode(cmd)

    current_x_mm += step_x
    current_y_mm += step_y

    delta_az = - step_x / K_AZ_MM_PER_DEG
    delta_el =   step_y / K_EL_MM_PER_DEG

    cam_az_abs_deg += delta_az
    cam_el_abs_deg += delta_el

    print(f"[CAM POS] {CAM_AXIS_X}={current_x_mm:.2f}mm "
          f"{CAM_AXIS_Y}={current_y_mm:.2f}mm "
          f"(cam_az={cam_az_abs_deg:.3f}°, cam_el={cam_el_abs_deg:.3f}°)")


# ============================================================
#  KAMERA AÇISINDAN LAZER AÇISINA GEÇİŞ
# ============================================================
def camera_to_laser_angles(cam_az_deg, cam_el_deg):
    """
    Kamera ve lazer yan yana, aynı yükseklikte.
    Kamera mutlak (az, el) açısını alır,
    aynı hologram noktasını görebilmesi için lazere gereken (az, el) açıyı hesaplar.
    """
    cam_az_rad = math.radians(cam_az_deg)
    cam_el_rad = math.radians(cam_el_deg)

    # Kameradan bakınca hedefin hologram düzlemindeki konumu
    X = DIST_TO_HOLOGRAM_MM * math.tan(cam_az_rad)
    Y = DIST_TO_HOLOGRAM_MM * math.tan(cam_el_rad)

    # Lazer koordinatında aynı nokta
    X_L = X - BASELINE_X_MM         # lazer kameradan +B kadar sağda
    Y_L = Y
    Z_L = DIST_TO_HOLOGRAM_MM

    # Lazer açısından azimut
    az_l_rad = math.atan2(X_L, Z_L)
    az_l_deg = math.degrees(az_l_rad)

    # Elevation: aynı yükseklikteyse pratikte kamerayı kopyalıyoruz
    el_l_deg = cam_el_deg

    return az_l_deg, el_l_deg


# ============================================================
#  LAZER GİMBALINI (Z/E) HEDEF AÇIYA SÜRME
# ============================================================
def send_to_laser_target(target_az_deg, target_el_deg):
    """
    Lazerin mutlak açısını (laser_az_abs_deg/el_abs_deg) globalden okur,
    hedef açıya göre farkı mm'ye çevirip LAS_AXIS_X / LAS_AXIS_Y eksenlerini sürer.
    """
    global laser_x_mm, laser_y_mm
    global laser_az_abs_deg, laser_el_abs_deg

    if ser is None:
        print(f"(DRY LZR) tgt_az={target_az_deg:.3f} tgt_el={target_el_deg:.3f}")
        return

    d_az = target_az_deg - laser_az_abs_deg
    d_el = target_el_deg - laser_el_abs_deg

    if abs(d_az) < AZ_DEADBAND_DEG and abs(d_el) < EL_DEADBAND_DEG:
        return

    step_x = - K_LAZ_AZ_MM_PER_DEG * d_az
    step_y =   K_LAZ_EL_MM_PER_DEG * d_el

    step_x = max(min(step_x, LASER_MAX_STEP_MM), -LASER_MAX_STEP_MM)
    step_y = max(min(step_y, LASER_MAX_STEP_MM), -LASER_MAX_STEP_MM)

    if abs(step_x) < 1e-3:
        step_x = 0.0
    if abs(step_y) < 1e-3:
        step_y = 0.0

    if step_x == 0.0 and step_y == 0.0:
        return

    cmd_parts = []
    if step_x != 0.0:
        cmd_parts.append(f"{LAS_AXIS_X}{step_x:.3f}")
    if step_y != 0.0:
        cmd_parts.append(f"{LAS_AXIS_Y}{step_y:.3f}")
    cmd = "G1 " + " ".join(cmd_parts) + f" F{FEEDRATE}"
    send_mks_gcode(cmd)

    laser_x_mm += step_x
    laser_y_mm += step_y

    delta_az = - step_x / K_LAZ_AZ_MM_PER_DEG
    delta_el =   step_y / K_LAZ_EL_MM_PER_DEG

    laser_az_abs_deg += delta_az
    laser_el_abs_deg += delta_el

    print(f"[LZR POS] {LAS_AXIS_X}={laser_x_mm:.2f}mm "
          f"{LAS_AXIS_Y}={laser_y_mm:.2f}mm "
          f"(laz_az={laser_az_abs_deg:.3f}°, laz_el={laser_el_abs_deg:.3f}°)")


# ============================================================
#  BİRDEN FAZLA NOKTA ALGILAMA (SENİN KODUNDAN)
# ============================================================
def find_white_points(img_bgr, min_area=30, thresh_val=220):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

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


# ============================================================
#  ANA TAKİP DÖNGÜSÜ (KAMERA + LAZER)
# ============================================================
def track_star():
    global current_x_mm, current_y_mm, cam_az_abs_deg, cam_el_abs_deg
    global laser_x_mm, laser_y_mm, laser_az_abs_deg, laser_el_abs_deg

    # Başlangıçta tüm pozisyon/açıları sıfırla.
    # Fiziksel olarak hem kamera hem lazeri hologramın ORTASINA baktırıp script'i öyle aç.
    current_x_mm = 0.0
    current_y_mm = 0.0
    cam_az_abs_deg = 0.0
    cam_el_abs_deg = 0.0

    laser_x_mm = 0.0
    laser_y_mm = 0.0
    laser_az_abs_deg = 0.0
    laser_el_abs_deg = 0.0

    cam = open_basler_camera()
    init_mks_serial()

    pixel_format = "Unknown"
    is_bayer = False

    # Rastgele hedef state
    current_target = None
    target_center_px = None
    target_centered_t = None
    rng = random.Random()

    try:
        try:
            if cam.PixelFormat.IsWritable():
                enum_entries = cam.PixelFormat.GetSymbolics()
                print("Mevcut PixelFormat seçenekleri:", enum_entries)

                if "BGR8" in enum_entries:
                    cam.PixelFormat.SetValue("BGR8")
                    print("🎨 PixelFormat → BGR8")
                elif "RGB8Packed" in enum_entries:
                    cam.PixelFormat.SetValue("RGB8Packed")
                    print("🎨 PixelFormat → RGB8Packed")
                else:
                    print("⚠ BGR8/RGB8Packed yok, mevcut formatla devam ediliyor.")
        except Exception as e:
            print("PixelFormat ayarlanırken hata:", e)

        try:
            pixel_format = cam.PixelFormat.GetValue()
            print("Aktif PixelFormat:", pixel_format)
            if pixel_format.startswith("Bayer"):
                is_bayer = True
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
                    gray_img = image
                    color_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
                    detection_mode = "mono"
            else:
                color_img = image
                detection_mode = "green"

            h, w = color_img.shape[:2]
            center = (w // 2, h // 2)

            # Birden fazla nokta tespiti
            if detection_mode == "green":
                points, aux_img = find_white_points(color_img)
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

            # Hedef seçimi: random
            if current_target is None:
                current_target = rng.choice(points)
                target_center_px = (current_target[0], current_target[1])
                target_centered_t = None
                print(f"🎯 Yeni rastgele hedef: {target_center_px}")
            else:
                tx_prev, ty_prev = target_center_px
                best = None
                best_d = None
                for (cx, cy, radius, area) in points:
                    d = math.hypot(cx - tx_prev, cy - ty_prev)
                    if best_d is None or d < best_d:
                        best_d = d
                        best = (cx, cy, radius, area)

                if best is None or best_d > TARGET_LOST_MAX_DIST_PX:
                    current_target = rng.choice(points)
                    target_center_px = (current_target[0], current_target[1])
                    target_centered_t = None
                    print(f"🎯 Hedef kayboldu, yeni rastgele hedef: {target_center_px}")
                else:
                    current_target = best
                    target_center_px = (current_target[0], current_target[1])

            cx, cy, radius, area = current_target
            dx = cx - center[0]
            dy = center[1] - cy

            az_err_deg, el_err_deg = pixels_to_angle(dx, dy)
            err_pix = math.hypot(dx, dy)
            mode_icon = "🟢" if detection_mode == "green" else "⚪"
            print(
                f"{mode_icon} mode={detection_mode} target={target_center_px}  "
                f"Δx={dx:4d} Δy={dy:4d}  "
                f"az_err={az_err_deg:7.3f}° el_err={el_err_deg:7.3f}°  |err|={err_pix:.1f}"
            )

            # 1) Kamera gimbali hedefi merkeze getirsin
            send_to_camera_gimbal(az_err_deg, el_err_deg)

            # 2) Kameranın mutlak açısına göre lazer hedef açı hesapla
            cam_az_deg = cam_az_abs_deg
            cam_el_deg = cam_el_abs_deg

            laz_az_tgt, laz_el_tgt = camera_to_laser_angles(cam_az_deg, cam_el_deg)

            # 3) Lazeri bu hedef açılara sür
            send_to_laser_target(laz_az_tgt, laz_el_tgt)

            # 5 saniye merkezde kalma mantığı
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
                target_centered_t = None

            # Görsel overlay
            annotated = color_img.copy()
            cv2.drawMarker(
                annotated, center, (0, 255, 0),
                cv2.MARKER_CROSS, 20, 2
            )
            for (px, py, r, a) in points:
                cv2.circle(annotated, (px, py), max(r, 3), (255, 0, 0), 1)

            cv2.circle(annotated, (cx, cy), max(radius, 5), (0, 0, 255), 2)
            cv2.circle(annotated, (cx, cy), 3, (0, 255, 255), -1)

            exp_text = f"Exp={current_exp:.0f} us" if current_exp is not None else "Exp:N/A"
            cv2.putText(
                annotated, f"dx={dx}px dy={dy}px",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2
            )
            cv2.putText(
                annotated, f"az_err={az_err_deg:.2f} el_err={el_err_deg:.2f}",
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
                print("MKS seri port kapatıldı.")
            except Exception:
                pass


if __name__ == "__main__":
    track_star()