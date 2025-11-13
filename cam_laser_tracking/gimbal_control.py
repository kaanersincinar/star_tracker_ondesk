from pypylon import pylon
import cv2
import numpy as np
import math
import time
import serial
import config as cfg

SER_ENABLED = True
# -------------------------------------------------
#  Seri port 
# -------------------------------------------------
def init_serial():
    """Marlin kart ile seri haberleşmeyi başlat."""
    global ser
    if not SER_ENABLED:
        print("Seri port devre dışı (SER_ENABLED = False).")
        ser = None
        return

    try:
        ser = serial.Serial(cfg.SER_MKS_PORT, cfg.BAUD_MKS, timeout=0.01)
        print(f"Seri port açıldı: {cfg.SER_MKS_PORT} @ {cfg.BAUD_MKS}")
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


