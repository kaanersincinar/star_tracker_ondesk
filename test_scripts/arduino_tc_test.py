import serial
import time

# --- Seri port ayarları ---
PORT = "COM7"   # Windows: "COM7" vb.
BAUD = 115200
INTERVAL_S = 5.0        # Her 5 saniyede bir RESET

# --- Protokol sabitleri ---
START_BYTE    = 0x7E
END_BYTE      = 0x7F
NODE_ID       = 2

MSG_SENSOR    = 0x10   # Arduino'dan gelen X/Y/AZ frame
MSG_CMD_RESET = 0x01   # PC -> Arduino reset komutu

# RESET frame: 7E 02 01 00 7F
RESET_FRAME = bytes([START_BYTE, NODE_ID, MSG_CMD_RESET, 0x00, END_BYTE])


def parse_frame(frame: bytes):
    """
    7E ... 7F arasındaki bir frame'i parse eder.
    Sensor frame (len=16, msg=0x10) ise X/Y/AZ int32'yi decode eder.
    Diğerlerini raw hex olarak gösterir.
    """
    if len(frame) < 5:
        print(f"RX (short) : {frame.hex(' ').upper()}")
        return

    start = frame[0]
    end = frame[-1]
    if start != START_BYTE or end != END_BYTE:
        print(f"RX (invalid boundary) : {frame.hex(' ').upper()}")
        return

    node = frame[1]
    msg_type = frame[2]

    # Sensör frame'i: 16 byte, length yok, sabit format
    if len(frame) == 16 and msg_type == MSG_SENSOR:
        # 7E | node | 0x10 | X(4) | Y(4) | AZ(4) | 7F
        x_bytes  = frame[3:7]
        y_bytes  = frame[7:11]
        az_bytes = frame[11:15]

        x = int.from_bytes(x_bytes,  byteorder='big', signed=True)
        y = int.from_bytes(y_bytes,  byteorder='big', signed=True)
        az = int.from_bytes(az_bytes, byteorder='big', signed=True)

        print(f"RX SENSOR  node={node}  X={x}  Y={y}  AZ={az}")
    else:
        # Komut/cevap vb. (reset için 7E 02 01 00 7F gibi)
        print(f"RX RAW     : {frame.hex(' ').upper()}")


def main():
    print(f"{PORT} @ {BAUD} açılıyor...")
    with serial.Serial(PORT, BAUD, timeout=0.01) as ser:
        time.sleep(2)
        print("Bağlantı kuruldu.")

        rx_buffer = bytearray()
        last_reset = time.time()
        reset_counter = 1

        while True:
            now = time.time()

            # --- 1) Periyodik RESET komutu gönder ---
            if now - last_reset >= INTERVAL_S:
                ser.write(RESET_FRAME)
                print(f"TX RESET [{reset_counter}] → {RESET_FRAME.hex(' ').upper()}")
                reset_counter += 1
                last_reset = now

            # --- 2) Gelen veriyi topla ---
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                rx_buffer.extend(data)

                # Start–End arası frame ayıklama
                while True:
                    start_idx = rx_buffer.find(bytes([START_BYTE]))
                    if start_idx == -1:
                        # Start yok, komple çöpe at
                        rx_buffer.clear()
                        break

                    end_idx = rx_buffer.find(bytes([END_BYTE]), start_idx + 1)
                    if end_idx == -1:
                        # End yok, frame tamamlanmamış → daha data bekle
                        # Start'tan öncesini drop et
                        if start_idx > 0:
                            del rx_buffer[:start_idx]
                        break

                    # Tam frame var
                    frame = rx_buffer[start_idx:end_idx + 1]
                    # Kullanılan kısmı buffer'dan düş
                    del rx_buffer[:end_idx + 1]

                    parse_frame(frame)

            # CPU'yu yakmamak için hafif uyku
            time.sleep(0.005)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDurduruldu.")
    except serial.SerialException as e:
        print(f"Seri port hatası: {e}")
