// DUMMY 3-AXIS MAGNETIC RULER SENDER
// Gercek encoder yokken Python tarafina ayni frame formatinda sahte veri yollar.

#include <Arduino.h>

// --- PROTOKOL ---
// Bu kartin Node ID'si (senin mevcut koddakiyle ayni olmasi lazim)
#define NODE_ID     1
#define MSG_SENSOR  0x10

const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE   = 0x7F;

// --- SERIAL & FREKANS ---
// Python tarafinda cfg.BAUD_ARDUINO ne ise, burada da o olmali (mevcut sketch'te 115200)
const unsigned long SEND_PERIOD_MS = 10; // 100 Hz frame

// --- DUMMY COUNT ARALIKLARI ---
// Bu sayilar tamamen keyfi; sadece ileri-geri gezinen bir pattern.
const int32_t X_MIN = -100000;
const int32_t X_MAX =  100000;

const int32_t Y_MIN = -50000;
const int32_t Y_MAX =  50000;

const int32_t AZ_MIN = -200000;
const int32_t AZ_MAX =  200000;

// Her adimda eklenecek count
int32_t xStep = 500;
int32_t yStep = 300;
int32_t azStep = 800;

// Dummy sayaclar
int32_t xCnt = 0;
int32_t yCnt = 0;
int32_t azCnt = 0;

// --- int32'yi big-endian olarak buffer'a yaz ---
void writeInt32(uint8_t *buf, uint8_t &idx, int32_t value)
{
  buf[idx++] = (uint8_t)((value >> 24) & 0xFF);
  buf[idx++] = (uint8_t)((value >> 16) & 0xFF);
  buf[idx++] = (uint8_t)((value >> 8)  & 0xFF);
  buf[idx++] = (uint8_t)( value        & 0xFF);
}

// --- X,Y,AZ frame gonder ---
// Frame:
// [0]  START_BYTE
// [1]  NODE_ID
// [2]  MSG_SENSOR
// [3..6]   X  (int32, big-endian, signed)
// [7..10]  Y  (int32, big-endian, signed)
// [11..14] AZ (int32, big-endian, signed)
// [15] END_BYTE
void sendXYZ(int32_t x, int32_t y, int32_t az)
{
  uint8_t buf[16];
  uint8_t idx = 0;

  buf[idx++] = START_BYTE;
  buf[idx++] = NODE_ID;
  buf[idx++] = MSG_SENSOR;

  writeInt32(buf, idx, x);
  writeInt32(buf, idx, y);
  writeInt32(buf, idx, az);

  buf[idx++] = END_BYTE;

  // idx her zaman 16 olmali
  Serial.write(buf, idx);
}

void setup()
{
  Serial.begin(115200);  // Mevcut projede kullandigin BAUD_ARDUINO ile ayni

  // Biraz info log
  Serial.println();
  Serial.println(F("=== DUMMY MAGNETIC RULER MODE ==="));
  Serial.println(F("Gercek encoder yok, 3 eksen icin sahte X/Y/AZ count gonderiyorum (100 Hz)."));
}

void loop()
{
  static unsigned long lastSend = 0;
  unsigned long now = millis();

  if (now - lastSend < SEND_PERIOD_MS) {
    return;
  }
  lastSend = now;

  // --- Dummy pattern guncelle ---

  // X ekseni: ileri-geri sawtooth
  xCnt += xStep;
  if (xCnt > X_MAX || xCnt < X_MIN) {
    xStep = -xStep;
    xCnt += xStep;  // sinir disina tasma olursa geri cek
  }

  // Y ekseni
  yCnt += yStep;
  if (yCnt > Y_MAX || yCnt < Y_MIN) {
    yStep = -yStep;
    yCnt += yStep;
  }

  // AZ ekseni
  azCnt += azStep;
  if (azCnt > AZ_MAX || azCnt < AZ_MIN) {
    azStep = -azStep;
    azCnt += azStep;
  }

  // Frame gonder
  sendXYZ(xCnt, yCnt, azCnt);
}
