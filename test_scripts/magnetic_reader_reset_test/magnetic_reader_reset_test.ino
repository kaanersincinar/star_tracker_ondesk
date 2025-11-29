// 3 EKSEN QUADRATURE ENCODER OKUMA + TELEKOMUT RESET
// AZIMUTH:  A_PIN 7,  B_PIN 14
// X EKSENI: AX_PIN 2, BX_PIN 3
// Y EKSENI: AY_PIN 4, BY_PIN 5
// Hepsi: DEBOUNCE + 4-state state machine + tek frame'de x,y,az gönderimi

#include <Arduino.h>

// --- PINLER ---
#define AZ_A_PIN 7
#define AZ_B_PIN 14

#define X_A_PIN  2
#define X_B_PIN  3

#define Y_A_PIN  4
#define Y_B_PIN  5

// --- PROTOKOL ---
// Her Arduino için benzersiz ID ver (bu kartta 2)
#define NODE_ID     2
#define MSG_SENSOR  0x10   // Telemetri: X,Y,AZ counts frame
#define MSG_CMD_RESET 0x01 // Telekomut: reset komutu

const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE   = 0x7F;

// Debounce filtresi için bekleme süresi (us)
const unsigned long DEBOUNCE_US = 90;

// --- QUADRATURE TRANSITION TABLOSU ---
const int8_t transitionTable[4][4] = {
  // new:   00   01   10   11
  // 00
  {  0,  -1,  +1,   0 },
  // 01
  { +1,   0,   0,  -1 },
  // 10
  { -1,   0,   0,  +1 },
  // 11
  {  0,  +1,  -1,   0 }
};

// --- DEBOUNCE YAPISI ---
struct DebouncedChannel {
  uint8_t pin;
  bool stableState;         // son onaylı (debounced) durum
  bool lastRaw;             // son okunan ham değer
  unsigned long lastChange; // ham değişimin zamanı (us)
};

// --- HER EKSEN IÇIN YAPI ---
struct EncoderAxis {
  DebouncedChannel chA;
  DebouncedChannel chB;
  uint8_t lastState;
  long position;
  unsigned long illegalTransitions;
};

// Eksen nesneleri
EncoderAxis azAxis = {
  { AZ_A_PIN, false, false, 0 },
  { AZ_B_PIN, false, false, 0 },
  0, 0, 0
};

EncoderAxis xAxis = {
  { X_A_PIN, false, false, 0 },
  { X_B_PIN, false, false, 0 },
  0, 0, 0
};

EncoderAxis yAxis = {
  { Y_A_PIN, false, false, 0 },
  { Y_B_PIN, false, false, 0 },
  0, 0, 0
};

// --- TELEKOMUT RX STATE MACHINE ---

enum RxState {
  RX_WAIT_START,
  RX_READ_NODE,
  RX_READ_TYPE,
  RX_READ_LENGTH,
  RX_READ_PAYLOAD,
  RX_WAIT_END
};

RxState rxState = RX_WAIT_START;
uint8_t rxNode = 0;
uint8_t rxMsgType = 0;
uint8_t rxLength = 0;

const uint8_t MAX_PAYLOAD_LEN = 16;
uint8_t rxPayload[MAX_PAYLOAD_LEN];
uint8_t rxIndex = 0;

// --- DEBOUNCE FONKSIYONU ---
void updateDebounced(DebouncedChannel &ch) {
  bool raw = digitalRead(ch.pin);
  unsigned long now = micros();

  if (raw != ch.lastRaw) {
    ch.lastRaw = raw;
    ch.lastChange = now;
  }

  if (raw != ch.stableState) {
    if ((now - ch.lastChange) > DEBOUNCE_US) {
      ch.stableState = raw;
    }
  }
}

// --- EKSEN STATE MACHINE GÜNCELLEME ---
void updateAxis(EncoderAxis &axis) {
  updateDebounced(axis.chA);
  updateDebounced(axis.chB);

  uint8_t a = axis.chA.stableState;
  uint8_t b = axis.chB.stableState;
  uint8_t currState = (a << 1) | b;

  if (currState != axis.lastState) {
    int8_t step = transitionTable[axis.lastState][currState];

    if (step == 0) {
      axis.illegalTransitions++;
    } else {
      // Yön düzeltmesi: pozitif yönde hareket için aynı tersleme
      axis.position -= step;
    }

    axis.lastState = currState;
  }
}

// --- FRAME YAZMA YARDIMCI FONKSIYONLARI ---
void writeInt32(uint8_t *buf, uint8_t &idx, int32_t value)
{
  buf[idx++] = (value >> 24) & 0xFF;
  buf[idx++] = (value >> 16) & 0xFF;
  buf[idx++] = (value >> 8)  & 0xFF;
  buf[idx++] = (value)       & 0xFF;
}

// x, y, az -> 3 adet int32 olarak gönderiyoruz
// Frame:
// [0]  START_BYTE
// [1]  NODE_ID
// [2]  MSG_SENSOR
// [3..6]   X  (int32)
// [7..10]  Y  (int32)
// [11..14] AZ (int32)
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

  Serial.write(buf, idx);   // her zaman 16 byte
}

// --- KOMUT İŞLEME ---
// Komut geldiğinde xCount, yCount, azCount -> aslında axis.position’lar sıfırlanıyor
void processCommand(uint8_t node, uint8_t msgType, uint8_t length, uint8_t* payload)
{
  // Adres kontrolü: sadece kendi NODE_ID veya broadcast (0xFF) için
  if (node != NODE_ID && node != 0xFF) {
    return;
  }

  switch (msgType) {
    case MSG_CMD_RESET:
      // Reset komutu: tüm eksen count'larını sıfırla
      xAxis.position  = 0;
      yAxis.position  = 0;
      azAxis.position = 0;
      // İstersen illegalTransitions da sıfırlarsın:
      // xAxis.illegalTransitions = 0;
      // yAxis.illegalTransitions = 0;
      // azAxis.illegalTransitions = 0;
      break;

    default:
      // Şimdilik diğer komutlar yok; sessiz ignore
      break;
  }
}

// --- TELEKOMUT RX HANDLER ---
void handleSerialRx()
{
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (rxState) {
      case RX_WAIT_START:
        if (b == START_BYTE) {
          rxState = RX_READ_NODE;
        }
        break;

      case RX_READ_NODE:
        rxNode = b;
        rxState = RX_READ_TYPE;
        break;

      case RX_READ_TYPE:
        rxMsgType = b;
        rxState = RX_READ_LENGTH;
        break;

      case RX_READ_LENGTH:
        rxLength = b;
        if (rxLength > MAX_PAYLOAD_LEN) {
          // saçma length, paketi çöpe at
          rxState = RX_WAIT_START;
        } else if (rxLength == 0) {
          rxState = RX_WAIT_END;
        } else {
          rxIndex = 0;
          rxState = RX_READ_PAYLOAD;
        }
        break;

      case RX_READ_PAYLOAD:
        rxPayload[rxIndex++] = b;
        if (rxIndex >= rxLength) {
          rxState = RX_WAIT_END;
        }
        break;

      case RX_WAIT_END:
        if (b == END_BYTE) {
          // Tam frame geldi
          processCommand(rxNode, rxMsgType, rxLength, rxPayload);
        }
        // END doğru da olsa yanlış da olsa, başa dön
        rxState = RX_WAIT_START;
        break;
    }
  }
}

// --- INIT FONKSIYONU: HER EKSEN IÇIN ILK DURUMU OKU ---
void initAxis(EncoderAxis &axis) {
  bool initialA = digitalRead(axis.chA.pin);
  bool initialB = digitalRead(axis.chB.pin);

  axis.chA.stableState = initialA;
  axis.chA.lastRaw     = initialA;
  axis.chA.lastChange  = micros();

  axis.chB.stableState = initialB;
  axis.chB.lastRaw     = initialB;
  axis.chB.lastChange  = micros();

  axis.lastState = (initialA << 1) | initialB;
  axis.position  = 0;
  axis.illegalTransitions = 0;
}

// --- SETUP / LOOP ---

void setup() {
  pinMode(AZ_A_PIN, INPUT_PULLUP);
  pinMode(AZ_B_PIN, INPUT_PULLUP);

  pinMode(X_A_PIN, INPUT_PULLUP);
  pinMode(X_B_PIN, INPUT_PULLUP);

  pinMode(Y_A_PIN, INPUT_PULLUP);
  pinMode(Y_B_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  initAxis(azAxis);
  initAxis(xAxis);
  initAxis(yAxis);

  Serial.println("3-axis quadrature encoder (DEBOUNCED + FRAME + TELEKOMUT) basladi.");
}

void loop() {
  static unsigned long lastSend = 0;
  const unsigned long sendPeriod = 10; // ms -> 100 Hz frame

  // 1) Gelen telekomutları işle
  handleSerialRx();

  // 2) Tüm eksenleri güncelle
  updateAxis(azAxis);
  updateAxis(xAxis);
  updateAxis(yAxis);

  // 3) Periyodik frame gönder
  unsigned long now = millis();
  if (now - lastSend >= sendPeriod) {
    lastSend = now;

    int32_t xCounts  = (int32_t)xAxis.position;
    int32_t yCounts  = (int32_t)yAxis.position;
    int32_t azCounts = (int32_t)azAxis.position;

    sendXYZ(xCounts, yCounts, azCounts);
  }
}
