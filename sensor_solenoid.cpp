#include "esp32-hal-adc.h"
#include "sensor_solenoid.h"
#include "esp32s3/ulp.h"  //  this also includes ulp_common.h
#include <HardwareSerial.h>
#include <stdbool.h>
#include "driver/uart.h"
#include <Arduino.h>

// Define your pin constants somewhere accessible:
// ---- Hardware mapping (Heltec E290 / ESP32-S3) ----
static const int PIN_TX = PIN_RS485_TX;  // UART1 TXD
static const int PIN_RX = PIN_RS485_RX;  // UART1 RXD
static const int PIN_RTS = RS485_DE;     // RS-485 DE/RE (wired together), driven by RTS

// ---- Modbus settings ----
static const uint8_t SLAVE_ID = 0x01;
static const uint32_t BAUDRATE = 9600;
static const uint16_t REG_START = 0x0000;  // holding register start
static const uint16_t REG_COUNT = 1;       // read one 16-bit register

extern uint32_t TxDutyCycle_hold;
extern uint32_t appTxDutyCycle;
extern uint32_t txDutyCycleTime;
// Modbus master instance

#define RS485_SERIAL Serial1
#define RS485_TX_ENABLE RS485_DE
#define RS485_EN RS485_DE
constexpr uart_port_t RS485_PORT = UART_NUM_1;  // match Serial1

uint8_t aTxBuffer0[8] = { 0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xcb };  // first soil moisture sensor message
uint8_t aTxBuffer1[8] = { 0x02, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xf8 };  //  second soil moisture message
uint8_t aTxBuffer3[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd };  //  for three metal proble soil moisture sensor
uint8_t aTxBuffer4[8] = { 0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xc5, 0xcb };  //  for three metal proble soil moisture sensor
uint8_t aSoilSensShallow[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09 };
uint8_t aSoilSensDeep[] = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x3a };
// Query to read 2 input registers (pressure), from address 0x0001
const uint8_t depthQuery[8] = { 0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xC5, 0xCB };
uint8_t wPres[2] = { 0x01, 0x02 };  // raw uncalibrated adc output
uint8_t sTempC[4], sMoist[4];       //  for final data from soil probes
uint8_t aRx[10];                    //  for rs-485 returned data from soil probes
uint8_t soilSensorOut[6];           //  for the two soil sensors including moisture, temp and pH
extern uint8_t bat_pct;

// forward decl
static void dumpHex(const char* tag, const uint8_t* buf, size_t len);



// Hardware pin setup
void hardware_pins_init(void) {
  // Set input pins
  pinMode(PIN_VE_CTL, OUTPUT);
  digitalWrite(PIN_VE_CTL, HIGH);  // power to external functions of the MCU
  pinMode(PIN_REED_P, INPUT_PULLUP);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);  // listen
  pinMode(ADC_CTL_PIN, OUTPUT);
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  pinMode(VBAT_READ_PIN, INPUT);                      // not needed
  analogSetPinAttenuation(VBAT_READ_PIN, ADC_2_5db);  //  6db reduces voltage to 1/2 of input

  pinMode(EPD_BUSY_PIN, INPUT);  //  status of the  epaper
  //analogReadResolution(12); // 12 bit res
  // Set solenoid control pins as outputs, start is locked high Z state
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Set EN_PWR as output, and start with this on
  pinMode(PIN_EN_SENSE_PWR, OUTPUT);
  digitalWrite(PIN_EN_SENSE_PWR, HIGH);
  // Set up UART pins if needed separately
  // (Typically UART driver will configure TX/RX pins automatically)
}

// Map valves 1→index 0, 2→index 1
static const uint8_t forwardPins[] = { PIN_IN1, PIN_IN3 };
static const uint8_t reversePins[] = { PIN_IN2, PIN_IN4 };
static const size_t valveCount = sizeof(forwardPins) / sizeof(forwardPins[0]);

void controlValve(uint8_t valve_number, uint8_t status) {
  if (valve_number >= valveCount) return;

  const uint8_t pinF = forwardPins[valve_number];
  const uint8_t pinR = reversePins[valve_number];

  // drive for the pulse
  digitalWrite(pinF, status ? HIGH : LOW);   // 1=ON, 0=OFF
  digitalWrite(pinR, status ? LOW  : HIGH);

  delay(120);

  // release to LOW (coast) for drv 8871
  digitalWrite(pinF, LOW);   // 
  digitalWrite(pinR, LOW);   // 
}

// return a 8 bit '% charged' that is  proportional to the range of voltages from 3.5 to 4.1
uint8_t bat_cap8() {
  const int16_t mid = 850;
  const int16_t span = 150;  // 1000–850

  Serial.print("in bat_cap8()\n");
  digitalWrite(ADC_CTL_PIN, HIGH);
  delay(40);
  uint16_t raw = analogRead(VBAT_READ_PIN);
  Serial.printf("read bat_cap8() pin_adc_ctl HIGH = %u\n", raw);
  digitalWrite(ADC_CTL_PIN, LOW);

  int16_t diff = raw - mid;    // –∞…+∞
  if (diff < 0) diff = -diff;  // abs(diff)
  uint16_t udiff = (uint16_t)diff;
  // now scale 0…150 → 0…100
  uint16_t pct = uint16_t(diff * 100 / span);
  if (pct > 100) pct = 100;  // clamp

  RTC_SLOW_MEM[ULP_BAT_PCT] = pct;  //  ULP_BAT_PCT is defined in the .h
  return ((uint8_t)(pct * 2.55));   //  lorawan.cpp wants this full byte form
}

// Replaces your readMCP3421avg_cont() with error handling and timeouts.
uint16_t readMCP3421avg_cont() {
  const uint8_t addr = 0x68;
  const uint8_t cfgCont = 0b10011000;  // continuous, 16-bit, PGA=1
  const uint8_t samples = 8;
  const uint32_t rdy_timeout_ms = 200;  // 16-bit conversion ≈ 67 ms → 200 ms cap
  const uint32_t poll_delay_ms = 5;

  // Persist across calls
  static uint8_t fail_streak = 0;
  static uint16_t last_good = 0;  // 0 until first valid read

  int32_t sum = 0;
  uint8_t ok = 0;

  // Start continuous conversions (ignore non-zero return as soft error)
  Wire.beginTransmission(addr);
  Wire.write(cfgCont);
  (void)Wire.endTransmission();

  for (uint8_t i = 0; i < samples; i++) {
    uint32_t deadline = millis() + rdy_timeout_ms;

    for (;;) {
      int n = Wire.requestFrom(addr, (uint8_t)3);
      if (n == 3) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint8_t stat = Wire.read();

        // RDY==0 → data valid
        if ((stat & 0x80) == 0) {
          int16_t raw = (int16_t)((uint16_t)msb << 8 | lsb);  // sign-correct
          sum  += raw;
          ok++;
          break;  // next sample
        }
      } else {
        // drain any partial bytes to keep the bus clean
        while (Wire.available()) (void)Wire.read();
      }

      if ((int32_t)(millis() - deadline) >= 0) {
        // give up on this sample and continue
        break;
      }
      delay(poll_delay_ms);
    }
  }

  // Stop continuous by triggering a one-shot (don’t care if this NACKs)
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0b10000000);
  (void)Wire.endTransmission();

  if (ok) {
    fail_streak = 0;
    last_good = (int16_t)(sum / (int32_t)ok);  // average of valid samples only
    uint16_t raw_u = (int16_t(last_good < 0)) ? 0 : (uint16_t)last_good;
    return raw_u;
  }

  // No valid samples — try a soft bus reset occasionally, then return last value
  if (++fail_streak >= 3) {
    Wire.end();
    delay(2);
    Wire.begin(PIN_SDA, PIN_SCL);  // assumes you’ve called Wire.begin(...) in setup()
    fail_streak = 0;
  }
  return uint16_t(last_good);  // graceful fallback instead of wedging
}

// Assume PIN_EN_SENSE_PWR was pinMode’d to OUTPUT in setup()
void setPowerEnable(uint8_t powerState) {
  digitalWrite(PIN_EN_SENSE_PWR, powerState ? HIGH : LOW);
}

void RS485Sub(uint8_t depth);

static const uint8_t* const buffers[] = { aTxBuffer0, aTxBuffer1 };
static const size_t bufLens[] = { sizeof(aTxBuffer0), sizeof(aTxBuffer1) };

// CHANGED: enforce Modbus silent gap and TX→RX turnaround
void RS485Send(uint8_t depth) {
  if (depth > 1) return;
  const uint32_t t_char_us = (11UL * 1000000UL) / 9600;  // 1 char at 9600 baud
  const uint32_t t3_5_us = (35UL * t_char_us) / 10UL;

  // Pre-frame silence
  delayMicroseconds(t3_5_us);

  digitalWrite(RS485_DE, HIGH);
  RS485_SERIAL.write(buffers[depth], bufLens[depth]);
  RS485_SERIAL.flush(true);
  delayMicroseconds(t_char_us);
  digitalWrite(RS485_DE, LOW);
}

// ----------------------
// Generic Modbus helpers
// ----------------------

//helper to send raw frame with silent gap and TX→RX turnaround
static void RS485SendFrame(const uint8_t* frame, size_t len, uint32_t baud = 9600) {
  const uint32_t t_char_us = (11UL * 1000000UL) / baud;
  const uint32_t t3_5_us = (35UL * t_char_us) / 10UL;
  delayMicroseconds(t3_5_us);
  delay(10);
  // print TX frame before driving DE high
  dumpHex("TX", frame, len);
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(t3_5_us);
  delay(10);
  RS485_SERIAL.write(frame, len);
  RS485_SERIAL.flush(true);
  delayMicroseconds(t_char_us);
  delayMicroseconds(200);
  digitalWrite(RS485_DE, LOW);
}

// generic Modbus RTU read (function 0x03)
bool readModbusFrame_dbg(const char* file, int line,
                         uint8_t addr, uint16_t startReg, uint16_t regCount,
                         uint8_t* outBuf, size_t outMax,
                         uint32_t baud, uint32_t frame_timeout_ms) {
  Serial.printf("TX req @%s:%d -> addr=%u start=0x%04X regs=%u\n", file, line, addr, startReg, regCount);


  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x03;
  req[2] = (startReg >> 8) & 0xFF;
  req[3] = startReg & 0xFF;
  req[4] = (regCount >> 8) & 0xFF;
  req[5] = regCount & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  Serial.printf("TX req: addr=%u start=0x%04X regs=%u\n", addr, startReg, regCount);
  RS485SendFrame(req, sizeof(req), baud);

  // expected normal reply length for function 0x03
  const size_t expected = 1 + 1 + 1 + (2 * regCount) + 2;  // addr + fc + byteCount + data + crc
  if (expected > outMax) return false;

  RS485_SERIAL.setTimeout(frame_timeout_ms);
  size_t len = RS485_SERIAL.readBytes(outBuf, expected);
  Serial.printf("RX %u/%u bytes\n", (unsigned)len, (unsigned)expected);

  if (len > 0) dumpHex("RX", outBuf, len);

  // NEW: handle Modbus exception frame (5 bytes: addr, 0x83, code, CRClo, CRChi)
  if (len == 5 && (outBuf[1] == (0x80 | 0x03))) {
    uint16_t exc_crc = modbusCRC(outBuf, 3);
    uint16_t exc_rx = (uint16_t)outBuf[3] | ((uint16_t)outBuf[4] << 8);
    if (exc_crc == exc_rx) {
      Serial.printf("Modbus exception: code=0x%02X (addr=%u)\n", outBuf[2], outBuf[0]);
    } else {
      Serial.println("Exception frame CRC mismatch");
    }
    return false;
  }

  // For a normal reply, we must have the full expected length
  if (len != expected) return false;

  // Basic header checks
  if (outBuf[0] != addr || outBuf[1] != 0x03) return false;

  // NEW: verify device-reported byte count matches requested register count
  if (outBuf[2] != (regCount * 2)) {
    Serial.printf("Byte count mismatch: got=%u expected=%u\n", outBuf[2], (unsigned)(regCount * 2));
    return false;
  }

  // CRC check
  uint16_t rx_crc = (uint16_t)outBuf[len - 2] | ((uint16_t)outBuf[len - 1] << 8);
  uint16_t calced = modbusCRC(outBuf, len - 2);
  if (rx_crc != calced) return false;

  return true;
}


bool readFrame(uint8_t depth, uint8_t header, int& outIdx) {
  constexpr uint8_t maxRetries = 3;
  for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
    RS485Sub(depth);
    memset(aRx, 0, sizeof(aRx));
    size_t len = Serial2.readBytes(aRx, sizeof(aRx));
    for (int i = 0; i + 3 < len; i++) {
      if (aRx[i] == header && aRx[i + 1] == 0x03 && aRx[i + 2] == 0x04) {
        outIdx = i;
        return true;
      }
    }
  }
  return false;
}

void RS485Get() {
  uint8_t buf[64];

  if (readModbusFrame(0x01, 0x0001, 2, buf, sizeof(buf))) {
    sTempC[0] = buf[3];
    sTempC[1] = buf[4];
    sMoist[0] = buf[5];
    sMoist[1] = buf[6];
  }

  if (readModbusFrame(0x02, 0x0001, 2, buf, sizeof(buf))) {
    sTempC[2] = buf[3];
    sTempC[3] = buf[4];
    sMoist[2] = buf[5];
    sMoist[3] = buf[6];
  }
}


void initRS485(uint16_t baud) {
  RS485_SERIAL.end();
  RS485_SERIAL.begin(baud, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);  // RX=44, TX=43
}

// ADDED: simple UART loopback self-test (TX->RX jumper at MCU pins 43↔44)
void rs485_uart_loopback_test(uint32_t baud = 9600) {
  Serial.printf("UART loopback: RX=%d TX=%d, baud=%u (jumper TX->RX on MCU pins)\n",
                PIN_RS485_RX, PIN_RS485_TX, (unsigned)baud);

  // Ensure RS-485 transceiver is NOT driving the bus
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);  // TX driver off, RX enabled (since DE=/RE tied)

  RS485_SERIAL.end();
  RS485_SERIAL.begin(baud, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // Known pattern
  const uint8_t tx[] = { 0x55, 0xAA, 0x12, 0x34, 0xBE, 0xEF, 0x00, 0xFF, 0x5A, 0xA5, 0xDE, 0xAD, 0xC0, 0xDE, 0x13, 0x37 };
  uint8_t rx[sizeof(tx)] = { 0 };

  // Flush & send
  RS485_SERIAL.flush();
  size_t wrote = RS485_SERIAL.write(tx, sizeof(tx));
  RS485_SERIAL.flush();

  // Read back exactly the same number of bytes
  RS485_SERIAL.setTimeout(100);  // 100 ms should be plenty for 16 bytes @ 9600
  size_t got = RS485_SERIAL.readBytes(rx, sizeof(rx));

  // Report
  Serial.printf("LOOPBACK wrote=%u, got=%u\n", (unsigned)wrote, (unsigned)got);
  if (got > 0) {
    // Use your existing dumpHex if present; otherwise inline dump:
    for (size_t i = 0; i < got; ++i) Serial.printf("%02X%s", rx[i], (i + 1 < got) ? " " : "\n");
  }
  bool pass = (got == sizeof(tx)) && (memcmp(tx, rx, sizeof(tx)) == 0);
  Serial.println(pass ? "LOOPBACK: PASS" : "LOOPBACK: FAIL");
}

// Wait until the bus is idle for >= 3.5 characters (Modbus RTU pre-silence)
static bool waitBusIdle(uint32_t baud, uint32_t timeout_ms = 100) {
  const uint32_t char_us = (uint32_t)(11.0e6 / baud);  // 8N1 ≈ 11 bits/char
  const uint32_t min_idle_us = 4 * char_us;            // ≥ 3.5 chars
  uint32_t idle_start = micros();
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeout_ms) {
    if (RS485_SERIAL.available()) {
      while (RS485_SERIAL.available()) (void)RS485_SERIAL.read();
      idle_start = micros();
    }
    if ((uint32_t)(micros() - idle_start) >= min_idle_us) return true;
    delayMicroseconds(50);
  }
  return false;  // bus stayed busy
}

inline uint16_t lake_depth_mm_from_raw(int32_t raw) {
  if (inv_m_u32 == 0) return 0;  // guard
  // wide, signed math; avoid overflow on *100
  int64_t num = static_cast<int64_t>(raw);
  int64_t scale = static_cast<int64_t>(inv_m_u32);
  int64_t bx10 = static_cast<int64_t>(b_x10);

  // depth_m = raw / inv_m_u32 + b_x10/10
  // return depth_m * 100 as uint16_t with clamp 0..65535
  int64_t depth = (num) / scale + (bx10/10);  // (b_x10/10)*100 = b_x10*10
  if (depth < 0) depth = 0;
  if (depth > 65535) depth = 65535;
  return static_cast<uint16_t>(depth);
}

// ---- Read depth sensor via raw Modbus RTU (manual DE) ---------------------------
// Sends: 01 03 00 04 00 01 C5 CB
// Expects: 01 03 02 HI LO CRC_L CRC_H
// Returns: (HI<<8)|LO on success, 0xFFFF on failure after retries.
bool readDepthSensor(unsigned long timeout_ms = 200, uint8_t max_tries = 7) {
  static bool inited = false;
  if (!inited) {
    // UART1 on pins RX=44, TX=43. 9600 8N1 (no parity) — as we found works.
    Serial1.begin(9600, SERIAL_8N1, 44, 43);
    pinMode(RS485_DE, OUTPUT);
    digitalWrite(RS485_DE, LOW);  // receive by default
    inited = true;
  }

  // Build request: [01][03][00 07][00 02][CRC_L][CRC_H]
  uint8_t req[8] = { 0x01, 0x03, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00 };
  {
    uint16_t crc = modbusCRC(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);
  }

  for (uint8_t attempt = 1; attempt <= max_tries; ++attempt) {
    // purge RX
    while (Serial1.available()) (void)Serial1.read();

    // --- transmit with DE high ---
    digitalWrite(RS485_DE, HIGH);  // drive bus
    size_t sent = Serial1.write(req, sizeof(req));
    Serial1.flush();               // wait for TX FIFO to drain
    delayMicroseconds(300);        // ~3.5 char times @ 9600
    digitalWrite(RS485_DE, LOW);   // back to receive

    if (sent != sizeof(req)) { delay(5); continue; }

    // --- collect until idle gap ---
    uint8_t rx[64];
    size_t n = 0;
    unsigned long t0 = millis(), last_rx = 0;
    const unsigned idle_gap_ms = 5;

    while ((millis() - t0) < timeout_ms) {
      while (Serial1.available()) {
        if (n < sizeof(rx)) rx[n++] = (uint8_t)Serial1.read();
        last_rx = millis();
      }
      if (n && Serial1.available() == 0 && (millis() - last_rx) >= idle_gap_ms) break;
    }

    // Need at least 9 bytes: 01 03 04 [HI7 LO7 HI8 LO8] CRC_L CRC_H
    if (n < 9) { delay(15); continue; }

    // Scan for a valid frame tail-first (handles line noise / concatenation)
    for (int i = (int)n - 9; i >= 0; --i) {
      if (rx[i+0] == 0x01 && rx[i+1] == 0x03 && rx[i+2] == 0x04) {
        // CRC over first 7 bytes
        uint16_t crc_calc = modbusCRC(&rx[i], 7);
        uint16_t crc_recv = (uint16_t)rx[i + 7] | ((uint16_t)rx[i + 8] << 8);
        if (crc_calc != crc_recv) continue;

        // Two 16-bit registers (big-endian within each register)
        uint16_t r7 = ((uint16_t)rx[i + 3] << 8) | rx[i + 4];
        uint16_t r8 = ((uint16_t)rx[i + 5] << 8) | rx[i + 6];

        // Combine into uint32_t (Modbus default word order: reg7=high word, reg8=low word)
        uint32_t v = ((uint32_t)r7 << 16) | (uint32_t)r8;
        Serial.printf("raw data for lake:  %u \n", v);
        lakeDepth32Raw = (v < 1000000000) ? 0 : (v - 1000000000);
        sensorMeasuredDepth = lake_depth_mm_from_raw(lakeDepth32Raw);    //  
        lake_level_mm = (int)sensorMeasuredDepth - (int)g_lake_depth_mm;
        Serial.printf("sensorMeasuredDepth:  %u \n", sensorMeasuredDepth);
        Serial.printf("lakeLevel:  %i \n", lake_level_mm);
        Serial.printf("g_lake_depth_mm:  %i \n", g_lake_depth_mm);
        g_need_display = true;
        return true;
      }
    }

    delay(20);
  }

  // Leave previous lakeDepth32Raw untouched on failure; signal false
  return false;
}
/* read soil sensor by RS-485, check CRC and fill global array 
    soilSensorOut with data
    call ith the number of  sensors in the node, but will work if one call with 2 has only one node
    as the second one will not reply and it wil report as zero.
    
*/

static bool readSoilSensor(uint8_t depth, uint8_t header, int& outIdx) {
  constexpr uint8_t maxRetries = 3;
  for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
    RS485Send(depth);
    memset(aRx, 0, sizeof(aRx));
    size_t len = RS485_SERIAL.readBytes(aRx, sizeof(aRx));
    for (int i = 0; i + 3 < len; i++) {
      if (aRx[i] == header && aRx[i + 1] == 0x03 && aRx[i + 2] == 0x04) {
        outIdx = i;
        return true;
      }
    }
  }
  return false;
}

bool buildModbusRequest(uint8_t slaveAddr, uint16_t regStart, uint16_t regCount, uint8_t (&request)[8]) {
  request[0] = slaveAddr;
  request[1] = 0x03;  // Function code: Read Holding Registers
  request[2] = (regStart >> 8) & 0xFF;
  request[3] = regStart & 0xFF;
  request[4] = (regCount >> 8) & 0xFF;
  request[5] = regCount & 0xFF;

  uint16_t crc = modbusCRC(request, 6);
  request[6] = crc & 0xFF;         // CRC Lo
  request[7] = (crc >> 8) & 0xFF;  // CRC Hi

  return true;  // for chaining or logging if desired
}

uint16_t modbusCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;  // LSB-first when sending over Modbus
}

bool appendSensorToPayload(uint8_t addr, uint16_t startReg, uint16_t regCount,
                           uint8_t* payload, size_t* pos, size_t max,
                           uint32_t baud = 9600, uint32_t timeout_ms = 80) {
  uint8_t rx[64];
  if (!readModbusFrame(addr, startReg, regCount, rx, sizeof(rx), baud, timeout_ms)) {
    return false;  // no append on failure
  }
  const size_t need = 1 + (size_t)regCount * 2;  // addr + data bytes
  if (*pos + need > max) return false;           // prevent overflow

  payload[(*pos)++] = addr;  // tag this chunk with sensor addr
  memcpy(&payload[*pos], &rx[3], regCount * 2);
  *pos += regCount * 2;
  return true;
}

// CHANGED: build a compact LoRaWAN payload for two soil probes
// Returns number of bytes written to `out`.
size_t buildLoRaWANPayload(uint8_t* out, size_t max,
                           uint32_t baud = 9600, uint32_t timeout_ms = 80) {
  size_t pos = 0;
  (void)appendSensorToPayload(0x01, 0x0001, 2, out, &pos, max, baud, timeout_ms);
  (void)appendSensorToPayload(0x02, 0x0001, 2, out, &pos, max, baud, timeout_ms);
  return pos;
}

// CHANGED: debug hex dump helper
static void dumpHex(const char* tag, const uint8_t* buf, size_t len) {
  Serial.printf("%s (%u):", tag, (unsigned)len);
  for (size_t i = 0; i < len; ++i) Serial.printf(" %02X", buf[i]);
  Serial.println();
}
// --- Safe jitter add with floor (ms) ---
inline uint32_t addJitterClampMin(uint32_t base_ms, int32_t jitter_ms, uint32_t min_ms) {
  int64_t t = (int64_t)base_ms + (int64_t)jitter_ms;  // signed math, no wrap
  if (t < (int64_t)min_ms) t = (int64_t)min_ms;       // enforce ≥ min
  if (t > 0xFFFFFFFFLL) t = 0xFFFFFFFFLL;             // paranoia clamp
  return (uint32_t)t;
}

// --- One-shot: save current cycle, set valve-on cycle, schedule next TX (ms) ---
// Uses globals: TxDutyCycle_hold, appTxDutyCycle, txDutyCycleTime
// Assumes CYCLE_TIME_VALVE_ON is in milliseconds (same units as appTxDutyCycle).
void scheduleValveOnCycle(void) {


  TxDutyCycle_hold = appTxDutyCycle;     // save current
  appTxDutyCycle = CYCLE_TIME_VALVE_ON;  // switch to valve-on cycle
  txDutyCycleTime = addJitterClampMin(
    appTxDutyCycle,
    randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND),
    10u * 60000u  // hard floor: 10 minutes
  );
}
