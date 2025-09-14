#include "esp32-hal-adc.h"
#include "sensor_solenoid.h"
#include "esp32s3/ulp.h"  //  this also includes ulp_common.h 
#include <HardwareSerial.h>
#include <stdbool.h>
#include "driver/uart.h"
#include <Arduino.h>

// Define your pin constants somewhere accessible:

#define RS485_SERIAL    Serial2
#define RS485_TX_ENABLE RS485_DE

CMCP3421 adc(0.1692);

uint8_t aTxBuffer0[8] = { 0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xcb }; // first soil moisture sensor message
uint8_t aTxBuffer1[8] = { 0x02, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xf8 }; //  second soil moisture message
uint8_t aTxBuffer3[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd };  //  for three metal proble soil moisture sensor
uint8_t aTxBuffer4[8] = { 0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xc5, 0xcb };  //  for three metal proble soil moisture sensor
uint8_t aSoilSensShallow[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09 };
uint8_t aSoilSensDeep[]    = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x3a };
// Query to read 2 input registers (pressure), from address 0x0001
const uint8_t depthQuery[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd };
uint8_t wPres[2] = {0x01, 0x02}; // raw uncalibrated adc output
uint8_t sTempC[4], sMoist[4];   //  for final data from soil probes
uint8_t aRx[10];    //  for rs-485 returned data from soil probes
uint8_t soilSensorOut[6];  //  for the two soil sensors including moisture, temp and pH
extern uint8_t bat_pct;
extern HardwareSerial Serial1;

// forward decl 
static void dumpHex(const char* tag, const uint8_t* buf, size_t len);



// Hardware pin setup
void hardware_pins_init(void) {
    // Set input pins
    pinMode(PIN_VE_CTL, OUTPUT);
    digitalWrite(PIN_VE_CTL, HIGH); // power to external functions of the MCU
    pinMode(PIN_REED_P, INPUT_PULLUP);
    pinMode(RS485_DE, OUTPUT);
    pinMode(ADC_CTL_PIN, OUTPUT);

    pinMode(VBAT_READ_PIN, INPUT);  // not needed
    analogSetPinAttenuation(VBAT_READ_PIN,ADC_2_5db);  //  6db reduces voltage to 1/2 of input

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
    if (valve_number < 0 || valve_number > 1) return;

    size_t idx = valve_number;
    uint8_t pinF = forwardPins[idx];
    uint8_t pinR = reversePins[idx];

    digitalWrite(pinF, status ? HIGH : LOW);
    digitalWrite(pinR, status ? LOW  : HIGH);

    delay(200);

    digitalWrite(pinF, LOW);
    digitalWrite(pinR, LOW);
}

// return a 8 bit '% charged' that is  proportional to the range of voltages from 3.5 to 4.1
uint8_t bat_cap8(){
  const int16_t mid = 850;
  const int16_t span = 150;  // 1000–850

  Serial.print("in bat_cap8()\n");
  digitalWrite(ADC_CTL_PIN, HIGH);
  delay(40);
  uint16_t raw = analogRead(VBAT_READ_PIN);
  Serial.printf("read bat_cap8() pin_adc_ctl HIGH = %u\n", raw);
  digitalWrite(ADC_CTL_PIN, LOW);

  int16_t diff = raw - mid;                // –∞…+∞
  if (diff < 0) diff = -diff;              // abs(diff)
  uint16_t udiff = (uint16_t)diff;
  // now scale 0…150 → 0…100
  uint16_t pct = uint16_t(diff * 100 / span);
  if (pct > 100) pct = 100;                // clamp

  RTC_SLOW_MEM[ULP_BAT_PCT] = pct;  //  ULP_BAT_PCT is defined in the .h
  return ((uint8_t)(pct * 2.55));   //  lorawan.cpp wants this full byte form
}

// Replaces your readMCP3421avg_cont() with error handling and timeouts.
uint16_t readMCP3421avg_cont() {
  const uint8_t addr    = 0x68;
  const uint8_t cfgCont = 0b10011000; // continuous, 16-bit, PGA=1
  const uint8_t samples = 8;
  const uint32_t rdy_timeout_ms = 200;  // 16-bit conversion ≈ 67 ms → 200 ms cap
  const uint32_t poll_delay_ms  = 5;

  // Persist across calls
  static uint8_t  fail_streak = 0;
  static uint16_t last_good   = 0;      // 0 until first valid read

  int32_t sum = 0;
  uint8_t ok  = 0;

  // Start continuous conversions (ignore non-zero return as soft error)
  Wire.beginTransmission(addr);
  Wire.write(cfgCont);
  (void)Wire.endTransmission();

  for (uint8_t i = 0; i < samples; i++) {
    uint32_t deadline = millis() + rdy_timeout_ms;

    for (;;) {
      int n = Wire.requestFrom(addr, (uint8_t)3);
      if (n == 3) {
        uint8_t msb  = Wire.read();
        uint8_t lsb  = Wire.read();
        uint8_t stat = Wire.read();

        // RDY==0 → data valid
        if ((stat & 0x80) == 0) {
          int16_t raw = (int16_t)((uint16_t)msb << 8 | lsb);
          sum += raw;
          ok++;
          break; // next sample
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
    last_good = (uint16_t)(sum / (int32_t)ok);  // average of valid samples only
    return last_good;
  }

  // No valid samples — try a soft bus reset occasionally, then return last value
  if (++fail_streak >= 3) {
    Wire.end();
    delay(2);
    Wire.begin(PIN_SDA, PIN_SCL);  // assumes you’ve called Wire.begin(...) in setup()
    fail_streak = 0;
  }
  return last_good;  // graceful fallback instead of wedging
}

// Assume PIN_EN_SENSE_PWR was pinMode’d to OUTPUT in setup()
void setPowerEnable(uint8_t powerState) {
    digitalWrite(PIN_EN_SENSE_PWR, powerState ? HIGH : LOW);
}

void RS485Sub(uint8_t depth);

static const uint8_t* const buffers[] = { aTxBuffer0, aTxBuffer1 };
static const size_t      bufLens[] = { sizeof(aTxBuffer0), sizeof(aTxBuffer1) };

// CHANGED: enforce Modbus silent gap and TX→RX turnaround
void RS485Send(uint8_t depth) {
  if (depth > 1) return;
  const uint32_t t_char_us = (11UL * 1000000UL) / 9600; // 1 char at 9600 baud
  const uint32_t t3_5_us   = (35UL * t_char_us) / 10UL;

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
  const uint32_t t3_5_us   = (35UL * t_char_us) / 10UL;
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
    uint16_t crc = modbusCRC(req,6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    Serial.printf("TX req: addr=%u start=0x%04X regs=%u\n", addr, startReg, regCount);
    RS485SendFrame(req, sizeof(req), baud);

    // expected normal reply length for function 0x03
    const size_t expected = 1 + 1 + 1 + (2 * regCount) + 2; // addr + fc + byteCount + data + crc
    if (expected > outMax) return false;

    RS485_SERIAL.setTimeout(frame_timeout_ms);
    size_t len = RS485_SERIAL.readBytes(outBuf, expected);
    Serial.printf("RX %u/%u bytes\n", (unsigned)len, (unsigned)expected);

    if (len > 0) dumpHex("RX", outBuf, len);

    // NEW: handle Modbus exception frame (5 bytes: addr, 0x83, code, CRClo, CRChi)
    if (len == 5 && (outBuf[1] == (0x80 | 0x03))) {
        uint16_t exc_crc = modbusCRC(outBuf, 3);
        uint16_t exc_rx  = (uint16_t)outBuf[3] | ((uint16_t)outBuf[4] << 8);
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
            if (aRx[i] == header && aRx[i+1] == 0x03 && aRx[i+2] == 0x04) {
                outIdx = i;
                return true;
            }
        }
    }
    return false;
}

void RS485Get() {
    uint8_t buf[64];

    if(readModbusFrame(0x01,0x0001,2,buf,sizeof(buf))) {
        sTempC[0] = buf[3];
        sTempC[1] = buf[4];
        sMoist[0] = buf[5];
        sMoist[1] = buf[6];
    }

    if(readModbusFrame(0x02,0x0001,2,buf,sizeof(buf))) {
        sTempC[2] = buf[3];
        sTempC[3] = buf[4];
        sMoist[2] = buf[5];
        sMoist[3] = buf[6];
    }
}


void initRS485(uint16_t baud) {
  RS485_SERIAL.begin(baud, SERIAL_8N1, 44, 43);  // RX=44, TX=43
  // CHANGED: hard bind UART2 to GPIOs (avoids any Arduino/Heltec re-mux weirdness)
  uart_set_pin(UART_NUM_2,
             (gpio_num_t)43,  // TX
             (gpio_num_t)44,  // RX
             UART_PIN_NO_CHANGE,
             UART_PIN_NO_CHANGE);
  uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_INV_DISABLE);
  Serial.println("UART2 pin bind: TX=43 RX=44");

  pinMode(RS485_TX_ENABLE, OUTPUT);
  digitalWrite(RS485_TX_ENABLE, LOW); // Listen by default

  // testing only: scope probe: drive DE and send 0x55 once
  digitalWrite(RS485_TX_ENABLE, HIGH);     // enable driver
  RS485_SERIAL.write(0x55);
  RS485_SERIAL.flush(true);
  digitalWrite(RS485_TX_ENABLE, LOW);      // back to RX
}

// ADDED: simple UART loopback self-test (TX->RX jumper at MCU pins 43↔44)
void rs485_uart_loopback_test(uint32_t baud = 9600) {
  Serial.printf("UART loopback: RX=%d TX=%d, baud=%u (jumper TX->RX on MCU pins)\n",
                PIN_RS485_RX, PIN_RS485_TX, (unsigned)baud);

  // Ensure RS-485 transceiver is NOT driving the bus
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);     // TX driver off, RX enabled (since DE=/RE tied)

  RS485_SERIAL.end();
  RS485_SERIAL.begin(baud, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // Known pattern
  const uint8_t tx[] = { 0x55,0xAA,0x12,0x34,0xBE,0xEF,0x00,0xFF,0x5A,0xA5,0xDE,0xAD,0xC0,0xDE,0x13,0x37 };
  uint8_t rx[sizeof(tx)] = {0};

  // Flush & send
  RS485_SERIAL.flush();
  size_t wrote = RS485_SERIAL.write(tx, sizeof(tx));
  RS485_SERIAL.flush();

  // Read back exactly the same number of bytes
  RS485_SERIAL.setTimeout(100);            // 100 ms should be plenty for 16 bytes @ 9600
  size_t got = RS485_SERIAL.readBytes(rx, sizeof(rx));

  // Report
  Serial.printf("LOOPBACK wrote=%u, got=%u\n", (unsigned)wrote, (unsigned)got);
  if (got > 0) {
    // Use your existing dumpHex if present; otherwise inline dump:
    for (size_t i = 0; i < got; ++i) Serial.printf("%02X%s", rx[i], (i+1<got)?" ":"\n");
  }
  bool pass = (got == sizeof(tx)) && (memcmp(tx, rx, sizeof(tx)) == 0);
  Serial.println(pass ? "LOOPBACK: PASS" : "LOOPBACK: FAIL");
}


bool readDepthSensor(uint16_t &depthRaw) {
  const uint8_t expected_len = 7;
  const uint8_t max_retries = 7;

  for (uint8_t attempt = 0; attempt < max_retries; ++attempt) {
    while (RS485_SERIAL.available()) RS485_SERIAL.read();  // Clear buffer

    digitalWrite(RS485_TX_ENABLE, HIGH);
    delay(10); // Settling time
    RS485_SERIAL.write(aTxBuffer4, sizeof(aTxBuffer3));
    RS485_SERIAL.flush(true);
    delayMicroseconds(200);
    digitalWrite(RS485_TX_ENABLE, LOW);

    uint32_t start = millis();
    while (RS485_SERIAL.available() < expected_len && millis() - start < 200) {
      delay(1);
    }

    if (RS485_SERIAL.available() >= expected_len) {
      uint8_t response[expected_len];
      for (int i = 0; i < expected_len; i++) {
        response[i] = RS485_SERIAL.read();
      }

      uint16_t crc = modbusCRC(response, expected_len - 2);
      uint16_t received_crc = response[5] | (response[6] << 8);

      if (crc == received_crc && (response[0] == 0x01 || response[0] == 0x02) && response[1] == 0x03) {
        depthRaw = ((uint16_t)response[3] << 8) | response[4];
        Serial.printf("raw depth 16 bit  %u\n", depthRaw);
        return true;
      } else {
        Serial.printf("Bad CRC or header on attempt %u\n", attempt + 1);
      }
    } else {
      Serial.printf("Timeout on attempt %u\n", attempt + 1);
    }

    delay(50);  // Short retry delay
  }

  Serial.println("Failed to read valid deth sensor rs-485 response after retries");
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
            if (aRx[i] == header && aRx[i+1] == 0x03 && aRx[i+2] == 0x04) {
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
    const size_t need = 1 + (size_t)regCount * 2; // addr + data bytes
    if (*pos + need > max) return false;         // prevent overflow

    payload[(*pos)++] = addr;                    // tag this chunk with sensor addr
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
