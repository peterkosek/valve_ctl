#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <Adafruit_MCP3421.h>
#include <stddef.h>
#include "LoRaWan_APP.h"

// GPIO Pin Assignments
#define PIN_SDA         39   // 
#define PIN_SCL         38   // 
#define PIN_RS485_RX    44  //  (U0RXD)
#define PIN_RS485_TX    43  //  (U0TXD)
#define RS485_DE        40  //  
#define PIN_IN1         47  // 
#define PIN_IN2         48  // 
#define PIN_IN3         42  // 
#define PIN_IN4         41  // 
#define PIN_VE_CTL      18  //  enables Ve, for eInk
#define PIN_EN_SENSE_PWR      45 //    for power on the sensor board, 
                          //  also LED on the vision master board
#define PIN_REED_P      17  // 
#define ADC_CTL_PIN     46  // to read bat voltage
#define VBAT_READ_PIN   7   // read voltge divider with 100k/490k divider.  

#define PULSE_THRESHOLD  100   // default, but MVS stores last uint16_t sent to port 8
                              // change this value to adjust the default wake-up count for the reed
#define RTC_GPIO_SENSOR_PIN GPIO_NUM_17  // GPIO pin connected to the sensor for ulp
#define RTC_GPIO_INDEX 17  //  RTCIO_CHANNEL_17 is 17

//  the eink display ready pin, this is high when it is busy
#define EPD_BUSY_PIN  6     // from display ctor (... busy=6 ...)

// Optional power rails
#define VDD_3V3         1
#define GND             0

// For the reed flow meter
#define TICKS_PER_MIN   28800u      //  based on   I_DELAY(08FFF), 480 Hz   //  integer math please
#define VOLUME_PER_TICK 100u         //  whatever the volume per tick of the meter is displayed as gpm

// ADC (MCP3421) Settings
#define ADC_ADDR            0x68   // 7-bit address for MCP3421
#define ADC_CFG_SINGLESHOT  0x90   // 16-bit, single-shot start, PGA=1

#define CYCLE_TIME_VALVE_ON   60000    // 10 min in ms, cycle time with valve on
#define MAX_TX_MS           21600000    //  6 hr max cycle time
// redirect all calls (works from .ino too)
#define readModbusFrame(...) readModbusFrame_dbg(__FILE__, __LINE__, __VA_ARGS__)

// typedef struct __attribute__((packed)) {

//     unsigned vlv_A_on_remaining  : 8;  // 8 bits, max 1023, 10 min incriments
//     unsigned vlv_B_on_remaining  : 8;  // 8 bits, max 1023, 10 min incriments
//     unsigned vlv_A_status        : 1;  // 1 bit
//     unsigned vlv_B_status        : 1;  // 1 bit
//     unsigned vlv_A_off           : 1;  //  instruction to shut off vlv A
//     unsigned vlv_B_off           : 1;  //  instruction to shut off vlv B
//     unsigned unused_also         : 4;  //  extra bits to fill the 32 bit str
//     unsigned unused              : 8;  // fill up to 32 bit word, packed as the last byte
// } ValveData_t;

// typedef union __attribute__((packed)){
//     ValveData_t bits;
//     uint8_t    raw[sizeof(ValveData_t)];
// } ValvePacket_t;

typedef struct __attribute__((packed)) {
    uint8_t time;       // 10-minute countdown
    union {
        struct {
            uint8_t onA : 1;  // on/off state
            uint8_t onB : 1;  // on/off state
            uint8_t latchA : 1;  // on/off latch state
            uint8_t latchB : 1;  // on/off latch state
            uint8_t offA    : 1;  // pending shutdown
            uint8_t offB    : 1;  // pending shutdown
            uint8_t pendLatchA    : 1;  // pending latch A
            uint8_t pendLatchB    : 1;  // pending latch B
        };
        uint8_t flags;
    };
} ValveState_t;
static_assert(sizeof(ValveState_t) == 2, "ValveState_t must be 2 bytes (time,flags)");
static_assert(offsetof(ValveState_t, time)  == 0, "time must be at byte 0");
static_assert(offsetof(ValveState_t, flags) == 1, "flags must be at byte 1");

enum {
  // count up each RTC_DATA_ATTR word…
  ULP_RSSI,            // 0
  ULP_SNR,             // 1
  ULP_BAT_PCT,         // 2
  ULP_LAST_SENT,       // 3
  ULP_COUNT_LO,        // 4
  ULP_COUNT_HI,        // 5
  ULP_PREV_STATE,      // 6
  ULP_VALVE_A,         // 7  (valve a status)
  ULP_VALVE_B,         // 8  (uvalve b status)
  ULP_DEBUG_PIN_STATE, // 9
  ULP_TICK_POP,          // 10   
  ULP_TS_DELTA_LO,      // 11
  ULP_TS_DELTA_HI,      // 12
  ULP_TS_DELTA_TICK_POP,   // 13
  ULP_TIMER_LO,        // 14 ← low 16 bits of running counter
  ULP_TIMER_HI,        // 15 ← high 16 bits
  ULP_REED_DELTA,       // 16 last reed delta
  ULP_FLOW_RATE,        // 17  calculated flow rate, used for display
  ULP_VOLUME_DELTA,             // 18  flow since last LoraWAN frame, in liters (reed_delta * vol_per_reed)
  ULP_WAKE_THRESHOLD,       //  19 
  ULP_TXCYCLETIME,        //  20 
  ULP_TXCYCLEFAST,        //  21
  ULP_COUNT_PENDING,               //  22
  ULP_COUNT,                //23
  ULP_PROG_START = 24,   // load instructions here
};

enum {
  ULP_ENTRY_LABEL   = 0,
  ULP_SKIP_HI_INC   = 1,
  ULP_SET_OVF   = 2,
  SKIP_HI_INC    = 3,
  ULP_STORE_PREV  = 4,
  ULP_NO_EDGE       = 5,
  ULP_WRAP_DONE       = 6,
  ULP_AFTER_COUNT       = 7,
  ULP_WRAP_CHECK_LABEL = 8,
  ULP_NO_WAKE   = 9,
  ULP_HI_INC_LABEL     = 10,
  ULP_NO_WRAP_LABEL    = 11,
  ULP_STORE_CUR = 12,
  ULP_EXIT_LABEL = 13,
  ULP_INC_LABEL        = 14,
  ULP_SET_TICK_POP,
  ULP_NO_TIMER_WRAP,
  ULP_SKIP_MERGE, 
  ULP_CPU_IS_AWAKE, 
  ULP_BUMP_HI,
  UL_BUMP_PEND_HI,
  ULP_SKIP_MERGE_BUMP,
  ULP_DO_MERGE,
  ULP_DO_MERGE_BUMP,
  RESET_PENDING_NO_WRAP,
  RESET_PENDING_WRAP,
  ULP_TEST_CPU,
  ULP_BUMP_HI_MERGE,
  ULP_CLR_PENDING,
  ULP_BUMP_HI_EDGE,
};

// Shared buffers defined in sensor_solenoid.cpp
extern uint8_t aTxBuffer0[8];     // first soil moisture sensor message
extern uint8_t aTxBuffer1[8];     // second soil moisture sensor message
extern uint8_t sTempC[4];         // temperature data from soil probes
extern uint8_t sMoist[4];         // moisture data from soil probes
extern uint8_t wPres[2];         // raw pressure from adc, needs calibration and conversion
extern uint8_t reedCount[2];  // reed pulses counted, MSB, LSB
extern uint8_t reedcyclesTenMin[2]; // intra reed pulse converted to frequency in activations in 10 min, MSB, LSB
extern uint8_t soilSensorOut[6];  //  for the two soil sensors including moisture, temp and pH

// Function prototypes
void hardware_pins_init();
void controlValve(uint8_t valve_number, uint8_t status);
uint8_t bat_cap8();

uint16_t readMCP3421avg_cont();

void setPowerEnable(uint8_t powerState);
void RS485Get();
void RS485Send(uint8_t depth);
void initRS485(uint16_t baud);
uint16_t readDepthSensor(unsigned long timeout_ms, uint8_t max_tries);
bool readFrame(uint8_t depth, uint8_t header, int& outIdx);
void sendModbusRequest();
uint16_t modbusCRC(const uint8_t* data, size_t length);
bool buildModbusRequest(uint8_t slaveAddr, uint16_t regStart, uint16_t regCount, uint8_t (&request)[8]);
// (removed stale prototype for readSoilSensor; implementation is internal/static in .cpp)
//bool readModbusFrame(uint8_t addr, uint16_t startReg, uint16_t regCount,
//                            uint8_t* outBuf, size_t outMax, uint32_t baud, uint32_t frame_timeout_ms);
static void dumpHex(const char* tag, const uint8_t* buf, size_t len);
void rs485_uart_loopback_test(uint32_t baud);
uint32_t addJitterClampMin(uint32_t base_ms, int32_t jitter_ms, uint32_t min_ms);
void scheduleValveOnCycle(void);
bool readModbusFrame_dbg(const char* file, int line,
uint8_t addr, uint16_t startReg, uint16_t regCount,
uint8_t* outBuf, size_t outMax,
uint32_t baud=9600, uint32_t frame_timeout_ms=80);