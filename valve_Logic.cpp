// valve_logic.cpp
#include "valve_logic.h"
#include "LoRaWan_APP.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

extern volatile ValveState_t *valveA, *valveB;
extern volatile ValveState_t vlv_packet_pend;
extern uint32_t appTxDutyCycle, TxDutyCycle_hold, txDutyCycleTime;
static portMUX_TYPE g_vlvMux = portMUX_INITIALIZER_UNLOCKED;
#define ENTER_CRIT() portENTER_CRITICAL(&g_vlvMux)
#define EXIT_CRIT() portEXIT_CRITICAL(&g_vlvMux)
// move your current set_vlv_status(), printValveState(), displayPacketBits() here unchanged

/* process the valve state:  Check for valve off commands first, then if on command set the TxDutyCycle
 * and turn on the valve, set the timer as cycles based on new TxDutyCycle */
void set_vlv_status() {
  ValveState_t s;  // local snapshot
  uint32_t seq_before, seq_after;

  portENTER_CRITICAL(&g_vlvMux);
  // snapshot without invoking operator= on a volatile source
  memcpy(&s, (const void *)&vlv_packet_pend, sizeof(s));
  // consume one-shot bits while still atomic
  vlv_packet_pend.onA = 0;
  vlv_packet_pend.onB = 0;
  vlv_packet_pend.offA = 0;
  vlv_packet_pend.offB = 0;
  vlv_packet_pend.pendLatchA = 0;
  vlv_packet_pend.pendLatchB = 0;
  vlv_packet_pend.flags = 0;
  portEXIT_CRITICAL(&g_vlvMux);

  // now use ONLY 's' below (no further reads of vlv_packet_pend)
  bool needTimedSchedule = false, didPlainOff = false, latchChange = false;

  if (s.pendLatchA) {
    const bool latch_on = (s.latchA != 0);
    controlValve(0, latch_on ? 1 : 0);
    valveA->latchA = latch_on ? 1 : 0;
    valveA->onA = 0;
    valveA->time = 0;
    latchChange = true;
  } else if (s.onA) {
    controlValve(0, 1);
    valveA->onA = 1;
    valveA->time = s.time;
    needTimedSchedule = true;
  } else if (s.offA) {
    controlValve(0, 0);
    valveA->onA = 0;
    valveA->time = 0;
    valveA->latchA = 0;
    didPlainOff = true;
  }
  if (!s.onA && s.time && valveA->onA) {
    valveA->time = s.time;
    needTimedSchedule = true;
  }


  if (s.pendLatchB) {
    const bool latch_on = (s.latchB != 0);
    controlValve(1, latch_on ? 1 : 0);
    valveB->latchB = latch_on ? 1 : 0;
    valveB->onB = 0;
    valveB->time = 0;
    latchChange = true;
  } else if (s.onB) {
    controlValve(1, 1);
    valveB->onB = 1;
    valveB->time = s.time;
    needTimedSchedule = true;
  } else if (s.offB) {
    controlValve(1, 0);
    valveB->onB = 0;
    valveB->time = 0;
    valveB->latchB = 0;
    didPlainOff = true;
  }

  if (needTimedSchedule) {
    scheduleValveOnCycle();
    LoRaWAN.cycle(txDutyCycleTime);
  } else if (didPlainOff && !latchChange) {
    const bool all_idle = (!valveA->onA && !valveB->onB && !valveA->latchA && !valveB->latchB);
    if (all_idle) {
      appTxDutyCycle = TxDutyCycle_hold;
      LoRaWAN.cycle(appTxDutyCycle);
    }
  }
  if (!s.onB && s.time && valveB->onB) {
    valveB->time = s.time;
    needTimedSchedule = true;
  }
  Serial.printf("flags 0x%02X, time %u\n", s.flags, s.time);
}

void printValveState(const ValveState_t &p) {
  Serial.printf("time=%u (x10min), flags=0x%02X  |  onA=%u onB=%u  latchA=%u latchB=%u  offA=%u offB=%u  pendLatchA=%u pendLatchB=%u\n",
                p.time, p.flags,
                p.onA, p.onB, p.latchA, p.latchB, p.offA, p.offB, p.pendLatchA, p.pendLatchB);
}

// AFTER (CHANGE #1)
void displayPacketBits(const ValveState_t &pkt) {
  // On-wire: [time][flags] -> low 8 = time, high 8 = flags
  const uint16_t v = ((uint16_t)pkt.flags << 8) | (uint16_t)pkt.time;

  // Take an atomic snapshot of the live state for display
  ValveState_t a, b, pend;
  ENTER_CRIT();
  memcpy(&a, (const void *)valveA, sizeof(a));
  memcpy(&b, (const void *)valveB, sizeof(b));
  memcpy(&pend, (const void *)&vlv_packet_pend, sizeof(pend));
  EXIT_CRIT();

  displayBits16(v, a, b, pend);
}

void displayBits16(uint16_t v,
                   const ValveState_t &a,
                   const ValveState_t &b,
                   const ValveState_t &pend) {
  for (int i = 15; i >= 0; --i) Serial.print((v >> i) & 1);

  // Show the *live* state snapshot (not the cleared globals)
  Serial.printf("\nvalve A time %u status %u off %u\n", (unsigned)a.time, (unsigned)a.onA, (unsigned)a.offA);
  Serial.printf("valve B time %u status %u off %u\n", (unsigned)b.time, (unsigned)b.onB, (unsigned)b.offB);
  Serial.printf("last pend   t=%u on %u:%u off %u:%u\n",
                (unsigned)pend.time, (unsigned)pend.onA, (unsigned)pend.onB,
                (unsigned)pend.offA, (unsigned)pend.offB);
}
