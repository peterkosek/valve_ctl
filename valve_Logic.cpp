// valve_logic.cpp
#include "valve_logic.h"
#include "LoRaWan_APP.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE g_vlvMux = portMUX_INITIALIZER_UNLOCKED;
#define ENTER_CRIT() portENTER_CRITICAL(&g_vlvMux)
#define EXIT_CRIT() portEXIT_CRITICAL(&g_vlvMux)
// move your current set_vlv_status(), printValveState(), displayPacketBits() here unchanged

// Take a one-shot snapshot and CLEAR it so it won't re-fire next tick
void snapshot_cmd(volatile ValveCmd_t* out) {
  uint8_t f, a, b;
  noInterrupts();
  f = g_cmd.flags;  a = g_cmd.unitsA;  b = g_cmd.unitsB;
  g_cmd.flags = 0;  g_cmd.unitsA = 0;  g_cmd.unitsB = 0;   // clear one-shot
  interrupts();
  out->flags  = f;
  out->unitsA = a;
  out->unitsB = b;
}

// Bit masks for readability
enum { startTimedA=1<<0, startTimedB=1<<1, pendLatchA=1<<2, pendLatchB=1<<3, offA=1<<4, offB=1<<5 };

void apply_downlink_snapshot(void) {
    uint8_t f, uA, uB;
    noInterrupts();
    f  = dl_flags;   uA = dl_unitsA;   uB = dl_unitsB;
    dl_flags = dl_unitsA = dl_unitsB = 0;  // consume (one-shot)
    interrupts();

    // OFF overrides
    if (f & OFF_A) { controlValve(0,0); valveState->onA=0; valveState->latchA=0; valveState->timeA=0; set_cycle_for_irrigation(0);g_need_display = true;}
    if (f & OFF_B) { controlValve(1,0); valveState->onB=0; valveState->latchB=0; valveState->timeB=0; set_cycle_for_irrigation(0);g_need_display = true;}

    // LATCH (force ON, no timer)
    if (f & LATCH_A) { controlValve(0,1); valveState->onA=0; valveState->latchA=1; valveState->timeA=0; }
    if (f & LATCH_B) { controlValve(1,1); valveState->onB=0; valveState->latchB=1; valveState->timeB=0; }

    // TIMED (force ON, clear latch, set ticks from units)
    if (f & START_TIMED_A) { controlValve(0,1); valveState->onA=1; valveState->latchA=0; valveState->timeA=uA; set_cycle_for_irrigation(1);}
    if (f & START_TIMED_B) { controlValve(1,1); valveState->onB=1; valveState->latchB=0; valveState->timeB=uB; set_cycle_for_irrigation(1);}
}

//  true: set to CYCLE_TIME_VALVE_ON when timed valves are turned on unless it is already set
//  false: return to previously saved cycle time only if both valves are off
//  only if both valves are off
void set_cycle_for_irrigation(bool set_it){
  if (set_it && (appTxDutyCycle != CYCLE_TIME_VALVE_ON)){
    TxDutyCycle_hold = appTxDutyCycle;
    appTxDutyCycle = CYCLE_TIME_VALVE_ON;
    LoRaWAN.cycle(appTxDutyCycle);
  } else if (valveState->onA==0 && valveState->onB==0) {
    appTxDutyCycle = TxDutyCycle_hold;
    LoRaWAN.cycle(appTxDutyCycle);
  }
}
//  display status draws the screen before sleep, after a delay from uploading to receive any download
void show_vlv_status(uint8_t vlv) {
  switch (vlv) {
    case 0:
      if (valveState->onA) {
        unsigned mins = (unsigned)((valveState->timeA * 10));  //   as the time has already been decremented
        snprintf(buffer, sizeof(buffer), "A %u min\n", mins);
      } else if (valveState->latchA) {
        snprintf(buffer, sizeof(buffer), "A latched\n");
      } else {
        snprintf(buffer, sizeof(buffer), "A off");
      }

      break;
    case 1:
      if (valveState->onB) {
        unsigned mins = (unsigned)((valveState->timeB * 10));
        snprintf(buffer, sizeof(buffer), "B %u min\n", mins);
      } else if (valveState->latchB) {
        snprintf(buffer, sizeof(buffer), "B latched\n");
      } else {
        snprintf(buffer, sizeof(buffer), "B off");
      }
      break;
  }
}

// run every tick (e.g., 10-min), AFTER apply_downlink and any UI button handling
void tick_timers(volatile ValveState_t *v){
    if (v->onA && !v->latchA && !g_skip_next_decrement && v->timeA) { v->timeA--;}
    if (v->onB && !v->latchB && !g_skip_next_decrement && v->timeB) { v->timeB--;}

    if (v->onA && !v->latchA && v->timeA == 0) { controlValve(0,0); v->onA=0; set_cycle_for_irrigation(0);g_need_display = true;}  // end timed A
    if (v->onB && !v->latchB && v->timeB == 0) { controlValve(1,0); v->onB=0; set_cycle_for_irrigation(0);g_need_display = true;}  // end timed B

    g_skip_next_decrement = true;  //  only once per wake cycle
}


