// valve_logic.h
#pragma once
#include "sensor_solenoid.h"
void set_vlv_status();
void printValveState(const ValveState_t &p);
void displayPacketBits(const ValveState_t &p);
void displayBits16(uint16_t v,
                   const ValveState_t &a,
                   const ValveState_t &b,
                   const ValveState_t &pend);
