#pragma once
#include <stdint.h>

// Raw sensor words (unchanged wire format)
float depth_m_from_raw(uint16_t raw, uint16_t inv_m_u16, int16_t b_x10);

// If you need pipe pressure, keep it SI too; tune gain/offset per your calib.
float line_pressure_kPa_from_raw(uint16_t raw, float gain_kPa_per_count, float offset_kPa);

// Convenience
static inline float kPa_to_bar(float kPa) { return kPa / 100.0f; }
static inline float kPa_to_MPa(float kPa) { return kPa / 1000.0f; }
