#include "units.h"

float depth_m_from_raw(uint16_t raw, uint16_t inv_m_u16, int16_t b_x10) {
  if (!inv_m_u16) return 0.0f;
  return ( (float)raw / (float)inv_m_u16 ) + ((float)b_x10 / 10.0f);
}

float line_pressure_kPa_from_raw(uint16_t raw, float gain_kPa_per_count, float offset_kPa) {
  return raw * gain_kPa_per_count + offset_kPa;
}
