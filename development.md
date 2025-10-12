# Ranch Map & Device Status – Development Spec

**Route:** `/map`  
**Owner:** Peter

## Goal
Map the ranch (Google Maps/Earth) with each valve controller, flow meter, and lake-level sensor shown at their actual locations. Display current status for each device. Clicking a device opens a page with historical data; for valve controllers, allow sending valve commands.

## Data Sources (MySQL Tables)

- `devices`
- `lake_data`
- `s1000_data`
- `sensor_data`
- `soil_air_data`
- `valve_data`
- `water_meter_data`

---

## Table: `valve_data`

| Field         | Type               | Null | Key | Default | Extra |
|---------------|--------------------|------|-----|---------|-------|
| devEui        | varchar(16)        | NO   | PRI | NULL    |       |
| devName       | varchar(16)        | NO   |     | NULL    |       |
| timestamp     | datetime           | NO   |     | NULL    |       |
| ts_epoch      | bigint unsigned    | NO   | PRI | NULL    |       |
| waterPressure | float              | YES  |     | NULL    |       |
| valveAStatus  | tinyint unsigned   | YES  |     | NULL    |       |
| valveBStatus  | tinyint unsigned   | YES  |     | NULL    |       |
| flags         | tinyint unsigned   | YES  |     | 0       |       |
| batPct        | float              | YES  |     | NULL    |       |

---

## Table: `lake_data`

| Field     | Type             | Null | Key | Default | Extra |
|-----------|------------------|------|-----|---------|-------|
| devEui    | varchar(16)      | NO   | PRI | NULL    |       |
| devName   | varchar(16)      | YES  |     | NULL    |       |
| timestamp | datetime         | NO   |     | NULL    |       |
| ts_epoch  | bigint unsigned  | NO   | PRI | NULL    |       |
| depth_raw | int unsigned     | YES  |     | NULL    |       |
| depth     | float            | YES  |     | NULL    |       |
| batPct    | tinyint unsigned | YES  |     | NULL    |       |

---

## Table: `water_meter_data`

| Field     | Type             | Null | Key | Default | Extra |
|-----------|------------------|------|-----|---------|-------|
| devEui    | varchar(16)      | NO   | PRI | NULL    |       |
| devName   | varchar(16)      | YES  |     | NULL    |       |
| timestamp | datetime         | NO   |     | NULL    |       |
| ts_epoch  | bigint unsigned  | NO   | PRI | NULL    |       |
| reedDelta | int unsigned     | YES  |     | NULL    |       |
| flowRate  | float            | YES  |     | NULL    |       |
| reedCount | int unsigned     | YES  |     | NULL    |       |
| batPct    | tinyint unsigned | YES  |     | NULL    |       |

---

## Table: `devices`

| Field         | Type                 | Null | Key | Default | Extra |
|---------------|----------------------|------|-----|---------|-------|
| devEui        | varchar(16)          | NO   | PRI | NULL    |       |
| AppKey        | varchar(64)          | YES  |     | NULL    |       |
| devName       | varchar(16)          | YES  |     | NULL    |       |
| location      | varchar(64)          | YES  |     | NULL    |       |
| description   | varchar(64)          | YES  |     | NULL    |       |
| fPort         | tinyint unsigned     | YES  |     | NULL    |       |
| cal_inv_m     | varchar(16)          | YES  |     | NULL    |       |
| cal_b         | float                | YES  |     | NULL    |       |
| upldCycleMin  | smallint unsigned    | YES  |     | NULL    |       |

---

## Table: `s1000_data`

| Field        | Type            | Null | Key | Default | Extra |
|--------------|-----------------|------|-----|---------|-------|
| devEui       | varchar(16)     | NO   | PRI | NULL    |       |
| timestamp    | datetime        | NO   | MUL | NULL    |       |
| ts_epoch     | bigint unsigned | NO   | PRI | NULL    |       |
| airTemp      | float           | NO   |     | NULL    |       |
| airHumid     | float           | NO   |     | NULL    |       |
| airPresBar   | float           | NO   |     | NULL    |       |
| lightLux     | float           | NO   |     | NULL    |       |
| minWindDir   | float           | NO   |     | NULL    |       |
| maxWindDir   | float           | NO   |     | NULL    |       |
| avgWindDir   | float           | NO   |     | NULL    |       |
| minWindSp    | float           | NO   |     | NULL    |       |
| maxWindSp    | float           | NO   |     | NULL    |       |
| avgWindSp    | float           | NO   |     | NULL    |       |
| accRain      | float           | NO   |     | NULL    |       |
| accRainDur   | float           | NO   |     | NULL    |       |
| rainInten    | float           | NO   |     | NULL    |       |
| maxRain      | float           | NO   |     | NULL    |       |
| pm_2_5       | float           | NO   |     | NULL    |       |
| pm_10        | float           | NO   |     | NULL    |       |
| c02          | float           | NO   |     | NULL    |       |

---

## Table: `soil_air_data`

| Field       | Type            | Null | Key | Default | Extra |
|-------------|-----------------|------|-----|---------|-------|
| devEui      | varchar(16)     | NO   | PRI | NULL    |       |
| devName     | varchar(16)     | YES  |     | NULL    |       |
| timestamp   | datetime        | NO   |     | NULL    |       |
| ts_epoch    | bigint unsigned | NO   | PRI | NULL    |       |
| soilTempCS  | float           | YES  |     | NULL    |       |
| soilTempCD  | float           | YES  |     | NULL    |       |
| soilMoistS  | float           | YES  |     | NULL    |       |
| soilMoistD  | float           | YES  |     | NULL    |       |
| soilpHS     | float           | YES  |     | NULL    |       |
| soilpHD     | float           | YES  |     | NULL    |       |
| airTempC    | float           | YES  |     | NULL    |       |
| airMoist    | float           | YES  |     | NULL    |       |
| batPct      | float           | YES  |     | NULL    |       |

---

## Timezone
- All timestamps stored as UTC epoch (`ts_epoch`).
- Display in `America/Los_Angeles`.

## Map Bounds (DMS)
- 37°08'09"N 122°19'02"W  
- 37°07'56"N 122°18'50"W  
- 37°08'24"N 122°18'06"W  
- 37°08'30"N 122°18'13"W
