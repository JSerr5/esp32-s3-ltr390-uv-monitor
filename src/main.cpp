/*
 * Indice UV - LTR-390UV-01 (Lite-On) en ESP32-S3 - ESP-IDF v5.x nativo
 *  - I2C: driver/i2c_master.h, GPIO4=SDA, GPIO5=SCL, 400 kHz (GPIO6/GPIO7 libres, sin pin INT)
 *  - Tarea FreeRTOS dedicada al sensor (core 1) + mutex para los datos compartidos
 *  - WiFi STA con reconexion automatica, mDNS "UVindice.local", servidor HTTP (esp_http_server)
 *  - Salida de diagnostico por consola serial a 115200
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "driver/i2c_master.h"

static const char *TAG = "LTR390";

// ================= CREDENCIALES WIFI (2.4 GHz) =================
#define WIFI_SSID      "TU_SSID"
#define WIFI_PASS      "TU_PASSWORD"
#define MDNS_HOSTNAME  "UVindice"      // -> http://UVindice.local

// ================= PINES HARDWARE ESP32-S3 =================
static constexpr gpio_num_t PIN_SDA = GPIO_NUM_4;
static constexpr gpio_num_t PIN_SCL = GPIO_NUM_5;
static constexpr uint32_t   I2C_FREQ_HZ = 400000;
static constexpr int        I2C_TIMEOUT_MS = 100;

// ================= DIRECCION Y REGISTROS =================
static constexpr uint8_t LTR390_ADDR     = 0x53;
static constexpr uint8_t REG_MAIN_CTRL   = 0x00;
static constexpr uint8_t REG_MEAS_RATE   = 0x04;
static constexpr uint8_t REG_GAIN        = 0x05;
static constexpr uint8_t REG_PART_ID     = 0x06;
static constexpr uint8_t REG_MAIN_STATUS = 0x07;
static constexpr uint8_t REG_ALS_DATA_0  = 0x0D;
static constexpr uint8_t REG_UVS_DATA_0  = 0x10;
static constexpr uint8_t REG_INT_CFG     = 0x19;
static constexpr uint8_t REG_INT_PST     = 0x1A;
static constexpr uint8_t REG_THRES_UP_0  = 0x21;
static constexpr uint8_t REG_THRES_LOW_0 = 0x24;

// ================= CALIBRACION =================
static constexpr float UVI_SENSITIVITY_DS_V1_1 = 2300.0f; // cuentas/UVI @18x, 20 bit (Rev V1.1)
static constexpr float UVI_SENSITIVITY_DS_V1_4 = 1400.0f; // cuentas/UVI @18x, 20 bit (Rev V1.4)
static constexpr float WINDOW_FACTOR           = 1.0f;

// ================= OPERACION CONTINUA =================
static constexpr uint8_t  UVS_GAIN_CODE = 4;
static constexpr float    UVS_GAIN_VAL  = 18.0f;
static constexpr uint8_t  UVS_RES_CODE  = 0;
static constexpr uint8_t  UVS_RES_BITS  = 20;
static constexpr float    UVS_INTEG_MS  = 400.0f;
static constexpr uint32_t UVS_MAX_CNT   = 1048575UL;
static constexpr uint8_t  UVS_RATE_CODE = 4;   // 500 ms

static constexpr uint8_t  ALS_GAIN_CODE = 1;
static constexpr float    ALS_GAIN_VAL  = 3.0f;
static constexpr uint8_t  ALS_RES_CODE  = 2;
static constexpr uint8_t  ALS_RES_BITS  = 18;
static constexpr float    ALS_INTEG_MS  = 100.0f;
static constexpr uint32_t ALS_MAX_CNT   = 262143UL;
static constexpr uint8_t  ALS_RATE_CODE = 3;   // 200 ms

static constexpr uint32_t CONVERSION_TIMEOUT_MS = 1500;
static constexpr int      MAX_I2C_FAILS = 5;

// ================= TABLAS =================
struct ResConfig  { uint8_t code; uint8_t bits; float integ_ms; uint32_t max_counts; };
struct GainConfig { uint8_t code; float factor; };

static const ResConfig RES_TABLE[6] = {
  {0, 20, 400.0f, 1048575UL},
  {1, 19, 200.0f,  524287UL},
  {2, 18, 100.0f,  262143UL},
  {3, 17,  50.0f,  131071UL},
  {4, 16,  25.0f,   65535UL},
  {5, 13,  12.5f,    8191UL}
};
static const GainConfig GAIN_TABLE[5] = {
  {0, 1.0f}, {1, 3.0f}, {2, 6.0f}, {3, 9.0f}, {4, 18.0f}
};

// ================= ESTADO DEL SENSOR (solo lo toca la tarea del sensor) =================
enum SensorState { STATE_MEASURING_UVS, STATE_MEASURING_ALS };

static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_dev = nullptr;

static bool        sensorReady  = false;
static SensorState currentState = STATE_MEASURING_UVS;
static uint32_t    stateStartMs = 0;
static uint32_t    lastUvsRaw   = 0;
static uint32_t    lastAlsRaw   = 0;
static bool        uvsSaturated = false;
static bool        alsSaturated = false;
static int         i2cFailCount = 0;

// ================= TELEMETRIA COMPARTIDA (protegida por mutex) =================
struct Telemetry {
  bool     valid;        // ya hay al menos un ciclo UVS+ALS completo
  bool     sensorOk;     // el sensor responde actualmente
  float    uvi11, uvi14, lux;
  uint32_t uvsRaw, alsRaw;
  bool     uvsSat, alsSat;
  uint32_t updatedMs;    // marca de tiempo de la ultima muestra
};

static Telemetry         g_tel = {};
static SemaphoreHandle_t g_telMutex = nullptr;

// ================= TIEMPO / ESPERAS =================
static inline uint32_t millis_ms() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

// Siempre cede la CPU al menos 1 tick -> la idle task alimenta el TWDT
static inline void delay_ms(uint32_t ms) {
  TickType_t t = pdMS_TO_TICKS(ms);
  vTaskDelay(t > 0 ? t : 1);
}

// ================= NIVEL DE RIESGO (OMS / EPA) =================
static const char *riskLevel(float uvi) {
  if (uvi < 3.0f)  return "BAJO";
  if (uvi < 6.0f)  return "MODERADO";
  if (uvi < 8.0f)  return "ALTO";
  if (uvi < 11.0f) return "MUY ALTO";
  return "EXTREMO";
}

static void telemetryPublish(float uvi11, float uvi14, float lux) {
  if (xSemaphoreTake(g_telMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
  g_tel.valid     = true;
  g_tel.sensorOk  = true;
  g_tel.uvi11     = uvi11;
  g_tel.uvi14     = uvi14;
  g_tel.lux       = lux;
  g_tel.uvsRaw    = lastUvsRaw;
  g_tel.alsRaw    = lastAlsRaw;
  g_tel.uvsSat    = uvsSaturated;
  g_tel.alsSat    = alsSaturated;
  g_tel.updatedMs = millis_ms();
  xSemaphoreGive(g_telMutex);
}

static void telemetrySetSensorOk(bool ok) {
  if (xSemaphoreTake(g_telMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
  g_tel.sensorOk = ok;
  xSemaphoreGive(g_telMutex);
}

// ================= I2C BAJO NIVEL =================
static esp_err_t initI2C() {
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = PIN_SDA;
  bus_cfg.scl_io_num = PIN_SCL;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_new_master_bus(&bus_cfg, &s_bus));
  if (!s_bus) return ESP_FAIL;

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = LTR390_ADDR;
  dev_cfg.scl_speed_hz = I2C_FREQ_HZ;
  return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
}

static bool writeReg8(uint8_t reg, uint8_t val) {
  uint8_t d[2] = { reg, val };
  bool ok = (i2c_master_transmit(s_dev, d, 2, I2C_TIMEOUT_MS) == ESP_OK);
  i2cFailCount = ok ? 0 : i2cFailCount + 1;
  return ok;
}

// Repeated start: START, addr+W, reg, RESTART, addr+R, n bytes (NACK en el ultimo), STOP
static bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
  bool ok = (i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, I2C_TIMEOUT_MS) == ESP_OK);
  i2cFailCount = ok ? 0 : i2cFailCount + 1;
  return ok;
}

static bool readReg8(uint8_t reg, uint8_t &val) { return readRegs(reg, &val, 1); }

static bool readData20Bit(uint8_t baseReg, uint32_t &out) {
  uint8_t b[3];
  if (!readRegs(baseReg, b, 3)) return false;
  out = ((uint32_t)(b[2] & 0x0F) << 16) | ((uint32_t)b[1] << 8) | b[0];
  return true;
}

static void writeThreshold20Bit(uint8_t baseReg, uint32_t thres) {
  thres &= 0x0FFFFFUL;
  writeReg8(baseReg,     (uint8_t)(thres & 0xFF));
  writeReg8(baseReg + 1, (uint8_t)((thres >> 8) & 0xFF));
  writeReg8(baseReg + 2, (uint8_t)((thres >> 16) & 0x0F));
}

static void softResetSensor() {
  // SW_Reset (0x10): el chip se reinicia antes de enviar ACK -> NACK esperado (error recuperable, se ignora).
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  uint8_t d[2] = { REG_MAIN_CTRL, 0x10 };
  (void)i2c_master_transmit(s_dev, d, 2, I2C_TIMEOUT_MS);
  esp_log_level_set("i2c.master", ESP_LOG_WARN);
  delay_ms(15);
  i2cFailCount = 0;
}

// ================= ETAPA 1: ESCANEO I2C =================
static bool detectSensorI2C() {
  printf("\n=== [1] ESCANEO DE BUS I2C Y DETECCION (SDA=GPIO%d, SCL=GPIO%d) ===\n", (int)PIN_SDA, (int)PIN_SCL);
  bool targetFound = false;

  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  for (uint8_t addr = 0x01; addr <= 0x7E; addr++) {
    if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
      if (addr == LTR390_ADDR) {
        targetFound = true;
        printf(" -> Dispositivo detectado en 0x%02X (Direccion LTR-390UV-01 CONFIRMADA)\n", addr);
      } else {
        printf(" -> Otro dispositivo I2C en 0x%02X\n", addr);
      }
    }
    if ((addr & 0x1F) == 0) delay_ms(1);
  }
  esp_log_level_set("i2c.master", ESP_LOG_WARN);

  if (!targetFound) {
    printf(" [ERROR CRITICO] No se detecto respuesta en 0x53. Verifica alimentacion 3V3 y cables SDA/SCL.\n");
    return false;
  }
  return true;
}

// ================= ETAPA 2: VOLCADO 0x00-0xFF =================
static void dumpAllRegistersReadOnly() {
  printf("\n=== [2] VOLCADO SOLO LECTURA DE REGISTROS (0x00 a 0xFF) ===\n");
  printf("Nota: direcciones fuera del mapa oficial = registros internos no documentados ('--' = sin respuesta).\n");
  printf("Nota: leer 0x07 limpia las banderas de estado (clear-on-read).\n");
  printf("     ");
  for (uint8_t col = 0; col < 16; col++) printf(" %02X", col);
  printf("\n-------------------------------------------------------\n");

  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  for (uint16_t row = 0; row < 16; row++) {
    printf("%02X | ", (unsigned)(row * 16));
    for (uint16_t col = 0; col < 16; col++) {
      uint8_t val = 0;
      if (readReg8((uint8_t)(row * 16 + col), val)) printf(" %02X", val);
      else                                          printf(" --");
    }
    printf("\n");
    delay_ms(1);
  }
  esp_log_level_set("i2c.master", ESP_LOG_WARN);
  i2cFailCount = 0;
}

// ================= ETAPA 3: RESET Y CAPACIDADES =================
static bool verifyResetAndCapabilities() {
  printf("\n=== [3] AUDITORIA DE RESET Y DETECCION DE CAPACIDADES INTERNAS ===\n");
  softResetSensor();

  struct RegCheck { uint8_t reg; const char *name; uint8_t expected; uint8_t mask; };
  static const RegCheck checks[] = {
    {0x00, "MAIN_CTRL",         0x00, 0xFF},
    {0x04, "ALS_UVS_MEAS_RATE", 0x22, 0xFF},
    {0x05, "ALS_UVS_GAIN",      0x01, 0xFF},
    {0x06, "PART_ID",           0xB2, 0xFF},
    {0x07, "MAIN_STATUS",       0x20, 0x38},
    {0x19, "INT_CFG",           0x10, 0xFF},
    {0x1A, "INT_PST",           0x00, 0xFF},
    {0x21, "THRES_UP_0",        0xFF, 0xFF},
    {0x22, "THRES_UP_1",        0xFF, 0xFF},
    {0x23, "THRES_UP_2",        0x0F, 0x0F},
    {0x24, "THRES_LOW_0",       0x00, 0xFF},
    {0x25, "THRES_LOW_1",       0x00, 0xFF},
    {0x26, "THRES_LOW_2",       0x00, 0x0F}
  };

  for (const auto &c : checks) {
    uint8_t actual = 0;
    bool ok = readReg8(c.reg, actual);
    bool match = ok && ((actual & c.mask) == (c.expected & c.mask));
    printf(" Reg 0x%02X (%-17s) | Leido: %s%02X | Esperado DS: 0x%02X | %s\n",
           c.reg, c.name, ok ? "0x" : "ERR", ok ? actual : 0, c.expected,
           !ok ? "ERROR_I2C" : (match ? "COINCIDE" : "DISCREPANCIA"));
  }

  uint8_t partId = 0;
  if (!readReg8(REG_PART_ID, partId) || partId != 0xB2) {
    printf(" [ERROR] PART_ID inesperado: 0x%02X (se esperaba 0xB2)\n", partId);
    return false;
  }
  printf(" -> Silicio verificado: Part Number = 0x%01X, Revision = 0x%01X\n",
         (partId >> 4) & 0x0F, partId & 0x0F);

  printf(" -> Verificando ganancias soportadas en hardware: ");
  for (uint8_t g = 0; g < 5; g++) {
    writeReg8(REG_GAIN, GAIN_TABLE[g].code);
    uint8_t rb = 0xFF;
    bool ok = readReg8(REG_GAIN, rb) && ((rb & 0x07) == GAIN_TABLE[g].code);
    printf("%.0fx(%s) ", GAIN_TABLE[g].factor, ok ? "OK" : "FALLO");
  }
  printf("\n");

  printf(" -> Verificando resoluciones ADC soportadas: ");
  for (uint8_t r = 0; r < 6; r++) {
    writeReg8(REG_MEAS_RATE, (RES_TABLE[r].code << 4) | 0x02);
    uint8_t rb = 0xFF;
    bool ok = readReg8(REG_MEAS_RATE, rb) && (((rb >> 4) & 0x07) == RES_TABLE[r].code);
    printf("%ub(%s) ", RES_TABLE[r].bits, ok ? "OK" : "FALLO");
  }
  printf("\n");
  return true;
}

// ================= ETAPA 4: BARRIDO 2x6x5 =================
static void sweepModesGainResolution() {
  printf("\n=== [4] BARRIDO DE CAPACIDADES: MODOS (ALS/UVS), GANANCIAS Y RESOLUCIONES ===\n");
  printf("Nota: Meas_Rate = 000b (25 ms) para medir el tiempo real de integracion del ADC.\n");
  printf("Modo | Bits | T_Teorico(ms) | Ganancia | T_Real(ms) | Cuentas Crudas | Techo Saturacion | Estado\n");
  printf("------------------------------------------------------------------------------------------------\n");

  for (uint8_t mode = 0; mode <= 1; mode++) {
    const char *modeName = (mode == 0) ? "ALS" : "UVS";
    const uint8_t ctrlVal = (mode == 0) ? 0x02 : 0x0A;
    const uint8_t dataReg = (mode == 0) ? REG_ALS_DATA_0 : REG_UVS_DATA_0;

    for (uint8_t r = 0; r < 6; r++) {
      for (uint8_t g = 0; g < 5; g++) {
        writeReg8(REG_MAIN_CTRL, 0x00);
        writeReg8(REG_MEAS_RATE, (RES_TABLE[r].code << 4) | 0x00);
        writeReg8(REG_GAIN, GAIN_TABLE[g].code);

        uint8_t dummy = 0;
        uint32_t dummyRaw = 0;
        readReg8(REG_MAIN_STATUS, dummy);
        readData20Bit(dataReg, dummyRaw);

        uint32_t tStart = millis_ms();
        writeReg8(REG_MAIN_CTRL, ctrlVal);

        bool ready = false;
        while ((millis_ms() - tStart) < 1200UL) {
          uint8_t st = 0;
          if (readReg8(REG_MAIN_STATUS, st) && (st & 0x08)) { ready = true; break; }
          delay_ms(1);
        }
        uint32_t tElapsed = millis_ms() - tStart;

        uint32_t raw = 0;
        bool readOk = readData20Bit(dataReg, raw);
        bool saturated = readOk && (raw >= RES_TABLE[r].max_counts);

        const char *estado = !readOk ? "ERROR_I2C" : (!ready ? "TIMEOUT" : (saturated ? "SATURADO" : "OK"));
        printf(" %-3s |  %2u  |    %6.1f     |   %2.0fx   |   %4lu     |    %7lu     |     %7lu      | %s\n",
               modeName, RES_TABLE[r].bits, RES_TABLE[r].integ_ms, GAIN_TABLE[g].factor,
               (unsigned long)tElapsed, (unsigned long)(readOk ? raw : 0),
               (unsigned long)RES_TABLE[r].max_counts, estado);
      }
    }
  }
  writeReg8(REG_MAIN_CTRL, 0x00);
}

// ================= ETAPA 5: LINEA BASE / RUIDO =================
static void testBaselineNoise() {
  printf("\n=== [5] MEDICION DE LINEA BASE / RUIDO (10 MUESTRAS POR CANAL A 18x, 18 BITS) ===\n");
  printf("Nota: sensor tapado = Dark Offset (0-5 cts); expuesto = fluctuacion ambiental.\n");

  for (uint8_t mode = 0; mode <= 1; mode++) {
    const uint8_t dataReg = (mode == 0) ? REG_ALS_DATA_0 : REG_UVS_DATA_0;
    writeReg8(REG_MAIN_CTRL, 0x00);
    writeReg8(REG_MEAS_RATE, (2 << 4) | 2);
    writeReg8(REG_GAIN, 4);
    uint8_t dummy = 0;
    readReg8(REG_MAIN_STATUS, dummy);
    writeReg8(REG_MAIN_CTRL, (mode == 0) ? 0x02 : 0x0A);

    uint32_t minVal = 0xFFFFFFFFUL, maxVal = 0;
    uint64_t sum = 0;
    int valid = 0;
    for (int i = 0; i < 10; i++) {
      bool ready = false;
      uint32_t t0 = millis_ms();
      while ((millis_ms() - t0) < 400UL) {
        uint8_t st = 0;
        if (readReg8(REG_MAIN_STATUS, st) && (st & 0x08)) { ready = true; break; }
        delay_ms(2);
      }
      uint32_t val = 0;
      if (ready && readData20Bit(dataReg, val)) {
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        sum += val;
        valid++;
      }
    }
    if (valid > 0) {
      printf(" Canal %s -> Min: %lu | Max: %lu | Promedio: %.2f cuentas (%d/10 muestras validas)\n",
             (mode == 0) ? "ALS" : "UVS", (unsigned long)minVal, (unsigned long)maxVal,
             (double)sum / valid, valid);
    } else {
      printf(" Canal %s -> sin muestras validas (error I2C/timeout)\n", (mode == 0) ? "ALS" : "UVS");
    }
  }
  writeReg8(REG_MAIN_CTRL, 0x00);
}

// ================= ETAPA 6: INTERRUPCION INTERNA POR REGISTRO =================
static void testInternalInterruptStatusFlag() {
  printf("\n=== [6] VERIFICACION DEL COMPARADOR DE UMBRALES POR I2C (SIN PIN FISICO INT) ===\n");
  writeReg8(REG_MAIN_CTRL, 0x00);
  writeReg8(REG_MEAS_RATE, (4 << 4) | 1);
  writeReg8(REG_GAIN, 4);

  writeThreshold20Bit(REG_THRES_UP_0,  0UL);
  writeThreshold20Bit(REG_THRES_LOW_0, 0UL);
  writeReg8(REG_INT_PST, 0x00);
  writeReg8(REG_INT_CFG, 0x14);             // INT_SEL=01 (ALS), INT_EN=1 (pin INT sin usar)

  uint8_t dummy = 0;
  readReg8(REG_MAIN_STATUS, dummy);
  writeReg8(REG_MAIN_CTRL, 0x02);

  uint8_t stAfterTrigger = 0;
  uint32_t t0 = millis_ms();
  bool readOk = false;
  delay_ms(80);
  while ((millis_ms() - t0) < 300UL) {
    uint8_t st = 0;
    if (readReg8(REG_MAIN_STATUS, st)) {
      readOk = true;
      stAfterTrigger |= st;
      if (st & 0x10) break;
    }
    delay_ms(5);
  }

  writeReg8(REG_MAIN_CTRL, 0x00);
  writeReg8(REG_INT_CFG, 0x10);
  writeThreshold20Bit(REG_THRES_UP_0,  0x0FFFFFUL);
  writeThreshold20Bit(REG_THRES_LOW_0, 0x000000UL);

  uint8_t stAfterClear = 0;
  bool clearOk = readReg8(REG_MAIN_STATUS, stAfterClear);

  printf(" Bit Interrupt_Status (0x07 Bit 4) tras superar umbral : %d (Esperado: 1 si cuentas > 0)%s\n",
         (stAfterTrigger & 0x10) ? 1 : 0, readOk ? "" : " [ERROR I2C]");
  printf(" Bit Interrupt_Status (0x07 Bit 4) tras lectura I2C    : %d (Esperado: 0, clear-on-read)%s\n",
         (stAfterClear & 0x10) ? 1 : 0, clearOk ? "" : " [ERROR I2C]");
  if (!(stAfterTrigger & 0x10) && readOk)
    printf(" Aviso: si ALS=0 cuentas (oscuridad total) el comparador no dispara con umbral 0; exponer a luz.\n");
}

// ================= OPERACION CONTINUA =================
static void startUvsMeasurement() {
  writeReg8(REG_MAIN_CTRL, 0x00);
  writeReg8(REG_GAIN, UVS_GAIN_CODE);
  writeReg8(REG_MEAS_RATE, (UVS_RES_CODE << 4) | UVS_RATE_CODE);
  uint8_t dummy = 0;
  readReg8(REG_MAIN_STATUS, dummy);
  writeReg8(REG_MAIN_CTRL, 0x0A);
  currentState = STATE_MEASURING_UVS;
  stateStartMs = millis_ms();
}

static void startAlsMeasurement() {
  writeReg8(REG_MAIN_CTRL, 0x00);
  writeReg8(REG_GAIN, ALS_GAIN_CODE);
  writeReg8(REG_MEAS_RATE, (ALS_RES_CODE << 4) | ALS_RATE_CODE);
  uint8_t dummy = 0;
  readReg8(REG_MAIN_STATUS, dummy);
  writeReg8(REG_MAIN_CTRL, 0x02);
  currentState = STATE_MEASURING_ALS;
  stateStartMs = millis_ms();
}

static float calculateUVI(uint32_t uvsCounts, float gain, float integMs, float refSensitivity) {
  float sensitivity = refSensitivity * (gain / 18.0f) * (integMs / 400.0f);
  return ((float)uvsCounts / sensitivity) * WINDOW_FACTOR;
}

static float calculateLux(uint32_t alsCounts, float gain, float integMs) {
  return ((0.6f * (float)alsCounts) / (gain * (integMs / 100.0f))) * WINDOW_FACTOR;
}

static bool runValidationPhase() {
  if (!detectSensorI2C()) return false;
  dumpAllRegistersReadOnly();
  if (!verifyResetAndCapabilities()) return false;
  sweepModesGainResolution();
  testBaselineNoise();
  testInternalInterruptStatusFlag();
  return true;
}

// ================= TAREA DEDICADA DEL SENSOR (core 1) =================
static void ltr390_task(void *arg) {
  (void)arg;
  delay_ms(1000);

  printf("\n==================================================================\n");
  printf(" DETECCION, AUDITORIA Y TELEMETRIA AUTOMATICA: LITE-ON LTR-390UV-01\n");
  printf(" ESP32-S3 | ESP-IDF nativo | I2C 400 kHz | SDA=GPIO4 SCL=GPIO5\n");
  printf("==================================================================\n");

  if (initI2C() != ESP_OK) {
    ESP_LOGE(TAG, "No se pudo inicializar el bus I2C maestro");
    telemetrySetSensorOk(false);
    vTaskDelete(NULL);
    return;
  }

  if (runValidationPhase()) {
    printf("\n==================================================================\n");
    printf(" VALIDACION COMPLETADA -> INICIANDO TELEMETRIA CONTINUA UVS / ALS\n");
    printf("==================================================================\n\n");
    sensorReady = true;
    startUvsMeasurement();
  } else {
    telemetrySetSensorOk(false);
  }

  while (true) {
    if (!sensorReady) {
      delay_ms(2000);
      if (runValidationPhase()) {
        sensorReady = true;
        printf("\n[OK] Sensor recuperado. Iniciando telemetria continua UVS / ALS...\n\n");
        startUvsMeasurement();
      }
      continue;
    }

    uint8_t status = 0;
    if (!readReg8(REG_MAIN_STATUS, status)) {
      if (i2cFailCount >= MAX_I2C_FAILS) {
        printf("[ERROR] Sensor sin respuesta I2C. Reintentando deteccion...\n");
        sensorReady = false;
        telemetrySetSensorOk(false);
      }
      delay_ms(50);
      continue;
    }

    // Power_On_Status (bit 5): brownout, el sensor perdio la configuracion
    if (status & 0x20) {
      printf("[AVISO] Reinicio electrico detectado en LTR390 (Power_On_Status=1). Reconfigurando...\n");
      startUvsMeasurement();
      continue;
    }

    const uint32_t elapsed = millis_ms() - stateStartMs;
    const bool dataReady = (status & 0x08) != 0;

    if (currentState == STATE_MEASURING_UVS) {
      if (dataReady && elapsed >= (uint32_t)UVS_INTEG_MS) {
        uint32_t raw;
        if (readData20Bit(REG_UVS_DATA_0, raw)) {
          lastUvsRaw   = raw;
          uvsSaturated = (raw >= UVS_MAX_CNT);
          startAlsMeasurement();
        }
      } else if (elapsed > CONVERSION_TIMEOUT_MS) {
        printf("[AVISO] Timeout en conversion UVS. Reconfigurando...\n");
        startUvsMeasurement();
      }
    } else {
      if (dataReady && elapsed >= (uint32_t)ALS_INTEG_MS) {
        uint32_t raw;
        if (readData20Bit(REG_ALS_DATA_0, raw)) {
          lastAlsRaw   = raw;
          alsSaturated = (raw >= ALS_MAX_CNT);

          float uvi_v1_1 = calculateUVI(lastUvsRaw, UVS_GAIN_VAL, UVS_INTEG_MS, UVI_SENSITIVITY_DS_V1_1);
          float uvi_v1_4 = calculateUVI(lastUvsRaw, UVS_GAIN_VAL, UVS_INTEG_MS, UVI_SENSITIVITY_DS_V1_4);
          float lux      = calculateLux(lastAlsRaw, ALS_GAIN_VAL, ALS_INTEG_MS);

          telemetryPublish(uvi_v1_1, uvi_v1_4, lux);

          printf("UVS: %7lu cts [G:%2.0fx, %2ub/%3.0fms%s] -> UVI(V1.1): %5.2f | UVI(V1.4): %5.2f  ||  "
                 "ALS: %6lu cts [G:%2.0fx, %2ub/%3.0fms%s] -> Lux: %8.2f lx\n",
                 (unsigned long)lastUvsRaw, UVS_GAIN_VAL, UVS_RES_BITS, UVS_INTEG_MS,
                 uvsSaturated ? ", SAT!" : "",
                 uvi_v1_1, uvi_v1_4,
                 (unsigned long)lastAlsRaw, ALS_GAIN_VAL, ALS_RES_BITS, ALS_INTEG_MS,
                 alsSaturated ? ", SAT!" : "",
                 lux);

          startUvsMeasurement();
        }
      } else if (elapsed > CONVERSION_TIMEOUT_MS) {
        printf("[AVISO] Timeout en conversion ALS. Reconfigurando...\n");
        startUvsMeasurement();
      }
    }

    delay_ms(10);
  }
}

// ================= PAGINA WEB EMBEBIDA (en Flash, sin CDNs) =================
static const char INDEX_HTML[] = R"UVHTML(<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#f0f6ff">
<title>Estaci&oacute;n Solar &middot; &Iacute;ndice UV</title>
<style>
:root{
  --bg:#f0f6ff;
  --bg-grad:radial-gradient(circle at 50% 0%,#fffbeb 0%,#f0f6ff 55%,#e0ecff 100%);
  --card:rgba(255,255,255,0.92);
  --card2:#f8fafc;
  --line:#e2e8f0;
  --txt:#0f172a;
  --mut:#475569;
  --sub:#64748b;
  --accent:#10b981;
  --accent-soft:rgba(16,185,129,0.12);
  --shadow:0 10px 30px -10px rgba(15,23,42,0.08),0 4px 12px -2px rgba(15,23,42,0.04);
}
[data-theme="dark"]{
  --bg:#0b1220;
  --bg-grad:radial-gradient(circle at 50% 0%,#172542 0%,#0b1220 70%);
  --card:rgba(19,28,46,0.92);
  --card2:#18233a;
  --line:#25324d;
  --txt:#f1f5f9;
  --mut:#94a3b8;
  --sub:#64748b;
  --shadow:0 12px 32px -10px rgba(0,0,0,0.45);
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{background:var(--bg);background-image:var(--bg-grad);background-attachment:fixed;color:var(--txt);font-family:system-ui,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;-webkit-text-size-adjust:100%;transition:background .3s,color .3s}
body{min-height:100vh;padding:max(16px,env(safe-area-inset-top)) 16px 32px}
.wrap{max-width:920px;margin:0 auto;display:grid;gap:16px}

/* Header */
header{display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap;background:var(--card);border:1px solid var(--line);border-radius:18px;padding:12px 18px;box-shadow:var(--shadow);backdrop-filter:blur(10px)}
.brand{display:flex;align-items:center;gap:12px}
.sun-logo{width:38px;height:38px;border-radius:12px;background:linear-gradient(135deg,#fbbf24,#f97316);display:grid;place-items:center;color:#fff;box-shadow:0 4px 12px rgba(249,115,22,0.3)}
.brand-txt h1{font-size:1.08rem;font-weight:800;letter-spacing:-.01em;color:var(--txt)}
.brand-txt p{font-size:.76rem;color:var(--mut);font-weight:500}
.hdr-right{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.live-pill{display:inline-flex;align-items:center;gap:8px;background:var(--card2);border:1px solid var(--line);border-radius:999px;padding:6px 13px;font-size:.8rem;font-weight:600;color:var(--mut)}
.dot{width:9px;height:9px;border-radius:50%;background:#94a3b8;transition:background .3s,transform .2s}
.dot.ok{background:#10b981;box-shadow:0 0 0 4px rgba(16,185,129,0.2)}
.dot.pulse{transform:scale(1.35)}
.dot.warn{background:#f59e0b;box-shadow:0 0 0 4px rgba(245,158,11,0.2)}
.dot.bad{background:#ef4444;box-shadow:0 0 0 4px rgba(239,68,68,0.2)}
.icon-btn{background:var(--card2);border:1px solid var(--line);color:var(--txt);border-radius:999px;padding:6px 12px;font:inherit;font-size:.8rem;font-weight:600;cursor:pointer;display:inline-flex;align-items:center;gap:6px;transition:border-color .2s,background .2s}
.icon-btn:hover{border-color:var(--accent)}

/* Banners */
.banner{display:none;padding:12px 16px;border-radius:14px;font-size:.9rem;font-weight:600;box-shadow:var(--shadow)}
.banner.show{display:flex;align-items:center;gap:10px}
.banner.err{background:#fef2f2;border:1px solid #fecaca;color:#b91c1c}
.banner.sat{background:#fffbeb;border:1px solid #fde68a;color:#b45309}
.banner.info{background:#eff6ff;border:1px solid #bfdbfe;color:#1d4ed8}
[data-theme="dark"] .banner.err{background:#3b1418;border-color:#ef4444;color:#fecaca}
[data-theme="dark"] .banner.sat{background:#3a2a0c;border-color:#f59e0b;color:#fde68a}
[data-theme="dark"] .banner.info{background:#13243f;border-color:#3b82f6;color:#bfdbfe}

/* Cards */
.card{background:var(--card);border:1px solid var(--line);border-radius:24px;padding:22px;box-shadow:var(--shadow);backdrop-filter:blur(10px)}
.hero{position:relative;overflow:hidden;border:2px solid var(--accent);box-shadow:0 16px 40px -18px var(--accent),var(--shadow);transition:border-color .35s,box-shadow .35s}
.hero-top{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}
.lbl{font-size:.76rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:var(--mut);display:inline-flex;align-items:center;gap:6px}
.seg{display:inline-flex;background:var(--card2);border:1px solid var(--line);border-radius:999px;padding:3px}
.seg button{background:none;border:0;color:var(--mut);font:inherit;font-size:.8rem;font-weight:600;padding:5px 14px;border-radius:999px;cursor:pointer;transition:all .2s}
.seg button.on{background:var(--accent);color:#fff;font-weight:700;box-shadow:0 2px 8px rgba(0,0,0,0.15)}

/* Gauge + Big UVI */
.hero-body{display:grid;grid-template-columns:1.05fr 1fr;gap:20px;align-items:center;margin-top:10px}
@media(max-width:680px){.hero-body{grid-template-columns:1fr;text-align:center}}
.gauge-box{position:relative;width:100%;max-width:360px;margin:0 auto}
.gauge-box svg{width:100%;display:block;overflow:visible}
.gnum{position:absolute;left:0;right:0;bottom:4%;text-align:center}
.uv{font-size:clamp(3.6rem,14vw,5.2rem);font-weight:900;line-height:.95;letter-spacing:-.03em;font-variant-numeric:tabular-nums;color:var(--accent);transition:color .35s}
.sub{font-size:.78rem;font-weight:600;color:var(--mut);margin-top:4px}

/* Risk details & scale bar */
.risk-box{display:flex;flex-direction:column;gap:12px}
@media(max-width:680px){.risk-box{align-items:center}}
.badge-wrap{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
@media(max-width:680px){.badge-wrap{justify-content:center}}
.badge{display:inline-flex;align-items:center;gap:6px;padding:7px 18px;border-radius:999px;font-weight:800;letter-spacing:.05em;font-size:.92rem;background:var(--accent);color:#fff;box-shadow:0 4px 14px -2px var(--accent);transition:background .35s}
.rec{font-size:1.02rem;line-height:1.5;color:var(--txt);font-weight:500;background:var(--card2);border:1px solid var(--line);border-left:4px solid var(--accent);padding:12px 14px;border-radius:12px;text-align:left}

/* Linear spectrum 0..11+ */
.scale-wrap{margin-top:4px;width:100%}
.scale-bar{position:relative;height:12px;border-radius:999px;background:linear-gradient(90deg,#10b981 0%,#10b981 22%,#f59e0b 27%,#f59e0b 48%,#f97316 54%,#f97316 68%,#ef4444 74%,#ef4444 92%,#a855f7 97%,#a855f7 100%);box-shadow:inset 0 1px 2px rgba(0,0,0,0.15)}
.scale-ptr{position:absolute;top:50%;left:0%;width:18px;height:18px;border-radius:50%;background:#fff;border:4px solid var(--txt);transform:translate(-50%,-50%);box-shadow:0 2px 6px rgba(0,0,0,0.25);transition:left .35s ease}
.scale-ticks{display:flex;justify-content:space-between;font-size:.72rem;font-weight:700;color:var(--mut);margin-top:6px;padding:0 2px}

.chips{display:flex;gap:10px;flex-wrap:wrap;margin-top:2px}
@media(max-width:680px){.chips{justify-content:center}}
.chip{flex:1;min-width:135px;background:var(--card2);border:1px solid var(--line);border-radius:14px;padding:9px 14px;font-size:.8rem;color:var(--mut);display:flex;justify-content:space-between;align-items:center;cursor:pointer;transition:border-color .2s,background .2s}
.chip b{color:var(--txt);font-variant-numeric:tabular-nums;font-size:1.05rem;font-weight:800}
.chip.on{border-color:var(--accent);background:var(--accent-soft)}

/* Bottom Grid */
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:680px){.grid{grid-template-columns:1fr}}

/* Lux Card */
.lux-head{display:flex;justify-content:space-between;align-items:center}
.lux{font-size:clamp(2.2rem,8vw,3rem);font-weight:900;letter-spacing:-.02em;font-variant-numeric:tabular-nums;margin-top:8px;color:var(--txt)}
.lux small{font-size:1rem;font-weight:700;color:var(--mut)}
.luxlbl{display:inline-block;margin-top:6px;padding:4px 12px;border-radius:999px;background:#e0f2fe;color:#0369a1;font-weight:700;font-size:.84rem}
[data-theme="dark"] .luxlbl{background:#0c4a6e;color:#7dd3fc}
.lux-meter{margin-top:14px;height:8px;background:var(--card2);border:1px solid var(--line);border-radius:999px;overflow:hidden}
.lux-fill{height:100%;width:0%;background:linear-gradient(90deg,#38bdf8,#fbbf24,#f97316);border-radius:999px;transition:width .35s ease}
.lux-scale{display:flex;justify-content:space-between;font-size:.7rem;color:var(--mut);margin-top:5px;font-weight:600}

/* Trend Sparkline */
.spark-box{margin-top:14px;padding-top:12px;border-top:1px solid var(--line)}
.spark-hdr{display:flex;justify-content:space-between;font-size:.75rem;color:var(--mut);font-weight:600;margin-bottom:6px}
.spark-svg{width:100%;height:44px;display:block;background:var(--card2);border:1px solid var(--line);border-radius:10px}

/* Tech rows */
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:8px 0;border-bottom:1px solid var(--line);font-size:.88rem}
.row:last-child{border-bottom:0;padding-bottom:0}
.row span:first-child{color:var(--mut);font-weight:500}
.row span:last-child{font-variant-numeric:tabular-nums;font-weight:700;text-align:right;color:var(--txt)}

.stale .uv,.stale .lux{opacity:.45}
footer{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px;font-size:.78rem;color:var(--mut);padding:0 6px;font-weight:500}
.rate-sel{display:inline-flex;align-items:center;gap:6px}
.rate-sel select{background:var(--card);border:1px solid var(--line);color:var(--txt);border-radius:8px;padding:3px 8px;font:inherit;font-weight:700;cursor:pointer}
</style>
</head>
<body>
<div class="wrap" id="app">
  <header>
    <div class="brand">
      <div class="sun-logo" aria-hidden="true">
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"><circle cx="12" cy="12" r="4"/><path d="M12 2v2m0 16v2M4.93 4.93l1.41 1.41m11.32 11.32l1.41 1.41M2 12h2m16 0h2M4.93 19.07l1.41-1.41m11.32-11.32l1.41-1.41"/></svg>
      </div>
      <div class="brand-txt">
        <h1>Monitor Solar UV &middot; LTR-390</h1>
        <p>Telemetr&iacute;a en tiempo real &middot; ESP32-S3</p>
      </div>
    </div>
    <div class="hdr-right">
      <div class="live-pill"><span class="dot" id="dot"></span><span id="connTxt">Conectando&hellip;</span></div>
      <button class="icon-btn" id="themeBtn" type="button" title="Cambiar tema claro/oscuro">&#9728;&#65039; <span id="themeTxt">Claro</span></button>
    </div>
  </header>

  <div class="banner err" id="bErr">&#9888;&#65039; Sin comunicaci&oacute;n con UVindice.local. Reintentando en tiempo real&hellip;</div>
  <div class="banner info" id="bInfo">&#9203; Autodiagn&oacute;stico inicial del sensor en curso. Los datos aparecer&aacute;n enseguida&hellip;</div>
  <div class="banner err" id="bSensor">&#9888;&#65039; El sensor LTR-390 no responde. Revisa alimentaci&oacute;n 3.3 V y cables SDA/SCL.</div>
  <div class="banner sat" id="bSat"></div>

  <section class="card hero" id="hero">
    <div class="hero-top">
      <span class="lbl">&#9728;&#65039; &Iacute;ndice Ultravioleta (OMS / EPA)</span>
      <div class="seg" role="group" aria-label="Calibraci&oacute;n">
        <button id="bv11" class="on" type="button">Est&aacute;ndar V1.1</button>
        <button id="bv14" type="button"> Sensibilidad V1.4</button>
      </div>
    </div>

    <div class="hero-body">
      <div class="gauge-box">
        <svg viewBox="0 0 220 128" aria-hidden="true">
          <path d="M20 112 A90 90 0 0 1 200 112" fill="none" stroke="var(--line)" stroke-width="16" stroke-linecap="round"/>
          <path id="arc" d="M20 112 A90 90 0 0 1 200 112" fill="none" stroke="#10b981" stroke-width="16" stroke-linecap="round" stroke-dasharray="0 300" style="transition:stroke-dasharray .35s ease,stroke .35s"/>
          <g fill="var(--mut)" font-size="8.5" font-weight="700" text-anchor="middle">
            <text x="18" y="126">0</text>
            <text x="58" y="42">3</text>
            <text x="110" y="16">6</text>
            <text x="162" y="42">8</text>
            <text x="202" y="126">11+</text>
          </g>
        </svg>
        <div class="gnum">
          <div class="uv" id="uv">--</div>
          <div class="sub" id="uvSub">calibraci&oacute;n V1.1</div>
        </div>
      </div>

      <div class="risk-box">
        <div class="badge-wrap">
          <span class="badge" id="badge">ESPERANDO</span>
        </div>
        <p class="rec" id="rec">Conectando con el sensor para evaluar el nivel de radiaci&oacute;n solar&hellip;</p>

        <div class="scale-wrap">
          <div class="scale-bar"><div class="scale-ptr" id="sPtr"></div></div>
          <div class="scale-ticks">
            <span>0 Bajo</span><span>3 Mod.</span><span>6 Alto</span><span>8 M.Alto</span><span>11+ Ext.</span>
          </div>
        </div>

        <div class="chips">
          <div class="chip on" id="c11"><span>UVI (V1.1)</span><b id="u11">--</b></div>
          <div class="chip" id="c14"><span>UVI (V1.4)</span><b id="u14">--</b></div>
        </div>
      </div>
    </div>
  </section>

  <div class="grid">
    <section class="card">
      <div class="lux-head">
        <span class="lbl">&#128161; Iluminancia Ambiental</span>
        <span class="luxlbl" id="luxLbl">--</span>
      </div>
      <div class="lux"><span id="lux">--</span> <small>lx</small></div>
      <div class="lux-meter"><div class="lux-fill" id="luxFill"></div></div>
      <div class="lux-scale"><span>Noche (&lt;10)</span><span>Interior (500)</span><span>Nublado (10k)</span><span>Sol (&gt;10k)</span></div>

      <div class="spark-box">
        <div class="spark-hdr"><span>Tendencia UVI en vivo (&uacute;ltimas muestras)</span><span id="sparkRange">--</span></div>
        <svg class="spark-svg" viewBox="0 0 240 44" preserveAspectRatio="none">
          <polyline id="sparkLine" fill="none" stroke="var(--accent)" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" points=""/>
        </svg>
      </div>
    </section>

    <section class="card">
      <span class="lbl">&#9881;&#65039; Diagn&oacute;stico en Tiempo Real</span>
      <div class="row" style="margin-top:8px"><span>Cuentas UVS (ADC)</span><span id="uvsRaw">--</span></div>
      <div class="row"><span>Cuentas ALS (ADC)</span><span id="alsRaw">--</span></div>
      <div class="row"><span>Configuraci&oacute;n UVS</span><span>18x &middot; 20 bits &middot; 400 ms</span></div>
      <div class="row"><span>Configuraci&oacute;n ALS</span><span>3x &middot; 18 bits &middot; 100 ms</span></div>
      <div class="row"><span>Edad de muestra sensor</span><span id="sampleAge">--</span></div>
      <div class="row"><span>Estado ESP32-S3</span><span id="esp">--</span></div>
    </section>
  </div>

  <footer>
    <div><span id="age">Iniciando&hellip;</span></div>
    <div class="rate-sel">
      <label for="rateSel">Refresco:</label>
      <select id="rateSel">
        <option value="800" selected>Tiempo real (0.8 s)</option>
        <option value="1500">R&aacute;pido (1.5 s)</option>
        <option value="2000">Cada 2 s</option>
        <option value="5000">Cada 5 s</option>
      </select>
    </div>
  </footer>
</div>

<script>
(function(){
"use strict";
var POLL_MS=800, FETCH_TIMEOUT_MS=2500;
var LEVELS=[
 {max:3, name:"RIESGO BAJO",     color:"#059669", msg:"Seguro salir. Riesgo m\u00ednimo para la piel y los ojos."},
 {max:6, name:"RIESGO MODERADO", color:"#d97706", msg:"Precauci\u00f3n al salir. Usa gafas de sol, gorra y protector solar SPF 30+ si estar\u00e1s m\u00e1s de 30 minutos al exterior."},
 {max:8, name:"RIESGO ALTO",     color:"#ea580c", msg:"Riesgoso salir sin protecci\u00f3n. Reduce la exposici\u00f3n entre las 10:00 y las 16:00, busca sombra y usa bloqueador solar."},
 {max:11,name:"RIESGO MUY ALTO", color:"#dc2626", msg:"Peligroso salir al sol directo. Da\u00f1o cut\u00e1neo r\u00e1pido; evita salir al mediod\u00eda y usa ropa de manga larga, sombrero y SPF 50+."},
 {max:Infinity,name:"RIESGO EXTREMO",color:"#9333ea", msg:"Peligro extremo: NO se recomienda salir al sol. Riesgo de quemaduras solares en pocos minutos."}
];
var ARC_LEN=282.74;
var $=function(id){return document.getElementById(id);};
var ver="v1_1", theme="light", history=[];
try{
  var sv=localStorage.getItem("uvver"); if(sv==="v1_4")ver="v1_4";
  var st=localStorage.getItem("uvtheme"); if(st==="dark")theme="dark";
  var sr=parseInt(localStorage.getItem("uvrate"),10); if(sr>=500&&sr<=15000){POLL_MS=sr;$("rateSel").value=String(sr);}
}catch(e){}
var last=null, lastOkAt=0, commFail=false, busy=false, timerId=null;

function applyTheme(){
  if(theme==="dark"){
    document.documentElement.setAttribute("data-theme","dark");
    $("themeTxt").textContent="Oscuro";
  }else{
    document.documentElement.removeAttribute("data-theme");
    $("themeTxt").textContent="Claro";
  }
}

function levelOf(u){for(var i=0;i<LEVELS.length;i++){if(u<LEVELS[i].max)return LEVELS[i];}return LEVELS[LEVELS.length-1];}
function luxLabel(l){
  if(l<10)return "Oscuridad / Noche";
  if(l<500)return "Iluminaci\u00f3n interior";
  if(l<=10000)return "Exterior nublado / Sombra";
  return "Luz solar directa";
}
function luxPercent(l){
  if(l<=0)return 0;
  var p=(Math.log10(l+1)/5)*100;
  return Math.min(100,Math.max(2,p));
}
function fmtLux(l){return l.toLocaleString("es-MX",{minimumFractionDigits:1,maximumFractionDigits:1});}
function show(id,on){$(id).classList.toggle("show",!!on);}

function setVer(v){
  ver=v;
  try{localStorage.setItem("uvver",v);}catch(e){}
  render();
}

function drawSpark(){
  var pts=$("sparkLine"), rng=$("sparkRange");
  if(!history.length){pts.setAttribute("points","");rng.textContent="--";return;}
  var min=Infinity,max=-Infinity;
  for(var i=0;i<history.length;i++){if(history[i]<min)min=history[i];if(history[i]>max)max=history[i];}
  var span=Math.max(0.2,max-min);
  var out=[];
  for(var j=0;j<history.length;j++){
    var x=(history.length===1)?120:(j/(history.length-1))*232+4;
    var y=38-((history[j]-min)/span)*32;
    out.push(x.toFixed(1)+","+y.toFixed(1));
  }
  pts.setAttribute("points",out.join(" "));
  rng.textContent="m\u00edn "+min.toFixed(2)+" / m\u00e1x "+max.toFixed(2);
}

function render(){
  var d=last, root=$("app");
  $("bv11").classList.toggle("on",ver==="v1_1");
  $("bv14").classList.toggle("on",ver==="v1_4");
  $("c11").classList.toggle("on",ver==="v1_1");
  $("c14").classList.toggle("on",ver==="v1_4");
  $("uvSub").textContent="calibraci\u00f3n "+(ver==="v1_1"?"V1.1":"V1.4");

  var stale=commFail||(d&&!d.sensor_ok);
  root.classList.toggle("stale",!!stale);
  show("bErr",commFail);
  show("bInfo",d&&!d.valid&&d.sensor_ok);
  show("bSensor",d&&!d.sensor_ok);

  var dot=$("dot"), ct=$("connTxt");
  dot.className="dot";
  if(commFail){dot.classList.add("bad");ct.textContent="Sin conexi\u00f3n";}
  else if(!d){ct.textContent="Conectando\u2026";}
  else if(!d.sensor_ok){dot.classList.add("warn");ct.textContent="Sensor sin respuesta";}
  else{dot.classList.add("ok");ct.textContent="En vivo ("+(POLL_MS/1000).toFixed(1)+"s)";}

  if(!d){return;}

  $("uvsRaw").textContent=d.uvs_raw.toLocaleString("es-MX")+(d.uvs_sat?"  SAT!":"");
  $("alsRaw").textContent=d.als_raw.toLocaleString("es-MX")+(d.als_sat?"  SAT!":"");
  var ageMs=(typeof d.data_age_ms==="number")?d.data_age_ms:(d.data_age_s*1000);
  $("sampleAge").textContent=ageMs+" ms";
  $("esp").textContent="activo "+fmtUp(d.uptime_s)+(d.rssi?" \u00b7 "+d.rssi+" dBm":"");

  if(!d.valid){
    $("uv").textContent="--";$("u11").textContent="--";$("u14").textContent="--";
    $("lux").textContent="--";$("luxLbl").textContent="--";$("badge").textContent="ESPERANDO";
    $("rec").textContent="Esperando datos del sensor\u2026";
    $("arc").setAttribute("stroke-dasharray","0 300");
    $("sPtr").style.left="0%";
    show("bSat",false);
    return;
  }

  var uvi=d["uvi_"+ver], lv=levelOf(uvi);
  var st=document.documentElement.style;
  st.setProperty("--accent",lv.color);
  $("uv").textContent=uvi.toFixed(2);
  $("u11").textContent=d.uvi_v1_1.toFixed(2);
  $("u14").textContent=d.uvi_v1_4.toFixed(2);
  $("badge").textContent=lv.name;
  $("rec").textContent=lv.msg;

  var p=Math.min(Math.max(uvi,0),11)/11;
  var a=$("arc"); a.setAttribute("stroke",lv.color); a.setAttribute("stroke-dasharray",(ARC_LEN*p).toFixed(1)+" 300");
  $("sPtr").style.left=(p*100).toFixed(1)+"%";

  $("lux").textContent=fmtLux(d.lux);
  $("luxLbl").textContent=luxLabel(d.lux);
  $("luxFill").style.width=luxPercent(d.lux).toFixed(1)+"%";

  var s=[]; if(d.uvs_sat)s.push("UVS"); if(d.als_sat)s.push("ALS");
  var b=$("bSat");
  if(s.length){b.textContent="\u26a0\ufe0f Saturaci\u00f3n del ADC en canal "+s.join(" y ")+": la lectura alcanz\u00f3 el l\u00edmite m\u00e1ximo.";}
  show("bSat",s.length>0);
  drawSpark();
}

function fmtUp(s){
  var h=Math.floor(s/3600), m=Math.floor((s%3600)/60), x=s%60;
  return (h?h+"h ":"")+(h||m?m+"m ":"")+x+"s";
}

function tick(){
  var now=Date.now();
  if(lastOkAt){
    var ms=Math.max(0,now-lastOkAt);
    $("age").textContent="Tiempo real \u00b7 recibido hace "+(ms<1000?(ms+" ms"):((ms/1000).toFixed(1)+" s"));
  }else{
    $("age").textContent="Conectando con UVindice.local\u2026";
  }
}

function poll(){
  if(busy)return;
  busy=true;
  var ctl=("AbortController" in window)?new AbortController():null;
  var to=setTimeout(function(){if(ctl)ctl.abort();},FETCH_TIMEOUT_MS);
  fetch("/api/telemetry",{cache:"no-store",signal:ctl?ctl.signal:undefined})
    .then(function(r){if(!r.ok)throw new Error("HTTP "+r.status);return r.json();})
    .then(function(j){
      last=j;lastOkAt=Date.now();commFail=false;
      if(j.valid){
        history.push(j["uvi_"+ver]);
        if(history.length>40)history.shift();
      }
      var dot=$("dot"); dot.classList.add("pulse");
      setTimeout(function(){dot.classList.remove("pulse");},180);
      render();tick();
    })
    .catch(function(){commFail=true;render();tick();})
    .then(function(){clearTimeout(to);busy=false;});
}

function restartTimer(){
  if(timerId)clearInterval(timerId);
  timerId=setInterval(poll,POLL_MS);
}

$("bv11").addEventListener("click",function(){setVer("v1_1");});
$("bv14").addEventListener("click",function(){setVer("v1_4");});
$("c11").addEventListener("click",function(){setVer("v1_1");});
$("c14").addEventListener("click",function(){setVer("v1_4");});
$("themeBtn").addEventListener("click",function(){
  theme=(theme==="light")?"dark":"light";
  try{localStorage.setItem("uvtheme",theme);}catch(e){}
  applyTheme();
});
$("rateSel").addEventListener("change",function(){
  POLL_MS=parseInt(this.value,10)||800;
  try{localStorage.setItem("uvrate",String(POLL_MS));}catch(e){}
  restartTimer();
  poll();
});

applyTheme();
render();tick();
poll();
restartTimer();
setInterval(tick,250);
})();
</script>
</body>
</html>
)UVHTML";

// ================= SERVIDOR HTTP =================
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t telemetry_get_handler(httpd_req_t *req) {
  Telemetry t;
  if (xSemaphoreTake(g_telMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"error\":\"busy\"}", HTTPD_RESP_USE_STRLEN);
  }
  t = g_tel;                       // copia consistente; el mutex no se mantiene durante el envio
  xSemaphoreGive(g_telMutex);

  const uint32_t nowMs = millis_ms();
  const uint32_t ageMs = t.valid ? (nowMs - t.updatedMs) : 0;
  const uint32_t age   = ageMs / 1000UL;
  int rssi = 0;
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;

  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"uvi_v1_1\":%.2f,\"uvi_v1_4\":%.2f,\"uvs_raw\":%lu,\"uvs_sat\":%s,"
           "\"lux\":%.1f,\"als_raw\":%lu,\"als_sat\":%s,\"risk_level\":\"%s\","
           "\"uptime_s\":%lu,\"valid\":%s,\"sensor_ok\":%s,\"data_age_s\":%lu,\"data_age_ms\":%lu,\"rssi\":%d}",
           (double)t.uvi11, (double)t.uvi14, (unsigned long)t.uvsRaw, t.uvsSat ? "true" : "false",
           (double)t.lux, (unsigned long)t.alsRaw, t.alsSat ? "true" : "false",
           t.valid ? riskLevel(t.uvi11) : "SIN_DATOS",
           (unsigned long)(nowMs / 1000UL), t.valid ? "true" : "false",
           t.sensorOk ? "true" : "false", (unsigned long)age, (unsigned long)ageMs, rssi);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;
  config.stack_size = 6144;
  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "No se pudo iniciar el servidor HTTP");
    return nullptr;
  }
  httpd_uri_t root = {};
  root.uri = "/";
  root.method = HTTP_GET;
  root.handler = root_get_handler;
  httpd_register_uri_handler(server, &root);

  httpd_uri_t tel = {};
  tel.uri = "/api/telemetry";
  tel.method = HTTP_GET;
  tel.handler = telemetry_get_handler;
  httpd_register_uri_handler(server, &tel);

  ESP_LOGI(TAG, "Servidor HTTP activo en el puerto 80 (GET / y GET /api/telemetry)");
  return server;
}

// ================= mDNS + HTTP (se arrancan una sola vez, tras obtener IP) =================
static volatile bool s_servicesStarted = false;

static void services_task(void *arg) {
  (void)arg;
  esp_err_t err = mdns_init();
  if (err == ESP_OK) {
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set("Indice UV - LTR390");
    mdns_service_add("Indice UV", "_http", "_tcp", 80, nullptr, 0);
    ESP_LOGI(TAG, "mDNS activo: http://%s.local", MDNS_HOSTNAME);
  } else {
    ESP_LOGE(TAG, "mdns_init fallo: %s", esp_err_to_name(err));
  }
  start_webserver();
  vTaskDelete(NULL);
}

// ================= WIFI STA CON RECONEXION AUTOMATICA =================
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
  (void)arg;
  if (base == WIFI_EVENT) {
    if (id == WIFI_EVENT_STA_START) {
      esp_wifi_connect();
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
      wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)data;
      ESP_LOGW(TAG, "WiFi desconectado (motivo %d). Reintentando conexion...", ev ? (int)ev->reason : -1);
      esp_wifi_connect();          // no bloquea: solo encola el intento
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "WiFi conectado. IP asignada: " IPSTR, IP2STR(&ev->ip_info.ip));
    if (!s_servicesStarted) {
      s_servicesStarted = true;
      xTaskCreate(services_task, "services_task", 4096, nullptr, 4, nullptr);
    }
  }
}

static void wifiInit() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

  wifi_config_t wc = {};
  strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
  strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
  wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wc.sta.pmf_cfg.capable = true;
  wc.sta.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_ps(WIFI_PS_NONE);   // mejor respuesta de mDNS/HTTP
}

extern "C" void app_main(void) {
  g_telMutex = xSemaphoreCreateMutex();
  configASSERT(g_telMutex != nullptr);

  wifiInit();                       // no bloquea: la conexion ocurre por eventos

  // Tarea dedicada al sensor, fijada al core 1 (WiFi/lwIP corren en el core 0)
  xTaskCreatePinnedToCore(ltr390_task, "ltr390_task", 8192, nullptr, 5, nullptr, 1);
}
