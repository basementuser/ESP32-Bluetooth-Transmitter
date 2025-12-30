#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include "driver/i2s.h"
#include "ES8388.h"
#include "esp_wifi.h"
#include "esp_pm.h"

// ---------------------------------------------------------------------
//                          POWER CONFIG
// ---------------------------------------------------------------------
#define CPU_FREQ_MHZ 120   // Stable for SBC A2DP

// ---------------------------------------------------------------------
//                               LED
// ---------------------------------------------------------------------
#define STATUS_LED_PIN 12

unsigned long ledTimer = 0;
uint8_t ledPhase = 0;

// Startup LED suppression
unsigned long startupTime = 0;
const unsigned long STARTUP_LED_DELAY_MS = 2000;

// ---------------------------------------------------------------------
//                               BLUETOOTH
// ---------------------------------------------------------------------
BluetoothA2DPSource a2dp;

struct BestDevice {
  esp_bd_addr_t addr;
  int rssi;
  bool valid = false;
} bestDevice;

// ---------------------------------------------------------------------
//                     DEVICE DISCOVERY CALLBACK
// ---------------------------------------------------------------------
bool ssidCallback(const char *ssid, esp_bd_addr_t address, int rssi) {

  if (!bestDevice.valid || rssi > bestDevice.rssi) {
    memcpy(bestDevice.addr, address, ESP_BD_ADDR_LEN);
    bestDevice.rssi = rssi;
    bestDevice.valid = true;
  }

  // Stop scanning once we have a strong device
  return (rssi >= -60);
}

// ---------------------------------------------------------------------
//                               AUDIO / I2S
// ---------------------------------------------------------------------
#define SAMPLE_RATE 44100
#define I2S_PORT I2S_NUM_0
#define MAX_FRAME_COUNT 512

#define I2S_BCK_PIN 27
#define I2S_WS_PIN  25
#define I2S_DATA_IN_PIN 35   // ADC SDOUT from ES8388
#define I2S_MCLK_PIN 0

ES8388 es8388(33, 32, 400000);  // SDA, SCL, I2C speed

static int16_t i2s_buffer[MAX_FRAME_COUNT * 2] __attribute__((aligned(4)));

void init_i2s_for_es8388() {

  i2s_config_t cfg = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 6,
  .dma_buf_len = 256,
  .use_apll = true,
  .tx_desc_auto_clear = false,
  .fixed_mclk = SAMPLE_RATE * 256
};

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num  = I2S_WS_PIN,
    .data_out_num = -1,
    .data_in_num  = I2S_DATA_IN_PIN
  };

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_set_sample_rates(I2S_PORT, SAMPLE_RATE);
}

void init_es8388_clean() {
        
  es8388.init();      
  
  // ADC path
  es8388.inputSelect(IN2);        // LINEIN
  es8388.setInputGain(1);
  es8388.setALCmode(DISABLE);

  // DAC path (for monitoring / headphones)
  es8388.outputSelect(OUT2);
  es8388.setOutputVolume(100);

  // Route ADC internally as well (headphone works even if I2S fails)
  es8388.mixerSourceSelect(MIXADC, MIXADC);
  es8388.mixerSourceControl(SRCSELOUT);
}

// ---------------------------------------------------------------------
//                      I2S → A2DP PASSTHROUGH
// ---------------------------------------------------------------------
int32_t get_data_from_es8388(Frame *frame, int32_t frame_count) {

  if (frame_count > MAX_FRAME_COUNT) frame_count = MAX_FRAME_COUNT;

  size_t bytes_read = 0;

  i2s_read(
    I2S_PORT,
    i2s_buffer,
    frame_count * 2 * sizeof(int16_t),
    &bytes_read,
    portMAX_DELAY
  );

  int frames = bytes_read / (sizeof(int16_t) * 2);

  for (int i = 0; i < frames; i++) {
    frame[i].channel1 = i2s_buffer[i * 2];
    frame[i].channel2 = i2s_buffer[i * 2 + 1];
  }

  return frames;
}

// ---------------------------------------------------------------------
//                         AVRCP BUTTON HANDLER
// ---------------------------------------------------------------------
const unsigned long PAUSE_DOUBLE_WINDOW_MS = 60000;
unsigned long lastPauseTime = 0;
int pausePressCount = 0;

void button_handler(uint8_t id, bool isReleased) {

  if (isReleased) return;

  unsigned long now = millis();

  if (id != 70) {
    pausePressCount = 0;
    lastPauseTime = 0;
  }

  switch (id) {

    case 75: // NEXT
      Serial2.write(
        (uint8_t*)"\xFFU\x03\x02\x00\x08\xF3\xFFU\x03\x02\x00\x00\xFB", 14);
      break;

    case 76: // PREV
      Serial2.write(
        (uint8_t*)"\xFFU\x03\x02\x00\x10\xEB\xFFU\x03\x02\x00\x00\xFB", 14);
      break;

    case 68: // PLAY
      Serial2.write(
        (uint8_t*)"\xFFU\x04\x02\x00\x00\x01\xF9\xFFU\x03\x02\x00\x00\xFB", 15);
      pausePressCount = 0;
      lastPauseTime = 0;
      break;

    case 70: // PAUSE / DOUBLE PAUSE = PLAY
      if (pausePressCount == 0) {
        Serial2.write(
          (uint8_t*)"\xFFU\x04\x02\x00\x00\x02\xF8\xFFU\x03\x02\x00\x00\xFB", 15);
        pausePressCount = 1;
        lastPauseTime = now;
      } else if (now - lastPauseTime <= PAUSE_DOUBLE_WINDOW_MS) {
        Serial2.write(
          (uint8_t*)"\xFFU\x04\x02\x00\x00\x01\xF9\xFFU\x03\x02\x00\x00\xFB", 15);
        pausePressCount = 0;
        lastPauseTime = 0;
      }
      break;

    default:
      pausePressCount = 0;
      lastPauseTime = 0;
      break;
  }
}

// ---------------------------------------------------------------------
//                                SETUP
// ---------------------------------------------------------------------
void setup() {
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  esp_pm_config_esp32_t pm_cfg = {
    .max_freq_mhz = CPU_FREQ_MHZ,
    .min_freq_mhz = 80,
    .light_sleep_enable = false
  };
  esp_pm_configure(&pm_cfg);

  Serial.begin(115200);
  delay(200);

  esp_wifi_stop();
  esp_wifi_deinit();

  Serial2.begin(19200, SERIAL_8N1, 22, 21);

  init_es8388_clean();
  // Allow analog + bias + internal refs to settle
  delay(200);   // <- important test delay

  // Now bring up I2S
  init_i2s_for_es8388();

  i2s_zero_dma_buffer(I2S_PORT);
  delay(50);

  a2dp.set_local_name("WyMod AeroStream");
  a2dp.set_auto_reconnect(false);
  a2dp.set_reset_ble(true);

  a2dp.set_valid_cod_service(
    ESP_BT_COD_SRVC_RENDERING |
    ESP_BT_COD_SRVC_AUDIO |
    ESP_BT_COD_SRVC_TELEPHONY
  );

  a2dp.set_ssid_callback(ssidCallback);
  a2dp.set_data_callback_in_frames(get_data_from_es8388);
  a2dp.set_avrc_passthru_command_callback(button_handler);
  a2dp.set_volume(100);

  a2dp.start();

  startupTime = millis();

  Serial.flush();
  Serial.end();
}

// ---------------------------------------------------------------------
//                                LOOP
// ---------------------------------------------------------------------
void loop() {

  unsigned long now = millis();

  if (now - startupTime < STARTUP_LED_DELAY_MS) {
    digitalWrite(STATUS_LED_PIN, LOW);
    ledPhase = 0;
    ledTimer = now;
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }

  if (!bestDevice.valid) {
    if (now - ledTimer >= 1000) {
      ledTimer = now;
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
  }
  else if (a2dp.is_connected()) {
    switch (ledPhase) {
      case 0:
        if (now - ledTimer >= 5000) {
          digitalWrite(STATUS_LED_PIN, HIGH);
          ledTimer = now;
          ledPhase = 1;
        }
        break;
      case 1:
        if (now - ledTimer >= 120) {
          digitalWrite(STATUS_LED_PIN, LOW);
          ledTimer = now;
          ledPhase = 2;
        }
        break;
      case 2:
        if (now - ledTimer >= 120) {
          digitalWrite(STATUS_LED_PIN, HIGH);
          ledTimer = now;
          ledPhase = 3;
        }
        break;
      case 3:
        if (now - ledTimer >= 120) {
          digitalWrite(STATUS_LED_PIN, LOW);
          ledTimer = now;
          ledPhase = 0;
        }
        break;
    }
  }
  else {
    digitalWrite(STATUS_LED_PIN, LOW);
    ledPhase = 0;
  }

  if (pausePressCount &&
      (millis() - lastPauseTime > PAUSE_DOUBLE_WINDOW_MS)) {
    pausePressCount = 0;
    lastPauseTime = 0;
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}
