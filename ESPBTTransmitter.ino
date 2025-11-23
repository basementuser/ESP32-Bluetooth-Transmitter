#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include "driver/i2s.h"
#include "ES8388.h"
#include <math.h>

// ---------------- Bluetooth ----------------
BluetoothA2DPSource a2dp_source;

// ---------------- I2S settings ----------------
#define SAMPLE_RATE 44100
#define I2S_NUM I2S_NUM_0
#define I2S_BITS I2S_BITS_PER_SAMPLE_16BIT
#define I2S_FORMAT I2S_CHANNEL_FMT_RIGHT_LEFT
#define I2S_BCK_PIN 27
#define I2S_WS_PIN 25
#define I2S_DATA_IN_PIN 35
#define I2S_DATA_OUT_PIN -1
#define MAX_FRAME_COUNT 512

// ---------------- ES8388 ----------------
ES8388 es8388(33, 32, 400000);

// ---------------- DSP PARAMETERS (user adjustable) ----------------
// Per-band gains in dB (default: gentle smile / V-curve that's commonly pleasing)
float EQ_GAIN_DB_31  = +3.0f;   // 31 Hz
float EQ_GAIN_DB_62  = +2.5f;   // 62 Hz
float EQ_GAIN_DB_125 = +1.5f;   // 125 Hz
float EQ_GAIN_DB_250 =  0.0f;   // 250 Hz
float EQ_GAIN_DB_500 = -0.5f;   // 500 Hz
float EQ_GAIN_DB_1K  =  0.0f;   // 1 kHz
float EQ_GAIN_DB_2K  = +0.5f;   // 2 kHz
float EQ_GAIN_DB_4K  = +1.5f;   // 4 kHz
float EQ_GAIN_DB_8K  = +2.5f;   // 8 kHz
float EQ_GAIN_DB_16K = +3.5f;   // 16 kHz

// Master controls (in dB). These add to the groups defined below.
float BASS_GAIN_DB   = +0.0f;   // affects 31, 62, 125, 250
float MID_GAIN_DB    = +0.0f;   // affects 500, 1k, 2k
float TREBLE_GAIN_DB = +0.0f;   // affects 4k, 8k, 16k

// Shared Q for peaking filters (bandwidth). Adjust to taste.
float EQ_Q = 1.0f; // 0.6..1.4 typical

// Optional slight fixed pre-gain and gentle soft clip (keeps peaks in check)
const float FIXED_GAIN = 0.95f; // base gain (slightly under unity)
const float SOFT_CLIP_GAIN = 0.06f; // small amount for tanhf soft clipping

// ---------------- Internal state ----------------
static float lpf_a = 0.0f;
static float lpf_prev_ch1 = 0.0f;
static float lpf_prev_ch2 = 0.0f;
const float LPF_CUTOFF_HZ = 10000.0f;

// ---------------- EQ band definitions ----------------
const int EQ_BAND_COUNT = 10;
const float EQ_FREQS[EQ_BAND_COUNT] = {31.0f, 62.0f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f, 8000.0f, 16000.0f};

// Biquad filter structure (Direct Form Transposed)
struct Biquad {
  float b0, b1, b2, a1, a2; // coefficients (a0 normalized out)
  float z1, z2;            // state
  void clear() { z1 = z2 = 0.0f; }
  inline float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x + z2 - a1 * y;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

// Two channels: left and right, each band needs independent state.
static Biquad eqL[EQ_BAND_COUNT];
static Biquad eqR[EQ_BAND_COUNT];

// ---------------- helper functions ----------------
inline float db_to_linear(float db) {
  return powf(10.0f, db / 20.0f);
}

// Setup peaking EQ coefficients (classic RBJ peaking EQ)
void setup_peaking(Biquad &bq, float centerFreq, float gainDB, float Q, float sampleRate) {
  // gainDB is the target boost/cut in dB
  float A = powf(10.0f, gainDB / 40.0f); // A = 10^(dBgain/40)
  float w0 = 2.0f * M_PI * centerFreq / sampleRate;
  float cosw0 = cosf(w0);
  float sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * Q);

  float b0 = 1.0f + alpha * A;
  float b1 = -2.0f * cosw0;
  float b2 = 1.0f - alpha * A;
  float a0 = 1.0f + alpha / A;
  float a1 = -2.0f * cosw0;
  float a2 = 1.0f - alpha / A;

  // normalize
  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
}

// Recalculate EQ coefficients for all bands (call this after changing any EQ dB/Q)
void update_all_eq() {
  // Compute per-band effective dB = band setting + master group adds
  float band_db[EQ_BAND_COUNT];

  // Assign per-band base gains from the variables, then add masters
  band_db[0] = EQ_GAIN_DB_31  + BASS_GAIN_DB;   // 31
  band_db[1] = EQ_GAIN_DB_62  + BASS_GAIN_DB;   // 62
  band_db[2] = EQ_GAIN_DB_125 + BASS_GAIN_DB;   // 125
  band_db[3] = EQ_GAIN_DB_250 + BASS_GAIN_DB;   // 250
  band_db[4] = EQ_GAIN_DB_500 + MID_GAIN_DB;    // 500
  band_db[5] = EQ_GAIN_DB_1K  + MID_GAIN_DB;    // 1k
  band_db[6] = EQ_GAIN_DB_2K  + MID_GAIN_DB;    // 2k
  band_db[7] = EQ_GAIN_DB_4K  + TREBLE_GAIN_DB; // 4k
  band_db[8] = EQ_GAIN_DB_8K  + TREBLE_GAIN_DB; // 8k
  band_db[9] = EQ_GAIN_DB_16K + TREBLE_GAIN_DB; // 16k

  for (int b = 0; b < EQ_BAND_COUNT; ++b) {
    // Setup coefficients for both left and right, but keep states (z1/z2)
    Biquad tmpL = eqL[b];
    Biquad tmpR = eqR[b];
    // Preserve state before overwriting coefficients
    float z1L = tmpL.z1, z2L = tmpL.z2;
    float z1R = tmpR.z1, z2R = tmpR.z2;

    // compute new coefficients into a temp Biquad then copy coefficients and restore state
    Biquad coeff;
    setup_peaking(coeff, EQ_FREQS[b], band_db[b], EQ_Q, (float)SAMPLE_RATE);

    // apply to left
    eqL[b].b0 = coeff.b0; eqL[b].b1 = coeff.b1; eqL[b].b2 = coeff.b2;
    eqL[b].a1 = coeff.a1; eqL[b].a2 = coeff.a2;
    // restore state
    eqL[b].z1 = z1L; eqL[b].z2 = z2L;

    // apply to right
    eqR[b].b0 = coeff.b0; eqR[b].b1 = coeff.b1; eqR[b].b2 = coeff.b2;
    eqR[b].a1 = coeff.a1; eqR[b].a2 = coeff.a2;
    // restore state
    eqR[b].z1 = z1R; eqR[b].z2 = z2R;
  }
}

// ---------------- I2S INIT ----------------
void init_i2s_for_es8388() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS,
    .channel_format = I2S_FORMAT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_DATA_OUT_PIN,
    .data_in_num = I2S_DATA_IN_PIN
  };
  esp_err_t r = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  if (r != ESP_OK) Serial.printf("i2s_driver_install failed: %d\n", r);
  r = i2s_set_pin(I2S_NUM, &pin_config);
  if (r != ESP_OK) Serial.printf("i2s_set_pin failed: %d\n", r);
  i2s_set_sample_rates(I2S_NUM, SAMPLE_RATE);

  // LPF coefficient for optional smoothing
  float dt = 1.0f / (float)SAMPLE_RATE;
  float rc = 1.0f / (2.0f * M_PI * LPF_CUTOFF_HZ);
  lpf_a = dt / (rc + dt);
}

// ---------------- ES8388 INIT ----------------
void init_es8388_clean() {
  Serial.println("Initializing ES8388…");
  if (!es8388.init()) {
    Serial.println("ES8388 init FAIL!");
    return;
  }
  es8388.inputSelect(IN2);         // LINE-IN
  es8388.setInputGain(0);          // cleanest
  es8388.outputSelect(OUT2);
  es8388.setOutputVolume(95);      // moderate DAC output
  es8388.mixerSourceSelect(MIXADC, MIXADC);
  es8388.mixerSourceControl(SRCSELOUT);
  Serial.println("ES8388 init done.");
}

// ---------------- SOFT CLIP ----------------
inline float mild_saturation(float x) {
  // small soft clipping; scale factor controls the curve
  return tanhf(x * SOFT_CLIP_GAIN) / SOFT_CLIP_GAIN;
}

// ---------------- READ + DSP ----------------
int32_t get_data_from_es8388(Frame *frame, int32_t frame_count) {
  if (frame_count <= 0) return 0;
  static int16_t i2s_buffer[MAX_FRAME_COUNT * 2];
  if (frame_count > MAX_FRAME_COUNT) frame_count = MAX_FRAME_COUNT;
  size_t bytes_to_read = (size_t)frame_count * 2 * sizeof(int16_t);
  size_t bytes_read = 0;
  esp_err_t res = i2s_read(I2S_NUM, (void*)i2s_buffer, bytes_to_read, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int16_t);
  int stereo_frames_read = samples_read / 2;
  if (stereo_frames_read > frame_count) stereo_frames_read = frame_count;

  // --- Apply DSP per-sample ---
  for (int i = 0; i < stereo_frames_read; i++) {
    int idx = i * 2;
    float chL = (float)i2s_buffer[idx]   / 32768.0f;
    float chR = (float)i2s_buffer[idx+1] / 32768.0f;

    // Optional LPF smoothing (keeps very high freq jitter down)
    lpf_prev_ch1 = lpf_prev_ch1 + lpf_a * (chL - lpf_prev_ch1);
    lpf_prev_ch2 = lpf_prev_ch2 + lpf_a * (chR - lpf_prev_ch2);
    chL = lpf_prev_ch1;
    chR = lpf_prev_ch2;

    // Apply fixed gain
    chL *= FIXED_GAIN;
    chR *= FIXED_GAIN;

    // --- EQ: run every band through biquads (L then R) ---
    // Process in series: band outputs feed into next band (graphic EQ style)
    for (int b = 0; b < EQ_BAND_COUNT; ++b) {
      chL = eqL[b].process(chL);
      chR = eqR[b].process(chR);
    }

    // Soft clip to tame extremes (gentle)
    chL = mild_saturation(chL);
    chR = mild_saturation(chR);

    frame[i].channel1 = (int16_t)constrain((int)roundf(chL * 32767.0f), -32768, 32767);
    frame[i].channel2 = (int16_t)constrain((int)roundf(chR * 32767.0f), -32768, 32767);
  }

  // Zero-fill remainder
  for (int i = stereo_frames_read; i < frame_count; i++) {
    frame[i].channel1 = 0;
    frame[i].channel2 = 0;
  }
  return stereo_frames_read;
}

// ---------------- AVRCP ----------------
void button_handler(uint8_t id, bool isReleased) {
  if (isReleased) Serial.printf("button id %u released\n", id);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Clear states
  for (int b = 0; b < EQ_BAND_COUNT; ++b) {
    eqL[b].clear();
    eqR[b].clear();
  }

  init_i2s_for_es8388();
  init_es8388_clean();

  // Initialize EQ coefficients with current parameters
  update_all_eq();

  a2dp_source.set_auto_reconnect(true);
  a2dp_source.set_local_name("WyMod Labs Pro Case");
  a2dp_source.set_data_callback_in_frames(get_data_from_es8388);
  a2dp_source.set_avrc_passthru_command_callback(button_handler);
  a2dp_source.set_volume(60);
  a2dp_source.start("Wesley's Bose QC Ultras");

  Serial.println("Setup complete.");
  Serial.println("EQ Bands (Hz) and current dB settings:");
  for (int b = 0; b < EQ_BAND_COUNT; ++b) {
    float effective_db;
    if (b <= 3) effective_db = (b==0?EQ_GAIN_DB_31: b==1?EQ_GAIN_DB_62: b==2?EQ_GAIN_DB_125:EQ_GAIN_DB_250) + BASS_GAIN_DB;
    else if (b <= 6) effective_db = (b==4?EQ_GAIN_DB_500: b==5?EQ_GAIN_DB_1K:EQ_GAIN_DB_2K) + MID_GAIN_DB;
    else effective_db = (b==7?EQ_GAIN_DB_4K: b==8?EQ_GAIN_DB_8K:EQ_GAIN_DB_16K) + TREBLE_GAIN_DB;
    Serial.printf("  %6.0f Hz : %+4.1f dB\n", EQ_FREQS[b], effective_db);
  }
  Serial.printf("EQ Q: %.2f\n", EQ_Q);
  Serial.printf("Bass master: %+4.1f dB  Mid master: %+4.1f dB  Treble master: %+4.1f dB\n",
                BASS_GAIN_DB, MID_GAIN_DB, TREBLE_GAIN_DB);
}

void loop() {
  // Nothing in loop; A2DP callback supplies audio.
  delay(1000);
}