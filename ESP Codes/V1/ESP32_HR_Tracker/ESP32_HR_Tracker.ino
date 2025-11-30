#include <Arduino.h>
#include "esp_timer.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>                              

// Standard Heart Rate Service / Characteristic UUIDs
#define HEART_RATE_SERVICE_UUID        "180D"
#define HEART_RATE_MEASUREMENT_UUID    "2A37"

BLEServer* pServer = nullptr;
BLECharacteristic* pHeartRateChar = nullptr;
bool deviceConnected = false;
// You can override the simulation header at build time: -DSIM_HEADER='"simecg_large.h"'
//#ifndef SIM_HEADER
//#define SIM_HEADER "simecg2.h"
//#endif
//#include SIM_HEADER

#define BAUDRATE        115200
#define ECG_PIN         D0
#define ADC_RES         12
#define SIM_USE         false

// Detection modes: 1=reactive, 2=RR_AVG2 with rate limiting, 3=reactive detection + rate-limited output
#define DETECTION_MODE  3

// Mode 3: Rate limiting parameters (adjust for more/less responsive output)
#define MAX_BPM_DELTA_PER_SEC  1.0f  // max BPM change per second (lower = smoother, higher = more responsive)

#define SAMPLE_HZ       200
#define PERIOD_US       (1000000 / SAMPLE_HZ)
#define BUFFER_LEN      1000

//const int SIM_LEN = sizeof(sim_arr) / sizeof(sim_arr[0]);
const int SIM_LEN = 1;

// Forward declarations for filter helpers used before their definitions
float lowpass_ringbuf(float yn1, float yn2, volatile uint16_t ring_buf[BUFFER_LEN], int bfidx);

// Ring Buffer
volatile uint16_t ecg_buffer[BUFFER_LEN];
volatile int buf_index = 0;
volatile unsigned long sample_counter = 0; // absolute sample count
volatile bool buffer_full = false;

// Filtering steps (Pan-Tompkins at 200 Hz)
volatile float lp_buffer[BUFFER_LEN];     // low-pass output
volatile float hp_buffer[BUFFER_LEN];     // high-pass output
volatile float der_buffer[BUFFER_LEN];    // derivative output
volatile float sq_buffer[BUFFER_LEN];     // squared output
volatile float mwi_buffer[BUFFER_LEN];    // moving window integration output

// State for IIR sections
float lp_yn1 = 0.00f, lp_yn2 = 0.00f;    // low-pass y[n-1], y[n-2]
float hp_yn1 = 0.00f;                    // high-pass y[n-1]

// Moving window integration running sum
const int MWI_WIN = 30;                  // 150 ms at 200 Hz
float mwi_sum = 0.0f;

// Peak detection/adaptive thresholds
const int REFRACTORY_SAMPLES = (int)(0.3f * SAMPLE_HZ); // 300 ms min separation
volatile long last_r_sample_abs = -REFRACTORY_SAMPLES;
float SPKI = 0.0f, NPKI = 0.0f;          // signal/noise levels for integrated signal
float THRESH_I1 = 0.0f;                  // adaptive threshold
// Startup gating: 0-5s warm-up, 5-10s initialize thresholds, >10s detect/HR
const unsigned long WARMUP_SAMPLES = (unsigned long)(5UL * SAMPLE_HZ);
const unsigned long INIT_SAMPLES   = (unsigned long)(10UL * SAMPLE_HZ);
volatile bool thresholds_initialized = false;
volatile float init_mwi_max = 0.0f;

// Filter padding to prevent DC gain and drift (from PanTompkinsFinal)
// This prevents filter startup transients and DC offset issues
const int LP_PADDING = 12;  // Low-pass filter needs 12 samples padding
const int HP_PADDING = 32;  // High-pass filter needs 32 samples padding
volatile bool lp_filter_ready = false;
volatile bool hp_filter_ready = false;

// R-peak storage (timestamps in samples)
volatile unsigned long rpeaks_idx[BUFFER_LEN];
volatile float rpeaks_amp[BUFFER_LEN];
volatile int rpeaks_count = 0;

// Track recent absolute R-peak times for HR calculation
volatile unsigned long recent_r_times[64];
volatile int recent_r_count = 0; // number of valid entries in recent_r_times

// Mode 1: Peak count limiting (max 2 peaks per 200ms window)
volatile unsigned long recent_peak_times[8]; // track last 8 peak times
volatile int recent_peak_count = 0;

// Mode 2: RR_AVG2 algorithm variables (improved from PanTompkinsFinal)
volatile float RR1 = 0.0f;        // most recent RR interval
volatile float RR2 = 0.0f;        // second most recent RR interval (fresh data)
volatile float RR_LOW = 0.0f;     // lower bound (92% of RR_AVG2)
volatile float RR_HIGH = 0.0f;    // upper bound (116% of RR_AVG2)
volatile float RR_AVG2 = 0.0f;    // average of last 2 RR intervals
volatile unsigned long last_accepted_peak_time = 0; // for 5-second timeout mechanism

// Mode 3: Rate-limited output variables
volatile float last_smoothed_bpm = 0.0f;  // last smoothed BPM output

// Timer Interrupt Handler
esp_timer_handle_t periodic_timer = nullptr;

// Sim setup
volatile int sim_ind = 0;
// Note: avoid large dynamic arrays derived from SIM_LEN to prevent RAM exhaustion

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected, restarting advertising...");
    pServer->getAdvertising()->start();
  }
};

// Timer Interrupt Function 
void periodic_cb(void* /*arg*/) {

  // Choose whether the data should come from the sim arr or from actual readings
  if (SIM_USE == true)
  {
    //ecg_buffer[buf_index] = sim_arr[sim_ind];
    sim_ind = (sim_ind + 1) % SIM_LEN;
  }
  else
  {
    ecg_buffer[buf_index] = analogRead(ECG_PIN);
  }

  // Increment the buffer index and absolute sample counter
  buf_index = (buf_index + 1) % BUFFER_LEN;
  sample_counter++;
  if(buf_index == 0) buffer_full = true;

  // Filtering + Detection per-sample once enough history exists
  // Low-pass filter (classic Pan-Tompkins, 200 Hz):
  // y[n] = 2*y[n-1] - y[n-2] + x[n] - 2*x[n-6] + x[n-12]
  float lp_y = 0.0f;
  if (buffer_full || buf_index >= LP_PADDING + 1) {
    lp_y = lowpass_ringbuf(lp_yn1, lp_yn2, ecg_buffer, buf_index);
    lp_yn2 = lp_yn1;
    lp_yn1 = lp_y;
    lp_filter_ready = true;
  } else {
    // During padding phase, use first sample value to prevent DC gain
    lp_y = (float)ecg_buffer[0];
  }
  lp_buffer[(buf_index - 1 + BUFFER_LEN) % BUFFER_LEN] = lp_y;

  // High-pass filter (PT, 200 Hz):
  // y[n] = y[n-1] - x[n]/32 + x[n-16] - x[n-17] + x[n-32]/32, where x is low-pass output
  float hp_y = 0.0f;
  if (lp_filter_ready && (buffer_full || buf_index >= HP_PADDING + 1)) {
    int i0 = (buf_index - 1 + BUFFER_LEN) % BUFFER_LEN;      // n
    int i16 = (buf_index - 17 + BUFFER_LEN) % BUFFER_LEN;    // n-16
    int i17 = (buf_index - 18 + BUFFER_LEN) % BUFFER_LEN;    // n-17
    int i32 = (buf_index - 33 + BUFFER_LEN) % BUFFER_LEN;    // n-32
    float x0 = lp_buffer[i0];
    float x16 = lp_buffer[i16];
    float x17 = lp_buffer[i17];
    float x32 = lp_buffer[i32];
    hp_y = hp_yn1 - (x0 / 32.0f) + x16 - x17 + (x32 / 32.0f);
    hp_yn1 = hp_y;
    hp_filter_ready = true;
  } else {
    // During padding phase, use zero to prevent DC gain
    hp_y = 0.0f;
  }
  hp_buffer[(buf_index - 1 + BUFFER_LEN) % BUFFER_LEN] = hp_y;

  // Derivative (PT, 5-point):
  // y[n] = (1/8) * (2x[n] + x[n-1] - x[n-3] - 2x[n-4]) where x is high-pass output
  float der_y = 0.0f;
  if (buffer_full || buf_index >= 5) {
    int i0 = (buf_index - 1 + BUFFER_LEN) % BUFFER_LEN;   // n
    int i1 = (buf_index - 2 + BUFFER_LEN) % BUFFER_LEN;   // n-1
    int i3 = (buf_index - 4 + BUFFER_LEN) % BUFFER_LEN;   // n-3
    int i4 = (buf_index - 5 + BUFFER_LEN) % BUFFER_LEN;   // n-4
    float x0 = hp_buffer[i0];
    float x1 = hp_buffer[i1];
    float x3 = hp_buffer[i3];
    float x4 = hp_buffer[i4];
    der_y = (2.0f * x0 + x1 - x3 - 2.0f * x4) * 0.125f; // 1/8
  }
  der_buffer[(buf_index - 1 + BUFFER_LEN) % BUFFER_LEN] = der_y;

  // Squaring
  float sq_y = der_y * der_y;
  sq_buffer[(buf_index - 1 + BUFFER_LEN) % BUFFER_LEN] = sq_y;

  // Moving Window Integration (boxcar, 30 samples)
  float mwi_y = 0.0f;
  int cur = (buf_index - 1 + BUFFER_LEN) % BUFFER_LEN;
  if (buffer_full || buf_index > 0) {
    // Update running sum for window
    mwi_sum += sq_y;
    int old = (cur - MWI_WIN + BUFFER_LEN) % BUFFER_LEN;
    if (buffer_full || buf_index >= MWI_WIN) {
      mwi_sum -= sq_buffer[old];
      mwi_y = mwi_sum / (float)MWI_WIN;
    } else {
      // Warm-up: average over available samples
      int win = buf_index;
      if (win < 1) win = 1;
      mwi_y = mwi_sum / (float)win;
    }
  }
  mwi_buffer[cur] = mwi_y;

  // Peak detection on MWI using adaptive threshold and refractory period
  // Detect a local maximum at mwi[n-1]
  if (buffer_full || buf_index >= 3) {
    int i_prev2 = (cur - 2 + BUFFER_LEN) % BUFFER_LEN;
    int i_prev1 = (cur - 1 + BUFFER_LEN) % BUFFER_LEN;
    int i_curr  = cur;
    float v0 = mwi_buffer[i_prev2];
    float v1 = mwi_buffer[i_prev1];
    float v2 = mwi_buffer[i_curr];
    
    // Startup gating
    if (sample_counter < WARMUP_SAMPLES) {
      // First 5 seconds: filters warm-up, no thresholding, no detection
      return;
    }
    if (sample_counter < INIT_SAMPLES) {
      // 5s..10s: collect initial stats for thresholds
      if (v1 > init_mwi_max) init_mwi_max = v1;
      return;
    }
    // If thresholds not set yet (first pass after 10s), initialize now
    if (!thresholds_initialized) {
      SPKI = 0.75f * init_mwi_max;
      NPKI = 0.25f * init_mwi_max;
      THRESH_I1 = NPKI + 0.25f * (SPKI - NPKI);
      thresholds_initialized = true;
    }

    bool is_peak = (v1 > v0) && (v1 >= v2);
    if (is_peak) {
      // Absolute sample number for v1 at index i_prev1 corresponds to (sample_counter - 1)
      long sample_abs = (long)sample_counter - 1;
      if ((sample_abs - last_r_sample_abs) > REFRACTORY_SAMPLES) {
        // Quick QRS width validation on MWI at 50% peak height
        bool width_ok = true;
        int max_span = (int)(0.30f * SAMPLE_HZ); // examine up to 300 ms around peak
        float half = v1 * 0.5f;
        int left = 0;
        for (int s = 1; s <= max_span; s++) {
          int ii = (i_prev1 - s + BUFFER_LEN) % BUFFER_LEN;
          if (mwi_buffer[ii] < half) { left = s; break; }
        }
        int right = 0;
        for (int s = 1; s <= max_span; s++) {
          int ii = (i_prev1 + s) % BUFFER_LEN;
          if (mwi_buffer[ii] < half) { right = s; break; }
        }
        int width_samples = left + right;
        if (left == 0 || right == 0) {
          width_ok = false; // could not bound width
        } else {
          // accept widths roughly 80..200 ms
          int min_w = (int)(0.08f * SAMPLE_HZ);
          int max_w = (int)(0.20f * SAMPLE_HZ);
          width_ok = (width_samples >= min_w && width_samples <= max_w);
        }

        if (width_ok && v1 > THRESH_I1) {
          // QRS detected - apply mode-specific validation
          bool accept_peak = true;
          
          if (DETECTION_MODE == 1 || DETECTION_MODE == 3) {
            // Mode 1 & 3: Check max 2 peaks per 200ms window
            const int WINDOW_MS = 200;
            const int WINDOW_SAMPLES = (int)(WINDOW_MS * SAMPLE_HZ / 1000.0f);
            int peaks_in_window = 0;
            
            // Count peaks in the last 200ms window
            for (int i = 0; i < recent_peak_count; i++) {
              if ((sample_abs - recent_peak_times[i]) <= WINDOW_SAMPLES) {
                peaks_in_window++;
              }
            }
            
            if (peaks_in_window >= 2) {
              accept_peak = false; // Reject: too many peaks in window
            }
            
            // Add this peak to recent times if accepted
            if (accept_peak) {
              if (recent_peak_count < 8) {
                recent_peak_times[recent_peak_count++] = sample_abs;
              } else {
                // Shift left and add new
                for (int s = 1; s < 8; s++) {
                  recent_peak_times[s - 1] = recent_peak_times[s];
                }
                recent_peak_times[7] = sample_abs;
              }
            }
          } else if (DETECTION_MODE == 2) {
            // Mode 2: RR_AVG2 rate limiting validation with timeout recovery
            if (last_r_sample_abs > 0) {
              float current_rr = (float)(sample_abs - last_r_sample_abs);
              
              // Check for timeout: if no peaks accepted for 5 seconds, reset RR_AVG2
              if (RR_AVG2 > 0.0f && last_accepted_peak_time > 0) {
                unsigned long time_since_last = sample_abs - last_accepted_peak_time;
                if (time_since_last > (5 * SAMPLE_HZ)) { // 5 seconds timeout
                  Serial.println("RR_AVG2 timeout - resetting to RR1");
                  RR_AVG2 = RR1; // Reset to just RR1
                  RR_LOW = RR_AVG2 * 0.92f;
                  RR_HIGH = RR_AVG2 * 1.16f;
                }
              }
              
              // For first few peaks, be more permissive to establish baseline
              if (RR_AVG2 == 0.0f) {
                // First RR interval - accept it to start the baseline
                accept_peak = true;
              } else if (RR_AVG2 > 0.0f) {
                // Check if current RR is within bounds (92% to 116% of RR_AVG2)
                if (current_rr < RR_LOW || current_rr > RR_HIGH) {
                  accept_peak = false; // Reject: outside rate limits
                  // Debug: show why peak was rejected
                  Serial.print("Rejected: RR="); Serial.print(current_rr);
                  Serial.print(" bounds=["); Serial.print(RR_LOW);
                  Serial.print(","); Serial.print(RR_HIGH); Serial.println("]");
                }
              }
            }
          }

          if (accept_peak) {
            SPKI = 0.125f * v1 + 0.875f * SPKI;
            
            if (DETECTION_MODE == 2) {
              // Update RR_AVG2 variables BEFORE updating last_r_sample_abs
              if (last_r_sample_abs > 0) {
                float current_rr = (float)(sample_abs - last_r_sample_abs);
                
                // Update RR1 with current interval
                RR1 = current_rr;
                
                // For RR2, use the most recent valid RR interval from recent_r_times
                // This ensures we're using fresh data, not stale RR2 (improved from PanTompkinsFinal)
                if (recent_r_count >= 2) {
                  // Find the most recent RR interval that's different from RR1
                  for (int i = recent_r_count - 2; i >= 0; i--) {
                    if (i + 1 < recent_r_count) {
                      unsigned long prev_peak = recent_r_times[i];
                      unsigned long curr_peak = recent_r_times[i + 1];
                      if (curr_peak > prev_peak) {
                        float candidate_rr = (float)(curr_peak - prev_peak);
                        // Use this RR interval if it's different from RR1 (avoid stale data)
                        if (candidate_rr != RR1) {
                          RR2 = candidate_rr;
                          break;
                        }
                      }
                    }
                  }
                }
                
                // Only calculate RR_AVG2 after we have at least 2 RR intervals
                if (RR2 > 0.0f) {
                  RR_AVG2 = (RR1 + RR2) / 2.0f;
                  RR_LOW = RR_AVG2 * 0.92f;   // 92% lower bound
                  RR_HIGH = RR_AVG2 * 1.16f;  // 116% upper bound
                  last_accepted_peak_time = sample_abs; // Update timeout tracker
                  
                  // Debug: show RR_AVG2 update
                  Serial.print("RR_AVG2 updated: RR1="); Serial.print(RR1);
                  Serial.print(", RR2="); Serial.print(RR2);
                  Serial.print(", RR_AVG2="); Serial.print(RR_AVG2);
                  Serial.print(", bounds=["); Serial.print(RR_LOW);
                  Serial.print(","); Serial.print(RR_HIGH); Serial.println("]");
                }
              }
            }
            
            last_r_sample_abs = sample_abs;
            
            if (rpeaks_count < BUFFER_LEN) {
              rpeaks_idx[rpeaks_count] = (unsigned long)sample_abs;
              rpeaks_amp[rpeaks_count] = v1;
              rpeaks_count++;
            }
            
            // Store recent absolute times (bounded)
            if (recent_r_count < (int)(sizeof(recent_r_times) / sizeof(recent_r_times[0]))) {
              recent_r_times[recent_r_count++] = (unsigned long)sample_abs;
            } else {
              // shift left to keep last values
              for (int s = 1; s < (int)(sizeof(recent_r_times) / sizeof(recent_r_times[0])); s++) {
                recent_r_times[s - 1] = recent_r_times[s];
              }
              recent_r_times[(int)(sizeof(recent_r_times) / sizeof(recent_r_times[0])) - 1] = (unsigned long)sample_abs;
            }
          } else {
            // Rejected peak - treat as noise
            NPKI = 0.125f * v1 + 0.875f * NPKI;
          }
        } else {
          // Noise peak
          NPKI = 0.125f * v1 + 0.875f * NPKI;
        }
        THRESH_I1 = NPKI + 0.25f * (SPKI - NPKI);
      }
    }
  }

  

}

// Turn circular buffer into a linear buffer
void getLinearBuffer(uint16_t* linear_buffer, int len) {
  int start;
  if (buffer_full) {
    // If buffer has wrapped, start from buf_index (oldest element)
    start = buf_index;
  } else {
    // If not full, start from 0
    start = 0;
    len = buf_index;  // only the filled part
  }

  for (int i = 0; i < len; i++) {
    linear_buffer[i] = ecg_buffer[(start + i) % BUFFER_LEN];
  }
}

// Low pass filter function - Linear Buffer
float lowpass_linbuf(float yn1, float yn2, uint16_t lin_buf[BUFFER_LEN])
{
  return 2 * yn1 - yn2 + lin_buf[BUFFER_LEN - 1] - 2 * lin_buf[BUFFER_LEN - 7] + lin_buf[BUFFER_LEN - 13];
}

// Low pass filter function - Circular buffer
float lowpass_ringbuf(float yn1, float yn2, volatile uint16_t ring_buf[BUFFER_LEN], int bfidx)
{
  int idx1 = (bfidx - 1  + BUFFER_LEN) % BUFFER_LEN; // x[n]
  int idx2 = (bfidx - 7  + BUFFER_LEN) % BUFFER_LEN; // x[n-6]
  int idx3 = (bfidx - 13 + BUFFER_LEN) % BUFFER_LEN; // x[n-12]
  return 2 * yn1 - yn2 + ring_buf[idx1] - 2 * ring_buf[idx2] + ring_buf[idx3];
}


// Circular buffer index
int idx(int i) { return (i + BUFFER_LEN) % BUFFER_LEN; }

void setup() {
  // Begin the Serial
  Serial.begin(BAUDRATE);
  delay(200);
  Serial.println("Pan-Tompkins ESP32C3 starting...");
  Serial.print("SIM_USE="); Serial.println(SIM_USE ? "true" : "false");
  Serial.print("SIM_LEN="); Serial.println(SIM_LEN);
  Serial.print("SAMPLE_HZ="); Serial.println(SAMPLE_HZ);
  Serial.print("DETECTION_MODE="); Serial.println(DETECTION_MODE);
  // Increase the ADC resolution from the default 10 to 12
  analogReadResolution(ADC_RES);

  // Configure the ECG Pin as input
  pinMode(ECG_PIN, INPUT);

  const esp_timer_create_args_t args = {
    .callback = &periodic_cb,
    .arg = nullptr,
    .name = "adc_periodic"
  };

  esp_err_t r = esp_timer_create(&args, &periodic_timer);
  if (r != ESP_OK) {
    Serial.printf("esp_timer_create failed: %d\n", r);
    while (1) delay(1000);
  }

  r = esp_timer_start_periodic(periodic_timer, PERIOD_US);
  if (r != ESP_OK) {
    Serial.printf("esp_timer_start_periodic failed: %d\n", r);
    while (1) delay(1000);
  }

  BLEDevice::init("ESP32 HRM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create Heart Rate Service
  BLEService* hrService = pServer->createService(HEART_RATE_SERVICE_UUID);

  // Heart Rate Measurement Characteristic (Notify only)
  pHeartRateChar = hrService->createCharacteristic(
      HEART_RATE_MEASUREMENT_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );

  // Descriptor required for notifications
  pHeartRateChar->addDescriptor(new BLE2902());

  hrService->start();

  // Advertising
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(HEART_RATE_SERVICE_UUID);
  pAdvertising->setAppearance(832); // Heart Rate Sensor
  pAdvertising->start();

  Serial.println("BLE HRM ready, advertising...");

}

void loop() {
  static unsigned long lastStatusMs = 0;

  // Optional debug print of detected peaks every second
  if (millis() - lastStatusMs > 1000) {
    lastStatusMs = millis();
    // Basic heartbeat to show the ISR is running and print the number of samples and recent R-peaks
    unsigned long sc;
    int rc;
    noInterrupts();
    sc = sample_counter;
    rc = recent_r_count;
    interrupts();
    Serial.print("tick: samples="); Serial.print(sc);
    Serial.print(" recentR="); Serial.println(rc);

    // Print the number of detected R-peaks and the timestamps and amplitudes of the R-peaks
    int local_count = 0;
    unsigned long local_r_idx[BUFFER_LEN < 64 ? BUFFER_LEN : 64];
    float local_r_amp[BUFFER_LEN < 64 ? BUFFER_LEN : 64];
    unsigned long local_recent[64];
    int local_recent_count = 0;
    noInterrupts();
    local_count = rpeaks_count;
    for (int i = 0; i < local_count; i++) { local_r_idx[i] = rpeaks_idx[i]; local_r_amp[i] = rpeaks_amp[i]; }
    rpeaks_count = 0;
    local_recent_count = recent_r_count;
    if (local_recent_count > 64) local_recent_count = 64;
    for (int i = 0; i < local_recent_count; i++) { local_recent[i] = recent_r_times[i]; }
    interrupts();

    // Print newly detected peaks for visibility
    if (local_count > 0) {
      Serial.print("Detected R-peaks (count=");
      Serial.print(local_count);
      Serial.println(") in last interval:");
      for (int i = 0; i < local_count; i++) {
        Serial.print(" idx="); Serial.print(local_r_idx[i]);
        Serial.print(" amp="); Serial.println(local_r_amp[i], 6);
      }
    }

    // Compute HR based on detection mode
    if (sample_counter >= INIT_SAMPLES && local_recent_count >= 2) {
      float bpm = 0.0f;
      
      if (DETECTION_MODE == 1) {
        // Mode 1: Use last up to 8 RR intervals (reactive)
        int use_times = local_recent_count;
        if (use_times > 9) use_times = 9;
        // Take the last use_times entries
        int start = local_recent_count - use_times;
        unsigned long prev = local_recent[start];
        float sum_rr = 0.0f;
        int rr_used = 0;
        for (int i = start + 1; i < local_recent_count; i++) {
          unsigned long curr = local_recent[i];
          if (curr > prev) {
            unsigned long rr_samples = curr - prev;
            sum_rr += (float)rr_samples;
            rr_used++;
          }
          prev = curr;
          if (rr_used >= 8) break;
        }
        if (rr_used > 0) {
          float mean_rr_samples = sum_rr / (float)rr_used;
          bpm = 60.0f * ((float)SAMPLE_HZ) / mean_rr_samples;
          Serial.print("HR (last "); Serial.print(rr_used); Serial.print(" RR): ");
          Serial.print(bpm, 1); Serial.println(" bpm");
        }
      } else if (DETECTION_MODE == 3) {
        // Mode 3: Reactive detection + rate-limited output
        int use_times = local_recent_count;
        if (use_times > 9) use_times = 9;
        // Take the last use_times entries
        int start = local_recent_count - use_times;
        unsigned long prev = local_recent[start];
        float sum_rr = 0.0f;
        int rr_used = 0;
        for (int i = start + 1; i < local_recent_count; i++) {
          unsigned long curr = local_recent[i];
          if (curr > prev) {
            unsigned long rr_samples = curr - prev;
            sum_rr += (float)rr_samples;
            rr_used++;
          }
          prev = curr;
          if (rr_used >= 8) break;
        }
        if (rr_used > 0) {
          float mean_rr_samples = sum_rr / (float)rr_used;
          float raw_bpm = 60.0f * ((float)SAMPLE_HZ) / mean_rr_samples;
          
          // Apply rate limiting to the output
          if (last_smoothed_bpm == 0.0f) {
            // First measurement - use raw value
            bpm = raw_bpm;
            last_smoothed_bpm = bpm;
          } else {
            // Rate limit: max 5 BPM change per second
            float delta = raw_bpm - last_smoothed_bpm;
            if (delta > MAX_BPM_DELTA_PER_SEC) {
              bpm = last_smoothed_bpm + MAX_BPM_DELTA_PER_SEC;
            } else if (delta < -MAX_BPM_DELTA_PER_SEC) {
              bpm = last_smoothed_bpm - MAX_BPM_DELTA_PER_SEC;
            } else {
              bpm = raw_bpm; // Within limits, use raw value
            }
            last_smoothed_bpm = bpm;
          }
          
          Serial.print("HR (raw="); Serial.print(raw_bpm, 1);
          Serial.print(", smoothed="); Serial.print(bpm, 1);
          Serial.print(", last "); Serial.print(rr_used); Serial.print(" RR): ");
          Serial.print(bpm, 1); Serial.println(" bpm");
        }
      } else if (DETECTION_MODE == 2) {
        // Mode 2: Use RR_AVG2 (rate-limited, motion-resistant)
        if (RR_AVG2 > 0.0f) {
          bpm = 60.0f * ((float)SAMPLE_HZ) / RR_AVG2;
          Serial.print("HR (RR_AVG2): ");
          Serial.print(bpm, 1); Serial.println(" bpm");
        } else {
          Serial.print("Mode 2: RR_AVG2 not ready (RR1=");
          Serial.print(RR1); Serial.print(", RR2=");
          Serial.print(RR2); Serial.print(", RR_AVG2=");
          Serial.print(RR_AVG2); Serial.print(", bounds=[");
          Serial.print(RR_LOW); Serial.print(","); Serial.print(RR_HIGH);
          Serial.println("])");
        }
      }

      if (bpm > 0.0f && deviceConnected) {
        // Heart Rate Measurement packet (Flags + HR value)
        uint8_t hrmPacket[2];
        hrmPacket[0] = 0x00; // Flags: 8-bit HR value
        hrmPacket[1] = (uint8_t)bpm;  // Heart rate

        pHeartRateChar->setValue(hrmPacket, 2);
        pHeartRateChar->notify();

        Serial.printf("Sent HR: %d bpm\n", (int)bpm);
      }
    }
  }

  // no other periodic work
}

