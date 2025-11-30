// Works on XIAO ESP32-C3 with esp32 Arduino core (uses esp_timer)
#include <Arduino.h>
#include "esp_timer.h"

#define ANALOG_PIN    D0
#define NUM_SAMPLES   20000
#define SAMPLE_HZ     200
#define PERIOD_US     (1000000 / SAMPLE_HZ)

volatile uint16_t samples[NUM_SAMPLES];
volatile int sampleIndex = 0;
volatile bool done = false;
int last_message = 0;
int seconds_left = 100;

esp_timer_handle_t periodic_timer = nullptr;

void periodic_cb(void* /*arg*/) {
  // Runs in esp_timer task context => it's safe to call analogRead()
  if (sampleIndex < NUM_SAMPLES) {
    samples[sampleIndex++] = analogRead(ANALOG_PIN);
    if (sampleIndex >= NUM_SAMPLES) {
      done = true;
      // stop timer (safe from timer callback)
      esp_timer_stop(periodic_timer);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Starting sampling (esp_timer) ...");
  analogReadResolution(12);      // 0..4095
  pinMode(ANALOG_PIN, INPUT);

  const esp_timer_create_args_t args = {
    .callback = &periodic_cb,
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

  last_message = millis();
}

void loop() {
  if (done) {
    Serial.println("Sampling complete. Printing samples:");
    Serial.print("[");
    for (int i = 0; i < NUM_SAMPLES; ++i) {
      Serial.print(samples[i]);
      Serial.print(", ");
    }
    Serial.println("]");
    // cleanup (optional)
    esp_timer_delete(periodic_timer);
    // done — stop here
    while (1) delay(1000);
  }
  else
  {
    if (millis() - last_message >= 1000)
    {
      seconds_left--;
      Serial.print("Seconds left for the data acquisition: ");
      Serial.println(seconds_left);
      last_message = millis();
    }
  }
}
