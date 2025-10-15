#include <Arduino.h>
#include "esp_sleep.h"
#include "esp_timer.h"

// ==== Piny ====
#define GPIO_OUT   4   // generuje obdélník (toggle každých 5 ms -> 100 Hz)
#define GPIO_IN    5   // vstup (propoj OUT -> IN)
#define MARK_GPIO  2   // značka do PPK2 (LOW=polling, HIGH=interrupt)

// ==== Časy ====
#define PHASE_MS       10000
#define GAP_MS         500
#define PREP_SLEEP_MS  300

// ==== Stav / čítače ====
static esp_timer_handle_t sq_timer;
volatile uint32_t isr_edges = 0;
static inline void IRAM_ATTR in_isr() { isr_edges++; }

// 100 Hz square: toggle každých 5000 us
static void toggle_cb(void*) {
  static bool lvl = false;
  lvl = !lvl;
  digitalWrite(GPIO_OUT, lvl);
}

// --- Pomocné ---
static inline void baseline_sleep() {
  esp_sleep_enable_timer_wakeup((uint64_t)PREP_SLEEP_MS * 1000ULL);
  esp_light_sleep_start();
}

// Polling fáze: čistý busy-loop
static inline void run_polling(uint32_t ms) {
  baseline_sleep();
  digitalWrite(MARK_GPIO, LOW);
  uint32_t t0 = millis();
  int last = digitalRead(GPIO_IN);
  while (millis() - t0 < ms) {
    int now = digitalRead(GPIO_IN);
    if (now != last) last = now; // žádný delay
  }
}

// Interrupt+LightSleep fáze: spí a probouzí se na každou změnu úrovně pinu
static inline void run_interrupt_sleep(uint32_t ms) {
  baseline_sleep();
  digitalWrite(MARK_GPIO, HIGH);

  // Zkusíme GPIO wake z Light Sleep. Když by pin neuměl, spadneme do fallbacku.
  bool gpio_wake_ok = true;
  if (gpio_wakeup_enable((gpio_num_t)GPIO_IN, GPIO_INTR_HIGH_LEVEL) != ESP_OK) gpio_wake_ok = false;
  if (esp_sleep_enable_gpio_wakeup() != ESP_OK) gpio_wake_ok = false;

  uint32_t t0 = millis();
  int last = digitalRead(GPIO_IN);

  if (gpio_wake_ok) {
    // Probuzení na každou změnu: střídáme HIGH/LOW level jako wake podmínku
    while (millis() - t0 < ms) {
      // Nastav probuzení na opačnou úroveň, než je aktuální stav
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
      if (last == LOW) {
        gpio_wakeup_enable((gpio_num_t)GPIO_IN, GPIO_INTR_HIGH_LEVEL);
      } else {
        gpio_wakeup_enable((gpio_num_t)GPIO_IN, GPIO_INTR_LOW_LEVEL);
      }
      esp_sleep_enable_gpio_wakeup();

      // Uspi čip – probudí se na další změnu úrovně
      esp_light_sleep_start();

      // Po probuzení přečti novou úroveň a pokračuj
      last = digitalRead(GPIO_IN);
    }
  } else {
    // Fallback: spíme po malých blocích na timer (ne tak „čisté“, ale spotřeba výrazně klesne)
    while (millis() - t0 < ms) {
      esp_sleep_enable_timer_wakeup(10 * 1000ULL); // 10 ms
      esp_light_sleep_start();
    }
  }

  digitalWrite(MARK_GPIO, LOW);
}

void setup() {
  pinMode(GPIO_OUT, OUTPUT);
  pinMode(GPIO_IN, INPUT_PULLDOWN);
  pinMode(MARK_GPIO, OUTPUT);
  digitalWrite(MARK_GPIO, LOW);

  // 100 Hz generátor
  esp_timer_create_args_t tcfg = {};
  tcfg.callback = &toggle_cb;
  tcfg.name = "sq100hz";
  esp_timer_create(&tcfg, &sq_timer);
  esp_timer_start_periodic(sq_timer, 5000); // 5 ms
}

void loop() {
  run_polling(PHASE_MS);
  delay(GAP_MS);
  run_interrupt_sleep(PHASE_MS);
  delay(GAP_MS);
}
