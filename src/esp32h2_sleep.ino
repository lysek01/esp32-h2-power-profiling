#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// ====== nastavení ======
#define MARK_GPIO          2        // digitální značka: H = aktivní okno
#define GPIO_WAKE_PIN      10       // RTC schopný pin; probuzení na HIGH

#define PRE_SLEEP_MS       500
#define POST_SLEEP_MS      3000
#define ACTIVE_WINDOW_MS   1500
#define PERIOD_US          1000
#define DUTY_US            250
#define DEEP_SLEEP_MS      5000

// ====== scénář přes deep sleep ======
RTC_DATA_ATTR uint8_t s = 0; // 0: light→active→light→hop, 1: deep(tmr), 2: deep(GPIO)

// „měkká“ zátěž (stabilní průměrný odběr)
static inline void soft_load_ms(uint32_t ms) {
  const uint32_t total_us = ms * 1000UL;
  uint32_t t = 0;
  while (t < total_us) {
    const uint32_t busy = DUTY_US;
    const uint32_t idle = (PERIOD_US > busy) ? (PERIOD_US - busy) : 0;

    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < busy) { __asm__ __volatile__("nop"); }
    if (idle) delayMicroseconds(idle);
    t += PERIOD_US;
  }
}

static inline void light_sleep_ms(uint32_t ms) {
  esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
  esp_light_sleep_start();
}

static inline void deep_sleep_ms(uint32_t ms) {
  esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
  gpio_hold_en((gpio_num_t)MARK_GPIO);     // drž marker LOW během deep
  esp_deep_sleep_start();                  // reset po probuzení
}

static inline void deep_sleep_gpio_high(int gpio) {
  // EXT1: probuzení na HIGH na zvoleném pinu
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  uint64_t mask = 1ULL << gpio;
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);

  // vstup + pulldown (doporučuju mít i externí 100k pulldown)
  pinMode(gpio, INPUT);
  gpio_pulldown_en((gpio_num_t)gpio);      // interní PD pro případ plovaní

  // marker stáhnout a usnout (marker držíme nízko)
  digitalWrite(MARK_GPIO, LOW);
  gpio_hold_en((gpio_num_t)MARK_GPIO);
  esp_deep_sleep_start();                  // čeká na HIGH → reset
}

void setup() {
  pinMode(MARK_GPIO, OUTPUT);
  digitalWrite(MARK_GPIO, LOW);

  switch (s) {
    default: s = 0; // fallthrough

    case 0: // light → aktivní okno → light → hop (pro oddělení)
      light_sleep_ms(PRE_SLEEP_MS);

      digitalWrite(MARK_GPIO, HIGH);
      soft_load_ms(ACTIVE_WINDOW_MS);
      digitalWrite(MARK_GPIO, LOW);

      light_sleep_ms(POST_SLEEP_MS);

      s = 1;
      deep_sleep_ms(100);                  // krátký DS hop do dalšího stavu
      break;

    case 1: // čisté deep sleep (timer) – minimum proudu
      s = 2;
      digitalWrite(MARK_GPIO, LOW);
      deep_sleep_ms(DEEP_SLEEP_MS);
      break;

    case 2: // deep sleep s probuzením na GPIO HIGH
      s = 0;                               // po probuzení se vrací na prezentaci
      deep_sleep_gpio_high(GPIO_WAKE_PIN);
      break;
  }
}

void loop() { /* nic */ }
