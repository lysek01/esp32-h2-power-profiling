#include <Arduino.h>
#include "esp_sleep.h"

// ====== Nastavení ======
#define MARK_GPIO           2                 // HIGH = běží zátěž (přiveď do PPK2 jako digitální značku)
#define WORK_ITERS          (10UL * 1000UL * 1000UL)  // délka zátěže (zvyšte/snižte dle požadované délky pulzu)
#define PREP_SLEEP_MS       500               // krátký light sleep před pulzem (baseline)
#define INTER_SCENARIO_MS   1200              // pauza mezi scénáři

// Požadované frekvence (MHz) – použij, co tvůj core skutečně umí (typicky 96 a 48 na H2)
int desired_freqs[] = {96, 64, 48, 32};

// ====== CPU-bound zátěž (deterministická) ======
static uint32_t do_workload(uint32_t iters) {
  volatile uint32_t acc = 1u;
  for (uint32_t i = 0; i < iters; ++i) {
    acc ^= (acc << 13);
    acc ^= (acc >> 17);
    acc ^= (acc << 5);
    acc = acc * 2654435761u + (i | 1u);
  }
  return acc; // hodnota se nepoužívá; brání optimalizaci
}

// Nastaví CPU frekvenci (MHz)
static inline void apply_freq_mhz(int mhz) {
  setCpuFrequencyMhz(mhz);
  delay(50); // krátká stabilizace
}

// Jeden scénář na aktuální frekvenci
static inline void run_scenario() {
  // baseline (light sleep) pro čistý klidový odběr
  esp_sleep_enable_timer_wakeup((uint64_t)PREP_SLEEP_MS * 1000ULL);
  esp_light_sleep_start();

  // zátěž – označ okno do PPK2
  digitalWrite(MARK_GPIO, HIGH);
  (void)do_workload(WORK_ITERS);
  digitalWrite(MARK_GPIO, LOW);

  delay(INTER_SCENARIO_MS);
}

void setup() {
  pinMode(MARK_GPIO, OUTPUT);
  digitalWrite(MARK_GPIO, LOW);

  // Projeď všechny požadované frekvence po sobě
  const int N = sizeof(desired_freqs) / sizeof(desired_freqs[0]);
  for (int i = 0; i < N; ++i) {
    apply_freq_mhz(desired_freqs[i]);
    run_scenario();
  }
}

void loop() {
  // Opakuj poslední frekvenci pro stabilní srovnání
  run_scenario();
}
