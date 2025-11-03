#include <Arduino.h>
#include "esp_bt.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_sleep.h"

// ===== User config =====
#define MARK_GPIO        2          // marker pin to PPK2
#define MEASURE_MS       5000      // ms to hold each phase = measure window
#define GAP_MS           500        // ms gap between phases

#define I2C_PORT         I2C_NUM_0
#define SPI_HOST_USED    SPI2_HOST  // ESP32-H2: use SPI2_HOST
#define UART_FOR_TEST    UART_NUM_1 // leave UART0 alone so chip can still debug if needed someday

// NOTE: adjust these pins to real pins on your board!
#define I2C_SDA_PIN      GPIO_NUM_8
#define I2C_SCL_PIN      GPIO_NUM_9
#define SPI_MOSI_PIN     GPIO_NUM_10
#define SPI_MISO_PIN     GPIO_NUM_11
#define SPI_SCLK_PIN     GPIO_NUM_12
#define UART1_TX_PIN     GPIO_NUM_6
#define UART1_RX_PIN     GPIO_NUM_7
#define ADC_PIN          4          // ADC channel pin (adjust to your board)

// ===== State flags =====
static bool i2c_on  = false;
static bool spi_on  = false;
static bool uart_on = false;
static bool bt_on   = false;
static bool adc_on  = false;
static volatile bool adc_reading = false; // flag for continuous ADC reading

// ===== Helpers =====
static inline void marker(bool on) {
  digitalWrite(MARK_GPIO, on ? HIGH : LOW);
}
static inline void hold_measure_window() {
  delay(MEASURE_MS);
}
static inline void gap() {
  delay(GAP_MS);
}

// ===== Init peripherals =====
static void init_i2c() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA_PIN;
  conf.scl_io_num = I2C_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000; // 100 kHz

  i2c_param_config(I2C_PORT, &conf);
  if (i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK) {
    i2c_on = true;
  }
}

static void init_spi() {
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = SPI_MOSI_PIN;
  buscfg.miso_io_num = SPI_MISO_PIN;
  buscfg.sclk_io_num = SPI_SCLK_PIN;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  if (spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_DISABLED) == ESP_OK) {
    spi_on = true;
  }
}

static void init_uart1() {
  uart_config_t uc = {};
  uc.baud_rate = 115200;
  uc.data_bits = UART_DATA_8_BITS;
  uc.parity    = UART_PARITY_DISABLE;
  uc.stop_bits = UART_STOP_BITS_1;
  uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  uart_param_config(UART_FOR_TEST, &uc);
  if (uart_set_pin(UART_FOR_TEST, UART1_TX_PIN, UART1_RX_PIN, -1, -1) == ESP_OK &&
      uart_driver_install(UART_FOR_TEST, 256, 0, 0, NULL, 0) == ESP_OK) {
    uart_on = true;
  }
}

static void init_bt() {
  esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if (esp_bt_controller_init(&cfg) == ESP_OK) {
    if (esp_bt_controller_enable(ESP_BT_MODE_BLE) == ESP_OK) {
      bt_on = true;
    } else {
      // enable failed -> cleanup init to not leave partial state
      esp_bt_controller_deinit();
    }
  }
}

static void init_adc() {
  // Configure ADC pin as analog input
  pinMode(ADC_PIN, ANALOG);
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  analogReadResolution(12);       // 12-bit resolution (0-4095)

  // Trigger first read to initialize ADC
  analogRead(ADC_PIN);
  adc_on = true;
  adc_reading = true;
}

// ===== Kill peripherals =====
static void kill_i2c() {
  if (i2c_on) {
    i2c_driver_delete(I2C_PORT);
    i2c_on = false;
  }
}

static void kill_spi() {
  if (spi_on) {
    spi_bus_free(SPI_HOST_USED);
    spi_on = false;
  }
}

static void kill_uart1() {
  if (uart_on) {
    // Disable UART TX/RX
    uart_disable_tx_intr(UART_FOR_TEST);
    uart_disable_rx_intr(UART_FOR_TEST);

    // Delete driver
    uart_driver_delete(UART_FOR_TEST);

    // Set pins to input to completely disable UART peripheral
    pinMode(UART1_TX_PIN, INPUT);
    pinMode(UART1_RX_PIN, INPUT);

    uart_on = false;
  }
}

static void kill_bt() {
  if (bt_on) {
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    bt_on = false;
  }
}

static void kill_adc() {
  if (adc_on) {
    adc_reading = false;
    // Set pin to digital input to disable ADC
    pinMode(ADC_PIN, INPUT);
    adc_on = false;
  }
}

// ===== Phase macro =====
//
// marker HIGH -> držíme stav periferie X zapnuto/vypnuto
// během toho měříš proud (PPK2 current)
// marker LOW  -> přechod, GAP_MS pauza
//
static void do_phase_and_then(void (*afterPhaseAction)()) {
  marker(true);

  // Actively use peripherals during measurement to show power consumption
  unsigned long start = micros();
  unsigned long target = (unsigned long)MEASURE_MS * 1000UL;
  int activity_counter = 0;

  while ((micros() - start) < target) {
    activity_counter++;

    // Every ~10000 iterations, do peripheral activity
    if (activity_counter >= 10000) {
      // I2C: Try to read from a dummy address (will fail but consumes power)
      if (i2c_on) {
        uint8_t dummy;
        i2c_master_read_from_device(I2C_PORT, 0x50, &dummy, 1, 10 / portTICK_PERIOD_MS);
      }

      // SPI: Just toggling the bus consumes power (even without device)
      if (spi_on) {
        // SPI peripheral is active when bus is initialized
        // Clock and data lines toggling consume power
      }

      // ADC: Continuous reading
      if (adc_reading && adc_on) {
        volatile int dummy_adc = analogRead(ADC_PIN);
        (void)dummy_adc;
      }

      // UART: Try to transmit dummy data
      if (uart_on) {
        uint8_t dummy_data = 0xAA;
        uart_write_bytes(UART_FOR_TEST, &dummy_data, 1);
      }

      activity_counter = 0;
    }

    // Busy wait with NOPs - keeps CPU active but minimizes state changes
    __asm__ __volatile__("nop");
  }

  marker(false);
  gap();

  if (afterPhaseAction) {
    afterPhaseAction();
  }
}

// ===== Arduino setup =====
void setup() {
  // Reduce CPU frequency for more stable current consumption
  // Lower frequency = less switching noise = more stable current
  // setCpuFrequencyMhz(32); // Options: 32, 64, 96 MHz (default is usually 96)

  pinMode(MARK_GPIO, OUTPUT);
  marker(false);

  // baseline: zapni všechno, co chceme později vypínat
  init_i2c();
  init_spi();
  init_uart1();
  init_bt();
  init_adc();

  delay(200); // krátká ustálení před první fází
}

// ===== Arduino loop =====
//
// sekvence:
//   F0: vše zapnuté (BT + I2C + SPI + UART1 + ADC)
//   kill ADC
//   F1: ADC off
//   kill BT
//   F2: BT off
//   kill I2C
//   F3: I2C off
//   kill SPI
//   F4: SPI off
//   kill UART1
//   F5: UART1 off
//   F6: light sleep snapshot
//
// Opakuje se to dokola.
//
void loop() {
  // F0: baseline (BT + I2C + SPI + UART1 + ADC zapnuto)
  do_phase_and_then(nullptr);

  // Vypni ADC
  kill_adc();
  // F1: ADC off
  do_phase_and_then(nullptr);

  // Vypni BT rádio
  kill_bt();
  // F2: BT off
  do_phase_and_then(nullptr);

  // Vypni I2C
  kill_i2c();
  // F3: I2C off
  do_phase_and_then(nullptr);

  // Vypni SPI
  kill_spi();
  // F4: SPI off
  do_phase_and_then(nullptr);

  // Vypni UART1
  kill_uart1();
  // F5: UART1 off
  do_phase_and_then(nullptr);

  // F6: light sleep snapshot = skoro minimum bez deep sleepu
  marker(true);

  // naplánuj probuzení po MEASURE_MS
  esp_sleep_enable_timer_wakeup((uint64_t)MEASURE_MS * 1000ULL);

  // light sleep = CPU off, většina clock domén off, RAM drží
  esp_light_sleep_start();

  marker(false);
  gap();

  // volitelně můžeš dát deep sleep místo opakování:
  // esp_deep_sleep_start();

  // opakujeme znovu: ale pozor,
  // po první smyčce už BT/I2C/SPI/UART1/ADC zůstávají vypnuté.
  // To je OK pokud ti stačí jednorázové pořadí pro zachycení na PPK2.
  //
  // Pokud chceš cyklus pokaždé od nuly (=zase zapnout vše),
  // odkomentuj následující 5 volání:
  //
  // init_bt();
  // init_i2c();
  // init_spi();
  // init_uart1();
  // init_adc();

  delay(5000); // klidová pauza před dalším kolem
}
