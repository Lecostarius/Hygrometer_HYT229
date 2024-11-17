#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <Fonts/FreeSans9pt7b.h>

#include <LittleFS.h>
#include <SingleFileDrive.h>

#include <stdio.h>
#include <inttypes.h>

// ---------------------------- stuff for sleeping
#include "pico/stdio.h"
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/powman.h"


#include "pico/stdlib.h"


#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/adc.h"
#include "hardware/structs/usb.h"
#include <stdint.h>

#include <inttypes.h>

#include "pico.h"

#include "hardware/gpio.h"
#include "hardware/powman.h"

#define SLEEP_TIME_MS 5000

void powman_example_init(uint64_t abs_time_ms);
int powman_example_off_until_gpio_high(int gpio);
int powman_example_off_until_time(uint64_t abs_time_ms);
int powman_example_off_for_ms(uint64_t duration_ms);

static powman_power_state off_state;
static powman_power_state on_state;
// ------------------- end power management stuff

#define OLED_ADDR 0x3C
#define HYT221_ADDR 0x28

#define SCALE_MAX 16384.0
#define TEMP_OFFSET 40.0
#define TEMP_SCALE 165.0
#define HUM_SCALE 100.0

#define TASTERPIN 10

//#define HYT_DEBUG 1
// https://forum.arduino.cc/t/hyt-221-from-hygrosens-instruments/54560/5

Adafruit_BMP280 bmp;

static void disable_usb() {
  usb_hw->phy_direct = USB_USBPHY_DIRECT_TX_PD_BITS | USB_USBPHY_DIRECT_RX_PD_BITS | USB_USBPHY_DIRECT_DM_PULLDN_EN_BITS | USB_USBPHY_DIRECT_DP_PULLDN_EN_BITS;
  usb_hw->phy_direct_override = USB_USBPHY_DIRECT_RX_DM_BITS | USB_USBPHY_DIRECT_RX_DP_BITS | USB_USBPHY_DIRECT_RX_DD_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_DIFFMODE_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_DM_PULLUP_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_FSSLEW_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_PD_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_RX_PD_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_DM_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_DP_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_DM_OE_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_TX_DP_OE_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_DM_PULLDN_EN_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_DP_PULLDN_EN_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_DM_PULLUP_HISEL_OVERRIDE_EN_BITS | USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_HISEL_OVERRIDE_EN_BITS;
}



int can_write = 1;
void myPlugCB(uint32_t data) {
  can_write = 0;
}
void myUnplugCB(uint32_t data) {
  can_write = 1;
}
void myDeleteCB(uint32_t data) {
  LittleFS.remove("littlefsfile.txt");
}

class HYT221 {
private:
  int address;
  int rawTemp;
  int rawHum;

public:
  HYT221(int I2Cadr);
  uint8_t begin(void);
  uint8_t read(void);

  int getRawHumidity(void);
  int getRawTemperature(void);
  double getHumidity(void);
  double getTemperature(void);
};
HYT221::HYT221(int I2Cadr) {
  address = I2Cadr;
}

uint8_t HYT221::begin(void) {
  return 1;
}

uint8_t HYT221::read(void) {
  Wire.beginTransmission(address);
  Wire.write((byte)0x00);
  Wire.available();
  int Ack = Wire.read();  // receive a byte

// DEBUG
#if HYT_DEBUG
  Serial.print("ACK: ");
  Serial.println(Ack);
#endif

  Wire.endTransmission();

// DEBUG ////////////////
//request 4 bytes
#if HYT_DEBUG
  Wire.requestFrom(address, 4);

  Wire.available();

  int a1 = Wire.read();  // receive a byte
  int a2 = Wire.read();  // receive a byte
  int a3 = Wire.read();  // receive a byte
  int a4 = Wire.read();  // receive a byte
#endif
  ////////////////////////////////

  // delay inteval !!! blocking MCU
  delay(100);

  //request 4 bytes
  Wire.requestFrom(address, 4);

  Wire.available();

  int b1 = Wire.read();  // receive a byte
  int b2 = Wire.read();  // receive a byte
  int b3 = Wire.read();  // receive a byte
  int b4 = Wire.read();  // receive a byte

// DEBUG
#if HYT_DEBUG
  Serial.print("a1: ");
  Serial.println(a1, BIN);
  Serial.print("b1: ");
  Serial.println(b1, BIN);
  Serial.print("a2: ");
  Serial.println(a2, BIN);
  Serial.print("b2: ");
  Serial.println(b2, BIN);
  Serial.print("a3: ");
  Serial.println(a3, BIN);
  Serial.print("b3: ");
  Serial.println(b3, BIN);
  Serial.print("a4: ");
  Serial.println(a4, BIN);
  Serial.print("b4: ");
  Serial.println(b4, BIN);
#endif


  // combine the bits
  rawHum = (b1 << 8 | b2) & 0x3FFF;

  // Mask away 2 last bits see HYT 221 doc
  rawTemp = b3 << 6 | (unsigned(b4) >> 2);

  return 1;
}

int HYT221::getRawHumidity(void) {
  return rawHum;
}

int HYT221::getRawTemperature(void) {
  return rawTemp;
}

double HYT221::getHumidity(void) {
  //hum = 100.0 / pow( 2, 14 ) * rawHum;
  return (HUM_SCALE * rawHum) / SCALE_MAX;
}

double HYT221::getTemperature(void) {
  return ((TEMP_SCALE * rawTemp) / SCALE_MAX) - TEMP_OFFSET;
}


// here we go:

Adafruit_SSD1306 display(128, 32, &Wire, -1);
HYT221 hyt(HYT221_ADDR);

uint8_t have_hyt = 0;
uint8_t have_oled = 0;
uint8_t logging = 0;

void setup() {
  int i;
  int ack;
  int rebooting = 0;  // flag that is 0 if we start fresh, 1 if this is a reboot initiated by powman (a wake up from deep sleep)

  set_sys_clock_48mhz();  // run everything from pll_usb pll and stop pll_sys

  Serial.begin(115200);
  Serial.println("Lecostarius");

  LittleFS.begin();
  singleFileDrive.onPlug(myPlugCB);
  singleFileDrive.onUnplug(myUnplugCB);
  singleFileDrive.onDelete(myDeleteCB);
  singleFileDrive.begin("littlefsfile.txt", "myfile.txt");  // export the real file littlefsfile.txt as myfile.txt

  pinMode(TASTERPIN, INPUT_PULLUP);
  pinMode(29, INPUT); // VSYS, to read battery voltage

  // check whether we have all devices on I2C bus:
  Wire.begin();
  Wire.requestFrom(HYT221_ADDR, 1);
  if (Wire.available()) have_hyt = 1;
  Wire.requestFrom(OLED_ADDR, 1);
  if (Wire.available()) have_oled = 1;

  if (have_oled == 0) {
    Serial.println("Found no OLED on I2C bus, stopping!");
    while (1)
      ;
    ;
  }

  hyt.begin();

  if (powman_hw->scratch[1] == 31415) rebooting = 1; // are we coming back from deep sleep?
  if (rebooting) {
    // this is a reboot, not an initial start. Do not clear screen etc!
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, (0 == 1));  // FALSE as third parameter says: no RESET of display
    powman_hw->scratch[0]++;                                   // "timer"
    logging = powman_hw->scratch[2];
    can_write = powman_hw->scratch[3];
  } else {
    // first time we come here, initial power-up, not some restart by powman
    powman_hw->scratch[1] = 31415;  // magic
    powman_hw->scratch[0] = 0;      // initialize timer, first time we boot after power up
    powman_hw->scratch[3] = can_write;
    powman_hw->scratch[2] = logging;
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);  // no third parameter says: RESET of display
    display.clearDisplay();
  }

  display.setTextColor(WHITE);
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  
  if (bmp.begin(0x76), 88) {
    if (!rebooting) {display.setCursor(0, 24); display.print("BMP280: OK");}
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. use FILTER_X2 or FILTER_X16 */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.print("BMP Sensor ID: "); Serial.println(bmp.sensorID(), 16);
  }

  // if this is a cold start, show our splash screen for a second:
  if (!rebooting) {
    display.setCursor(0, 10);
    display.println("Lecostarius");

    display.setCursor(0, 16);
    display.print("HYT: ");
    display.println(have_hyt);
    
    display.display();
    delay(1000);
  }


}

// this will return the y pixel coordinate for the battery bar if
// the voltage of the battery is "bat" volts
// highest value is 4.2V which will result in 0
// lowest is 3.0V which will result in 31
int get_y_pixel(float bat) {
  int bat0 = int((bat - 3.0) * 25);
  if (bat0 < 0) bat0 = 0;
  if (bat0 > 31) bat0 = 31;
  return (31 - bat0);
}
void loop() {
  float bat = analogRead(29) * 3.0 * 3.3 / 1024.0;  // Spannung an VSYS, also Batteriespannung



  hyt.read();
  Serial.println("finished read");
  double h = hyt.getHumidity();
  double t = hyt.getTemperature();
  double p = bmp.readPressure();
  int hraw = hyt.getRawHumidity();
  int traw = hyt.getRawTemperature();
  float alt = bmp.readAltitude();

  // three lines of info about sensor data:
  Serial.print("U bat= ");
  Serial.println(bat);
  Serial.print(t);
  Serial.print(" C, ");
  Serial.print(traw);
  Serial.print(" , ");
  Serial.print(h);
  Serial.print(" %, ");
  Serial.println(hraw);
  Serial.print("pressure: ");
  Serial.println(p);

  if (digitalRead(TASTERPIN) == LOW) {
    logging = 1 - logging;
  }
  powman_hw->scratch[2] = logging;  // keep logging status alive through power cycle


  // now serve the display:
  display.clearDisplay();
  display.setCursor(0, 15);
  display.print(0.1 * int(t * 10));
  display.print("°C/");

  display.print(int(h));
  display.print("%rF");
  display.setCursor(0, 31);
  display.setFont();  // standard font
  display.print(0.1 * int(p / 10));
  display.print(" hPa");
  display.setFont(&FreeSans9pt7b);
  display.print(alt);
  display.print("m");

  // battery line:

  display.drawLine(127, 31, 127, get_y_pixel(bat), 1);
  display.drawPixel(126, get_y_pixel(4.0), 1);
  display.drawPixel(126, get_y_pixel(3.4), 1);
  display.drawPixel(126, get_y_pixel(3.6), 1);
  display.drawPixel(126, get_y_pixel(3.8), 1);
  display.drawPixel(126, get_y_pixel(4.2), 1);

  // logging line:
  if (logging) {
    display.drawLine(123, 31, 127, 31, 1);
  }

  display.display();

  if (logging) {
    if (can_write) {
      //Serial.println("writing");
      noInterrupts();
      File f = LittleFS.open("littlefsfile.txt", "a");
      if (h >= 99.9) { h = 99.9; }
      f.printf("%d %5.1f %3.1f %3.1f %4.2f\n", int((millis() + SLEEP_TIME_MS * powman_hw->scratch[0])/1000), p, h, t, alt);
      f.close();
      interrupts();
    }
#define SLEEP
#ifdef SLEEP
    // go to deep sleep using powman
    // Unlock the VREG control interface
    hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
    // Turn off USB PHY and apply pull downs on DP & DM
    disable_usb();
    // Initialise the example
    powman_example_init(1704067200000);
    // Scratch register survives power down
    //printf("Wake up, test run: %u\n", powman_hw->scratch[0]++);
    // power off
    int rc = powman_example_off_for_ms(SLEEP_TIME_MS);
    // we should not get here... instead we re-start the Pico 2
#else
    delay(SLEEP_TIME_MS);
#endif
  } else {
    // no logging, we want to read from USB drive the logfile
    delay(SLEEP_TIME_MS);
  }
}

// powman stuff

// Initialise everything
void powman_example_init(uint64_t abs_time_ms) {
  // start powman and set the time
  powman_timer_start();
  powman_timer_set_ms(abs_time_ms);

  // Allow power down when debugger connected
  powman_set_debug_power_request_ignored(true);

  // Power states
  powman_power_state P1_7 = POWMAN_POWER_STATE_NONE;

  powman_power_state P0_3 = POWMAN_POWER_STATE_NONE;
  P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
  P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_XIP_CACHE);

  off_state = P1_7;
  on_state = P0_3;
}

// Initiate power off
static int powman_example_off(void) {
  // Get ready to power off
  // stdio_flush(); TK change

  // Set power states
  bool valid_state = powman_configure_wakeup_state(off_state, on_state);
  if (!valid_state) {
    return PICO_ERROR_INVALID_STATE;
  }

  // reboot to main
  powman_hw->boot[0] = 0;
  powman_hw->boot[1] = 0;
  powman_hw->boot[2] = 0;
  powman_hw->boot[3] = 0;

  // Switch to required power state
  int rc = powman_set_power_state(off_state);
  if (rc != PICO_OK) {
    return rc;
  }

  // Power down
  while (true) __wfi();
}

// Power off until a gpio goes high
int powman_example_off_until_gpio_high(int gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, false);
  if (gpio_get(gpio)) {
    printf("Waiting for gpio %d to go low\n", gpio);
    while (gpio_get(gpio)) {
      sleep_ms(100);
    }
  }
  printf("Powering off until GPIO %d goes high\n", gpio);
  powman_enable_gpio_wakeup(0, gpio, false, true);
  return powman_example_off();
}

// Power off until an absolute time
int powman_example_off_until_time(uint64_t abs_time_ms) {
  // Start powman timer and turn off
  //printf("Powering off for %"PRIu64"ms\n", abs_time_ms - powman_timer_get_ms());
  powman_enable_alarm_wakeup_at_ms(abs_time_ms);
  return powman_example_off();
}

// Power off for a number of milliseconds
int powman_example_off_for_ms(uint64_t duration_ms) {
  uint64_t ms = powman_timer_get_ms();
  return powman_example_off_until_time(ms + duration_ms);
}

/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// the following is powman.c


#ifndef PICO_POWMAN_DEBUG
#define PICO_POWMAN_DEBUG 0
#endif

#if PICO_POWMAN_DEBUG
bool powman_debug_printf = false;
void powman_enable_debug_printf(void) {
  powman_debug_printf = true;
}
#define powman_debug(format, args...) \
  if (powman_debug_printf) printf(format, ##args)
#else
#define powman_debug(...)
#endif

static inline void powman_write(volatile uint32_t *reg, uint32_t value) {
  // Write needs a password in top 16 bits
  invalid_params_if(HARDWARE_POWMAN, value >> 16);
  *reg = POWMAN_PASSWORD_BITS | value;
}

void powman_timer_set_ms(uint64_t time_ms) {
  bool was_running = powman_timer_is_running();
  if (was_running) powman_timer_stop();
  powman_write(&powman_hw->set_time_15to0, time_ms & 0xffff);
  powman_write(&powman_hw->set_time_31to16, (time_ms >> 16) & 0xffff);
  powman_write(&powman_hw->set_time_47to32, (time_ms >> 32) & 0xffff);
  powman_write(&powman_hw->set_time_63to48, (time_ms >> 48) & 0xffff);
  if (was_running) powman_timer_start();
}

uint64_t powman_timer_get_ms(void) {
  // Need to make sure that the upper 32 bits of the timer
  // don't change, so read that first
  uint32_t hi = powman_hw->read_time_upper;
  uint32_t lo;
  do {
    // Read the lower 32 bits
    lo = powman_hw->read_time_lower;
    // Now read the upper 32 bits again and
    // check that it hasn't incremented. If it has loop around
    // and read the lower 32 bits again to get an accurate value
    uint32_t next_hi = powman_hw->read_time_upper;
    if (hi == next_hi) break;
    hi = next_hi;
  } while (true);
  return ((uint64_t)hi << 32u) | lo;
}

void powman_timer_set_1khz_tick_source_lposc(void) {
  powman_timer_set_1khz_tick_source_lposc_with_hz(32768);
}

void powman_timer_set_1khz_tick_source_lposc_with_hz(uint32_t lposc_freq_hz) {
  bool was_running = powman_timer_is_running();
  if (was_running) powman_timer_stop();
  uint32_t lposc_freq_khz = lposc_freq_hz / 1000;
  uint32_t lposc_freq_khz_frac16 = (lposc_freq_khz % 1000) * 65536 / 1000;
  powman_write(&powman_hw->lposc_freq_khz_int, lposc_freq_khz);
  powman_write(&powman_hw->lposc_freq_khz_frac, lposc_freq_khz_frac16);
  powman_set_bits(&powman_hw->timer, POWMAN_TIMER_USE_LPOSC_BITS);
  if (was_running) {
    powman_timer_start();
    while (!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS))
      ;
  }
}

void powman_timer_set_1khz_tick_source_xosc(void) {
  powman_timer_set_1khz_tick_source_xosc_with_hz(XOSC_HZ);
}

void powman_timer_set_1khz_tick_source_xosc_with_hz(uint32_t xosc_freq_hz) {
  bool was_running = powman_timer_is_running();
  if (was_running) powman_timer_stop();
  uint32_t xosc_freq_khz = xosc_freq_hz / 1000;
  uint32_t xosc_freq_khz_frac16 = (xosc_freq_khz % 1000) * 65536 / 1000;
  powman_write(&powman_hw->xosc_freq_khz_int, xosc_freq_khz);
  powman_write(&powman_hw->xosc_freq_khz_frac, xosc_freq_khz_frac16);
  powman_set_bits(&powman_hw->timer, POWMAN_TIMER_USE_XOSC_BITS);
  if (was_running) {
    powman_timer_start();
    while (!(powman_hw->timer & POWMAN_TIMER_USING_XOSC_BITS))
      ;
  }
}

static void powman_timer_use_gpio(uint32_t gpio, uint32_t use, uint32_t usinng) {
  bool was_running = powman_timer_is_running();
  if (was_running) powman_timer_stop();
  invalid_params_if(HARDWARE_POWMAN, !((gpio == 12) || (gpio == 14) || (gpio == 20) || (gpio == 22)));
  gpio_set_input_enabled(gpio, true);
  powman_write(&powman_hw->ext_time_ref, gpio);
  powman_set_bits(&powman_hw->timer, use);
  if (was_running) {
    powman_timer_start();
    while (!(powman_hw->timer & usinng))
      ;
  }
}

void powman_timer_set_1khz_tick_source_gpio(uint32_t gpio) {
  // todo check if we're using the GPIO setup already?
  powman_timer_use_gpio(gpio, POWMAN_TIMER_USE_GPIO_1KHZ_BITS, POWMAN_TIMER_USING_GPIO_1KHZ_BITS);
}

void powman_timer_enable_gpio_1hz_sync(uint32_t gpio) {
  // todo check if we're using the GPIO setup already?
  powman_timer_use_gpio(gpio, POWMAN_TIMER_USE_GPIO_1HZ_BITS, POWMAN_TIMER_USING_GPIO_1HZ_BITS);
}

void powman_timer_disable_gpio_1hz_sync(void) {
  powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_USE_GPIO_1HZ_BITS);
}

powman_power_state powman_get_power_state(void) {
  uint32_t state_reg = powman_hw->state & POWMAN_STATE_CURRENT_BITS;
  // todo we should have hardware/regs/powman.h values for these
  static_assert(POWMAN_POWER_DOMAIN_SRAM_BANK1 == 0, "");
  static_assert(POWMAN_POWER_DOMAIN_SRAM_BANK0 == 1, "");
  static_assert(POWMAN_POWER_DOMAIN_XIP_CACHE == 2, "");
  static_assert(POWMAN_POWER_DOMAIN_SWITCHED_CORE == 3, "");
  static_assert(POWMAN_STATE_CURRENT_BITS == 0xf, "");
  return (powman_power_state)state_reg;
}

// TODO: Should this fail to go to sleep if there is no wakeup alarm
int powman_set_power_state(powman_power_state state) {
  // Clear req ignored in case it has been set
  powman_clear_bits(&powman_hw->state, POWMAN_STATE_REQ_IGNORED_BITS);
  powman_debug("powman: Requesting state %x\n", state);
  powman_write(&powman_hw->state, (~state << POWMAN_STATE_REQ_LSB) & POWMAN_STATE_REQ_BITS);

  // Has it been ignored?
  if (powman_hw->state & POWMAN_STATE_REQ_IGNORED_BITS) {
    //powman_debug("State req ignored because of a pending pwrup req: %"PRIx32"\n", powman_hw->current_pwrup_req);
    powman_debug("State req ignored because of a pending pwrup req");  // TK change
    return PICO_ERROR_PRECONDITION_NOT_MET;
  }

  bool state_valid = (powman_hw->state & POWMAN_STATE_BAD_SW_REQ_BITS) == 0;
  if (!state_valid) {
    powman_debug("powman: Requested state invalid\n");
    return PICO_ERROR_INVALID_ARG;
  } else {
    powman_debug("powman: Requested state valid\n");
  }
  if (!powman_power_state_is_domain_on(state, POWMAN_POWER_DOMAIN_SWITCHED_CORE)) {
    // If we are turning off switched core then POWMAN_STATE_WAITING_BITS will be
    // set because we are waiting for proc to go to sleep, so return ok and then the proc
    // can go to sleep

    // Note if the powerdown is being blocked by a pending pwrup request we will break out of this and return a failure

    // Clk pow is slow so can take a few clk_pow cycles for waiting to turn up
    for (int i = 0; i < 100; i++) {
      if (powman_hw->state & POWMAN_STATE_WAITING_BITS) {
        return PICO_OK;
      }
    }

    // If it hasn't turned up then false
    powman_debug("powman: STATE_WAITING hasn't turned up\n");
    return PICO_ERROR_TIMEOUT;
  }
  // Wait while the state is changing then return true as we will be in the new state
  powman_debug("powman: waiting for state change\n");
  while (powman_hw->state & POWMAN_STATE_CHANGING_BITS) tight_loop_contents();
  powman_debug("powman: state changed to %x\n", state);
  return PICO_OK;
}

bool powman_configure_wakeup_state(powman_power_state sleep_state, powman_power_state wakeup_state) {
  // When powman wakes up it can keep the state of the sram0 and sram1 banks. Note, it can't
  // explicitly
  bool valid = powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_XIP_CACHE);
  valid &= powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
  valid &= powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK0) == powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_SRAM_BANK0);
  valid &= powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK1) == powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_SRAM_BANK1);
  if (valid) {
    powman_clear_bits(&powman_hw->seq_cfg, POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS | POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS);
    uint32_t seq_cfg_set = 0;
    if (!powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK0)) seq_cfg_set |= POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS;
    if (!powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK1)) seq_cfg_set |= POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS;
    powman_set_bits(&powman_hw->seq_cfg, seq_cfg_set);
  }
  return valid;
}

void powman_timer_enable_alarm_at_ms(uint64_t alarm_time_ms) {
  powman_set_bits(&powman_hw->inte, POWMAN_INTE_TIMER_BITS);
  powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
  // Alarm must be disabled to set the alarm time
  powman_write(&powman_hw->alarm_time_15to0, alarm_time_ms & 0xffff);
  powman_write(&powman_hw->alarm_time_31to16, (alarm_time_ms >> 16) & 0xffff);
  powman_write(&powman_hw->alarm_time_47to32, (alarm_time_ms >> 32) & 0xffff);
  powman_write(&powman_hw->alarm_time_63to48, (alarm_time_ms >> 48) & 0xffff);
  powman_clear_alarm();
  // TODO: Assuming pwrup on alarm has no bad side effects if already powered up
  powman_set_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
}

void powman_timer_disable_alarm(void) {
  powman_clear_bits(&powman_hw->inte, POWMAN_INTE_TIMER_BITS);
  powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
}

void powman_enable_alarm_wakeup_at_ms(uint64_t alarm_time_ms) {
  powman_timer_enable_alarm_at_ms(alarm_time_ms);
  powman_set_bits(&powman_hw->timer, POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
}

void powman_disable_alarm_wakeup(void) {
  powman_timer_disable_alarm();
  powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
}

void powman_enable_gpio_wakeup(uint gpio_wakeup_num, uint32_t gpio, bool edge, bool high) {
  invalid_params_if(HARDWARE_POWMAN, gpio_wakeup_num >= count_of(powman_hw->pwrup));

  // Need to make sure pad is input enabled
  gpio_set_input_enabled(gpio, true);

  // Set up gpio hardware for what we want
  uint32_t pwrup = (edge ? POWMAN_PWRUP0_MODE_VALUE_EDGE : POWMAN_PWRUP0_MODE_VALUE_LEVEL) << POWMAN_PWRUP0_MODE_LSB;
  pwrup |= (high ? POWMAN_PWRUP0_DIRECTION_BITS : 0);
  pwrup |= gpio << POWMAN_PWRUP0_SOURCE_LSB;
  powman_write(&powman_hw->pwrup[gpio_wakeup_num], pwrup);

  // Clear the status bit in case an edge is already latched
  powman_clear_bits(&powman_hw->pwrup[gpio_wakeup_num], POWMAN_PWRUP0_STATUS_BITS);

  // Important to enable it separately to allow the gpio to change
  powman_set_bits(&powman_hw->pwrup[gpio_wakeup_num], POWMAN_PWRUP0_ENABLE_BITS);
}

void powman_disable_gpio_wakeup(uint gpio_wakeup_num) {
  invalid_params_if(HARDWARE_POWMAN, gpio_wakeup_num >= count_of(powman_hw->pwrup));
  powman_clear_bits(&powman_hw->pwrup[gpio_wakeup_num], POWMAN_PWRUP0_ENABLE_BITS);
}

void powman_disable_all_wakeups(void) {
  for (uint i = 0; i < count_of(powman_hw->pwrup); i++) {
    powman_disable_gpio_wakeup(i);
  }
  powman_disable_alarm_wakeup();
}
