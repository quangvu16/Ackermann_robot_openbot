// rc_input.h - RC receiver input and the MANUAL / AUTO / HOLD mode switch.
//
// Made for a FlySky FS-i6 transmitter with either receiver:
//
//   RC_IBUS  FS-iA6B: one signal wire from its "SERVO" iBUS port (not the
//            "SENS" port). 14 channels at 115200 baud, a frame every ~7 ms.
//   RC_PWM   FS-iA6 (no iBUS): one wire per channel - steering, throttle
//            and the mode switch.
//
// A switch on the transmitter (by default SwC on CH5) picks who drives:
//
//   MANUAL  the RC sticks drive the car directly; app drive commands are
//           ignored (lights and everything else still work)
//   AUTO    the phone / computer drives, exactly as without a receiver
//   HOLD    throttle neutral, wheels straight
//
// If no receiver has been heard since power-on the car behaves as if none
// were fitted (AUTO, the app drives). Once one has been heard, losing it for
// RC_TIMEOUT_MS is a failsafe: the car holds (or, with RC_LOST_ACTION 1,
// keeps driving if it was in AUTO). Also set the transmitter's own failsafe
// (see docs/FIRMWARE_GUIDE.md), because a FlySky receiver may keep sending
// the last stick positions after the link drops.
//
// The pure logic (iBUS decoding, stick mapping, mode choice) comes first and
// uses no Arduino calls, so it can be tested on a computer with
// -DRC_HOST_TEST. The hardware glue follows.
#pragma once
#include <stdint.h>
#include <stdlib.h>

enum RcMode : uint8_t { RC_NOT_FITTED, RC_MANUAL, RC_AUTO, RC_HOLD, RC_LOST };

static inline const char *rcModeName(RcMode m) {
  switch (m) {
    case RC_MANUAL: return "MANUAL";
    case RC_AUTO: return "AUTO";
    case RC_HOLD: return "HOLD";
    case RC_LOST: return "LOST (failsafe)";
    default: return "no receiver";
  }
}

// ------------------------------------------------------------ pure logic

static const int RC_IBUS_CHANNELS = 14;
static const uint16_t RC_US_VALID_MIN = 800;   // a FlySky channel is 1000-2000
static const uint16_t RC_US_VALID_MAX = 2200;  // outside this is not a pulse

// iBUS frame: 0x20 0x40, 14 channels as uint16 little-endian (us), then a
// uint16 little-endian checksum = 0xFFFF minus the sum of the first 30 bytes.
struct IbusParser {
  uint8_t buf[32];
  uint8_t idx = 0;

  // Feeds one byte. Returns true when a complete frame with a good checksum
  // has been decoded into ch[0..13] (ch[0] = CH1).
  bool feed(uint8_t b, uint16_t ch[RC_IBUS_CHANNELS]) {
    if (idx == 0) {
      if (b == 0x20) buf[idx++] = b;
      return false;
    }
    if (idx == 1) {
      if (b == 0x40) {
        buf[idx++] = b;
      } else {
        idx = (b == 0x20) ? 1 : 0;  // a new start byte resynchronises
      }
      return false;
    }
    buf[idx++] = b;
    if (idx < 32) return false;
    idx = 0;
    uint16_t sum = 0xFFFF;
    for (int i = 0; i < 30; i++) sum -= buf[i];
    uint16_t rx = (uint16_t)(buf[30] | (buf[31] << 8));
    if (sum != rx) return false;
    for (int c = 0; c < RC_IBUS_CHANNELS; c++) {
      ch[c] = (uint16_t)(buf[2 + 2 * c] | (buf[3 + 2 * c] << 8));
    }
    return true;
  }
};

static inline bool rcUsValid(int us) {
  return us >= RC_US_VALID_MIN && us <= RC_US_VALID_MAX;
}

// Maps a stick pulse onto the firmware's -510..510 drive range. Inside the
// deadband around 1500 us the result is 0, so a centred stick really stops the
// car; outside it the range is stretched so full stick is still full drive.
static inline int rcUsToDrive(int us, int deadband_us) {
  int d = us - 1500;
  if (abs(d) <= deadband_us) return 0;
  d = (d > 0) ? d - deadband_us : d + deadband_us;
  long v = (long)d * 510 / (500 - deadband_us);
  if (v > 510) v = 510;
  if (v < -510) v = -510;
  return (int)v;
}

// Which position the mode switch is in. A 3-position switch gives about
// 1000 / 1500 / 2000 us, a 2-position switch 1000 / 2000 us.
static inline RcMode rcModeFromUs(int us, RcMode low, RcMode mid, RcMode high) {
  if (us < 1250) return low;
  if (us > 1750) return high;
  return mid;
}

// Puts it together. seen: a valid frame has arrived since power-on. fresh: one
// arrived within RC_TIMEOUT_MS. switch_mode: what the switch said in the last
// valid frame. lost_keeps_auto: RC_LOST_ACTION 1.
static inline RcMode rcDecideMode(bool seen, bool fresh, RcMode switch_mode,
                                  bool lost_keeps_auto) {
  if (!seen) return RC_NOT_FITTED;
  if (!fresh) return (lost_keeps_auto && switch_mode == RC_AUTO) ? RC_AUTO : RC_LOST;
  return switch_mode;
}

// ------------------------------------------------------------ hardware glue
#ifndef RC_HOST_TEST
#if (HAS_RC)
#include <Arduino.h>
#include "soc/gpio_reg.h"

// Refuse pins that would brick the N16R8 or steal the phone link (see the
// SETTINGS pin notes): 26-37 flash/PSRAM, 0/3/45/46 strapping, 19/20 USB,
// 43/44 UART0.
#define RC_PIN_RESERVED(p) \
  (((p) >= 26 && (p) <= 37) || (p) == 0 || (p) == 3 || (p) == 45 || (p) == 46 || \
   (p) == 19 || (p) == 20 || (p) == 43 || (p) == 44)

static uint16_t rc_steer_us = 1500;
static uint16_t rc_throttle_us = 1500;
static uint16_t rc_mode_us = 1500;
static unsigned long rc_last_valid_ms = 0;
static bool rc_seen = false;

#if (RC_PROTOCOL == RC_IBUS)
#if RC_PIN_RESERVED(RC_IBUS_PIN)
#error "RC_IBUS_PIN is a reserved pin on the ESP32-S3 N16R8 - pick a free GPIO such as 14"
#endif
// UART0 is the USB serial link and UART1 the LiDAR, so iBUS gets UART2.
HardwareSerial RcSerial(2);
static IbusParser rc_parser;

static void rcBegin() {
  // Room for ~400 ms of frames, so a slow loop() pass (the BLE reconnect
  // path waits 500 ms) does not drop the link and trip the failsafe.
  RcSerial.setRxBufferSize(2048);
  RcSerial.begin(115200, SERIAL_8N1, RC_IBUS_PIN, -1);
  Serial.printf("RC     iBUS on GPIO%d: CH%d steering, CH%d throttle, CH%d mode switch\n",
                RC_IBUS_PIN, RC_CH_STEER, RC_CH_THROTTLE, RC_CH_MODE);
}

static void rcPoll() {
  uint16_t ch[RC_IBUS_CHANNELS];
  while (RcSerial.available() > 0) {
    if (!rc_parser.feed((uint8_t)RcSerial.read(), ch)) continue;
    uint16_t s = ch[RC_CH_STEER - 1], t = ch[RC_CH_THROTTLE - 1], m = ch[RC_CH_MODE - 1];
    if (!rcUsValid(s) || !rcUsValid(t) || !rcUsValid(m)) continue;
    rc_steer_us = s;
    rc_throttle_us = t;
    rc_mode_us = m;
    rc_last_valid_ms = millis();
    rc_seen = true;
  }
}

#elif (RC_PROTOCOL == RC_PWM)
#if RC_PIN_RESERVED(RC_PWM_PIN_STEER) || RC_PIN_RESERVED(RC_PWM_PIN_THROTTLE) || \
    RC_PIN_RESERVED(RC_PWM_PIN_MODE)
#error "An RC_PWM_PIN_* is a reserved pin on the ESP32-S3 N16R8 - pick free GPIOs such as 14, 15, 16"
#endif
#if (RC_PWM_PIN_STEER > 31) || (RC_PWM_PIN_THROTTLE > 31) || (RC_PWM_PIN_MODE > 31)
#error "RC_PWM pins must be GPIO 0-31 (the interrupt reads GPIO_IN_REG)"
#endif

// One pulse-width timer per channel, run from a pin-change interrupt. The ISR
// reads the pin straight from the GPIO register so it stays in IRAM and is
// safe while flash is busy (WiFi writes to flash).
struct RcPwmIn {
  uint8_t pin;
  volatile uint32_t rise_us;
  volatile uint16_t width_us;
  volatile uint32_t last_ms;
};
static RcPwmIn rc_pwm[3] = {
  { RC_PWM_PIN_STEER, 0, 1500, 0 },
  { RC_PWM_PIN_THROTTLE, 0, 1500, 0 },
  { RC_PWM_PIN_MODE, 0, 1500, 0 },
};

static void IRAM_ATTR rcPwmIsr(void *arg) {
  RcPwmIn *p = (RcPwmIn *)arg;
  uint32_t now = micros();
  if ((REG_READ(GPIO_IN_REG) >> p->pin) & 1) {
    p->rise_us = now;
  } else {
    uint32_t w = now - p->rise_us;
    if (w >= RC_US_VALID_MIN && w <= RC_US_VALID_MAX) {
      p->width_us = (uint16_t)w;
      p->last_ms = millis();
    }
  }
}

static void rcBegin() {
  for (int i = 0; i < 3; i++) {
    pinMode(rc_pwm[i].pin, INPUT_PULLDOWN);  // an unplugged wire reads as no pulse
    attachInterruptArg(rc_pwm[i].pin, rcPwmIsr, &rc_pwm[i], CHANGE);
  }
  Serial.printf("RC     PWM: steering GPIO%d, throttle GPIO%d, mode switch GPIO%d\n",
                RC_PWM_PIN_STEER, RC_PWM_PIN_THROTTLE, RC_PWM_PIN_MODE);
}

static void rcPoll() {
  unsigned long now = millis();
  // Valid only when all three channels pulsed recently: a missing steering or
  // throttle wire must never leave a stale command driving the car.
  for (int i = 0; i < 3; i++) {
    uint32_t last = rc_pwm[i].last_ms;
    if (last == 0 || now - last > RC_TIMEOUT_MS) return;
  }
  rc_steer_us = rc_pwm[0].width_us;
  rc_throttle_us = rc_pwm[1].width_us;
  rc_mode_us = rc_pwm[2].width_us;
  rc_last_valid_ms = now;
  rc_seen = true;
}
#else
#error "RC_PROTOCOL must be RC_IBUS or RC_PWM"
#endif

static RcMode rcMode() {
  bool fresh = rc_seen && (millis() - rc_last_valid_ms) <= RC_TIMEOUT_MS;
  RcMode sw = rcModeFromUs(rc_mode_us, RC_MODE_LOW, RC_MODE_MID, RC_MODE_HIGH);
  return rcDecideMode(rc_seen, fresh, sw, RC_LOST_ACTION == 1);
}

static int rcSteerUs() { return rc_steer_us; }
static int rcThrottleUs() { return rc_throttle_us; }

#else  // !HAS_RC
static void rcBegin() {}
static void rcPoll() {}
static RcMode rcMode() { return RC_NOT_FITTED; }
static int rcSteerUs() { return 1500; }
static int rcThrottleUs() { return 1500; }
#endif  // HAS_RC
#endif  // RC_HOST_TEST
