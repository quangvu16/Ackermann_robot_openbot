// lidar_serial.h - LiDAR scans down the USB cable to the phone.
//
// The UDP side channel (lidar_udp.h) needs WiFi, and it sends to a single
// address. When the phone is plugged into the car with a USB cable, the same
// "LDS1" revolution can ride the serial link instead, so the app's LiDAR plan
// view works with no WiFi at all.
//
// Wire format, one line per revolution:
//
//   x<base64 of one "LDS1" datagram>\n
//
// The datagram is byte-for-byte the UDP format (see lidar_udp.h): 14-byte
// header then little-endian uint16 ranges in mm, 0 = no return, bins in the
// sensor's clockwise order. The app decodes it with the parser it already uses
// for UDP, so both paths share one set of checks. Base64 keeps the line plain
// text for the OpenBot line protocol: no '\n' can appear inside it.
//
// Off until asked. The app sends "x<ms>" (minimum interval between scans, 0 =
// off) once it sees the 'x' feature flag on a USB link. A serial monitor or the
// Linux robot never asks, so they never see these lines. The interval resets
// to off on every boot.
//
// SERIAL_SCAN_STEP folds that many neighbouring bins into one (nearest return
// wins) before sending. With the default 360 bins and step 2 a line is 180
// bins = 374 bytes = 500 base64 characters; at 5 scans/s that is about 2.5 kB/s,
// about a fifth of a 115200 baud UART. On the native USB port the baud rate is
// ignored and step 1 (full resolution) is fine.
//
// Never blocks: a line is only written when the whole thing fits in the
// serial transmit buffer, otherwise that revolution is skipped. A late scan is
// worthless; a stalled control loop is dangerous.

#pragma once

#include <Arduino.h>

#if (LIDAR_SERIAL_SCAN)

static_assert(SERIAL_SCAN_STEP >= 1 && SERIAL_SCAN_STEP <= 8,
              "SERIAL_SCAN_STEP must be 1..8");

static const uint16_t SERIAL_SCAN_BINS =
    (SCAN_BINS + SERIAL_SCAN_STEP - 1) / SERIAL_SCAN_STEP;
static const size_t SERIAL_SCAN_RAW_LEN = 14 + 2 * (size_t)SERIAL_SCAN_BINS;
static const size_t SERIAL_SCAN_B64_LEN = 4 * ((SERIAL_SCAN_RAW_LEN + 2) / 3);
// 'x' + base64 + '\n'
static const size_t SERIAL_SCAN_LINE_LEN = 1 + SERIAL_SCAN_B64_LEN + 1;

// Serial TX buffer asked for in setup(). Big enough for one scan line plus the
// short 's' and 'p' reports that share the link.
static const size_t SERIAL_TX_BUFFER = SERIAL_SCAN_LINE_LEN + 256;

static uint32_t serial_scan_interval_ms = 0;  // 0 = off until the app asks
static uint32_t serial_scan_last_ms = 0;
static uint32_t serial_scan_last_seq = 0;
static uint32_t serial_scan_id = 0;

static uint8_t serial_scan_raw[SERIAL_SCAN_RAW_LEN];
static char    serial_scan_line[SERIAL_SCAN_LINE_LEN];

static size_t base64Encode(const uint8_t *in, size_t len, char *out) {
  static const char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t o = 0;
  size_t i = 0;
  for (; i + 2 < len; i += 3) {
    uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    out[o++] = tbl[(v >> 6) & 63];
    out[o++] = tbl[v & 63];
  }
  if (i < len) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < len)
      v |= (uint32_t)in[i + 1] << 8;
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    out[o++] = (i + 1 < len) ? tbl[(v >> 6) & 63] : '=';
    out[o++] = '=';
  }
  return o;
}

// Called from setup() BEFORE Serial.begin(): the transmit buffer can only be
// resized while the port is closed. Without this the UART keeps its 128-byte
// hardware FIFO and a scan line would never fit.
static void lidarSerialPrepare() {
#if ARDUINO_USB_CDC_ON_BOOT && !ARDUINO_USB_MODE
  // "USB Mode: USB-OTG (TinyUSB)" with CDC on boot: Serial is a TinyUSB CDC
  // port with its own FIFO and no resize call. Scans still go out whenever a
  // whole line fits; on this port that may be rarely. Prefer the default
  // "Hardware CDC and JTAG" mode, or the COM/UART port.
#else
  Serial.setTxBufferSize(SERIAL_TX_BUFFER);
#endif
}

// "x<ms>": minimum interval between scan lines, 0 turns them off. Faster than
// 50 ms is clamped - the LD14P tops out near 11 revolutions a second anyway.
static void process_scan_request(const char *body) {
  long ms = atol(body);
  if (ms <= 0) {
    serial_scan_interval_ms = 0;
    return;
  }
  serial_scan_interval_ms = (ms < 50) ? 50 : (uint32_t)ms;
  serial_scan_last_ms = 0;
}

static void lidarSendSerialScan() {
  if (serial_scan_interval_ms == 0)
    return;
  uint32_t now = millis();
  if (now - serial_scan_last_ms < serial_scan_interval_ms)
    return;
  if (shared_scan_seq == serial_scan_last_seq)
    return;  // nothing new since the last line
  if ((size_t)Serial.availableForWrite() < SERIAL_SCAN_LINE_LEN)
    return;  // link busy: skip this revolution rather than block
  if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(2)) != pdTRUE)
    return;

  // Header, identical to the UDP datagram.
  serial_scan_raw[0] = 'L';
  serial_scan_raw[1] = 'D';
  serial_scan_raw[2] = 'S';
  serial_scan_raw[3] = '1';
  serial_scan_raw[4] = (uint8_t)(SERIAL_SCAN_BINS);
  serial_scan_raw[5] = (uint8_t)(SERIAL_SCAN_BINS >> 8);
  serial_scan_raw[6] = (uint8_t)(serial_scan_id);
  serial_scan_raw[7] = (uint8_t)(serial_scan_id >> 8);
  serial_scan_raw[8] = (uint8_t)(serial_scan_id >> 16);
  serial_scan_raw[9] = (uint8_t)(serial_scan_id >> 24);
  serial_scan_raw[10] = 0;
  serial_scan_raw[11] = 0;
  serial_scan_raw[12] = 0;
  serial_scan_raw[13] = 0;

  // Fold SERIAL_SCAN_STEP bins into one, nearest return wins. Output bin j
  // then covers input bins j*step .. j*step+step-1, so its centre is still at
  // (j + 0.5) * 360 / SERIAL_SCAN_BINS - the convention every receiver uses.
  for (uint16_t j = 0; j < SERIAL_SCAN_BINS; j++) {
    uint16_t best = 0;
    for (uint16_t k = 0; k < SERIAL_SCAN_STEP; k++) {
      uint16_t bin = j * SERIAL_SCAN_STEP + k;
      if (bin >= SCAN_BINS)
        break;
      uint16_t mm = shared_scan.range_mm[bin];
      if (mm != 0 && (best == 0 || mm < best))
        best = mm;
    }
    serial_scan_raw[14 + j * 2]     = (uint8_t)(best);
    serial_scan_raw[14 + j * 2 + 1] = (uint8_t)(best >> 8);
  }
  serial_scan_last_seq = shared_scan_seq;
  xSemaphoreGive(scan_mutex);

  serial_scan_line[0] = 'x';
  size_t n = base64Encode(serial_scan_raw, SERIAL_SCAN_RAW_LEN, serial_scan_line + 1);
  serial_scan_line[1 + n] = '\n';
  Serial.write((const uint8_t *)serial_scan_line, n + 2);

  serial_scan_last_ms = now;
  serial_scan_id++;
}

#else  // !LIDAR_SERIAL_SCAN

static void lidarSerialPrepare() {}
static void process_scan_request(const char *body) { (void)body; }
static void lidarSendSerialScan() {}

#endif  // LIDAR_SERIAL_SCAN
