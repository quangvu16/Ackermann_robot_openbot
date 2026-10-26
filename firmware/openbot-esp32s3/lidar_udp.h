// lidar_udp.h - "LDS1" scan datagrams for ROS 2.
//
// One datagram per finished revolution, the same wire format as
// ldrobot_ld14p_esp32_mpu9250_ros/lidar_ld14.h and the kaiaai firmware's
// scan_udp.h, so robot_ws's udp_lidar_receiver_node can publish all of them on
// /scan without knowing which robot sent them:
//
//   offset  size          field
//        0     4          "LDS1"
//        4     2          bin count           uint16 little-endian
//        6     4          scan id             uint32 little-endian
//       10     4          reserved, zero      (the receiver skips it)
//       14  2*bins        ranges, millimetres uint16 little-endian each
//
// 0 mm means "no return"; the receiver publishes +inf for that bin.
//
// Bins go out in the sensor's own clockwise order and are never reordered.
// The receiver does the ROS-side mirroring (flip_lr, default true) and the
// mounting rotation (angle0_deg, default 180). Fix a mounting offset there or
// with LIDAR_FORWARD_DEG in the settings, never by shifting bins here - the
// ranges array must not be reordered on either side or the two corrections
// fight.
//
// WiFi here is deliberately non-blocking and entirely optional. The car is
// driven over BLE; WiFi is only the side channel that carries scans to ROS.
// If the router is down, or LIDAR_UDP is 0, the car drives exactly as before
// and simply sends no scans. Nothing in the control path waits on the network.

#pragma once

#include <Arduino.h>

#if (LIDAR_UDP)

#include <WiFi.h>
#include <WiFiUdp.h>

static const uint16_t UDP_HEADER_LEN = 14;
static const uint16_t UDP_BINS_MAX   = 720;   // 14 + 720*2 fits one MTU

static_assert(SCAN_BINS >= 1 && SCAN_BINS <= UDP_BINS_MAX,
  "SCAN_BINS must fit one UDP datagram: 14 + 2*SCAN_BINS bytes, and the "
  "receiver reads at most 4096 bytes per recvfrom()");

static WiFiUDP   udp;
static uint8_t   udp_buf[UDP_HEADER_LEN + UDP_BINS_MAX * 2];
static uint32_t  scan_id = 0;

static bool      dest_valid = false;
static IPAddress dest_ip;

static uint32_t  sent_count = 0;
static uint32_t  sent_prev  = 0;
static uint32_t  fail_count = 0;

static bool     wifi_was_connected = false;
static uint32_t wifi_retry_ms = 0;

// Fire and forget: WiFi.begin() is called once and the status is polled. A
// wrong SSID or a router still booting never wedges the sketch - the car keeps
// taking BLE commands throughout and simply sends no scans until the link is up.
static void wifiBegin() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // power save adds ~100 ms of latency to every packet
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifi_retry_ms = millis();
}

static void wifiLoop() {
  const bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wifi_was_connected) {
    Serial.printf("WiFi up as %s\n", WiFi.localIP().toString().c_str());
    udp.begin(0);   // any local port; the receiver never replies
    sent_count = 0;
    sent_prev  = 0;
    fail_count = 0;
  } else if (!connected && wifi_was_connected) {
    Serial.println("WiFi lost, reconnecting");
  }
  wifi_was_connected = connected;

  if (!connected && millis() - wifi_retry_ms >= WIFI_RETRY_MS) {
    Serial.printf("WiFi \"%s\" still down, retrying\n", WIFI_SSID);
    wifiBegin();
  }
}

// Take the newest finished revolution, if there is one, and send it. Never
// blocks the control loop for longer than the mutex hold.
static void lidarSendPendingScan() {
  if (!shared_scan_ready || !wifi_was_connected || !dest_valid)
    return;
  if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    return;

  float    freq         = shared_scan.freq_hz;
  uint16_t points       = shared_scan.points;
  uint16_t valid_points = shared_scan.valid_points;

  udp_buf[0] = 'L';
  udp_buf[1] = 'D';
  udp_buf[2] = 'S';
  udp_buf[3] = '1';
  udp_buf[4] = (uint8_t)(SCAN_BINS);
  udp_buf[5] = (uint8_t)(SCAN_BINS >> 8);
  udp_buf[6] = (uint8_t)(scan_id);
  udp_buf[7] = (uint8_t)(scan_id >> 8);
  udp_buf[8] = (uint8_t)(scan_id >> 16);
  udp_buf[9] = (uint8_t)(scan_id >> 24);
  udp_buf[10] = 0;    // reserved, the receiver skips bytes 10..13
  udp_buf[11] = 0;
  udp_buf[12] = 0;
  udp_buf[13] = 0;

  for (uint16_t i = 0; i < SCAN_BINS; i++) {
    // An empty bin means "nothing measured here"; the receiver spells that
    // +inf. Bins go out in the sensor's own clockwise order, never reversed.
    uint16_t mm = shared_scan.range_mm[i];
    udp_buf[UDP_HEADER_LEN + i * 2]     = (uint8_t)(mm);
    udp_buf[UDP_HEADER_LEN + i * 2 + 1] = (uint8_t)(mm >> 8);
  }
  shared_scan_ready = false;
  xSemaphoreGive(scan_mutex);

  const size_t len = UDP_HEADER_LEN + (size_t)SCAN_BINS * 2;
  bool ok = false;
  if (udp.beginPacket(dest_ip, (uint16_t)LIDAR_UDP_PORT) == 1) {
    udp.write(udp_buf, len);
    ok = udp.endPacket() == 1;
  }

  scan_id++;
  if (ok) {
    sent_count++;
  } else {
    fail_count++;
    // UDP is fire-and-forget, so a failure here is local: no route, or the
    // WiFi stack is out of buffers. Say so instead of going quiet - a silently
    // dropped scan looks exactly like a dead LiDAR from the ROS side.
    static uint32_t last_fail_ms = 0;
    if (millis() - last_fail_ms >= 2000) {
      last_fail_ms = millis();
      Serial.printf("UDP send to %s:%d failed (%lu total)\n",
                    LIDAR_UDP_DEST_IP, LIDAR_UDP_PORT, (unsigned long)fail_count);
    }
  }

  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms >= 1000) {
    uint32_t scans = sent_count - sent_prev;
    sent_prev = sent_count;
    last_log_ms = now;
    Serial.printf("scan %5.2f Hz  %3u pts (%3u valid)  %lu pkt  %lu crc err  "
                  "%lu scan/s sent  %lu err  rssi %d\n",
                  freq, (unsigned)points, (unsigned)valid_points,
                  (unsigned long)lidar.getPacketCount(),
                  (unsigned long)lidar.getCrcErrorCount(),
                  (unsigned long)scans, (unsigned long)fail_count,
                  (int)WiFi.RSSI());
  }
}

static void lidarUdpBegin() {
  dest_valid = dest_ip.fromString(LIDAR_UDP_DEST_IP);
  if (!dest_valid)
    Serial.println("LIDAR_UDP_DEST_IP is not a valid IPv4 address; no scans will be sent");

  Serial.printf("LiDAR scans -> UDP %s:%d (%u bins, %u B each)\n",
                LIDAR_UDP_DEST_IP, LIDAR_UDP_PORT,
                (unsigned)SCAN_BINS, (unsigned)(UDP_HEADER_LEN + SCAN_BINS * 2));
  wifiBegin();
}

#else  // !LIDAR_UDP

static void wifiLoop() {}
static void lidarSendPendingScan() {}
static void lidarUdpBegin() {}

#endif  // LIDAR_UDP
