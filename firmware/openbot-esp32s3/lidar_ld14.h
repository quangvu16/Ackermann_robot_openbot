// lidar_ld14.h - LDROBOT LD14/LD14P decoder and scan accumulator.
//
// Ported from ldrobot_ld14p_esp32_mpu9250_ros/lidar_ld14.h, which is itself
// ported from LDROBOT's official SDK (MIT licence). What changed here:
//
//   - the UDP "LDS1" sender moved to lidar_udp.h, so this header has no WiFi
//     dependency and the car can run BLE-only
//   - the accumulator also tracks the nearest return in a forward cone, which
//     is what the OpenBot sonar channel and stop_at_obstacle() consume
//   - diagnostics are rewritten for the ESP32-S3 pin map
//
// Decode, CRC8, angle interpolation and the LD14/LD14P ranging-centre
// correction are unchanged. Do not "fix" the angle convention:
//
//   *** Bin i is the measurement i*(360/bins) degrees CLOCKWISE from the
//   LiDAR's own zero mark. *** That is the raw convention the LDROBOT hardware
//   reports in. A mounting offset is corrected with LIDAR_FORWARD_DEG in the
//   settings block, never by shifting bins here.
//
// The SDK's ranging-centre correction is applied per point because it depends
// on the measured distance and cannot be undone downstream.
//
// Wiring (ESP32-S3 N16R8):
//   LiDAR TX  -> LIDAR_RX_PIN           3.3 V logic, no level shifter needed
//   LiDAR RX  -> unwired                the LD14 free-runs; LIDAR_TX_PIN -1
//   LiDAR GND -> GND                    must share ground with the ESP32
//   LiDAR VCC -> 5V                     ~200 mA; power it from the robot's 5 V
//                                       rail, not the ESP32's 3V3 pin
//
// The reader runs as its own FreeRTOS task so the UART is drained even while
// loop() is busy with BLE or WiFi. It and the sender exchange one finished
// revolution at a time through a mutex-protected snapshot.

#pragma once

#include <Arduino.h>
#include <math.h>
#include <string.h>

// The LiDAR is the only user of UART1. UART0 is the USB Serial monitor.
HardwareSerial LidarSerial(1);

// ------------------------------------------------------------- on-wire packets
static const uint8_t PKG_HEADER        = 0x54;
static const uint8_t DATA_PKG_INFO     = 0x2C;
static const uint8_t HEALTH_PKG_INFO   = 0xE0;
static const uint8_t MANUFACT_PKG_INFO = 0x0F;
static const int     POINT_PER_PACK    = 12;

#pragma pack(push, 1)
struct LidarPoint {
  uint16_t distance;      // mm, 0 = no return
  uint8_t  intensity;     // 0..255
};

struct MeasurePacket {    // 47 bytes
  uint8_t    header;      // 0x54
  uint8_t    ver_len;     // 0x2C = 12 points per packet
  uint16_t   speed;       // deg/s
  uint16_t   start_angle; // 0.01 deg
  LidarPoint point[POINT_PER_PACK];
  uint16_t   end_angle;   // 0.01 deg
  uint16_t   timestamp;   // ms, wraps
  uint8_t    crc8;
};

struct HealthPacket {     // 4 bytes
  uint8_t header;
  uint8_t information;    // 0xE0
  uint8_t error_code;
  uint8_t crc8;
};

struct ManufacturePacket {  // 23 bytes
  uint8_t  header;
  uint8_t  information;     // 0x0F
  uint16_t speed;
  uint16_t product_version;
  uint32_t sn_high;
  uint32_t sn_low;
  uint32_t hardware_version;
  uint32_t firmware_version;
  uint8_t  crc8;
};
#pragma pack(pop)

static_assert(sizeof(MeasurePacket) == 47, "MeasurePacket must not be padded");
static_assert(sizeof(HealthPacket) == 4, "HealthPacket must not be padded");
static_assert(sizeof(ManufacturePacket) == 23, "ManufacturePacket must not be padded");

// ------------------------------------------------------------------------ CRC8
static const uint8_t kCrcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
  0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
  0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
  0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
  0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
  0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
  0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
  0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
  0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
  0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
  0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
  0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
  0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
  0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
  0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
  0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
  0x7f, 0x32, 0xe5, 0xa8
};

static uint8_t calcCRC8(const uint8_t * data, uint16_t len) {
  uint8_t crc = 0;
  while (len--)
    crc = kCrcTable[(crc ^ *data++) & 0xff];
  return crc;
}

// ----------------------------------------------------------------- the driver
class LD14P {
public:
  // A measurement point. angle_deg is the ranging-centre-corrected angle,
  // CLOCKWISE from the LiDAR's zero mark. scan_completed marks the first point
  // of a new revolution.
  typedef void (*ScanPointCallback)(float angle_deg, uint16_t distance_mm,
                                    uint8_t intensity, bool scan_completed);
  typedef void (*HealthCallback)(uint8_t error_code);
  typedef void (*DeviceInfoCallback)(const ManufacturePacket & info);

  void setScanPointCallback(ScanPointCallback cb)   { scan_point_cb_ = cb; }
  void setHealthCallback(HealthCallback cb)         { health_cb_ = cb; }
  void setDeviceInfoCallback(DeviceInfoCallback cb) { device_info_cb_ = cb; }

  // Feed the driver everything the UART has. Call this often.
  void poll(Stream & in) {
    while (in.available() > 0) {
      int b = in.read();
      if (b < 0)
        break;
      raw_byte_count_++;
      if (sample_len_ < SAMPLE_LEN)
        sample_[sample_len_++] = (uint8_t)b;
      feed((uint8_t)b);
    }
  }

  // Throw away everything learned so far, e.g. after changing the baud rate.
  void resetStats() {
    reset();
    raw_byte_count_ = 0;
    sample_len_ = 0;
    packet_count_ = 0;
    crc_error_count_ = 0;
    last_packet_ms_ = 0;
  }

  float    getScanFreqHz() const    { return speed_ / 360.0f; }  // revolutions/s
  uint16_t getSpeedDegPerS() const  { return speed_; }
  uint32_t getPacketCount() const   { return packet_count_; }
  uint32_t getCrcErrorCount() const { return crc_error_count_; }
  uint32_t getLastPacketMs() const  { return last_packet_ms_; }
  uint32_t getRawByteCount() const  { return raw_byte_count_; }
  const uint8_t * getSample() const { return sample_; }
  uint8_t  getSampleLen() const     { return sample_len_; }

private:
  // Ranging-centre offsets for LD14/LD14P, in mm (sl_transform.cpp).
  static constexpr float OFFSET_X = 5.9f;
  static constexpr float OFFSET_Y = -18.975571f;

  enum State { HEADER, VER_LEN, DATA, DATA_HEALTH, DATA_MANUFACTURE };

  void feed(uint8_t byte) {
    switch (state_) {
      case HEADER:
        if (byte == PKG_HEADER) {
          buf_[count_++] = byte;
          state_ = VER_LEN;
        }
        break;

      case VER_LEN:
        buf_[count_++] = byte;
        if (byte == DATA_PKG_INFO)
          state_ = DATA;
        else if (byte == HEALTH_PKG_INFO)
          state_ = DATA_HEALTH;
        else if (byte == MANUFACT_PKG_INFO)
          state_ = DATA_MANUFACTURE;
        else
          reset();  // that 0x54 was payload, not a header
        break;

      case DATA:
        buf_[count_++] = byte;
        if (count_ >= sizeof(MeasurePacket)) {
          if (checkCRC(sizeof(MeasurePacket)))
            handleMeasurePacket(*(const MeasurePacket *)buf_);
          reset();
        }
        break;

      case DATA_HEALTH:
        buf_[count_++] = byte;
        if (count_ >= sizeof(HealthPacket)) {
          if (checkCRC(sizeof(HealthPacket)) && health_cb_ != nullptr)
            health_cb_(((const HealthPacket *)buf_)->error_code);
          reset();
        }
        break;

      case DATA_MANUFACTURE:
        buf_[count_++] = byte;
        if (count_ >= sizeof(ManufacturePacket)) {
          if (checkCRC(sizeof(ManufacturePacket)) && device_info_cb_ != nullptr)
            device_info_cb_(*(const ManufacturePacket *)buf_);
          reset();
        }
        break;
    }
  }

  bool checkCRC(size_t len) {
    bool ok = calcCRC8(buf_, len - 1) == buf_[len - 1];
    if (!ok)
      crc_error_count_++;
    return ok;
  }

  void reset() {
    state_ = HEADER;
    count_ = 0;
  }

  void handleMeasurePacket(const MeasurePacket & pkt) {
    packet_count_++;
    last_packet_ms_ = millis();
    speed_ = pkt.speed;
    timestamp_ = pkt.timestamp;

    // Angle swept by this packet, handling the 360 -> 0 wrap.
    uint32_t sweep = ((uint32_t)pkt.end_angle + 36000u - (uint32_t)pkt.start_angle) % 36000u;

    // Sanity check from the SDK: at the reported rotation speed, 12 points
    // cannot span much more than speed * 12 / measure_freq degrees. A wider
    // sweep means we resynced onto a corrupt packet whose CRC happened to pass.
    if (speed_ > 0) {
      float max_sweep_deg =
        (float)speed_ * POINT_PER_PACK / LIDAR_MEASURE_FREQ * 1.5f;
      if (sweep / 100.0f > max_sweep_deg)
        return;
    }

    float step  = (sweep / 100.0f) / (POINT_PER_PACK - 1);
    float start = pkt.start_angle / 100.0f;

    for (int i = 0; i < POINT_PER_PACK; i++) {
      float raw_angle = start + i * step;
      if (raw_angle >= 360.0f)
        raw_angle -= 360.0f;

      bool scan_completed = (raw_angle < 20.0f) && (last_raw_angle_ > 340.0f);
      last_raw_angle_ = raw_angle;

      uint16_t distance = pkt.point[i].distance;
      uint8_t intensity = (distance == 0) ? 0 : pkt.point[i].intensity;

      if (scan_point_cb_ != nullptr)
        scan_point_cb_(correctAngle(raw_angle, distance), distance, intensity,
                       scan_completed);
    }
  }

  // Move the origin from the sensor's ranging centre to the centre of the
  // LiDAR circle (sl_transform.cpp). The SDK spells this as (360 - raw) +
  // shift, i.e. in the right-hand frame it flips into at the same time; the
  // datagram wants the clockwise angle, which is the same correction with the
  // flip left out: raw - shift.
  float correctAngle(float raw_angle, uint16_t distance) {
    if (distance > 0) {
      float x = distance + OFFSET_X;
      float y = distance * 0.11923f + OFFSET_Y;
      last_shift_deg_ = atanf(y / x) * 180.0f / (float)M_PI;
    }
    // A zero-distance point reuses the previous point's correction.
    float angle = raw_angle - last_shift_deg_;
    if (angle >= 360.0f)
      angle -= 360.0f;
    if (angle < 0.0f)
      angle += 360.0f;
    return angle;
  }

  State    state_ = HEADER;
  uint16_t count_ = 0;
  uint8_t  buf_[sizeof(MeasurePacket)];

  uint16_t speed_ = 0;
  uint16_t timestamp_ = 0;
  float    last_raw_angle_ = 0.0f;
  float    last_shift_deg_ = 0.0f;

  uint32_t packet_count_ = 0;
  uint32_t crc_error_count_ = 0;
  uint32_t last_packet_ms_ = 0;

  // Raw-byte diagnostics: they answer "is anything reaching the RX pin at all?"
  static const uint8_t SAMPLE_LEN = 32;
  uint32_t raw_byte_count_ = 0;
  uint8_t  sample_[SAMPLE_LEN];
  uint8_t  sample_len_ = 0;

  ScanPointCallback  scan_point_cb_ = nullptr;
  HealthCallback     health_cb_ = nullptr;
  DeviceInfoCallback device_info_cb_ = nullptr;
};

LD14P lidar;

// ------------------------------------------------- scan handoff between tasks
// RANGE_MIN_MM, RANGE_MAX_MM, SCAN_MAX_AGE_MS and the forward-cone settings are
// in the main sketch's SETTINGS block.

struct ScanSnapshot {
  uint16_t range_mm[SCAN_BINS];   // 0 = no valid return in this bin
  uint16_t sector_mm[PROX_SECTORS];  // nearest return per vehicle octant, 0 = clear
  float    freq_hz;
  uint32_t start_ms;              // millis() when this revolution began
  uint32_t end_ms;
  uint16_t points;
  uint16_t valid_points;
};

// Written by the LiDAR task, read by the sender and by loop(), all under
// scan_mutex.
static ScanSnapshot      shared_scan;
static volatile bool     shared_scan_ready = false;
// Bumped once per published revolution. shared_scan_ready is consumed (cleared)
// by the UDP sender, so the serial sender (lidar_serial.h) tracks this counter
// instead and the two never steal scans from each other.
static volatile uint32_t shared_scan_seq = 0;
static SemaphoreHandle_t scan_mutex = nullptr;

// Nearest return in the forward cone, in mm, 0 = nothing seen. Written by the
// LiDAR task and read by the control path, so it is a bare aligned uint16_t:
// a torn read is not possible on Xtensa and the control path must never block
// on a mutex.
static volatile uint16_t obstacle_mm = 0;

// millis() of the last published measurement, and a flag to tell "nothing in
// the cone" apart from "no scan has ever completed". Without this a stalled
// LiDAR freezes obstacle_mm at its last value, and if that value was clear
// road the car drives on blind. See lidarObstacleCm().
//
// These three are written on the LiDAR task's core and read from loop()'s, and
// the group is not updated atomically - only each individual access is. A
// reader can therefore catch one control cycle with a new distance against an
// old timestamp (which makes the age check err towards stopping) or the reverse
// (one cycle of stale distance). Each is corrected on the next loop pass, which
// runs far faster than the ~10 Hz scan rate, so this is not closed with a
// critical section. The permanent-freeze case is what the timestamp is for.
static volatile uint32_t obstacle_ms = 0;
static volatile bool     obstacle_valid = false;

// Owned by the LiDAR task alone.
static ScanSnapshot acc;
static uint16_t     acc_forward_mm = 0;

static void resetAccumulator(uint32_t now_ms) {
  memset(acc.range_mm, 0, sizeof(acc.range_mm));
  acc.points = 0;
  acc.valid_points = 0;
  acc.start_ms = now_ms;
  acc_forward_mm = 0;
}

// True when a bin angle falls inside the forward cone. Angles here are
// CLOCKWISE from the LiDAR's zero mark; LIDAR_FORWARD_DEG is where the car's
// nose sits in that frame, so 0 means "mounted with the zero mark facing
// forward". The cone is centred on the nose and half-angles handle the 0/360
// wrap.
static bool inForwardCone(float bin_angle_deg) {
  float delta = bin_angle_deg - LIDAR_FORWARD_DEG;
  while (delta > 180.0f)  delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return fabsf(delta) <= OBSTACLE_HALF_ANGLE_DEG;
}

// Vehicle-frame bearing of a bin: 0 is straight ahead, increasing CLOCKWISE,
// so 90 is the car's right and 270 its left. This is the frame the proximity
// sectors and the forward cone are expressed in.
static float vehicleAngleDeg(float lidar_angle_deg) {
  float a = lidar_angle_deg - LIDAR_FORWARD_DEG;
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f)    a += 360.0f;
  return a;
}

// Fold the 360-degree scan into PROX_SECTORS octants, keeping the nearest
// return in each. Sector 0 is straight ahead and the numbering goes clockwise,
// which is the MAVLink DISTANCE_SENSOR orientation layout (YAW_0, YAW_45, ...
// YAW_315) that ArduPilot's proximity sensor uses.
//
// This is what the app's object avoidance consumes. Collapsing to octants here
// rather than sending the whole scan keeps the message small enough for the
// OpenBot line protocol, and eight directions is all a stop-law needs.
static void computeSectors(ScanSnapshot & s) {
  for (int i = 0; i < PROX_SECTORS; i++)
    s.sector_mm[i] = 0;

  for (int bin = 0; bin < (int)SCAN_BINS; bin++) {
    uint16_t mm = s.range_mm[bin];
    if (mm == 0)
      continue;
    float bin_angle = ((float)bin + 0.5f) * 360.0f / (float)SCAN_BINS;
    float a = vehicleAngleDeg(bin_angle);

    // Sector i is centred on i * (360/PROX_SECTORS) and spans half of one
    // sector step either side.
    float step = 360.0f / (float)PROX_SECTORS;
    int sector = (int)((a + step * 0.5f) / step) % PROX_SECTORS;
    if (sector < 0)
      continue;
    if (s.sector_mm[sector] == 0 || mm < s.sector_mm[sector])
      s.sector_mm[sector] = mm;
  }
}

// Hand the finished revolution to the sender. A snapshot that was never picked
// up is simply overwritten - a stale scan is worse than none.
static void publishAccumulator(uint32_t now_ms) {
  acc.freq_hz = lidar.getScanFreqHz();
  acc.end_ms  = now_ms;
  // Fold the revolution into proximity octants here, on the LiDAR task, so the
  // 'p' report and any later reader see a consistent set that belongs to this
  // same scan rather than recomputing from bins that may have moved on.
  computeSectors(acc);
  if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    memcpy(&shared_scan, &acc, sizeof(ScanSnapshot));
    shared_scan_ready = true;
    shared_scan_seq = shared_scan_seq + 1;
    xSemaphoreGive(scan_mutex);
  }
  obstacle_mm = acc_forward_mm;
  obstacle_ms = now_ms;
  obstacle_valid = true;
  resetAccumulator(now_ms);
}

// Called from the LiDAR task for every point the driver decodes. The scan
// datagram carries ranges only, so the intensity byte is decoded and checked
// but not forwarded.
void lidar_scan_point_callback(float angle_deg, uint16_t distance_mm,
                               uint8_t intensity, bool scan_completed) {
  (void)intensity;
  uint32_t now = millis();

  if (scan_completed && acc.points > 0)
    publishAccumulator(now);

  acc.points++;
  if (distance_mm < RANGE_MIN_MM || distance_mm > RANGE_MAX_MM)
    return;  // out of range, or 0 = no return

  float a = fmodf(angle_deg, 360.0f);
  if (a < 0.0f)
    a += 360.0f;

  // Forward-cone obstacle distance: nearest return wins. A range that reads
  // slightly short is a much cheaper mistake for an obstacle stop than one
  // that reads long and drives the car into a wall.
  if (inForwardCone(a)) {
    if (acc_forward_mm == 0 || distance_mm < acc_forward_mm)
      acc_forward_mm = distance_mm;
  }

  int bin = (int)(a * (float)SCAN_BINS / 360.0f + 0.5f);
  if (bin >= (int)SCAN_BINS)
    bin -= SCAN_BINS;
  if (bin < 0)
    return;

  // Several points can land in one bin. Keep the nearest: a range that reads
  // slightly short is a much cheaper mistake for the costmap than an obstacle
  // that disappears behind a farther return.
  if (acc.range_mm[bin] == 0 || distance_mm < acc.range_mm[bin])
    acc.range_mm[bin] = distance_mm;
  acc.valid_points++;
}

void lidar_health_callback(uint8_t error_code) {
  Serial.printf("LiDAR health: error code 0x%02X%s\n", error_code,
                error_code == 0 ? " (ok)" : "");
}

void lidar_device_info_callback(const ManufacturePacket & info) {
  Serial.printf("LiDAR SN %08lX%08lX  hw %lu  fw %lu  product ver %u\n",
                (unsigned long)info.sn_high, (unsigned long)info.sn_low,
                (unsigned long)info.hardware_version,
                (unsigned long)info.firmware_version,
                (unsigned)info.product_version);
}

// ------------------------------------------------------------- the LiDAR task
// CANDIDATE_BAUD (the rates tried when the LiDAR stays silent) is in SETTINGS.
static const int CANDIDATE_BAUD_COUNT =
  (int)(sizeof(CANDIDATE_BAUD) / sizeof(CANDIDATE_BAUD[0]));
static int baud_index = 0;

static void diagnoseSilentLidar() {
  if (lidar.getRawByteCount() == 0) {
    // Not one byte has reached the RX pin, so the baud rate is not the issue.
    Serial.printf("nothing at all on GPIO%d - not a single byte received\n",
                  LIDAR_RX_PIN);
    Serial.println("  1. is the LiDAR motor actually spinning? if not it is a "
                   "power problem: the LD14/LD14P wants 5V and ~200 mA");
    Serial.printf("  2. LiDAR TX must go to GPIO%d, not the LiDAR RX pin - "
                  "try swapping the two signal wires\n", LIDAR_RX_PIN);
    Serial.println("  3. LiDAR GND and ESP32 GND must be connected");
    Serial.printf("  4. GPIO%d must not be one of the N16R8's reserved pins "
                  "(26-37 are flash/PSRAM); see the SETTINGS pin notes\n",
                  LIDAR_RX_PIN);
    return;
  }

  // Bytes are arriving but nothing frames up, which usually means the baud
  // rate is wrong. Show what came in, then try the next candidate.
  Serial.printf("%lu bytes at %lu baud, but no valid packet (%lu CRC errors)."
                " first bytes:", (unsigned long)lidar.getRawByteCount(),
                (unsigned long)CANDIDATE_BAUD[baud_index],
                (unsigned long)lidar.getCrcErrorCount());
  const uint8_t * sample = lidar.getSample();
  for (uint8_t i = 0; i < lidar.getSampleLen(); i++)
    Serial.printf(" %02X", sample[i]);
  Serial.println();

  baud_index = (baud_index + 1) % CANDIDATE_BAUD_COUNT;
  Serial.printf("trying %lu baud...\n",
                (unsigned long)CANDIDATE_BAUD[baud_index]);
  LidarSerial.updateBaudRate(CANDIDATE_BAUD[baud_index]);
  lidar.resetStats();
}

static void lidarTask(void * arg) {
  (void)arg;
  uint32_t last_report_ms = 0;

  for (;;) {
    lidar.poll(LidarSerial);

    uint32_t now = millis();

    // A revolution whose wrap-around point never arrived still goes out.
    if (acc.points > 0 && now - acc.start_ms > SCAN_MAX_AGE_MS)
      publishAccumulator(now);

    if (lidar.getLastPacketMs() == 0 || now - lidar.getLastPacketMs() >= 1000) {
      if (now - last_report_ms >= 3000) {
        last_report_ms = now;
        diagnoseSilentLidar();
      }
    }

    vTaskDelay(1);  // one tick; the RX buffer holds ~180 ms of LD14 data
  }
}

// ------------------------------------------------------------ lifecycle hooks
static void lidarBegin() {
  LidarSerial.setRxBufferSize(2048);  // must be called before begin()
  // The LD14/LD14P free-runs and never needs a command, so TX is left
  // unwired and unassigned.
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setHealthCallback(lidar_health_callback);
  lidar.setDeviceInfoCallback(lidar_device_info_callback);

  scan_mutex = xSemaphoreCreateMutex();
  resetAccumulator(millis());

  Serial.printf("LiDAR  UART1 @ %lu baud, RX=GPIO%d TX=%d (-1 = unwired)\n",
                (unsigned long)LIDAR_BAUD, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.printf("       %u bins, forward cone %+.0f deg +/- %.0f deg\n",
                (unsigned)SCAN_BINS, LIDAR_FORWARD_DEG, OBSTACLE_HALF_ANGLE_DEG);
}

// The forward-cone distance in centimetres, capped to what the OpenBot sonar
// channel is specced for.
//
// Fail-safe on stale data. If no revolution has completed within
// SCAN_MAX_AGE_MS - the LiDAR jammed, its power dropped, or the cable fell off
// - this returns 0 rather than the last reading. That is deliberate: a frozen
// "clear road" is exactly the reading that drives a car into a wall, whereas a
// frozen "0 cm" just stops it. The driver sees 0 cm on the obstacle bar and
// knows the sensor is down.
//
// Before the first revolution completes it also returns 0, for the same reason.
static unsigned int lidarObstacleCm() {
  if (!obstacle_valid)
    return 0;
  uint32_t age = millis() - obstacle_ms;
  if (age > SCAN_MAX_AGE_MS)
    return 0;

  uint16_t mm = obstacle_mm;
  if (mm == 0)
    return MAX_SONAR_DISTANCE;  // cone is clear
  unsigned int cm = (unsigned int)((mm + 5) / 10);  // round to nearest cm
  if (cm < 1) cm = 1;
  if (cm > MAX_SONAR_DISTANCE) cm = MAX_SONAR_DISTANCE;
  return cm;
}

// Fill out_cm[PROX_SECTORS] with the nearest return per vehicle octant, in cm.
// 0 means "nothing in range in this sector", NOT "an obstacle at zero" - open
// space and no return are the same reading here. That is the opposite of the
// 's' channel, where 0 means stale and the car stops; staleness on 'p' is
// signalled by the report simply not arriving, which is what the app's
// Proximity age check watches for.
//
// Returns false when there is no fresh revolution (no scan yet, or one older
// than SCAN_MAX_AGE_MS), or when the snapshot is momentarily locked. The caller
// must leave its previous sectors alone in that case rather than reporting
// zeros as "clear road". Unlike lidarObstacleCm(), this is not on the hard
// control path - it runs at the report rate - so taking the mutex is fine.
static bool lidarProximityCm(unsigned int out_cm[PROX_SECTORS]) {
  if (!obstacle_valid)
    return false;
  if (millis() - obstacle_ms > SCAN_MAX_AGE_MS)
    return false;
  if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    return false;

  for (int i = 0; i < PROX_SECTORS; i++) {
    uint16_t mm = shared_scan.sector_mm[i];
    unsigned int cm = 0;
    if (mm != 0) {
      cm = (unsigned int)((mm + 5) / 10);
      if (cm < 1) cm = 1;
      if (cm > MAX_SONAR_DISTANCE) cm = MAX_SONAR_DISTANCE;
    }
    out_cm[i] = cm;
  }
  xSemaphoreGive(scan_mutex);
  return true;
}
