package org.openbot.app.robot.vehicle;

import android.os.SystemClock;

/**
 * The eight proximity octants the car reports on the OpenBot {@code p} channel, with the staleness
 * tracking object avoidance needs.
 *
 * <p>The layout is the MAVLink {@code DISTANCE_SENSOR} octant arrangement that ArduPilot's
 * proximity library uses, and the same one the firmware's {@code PROX_SECTORS} folding produces:
 * sector 0 is straight ahead and the numbering goes clockwise, one sector every 45 degrees.
 *
 * <pre>
 *        7   0   1        0 = front        4 = back
 *      6   +   2          1 = front-right  5 = back-left
 *        5   4   3        2 = right        6 = left
 *                          3 = back-right   7 = front-left
 * </pre>
 *
 * <p><b>Units and the zero convention.</b> The wire carries centimetres to match the {@code s}
 * sonar channel, and the firmware spells "nothing measured in this sector" as {@code 0}. This class
 * converts on ingest: metres internally, and a 0 becomes {@link Float#POSITIVE_INFINITY} so an
 * empty sector can never be mistaken for an obstacle sitting on top of the car. That is the
 * opposite of the {@code s} channel, where 0 means stale and the car stops - on {@code p},
 * staleness is signalled by the report simply not arriving, which is what {@link #getAge()} and
 * {@link #getDataState()} watch for.
 *
 * <p>Timestamps follow {@link org.openbot.app.robot.env.SensorReading}, but the age it reports is
 * the gap between the last two updates - which freezes at the reporting period if updates stop,
 * making a dead sensor look permanently fresh. Staleness here is therefore measured as time since
 * the last accepted report, which is what actually has to expire for the car to notice the LiDAR
 * has stopped.
 */
public class Proximity {

  public static final int SECTORS = 8;

  /** How old the last report may be before it is considered stale, in milliseconds. */
  public static final long STALE_AFTER_MS = 500;

  /**
   * Age reported before any reading has arrived, so a never-fed instance reads as stale rather than
   * fresh.
   */
  private static final long NEVER_FED_AGE_MS = 1000;

  private final float[] sectorMeters = new float[SECTORS];
  private long timestamp = 0;

  public Proximity() {
    for (int i = 0; i < SECTORS; i++) {
      sectorMeters[i] = Float.POSITIVE_INFINITY;
    }
  }

  /**
   * Parse the body of a {@code p} message: eight comma-separated distances in centimetres. Returns
   * null for anything malformed, so a truncated BLE notification cannot be mistaken for a scan of
   * eight clear sectors.
   */
  public static float[] parseSectorsCm(String body) {
    if (body == null) {
      return null;
    }
    String[] parts = body.split(",");
    if (parts.length != SECTORS) {
      return null;
    }
    float[] out = new float[SECTORS];
    for (int i = 0; i < SECTORS; i++) {
      try {
        out[i] = Float.parseFloat(parts[i].trim());
      } catch (NumberFormatException e) {
        return null;
      }
    }
    return out;
  }

  /**
   * Ingest one {@code p} report.
   *
   * @param sectorCm the eight octant distances in centimetres, as received; {@code 0} means "no
   *     return" and is stored as infinite. A null, short or otherwise unusable array is rejected
   *     and the previous reading is left alone rather than being replaced with zeros that look like
   *     clear road.
   * @return true if the report was accepted
   */
  public synchronized boolean setSectorsCm(float[] sectorCm) {
    if (sectorCm == null || sectorCm.length != SECTORS) {
      return false;
    }
    for (int i = 0; i < SECTORS; i++) {
      float cm = sectorCm[i];
      if (Float.isNaN(cm) || cm < 0f) {
        return false;
      }
    }
    for (int i = 0; i < SECTORS; i++) {
      float cm = sectorCm[i];
      // 0 cm on the wire means "nothing measured here", not "an obstacle at zero". A real return
      // at 0 is outside the LD14's 20 mm minimum anyway, so the two cannot be confused.
      sectorMeters[i] = cm == 0f ? Float.POSITIVE_INFINITY : cm / 100f;
    }
    this.timestamp = SystemClock.elapsedRealtime();
    return true;
  }

  /**
   * The eight octant distances in metres. Sectors with no return are {@link
   * Float#POSITIVE_INFINITY}. The array is a copy, safe to hold onto.
   */
  public synchronized float[] getSectorMeters() {
    return sectorMeters.clone();
  }

  /**
   * Distance of one sector in metres, or {@link Float#POSITIVE_INFINITY} when there is no return.
   */
  public synchronized float getSectorMeters(int sector) {
    if (sector < 0 || sector >= SECTORS) {
      return Float.POSITIVE_INFINITY;
    }
    return sectorMeters[sector];
  }

  /**
   * Milliseconds since the last accepted report, or {@link #NEVER_FED_AGE_MS} before the first one.
   * Unlike {@code SensorReading#getAge()} this keeps growing when reports stop - that is the whole
   * point of it.
   */
  public synchronized long getAge() {
    if (timestamp == 0) {
      return NEVER_FED_AGE_MS;
    }
    return SystemClock.elapsedRealtime() - timestamp;
  }

  /**
   * {@code SystemClock.elapsedRealtime()} of the last accepted report, or 0 before the first one.
   */
  public synchronized long getTimestamp() {
    return timestamp;
  }

  /** True once at least one report has been ingested, so the car is known to carry a LiDAR. */
  public synchronized boolean hasEverReported() {
    return timestamp > 0;
  }

  /**
   * How fresh the picture is. See {@link ObjectAvoidance.DataState} for what the law does with each
   * value - notably, {@code NEVER} never blocks driving (a car with no LiDAR must stay drivable)
   * while {@code STALE} blocks forward motion (a LiDAR that died mid-drive must not leave the car
   * driving blind).
   */
  public synchronized ObjectAvoidance.DataState getDataState() {
    if (timestamp == 0) {
      return ObjectAvoidance.DataState.NEVER;
    }
    return getAge() > STALE_AFTER_MS
        ? ObjectAvoidance.DataState.STALE
        : ObjectAvoidance.DataState.FRESH;
  }

  /** Reset to "nothing has ever been measured". */
  public synchronized void reset() {
    for (int i = 0; i < SECTORS; i++) {
      sectorMeters[i] = Float.POSITIVE_INFINITY;
    }
    timestamp = 0;
  }
}
