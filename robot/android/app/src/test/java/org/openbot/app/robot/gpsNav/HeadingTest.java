package org.openbot.app.robot.gpsNav;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import android.hardware.SensorManager;

import org.junit.Test;

/**
 * Compass heading for a phone standing upright in the OpenBot mount. Matrices are row-major
 * device-to-world (X east, Y north, Z up); column j is device axis j in world coordinates.
 */
public class HeadingTest {
  private static final float EPS = 0.01f;

  @Test
  public void uprightPortraitCameraNorthIsZero() {
    // X east, Y up, Z south (so the back camera, -Z, looks north).
    float[] r = {1, 0, 0, 0, 0, -1, 0, 1, 0};
    assertEquals(0f, GpsNavLocationProvider.headingFromRotationMatrix(r), EPS);
  }

  @Test
  public void uprightLandscapeCameraEastIsNinety() {
    // X up, Y north, Z west (back camera looks east).
    float[] r = {0, 0, -1, 0, 1, 0, 1, 0, 0};
    assertEquals(90f, GpsNavLocationProvider.headingFromRotationMatrix(r), EPS);
  }

  @Test
  public void flatPhoneUsesTopEdge() {
    // Lying flat, screen up, top edge pointing west.
    float[] r = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    assertEquals(270f, GpsNavLocationProvider.headingFromRotationMatrix(r), EPS);
  }

  @Test
  public void compassTrustedWithEarthLikeFieldAndGoodAccuracy() {
    assertTrue(
        GpsNavLocationProvider.isCompassReliable(SensorManager.SENSOR_STATUS_ACCURACY_HIGH, 42f));
    assertTrue(
        GpsNavLocationProvider.isCompassReliable(SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM, 30f));
  }

  @Test
  public void compassDistrustedWhenUncalibrated() {
    assertFalse(GpsNavLocationProvider.isCompassReliable(SensorManager.SENSOR_STATUS_UNRELIABLE, 42f));
    assertFalse(
        GpsNavLocationProvider.isCompassReliable(SensorManager.SENSOR_STATUS_ACCURACY_LOW, 42f));
  }

  @Test
  public void compassDistrustedNearMotors() {
    // A motor magnet or the ESC next to the phone: field far stronger than Earth's.
    assertFalse(
        GpsNavLocationProvider.isCompassReliable(SensorManager.SENSOR_STATUS_ACCURACY_HIGH, 140f));
    // Shielded by steel: field far weaker.
    assertFalse(
        GpsNavLocationProvider.isCompassReliable(SensorManager.SENSOR_STATUS_ACCURACY_HIGH, 8f));
  }

  @Test
  public void noReadingYetDoesNotNag() {
    assertTrue(
        GpsNavLocationProvider.isCompassReliable(
            SensorManager.SENSOR_STATUS_ACCURACY_HIGH, Float.NaN));
  }
}
