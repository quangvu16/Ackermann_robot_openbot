package org.openbot.app.robot.gpsNav;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Looper;
import android.os.SystemClock;
import androidx.core.content.ContextCompat;
import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.location.LocationServices;
import org.openbot.app.robot.sensorlink.SensorLink;
import timber.log.Timber;

/**
 * Supplies GPS fixes and a heading for the navigation loop.
 *
 * <p>Position comes from the fused location provider at 1 Hz, the same cadence the data logger
 * uses. Heading prefers the GPS course-over-ground while the robot is actually moving (that is the
 * direction the wheels are taking us) and falls back to the device rotation vector when the robot
 * is stationary or still accelerating, so a mission can start from a standstill.
 */
public class GpsNavLocationProvider implements SensorEventListener {

  public interface Listener {
    void onLocationUpdate(Location location);

    void onHeadingUpdate(float headingDegrees);

    void onLocationUnavailable(String reason);
  }

  /** Speed in m/s above which GPS course is trusted as the heading. */
  private static final float COURSE_SPEED_THRESHOLD_MPS = 0.5f;

  private final FusedLocationProviderClient fusedLocationClient;
  private final SensorManager sensorManager;
  private final Sensor rotationVectorSensor;
  private final Sensor magneticSensor;

  /**
   * Earth's field is about 25-65 microtesla everywhere (about 42 in southern Vietnam). A reading
   * well outside that means something on the robot - motors, the ESC, a speaker magnet, steel - is
   * swamping it, and the compass heading cannot be trusted.
   */
  static final float FIELD_MIN_UT = 25f;

  static final float FIELD_MAX_UT = 70f;

  private volatile int magneticAccuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH;
  private volatile float fieldMicroTesla = Float.NaN;
  private final Listener listener;

  private LocationCallback locationCallback;
  private boolean running = false;
  private float lastHeadingDegrees = Float.NaN;
  /** True when the most recent fix carried a trustworthy course-over-ground. */
  private boolean headingFromCourse = false;

  public GpsNavLocationProvider(Context context, Listener listener) {
    this.listener = listener;
    this.fusedLocationClient = LocationServices.getFusedLocationProviderClient(context);
    this.sensorManager =
        (SensorManager) context.getApplicationContext().getSystemService(Context.SENSOR_SERVICE);
    this.rotationVectorSensor =
        sensorManager == null ? null : sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    this.magneticSensor =
        sensorManager == null ? null : sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
  }

  public void start(Context context) {
    if (running) {
      return;
    }
    if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
        != PackageManager.PERMISSION_GRANTED) {
      listener.onLocationUnavailable("Location permission not granted");
      return;
    }

    locationCallback =
        new LocationCallback() {
          @Override
          public void onLocationResult(LocationResult locationResult) {
            Location location = locationResult.getLastLocation();
            if (location == null || isRemoteActive()) {
              // While another phone's GPS is feeding us (Sensor link), ignore our own.
              return;
            }
            handleFix(location);
          }
        };

    // play-services-location 19.x API (see SensorService#getLocationRequest): the Builder/Priority
    // API only arrived in 21.0.0, which this project does not use yet.
    LocationRequest request = new LocationRequest();
    request.setInterval(1000L);
    request.setFastestInterval(500L);
    request.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);

    try {
      fusedLocationClient.requestLocationUpdates(request, locationCallback, Looper.getMainLooper());
    } catch (SecurityException e) {
      Timber.e(e, "Lost location permission while starting GPS navigation");
      listener.onLocationUnavailable("Location permission not granted");
      return;
    }

    if (rotationVectorSensor != null) {
      sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_GAME);
    }
    if (magneticSensor != null) {
      // Watched only to judge whether the compass can be trusted; the heading itself comes from
      // the rotation vector.
      sensorManager.registerListener(this, magneticSensor, SensorManager.SENSOR_DELAY_UI);
    }
    running = true;
  }

  public void stop() {
    if (!running) {
      return;
    }
    running = false;
    if (locationCallback != null) {
      fusedLocationClient.removeLocationUpdates(locationCallback);
      locationCallback = null;
    }
    if (sensorManager != null) {
      sensorManager.unregisterListener(this);
    }
  }

  /** Last known heading in degrees, or {@link Float#NaN} if none yet. */
  public float getLastHeadingDegrees() {
    return lastHeadingDegrees;
  }

  /** True while the heading comes from GPS course-over-ground (moving) rather than the compass. */
  public boolean isHeadingFromCourse() {
    return headingFromCourse;
  }

  /**
   * False when the compass should not be trusted: Android rates the magnetometer unreliable or low
   * (it needs a figure-8 calibration), or the field strength is not Earth-like (something on the
   * robot is disturbing it). Unknown - no magnetometer reading yet - counts as reliable, so a phone
   * that simply has not reported does not nag.
   */
  public boolean isCompassReliable() {
    return isCompassReliable(magneticAccuracy, fieldMicroTesla);
  }

  static boolean isCompassReliable(int accuracy, float fieldMicroTesla) {
    if (accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE
        || accuracy == SensorManager.SENSOR_STATUS_ACCURACY_LOW) {
      return false;
    }
    if (Float.isNaN(fieldMicroTesla)) {
      return true;
    }
    return fieldMicroTesla >= FIELD_MIN_UT && fieldMicroTesla <= FIELD_MAX_UT;
  }

  private void handleFix(Location location) {
    updateDeclination(location);
    listener.onLocationUpdate(location);
    if (isCourseValid(location)) {
      headingFromCourse = true;
      lastHeadingDegrees = location.getBearing();
      listener.onHeadingUpdate(lastHeadingDegrees);
    } else {
      // Stationary or creeping: GPS course is meaningless, fall back to the magnetometer.
      headingFromCourse = false;
      if (!Float.isNaN(lastHeadingDegrees)) {
        listener.onHeadingUpdate(lastHeadingDegrees);
      }
    }
  }

  // region Remote sensors (Sensor link client: a SensorServer app on another phone)

  private long remoteFixAtMs = 0L;

  /** True while fixes from the other phone are fresh; our own GPS and compass then stand by. */
  public boolean isRemoteActive() {
    return remoteFixAtMs != 0L
        && SystemClock.elapsedRealtime() - remoteFixAtMs < SensorLink.REMOTE_STALE_MS;
  }

  /** A fix from the other phone; handled exactly like one of ours. */
  public void onRemoteLocation(Location location) {
    if (!running || location == null) {
      return;
    }
    remoteFixAtMs = SystemClock.elapsedRealtime();
    handleFix(location);
  }

  /**
   * The other phone's rotation vector. Same mounting assumption as our own: upright, back camera
   * facing forward (or lying flat, top edge forward).
   */
  public void onRemoteRotationVector(float[] values) {
    if (!running || !isRemoteActive()) {
      return;
    }
    handleRotationVector(values);
  }

  // endregion

  private static boolean isCourseValid(Location location) {
    return location.hasBearing() && location.getSpeed() >= COURSE_SPEED_THRESHOLD_MPS;
  }

  // region SensorEventListener (rotation vector → magnetic heading)

  private final float[] rotationMatrix = new float[9];

  /**
   * Magnetic declination at the robot, in degrees east. The rotation vector points at magnetic
   * north while GPS bearings are true north; updated from each fix.
   */
  private float declinationDegrees = 0f;

  private void updateDeclination(Location location) {
    GeomagneticField field =
        new GeomagneticField(
            (float) location.getLatitude(),
            (float) location.getLongitude(),
            (float) location.getAltitude(),
            location.getTime());
    declinationDegrees = field.getDeclination();
  }

  /**
   * Compass heading of the robot's nose from a rotation matrix, in {@code [0, 360)} degrees from
   * magnetic north.
   *
   * <p>On OpenBot the phone stands upright in its mount with the back camera looking where the car
   * is going, so the car's forward direction is the device's -Z axis, whichever way round
   * (portrait or either landscape) the phone sits. {@code SensorManager.getOrientation()} instead
   * reports the azimuth of the device's top edge, which is only right with the phone lying flat and
   * becomes unstable (gimbal lock) when it is upright. So: when the phone is upright, use the
   * camera axis; when it is lying flat (|world-up component of Z| large), fall back to the top edge.
   *
   * <p>{@code r} is row-major device-to-world (X east, Y north, Z up), as filled by {@link
   * SensorManager#getRotationMatrixFromVector}. Column j is device axis j in world coordinates.
   */
  static float headingFromRotationMatrix(float[] r) {
    double east;
    double north;
    if (Math.abs(r[8]) < 0.7) {
      // Upright: the back camera (device -Z) is near horizontal.
      east = -r[2];
      north = -r[5];
    } else {
      // Flat: the top edge (device +Y) is near horizontal.
      east = r[1];
      north = r[4];
    }
    float heading = (float) Math.toDegrees(Math.atan2(east, north));
    if (heading < 0) {
      heading += 360f;
    }
    return heading;
  }

  @Override
  public void onSensorChanged(SensorEvent event) {
    if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
      float x = event.values[0];
      float y = event.values[1];
      float z = event.values[2];
      fieldMicroTesla = (float) Math.sqrt(x * x + y * y + z * z);
      magneticAccuracy = event.accuracy;
      return;
    }
    if (event.sensor.getType() != Sensor.TYPE_ROTATION_VECTOR) {
      return;
    }
    if (isRemoteActive()) {
      return; // the other phone's compass is in charge
    }
    handleRotationVector(event.values);
  }

  private void handleRotationVector(float[] values) {
    SensorManager.getRotationMatrixFromVector(rotationMatrix, values);
    float heading = headingFromRotationMatrix(rotationMatrix) + declinationDegrees;
    if (heading >= 360f) {
      heading -= 360f;
    } else if (heading < 0f) {
      heading += 360f;
    }

    // Only trust the magnetometer while GPS course is unusable; when moving, the direction of
    // travel is what the steering law actually needs.
    if (!headingFromCourse) {
      lastHeadingDegrees = heading;
      listener.onHeadingUpdate(heading);
    }
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {
    if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
      magneticAccuracy = accuracy;
    }
  }

  // endregion
}
