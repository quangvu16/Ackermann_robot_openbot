package org.openbot.app.robot.gpsNav;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.preference.PreferenceManager;
import android.util.AttributeSet;
import java.util.ArrayList;
import java.util.List;
import org.osmdroid.config.Configuration;
import org.osmdroid.events.MapEventsReceiver;
import org.osmdroid.tileprovider.MapTileProviderBasic;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.CustomZoomButtonsController;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.CopyrightOverlay;
import org.osmdroid.views.overlay.MapEventsOverlay;
import org.osmdroid.views.overlay.Marker;
import org.osmdroid.views.overlay.Polygon;
import org.osmdroid.views.overlay.Polyline;
import org.osmdroid.views.overlay.TilesOverlay;

/**
 * ArduPilot Mission Planner style interactive map view based on OpenStreetMap tile engine.
 *
 * <p>Provides real-world satellite / street tile mapping, interactive pan and pinch-to-zoom,
 * mission path visualization, numbered waypoint markers with acceptance radius circles, robot
 * heading & position marker, Return-to-Launch (RTL) home marker, breadcrumb tracking, and
 * tap-to-add waypoint interactions.
 */
public class MissionMapView extends MapView {

  /**
   * Map backdrop, matching Mission Planner's map provider dropdown. Satellite is the default
   * because that is what Mission Planner shows for outdoor vehicle missions.
   */
  public enum MapType {
    SATELLITE,
    HYBRID,
    STREET,
    TERRAIN
  }

  /** Callback for taps on the map. */
  public interface OnMapTapListener {
    void onMapTap(double latitude, double longitude);
  }

  private OnMapTapListener onMapTapListener;

  private Marker robotMarker;
  private Marker homeMarker;
  private final List<Marker> waypointMarkers = new ArrayList<>();
  private final List<Polygon> waypointAcceptanceCircles = new ArrayList<>();
  private Polyline missionPolyline;
  private Polyline trackPolyline;
  private TilesOverlay labelsOverlay;
  private MapType mapType = MapType.SATELLITE;
  private CopyrightOverlay copyrightOverlay;

  private double robotLatitude = Double.NaN;
  private double robotLongitude = Double.NaN;
  private double robotHeadingDegrees = 0.0;
  private boolean hasCenteredOnRobot = false;

  private Drawable robotDrawable;
  private Drawable homeDrawable;

  public MissionMapView(Context context) {
    this(context, null);
  }

  public MissionMapView(Context context, AttributeSet attrs) {
    super(context, attrs);
    initMap(context);
  }

  private void initMap(Context context) {
    Configuration.getInstance()
        .load(context, PreferenceManager.getDefaultSharedPreferences(context));
    Configuration.getInstance().setUserAgentValue(context.getPackageName());

    setMultiTouchControls(true);
    getZoomController().setVisibility(CustomZoomButtonsController.Visibility.SHOW_AND_FADEOUT);
    applyMapType();

    // Default to the centre of North America until a GPS fix arrives. Zoom 17 shows the
    // satellite backdrop at a scale where a robot's route is legible.
    getController().setZoom(17);
    getController().setCenter(new GeoPoint(37.4219983, -122.0840000));

    robotDrawable = createRobotDrawable(context);
    homeDrawable = createHomeDrawable(context);

    // Map tap receiver
    MapEventsReceiver receiver =
        new MapEventsReceiver() {
          @Override
          public boolean singleTapConfirmedHelper(GeoPoint p) {
            if (onMapTapListener != null) {
              onMapTapListener.onMapTap(p.getLatitude(), p.getLongitude());
            }
            return true;
          }

          @Override
          public boolean longPressHelper(GeoPoint p) {
            return false;
          }
        };
    getOverlays().add(0, new MapEventsOverlay(receiver));
  }

  public void setOnMapTapListener(OnMapTapListener listener) {
    this.onMapTapListener = listener;
  }

  /** Switches the map backdrop, keeping the mission overlays in place. */
  public void setMapType(MapType type) {
    if (type == null) {
      return;
    }
    this.mapType = type;
    applyMapType();
  }

  public MapType getMapType() {
    return mapType;
  }

  private void applyMapType() {
    if (labelsOverlay != null) {
      getOverlays().remove(labelsOverlay);
      labelsOverlay = null;
    }

    switch (mapType) {
      case HYBRID:
        // GMap.NET's GoogleHybridMapProvider.Overlays is [GoogleSatellite, GoogleHybrid], so
        // hybrid is the satellite tile with the label tile drawn on top of it.
        setTileSource(MapTileSources.GOOGLE_SATELLITE);
        labelsOverlay =
            new TilesOverlay(
                new MapTileProviderBasic(getContext(), MapTileSources.GOOGLE_LABELS), getContext());
        labelsOverlay.setLoadingBackgroundColor(Color.TRANSPARENT);
        labelsOverlay.setLoadingLineColor(Color.TRANSPARENT);
        // Index 0 draws the labels just above the base tiles, underneath every mission
        // marker and path. MapEventsOverlay is also at 0 but paints nothing.
        getOverlays().add(0, labelsOverlay);
        break;
      case STREET:
        setTileSource(MapTileSources.GOOGLE_STREET);
        break;
      case TERRAIN:
        setTileSource(MapTileSources.GOOGLE_TERRAIN);
        break;
      case SATELLITE:
      default:
        setTileSource(MapTileSources.GOOGLE_SATELLITE);
        break;
    }

    if (copyrightOverlay == null) {
      copyrightOverlay = new CopyrightOverlay(getContext());
    }
    if (!getOverlays().contains(copyrightOverlay)) {
      getOverlays().add(copyrightOverlay);
    }
    invalidate();
  }

  public void setRobot(double latitude, double longitude, double headingDegrees) {
    if (Double.isNaN(latitude) || Double.isNaN(longitude)) {
      return;
    }
    this.robotLatitude = latitude;
    this.robotLongitude = longitude;
    this.robotHeadingDegrees = headingDegrees;

    GeoPoint robotPoint = new GeoPoint(latitude, longitude);

    if (robotMarker == null) {
      robotMarker = new Marker(this);
      robotMarker.setTitle("Robot");
      robotMarker.setIcon(robotDrawable);
      robotMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
      robotMarker.setFlat(true);
      getOverlays().add(robotMarker);
    }

    robotMarker.setPosition(robotPoint);
    robotMarker.setRotation((float) headingDegrees);

    if (!hasCenteredOnRobot) {
      getController().setCenter(robotPoint);
      hasCenteredOnRobot = true;
    }

    invalidate();
  }

  public void setHome(double latitude, double longitude) {
    if (Double.isNaN(latitude) || Double.isNaN(longitude)) {
      if (homeMarker != null) {
        getOverlays().remove(homeMarker);
        homeMarker = null;
      }
      return;
    }

    GeoPoint homePoint = new GeoPoint(latitude, longitude);
    if (homeMarker == null) {
      homeMarker = new Marker(this);
      homeMarker.setTitle("Home (RTL)");
      homeMarker.setIcon(homeDrawable);
      homeMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
      getOverlays().add(homeMarker);
    }
    homeMarker.setPosition(homePoint);
    invalidate();
  }

  public void setWaypoints(List<Waypoint> updatedWaypoints, int activeIndex) {
    // Remove previous waypoint markers and circles
    for (Marker marker : waypointMarkers) {
      getOverlays().remove(marker);
    }
    waypointMarkers.clear();

    for (Polygon circle : waypointAcceptanceCircles) {
      getOverlays().remove(circle);
    }
    waypointAcceptanceCircles.clear();

    if (missionPolyline != null) {
      getOverlays().remove(missionPolyline);
      missionPolyline = null;
    }

    if (updatedWaypoints != null && !updatedWaypoints.isEmpty()) {
      List<GeoPoint> pathPoints = new ArrayList<>();

      for (int i = 0; i < updatedWaypoints.size(); i++) {
        Waypoint wp = updatedWaypoints.get(i);
        GeoPoint point = new GeoPoint(wp.getLatitude(), wp.getLongitude());
        pathPoints.add(point);

        boolean isActive = i == activeIndex;

        // Acceptance radius circle
        Polygon circle = new Polygon(this);
        circle.setPoints(
            Polygon.pointsAsCircle(point, Math.max(1.0, wp.getAcceptanceRadiusMeters())));
        circle.setFillColor(
            isActive ? Color.parseColor("#33FFB300") : Color.parseColor("#330288D1"));
        circle.setStrokeColor(
            isActive ? Color.parseColor("#80FFB300") : Color.parseColor("#800288D1"));
        circle.setStrokeWidth(2f);
        getOverlays().add(circle);
        waypointAcceptanceCircles.add(circle);

        // Waypoint Marker
        Marker wpMarker = new Marker(this);
        wpMarker.setPosition(point);
        wpMarker.setTitle("WP " + (i + 1) + (isActive ? " (Active)" : ""));
        wpMarker.setIcon(createWaypointDrawable(getContext(), i + 1, isActive));
        wpMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
        getOverlays().add(wpMarker);
        waypointMarkers.add(wpMarker);
      }

      // Mission Path Polyline
      missionPolyline = new Polyline();
      missionPolyline.setPoints(pathPoints);
      missionPolyline.setColor(Color.parseColor("#FF0288D1"));
      missionPolyline.setWidth(6f);
      getOverlays().add(1, missionPolyline);
    }

    invalidate();
  }

  public void setTrack(List<double[]> trackPoints) {
    if (trackPolyline != null) {
      getOverlays().remove(trackPolyline);
      trackPolyline = null;
    }

    if (trackPoints != null && !trackPoints.isEmpty()) {
      List<GeoPoint> geoPoints = new ArrayList<>();
      for (double[] pt : trackPoints) {
        geoPoints.add(new GeoPoint(pt[0], pt[1]));
      }

      trackPolyline = new Polyline();
      trackPolyline.setPoints(geoPoints);
      trackPolyline.setColor(Color.parseColor("#FF00E676")); // Bright Green breadcrumbs
      trackPolyline.setWidth(5f);
      getOverlays().add(1, trackPolyline);
    }

    invalidate();
  }

  // region Custom Marker Icon Generators

  private Drawable createWaypointDrawable(Context context, int number, boolean isActive) {
    int size = 72;
    Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(bitmap);

    Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    fillPaint.setStyle(Paint.Style.FILL);
    fillPaint.setColor(isActive ? Color.parseColor("#FFFFB300") : Color.parseColor("#FF0288D1"));

    Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    strokePaint.setStyle(Paint.Style.STROKE);
    strokePaint.setStrokeWidth(4f);
    strokePaint.setColor(Color.WHITE);

    Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    textPaint.setColor(Color.WHITE);
    textPaint.setTextSize(32f);
    textPaint.setTextAlign(Paint.Align.CENTER);
    textPaint.setFakeBoldText(true);

    canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, fillPaint);
    canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, strokePaint);

    float yPos = (size / 2f) - ((textPaint.descent() + textPaint.ascent()) / 2f);
    canvas.drawText(String.valueOf(number), size / 2f, yPos, textPaint);

    return new BitmapDrawable(context.getResources(), bitmap);
  }

  private Drawable createRobotDrawable(Context context) {
    int size = 80;
    Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(bitmap);

    Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    fillPaint.setStyle(Paint.Style.FILL);
    fillPaint.setColor(Color.parseColor("#FF00E5FF"));

    Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    strokePaint.setStyle(Paint.Style.STROKE);
    strokePaint.setStrokeWidth(4f);
    strokePaint.setColor(Color.WHITE);

    Path arrowPath = new Path();
    arrowPath.moveTo(size / 2f, 8f);
    arrowPath.lineTo(size - 14f, size - 14f);
    arrowPath.lineTo(size / 2f, size - 26f);
    arrowPath.lineTo(14f, size - 14f);
    arrowPath.close();

    canvas.drawPath(arrowPath, fillPaint);
    canvas.drawPath(arrowPath, strokePaint);

    return new BitmapDrawable(context.getResources(), bitmap);
  }

  private Drawable createHomeDrawable(Context context) {
    int size = 72;
    Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(bitmap);

    Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    fillPaint.setStyle(Paint.Style.FILL);
    fillPaint.setColor(Color.parseColor("#FFFF6D00"));

    Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    strokePaint.setStyle(Paint.Style.STROKE);
    strokePaint.setStrokeWidth(4f);
    strokePaint.setColor(Color.WHITE);

    Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    textPaint.setColor(Color.WHITE);
    textPaint.setTextSize(32f);
    textPaint.setTextAlign(Paint.Align.CENTER);
    textPaint.setFakeBoldText(true);

    canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, fillPaint);
    canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, strokePaint);

    float yPos = (size / 2f) - ((textPaint.descent() + textPaint.ascent()) / 2f);
    canvas.drawText("H", size / 2f, yPos, textPaint);

    return new BitmapDrawable(context.getResources(), bitmap);
  }

  // endregion
}
