package org.openbot.app.robot.valetudo;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.MotionEvent;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.openbot.app.robot.slam.RoomMap;
import org.openbot.app.robot.slam.SlamMapView;

/**
 * The SLAM map with Valetudo-style layers on top: coloured rooms with their names, zones (blue),
 * no-go areas (red), virtual walls (red lines), home, and the lanes still to drive. In the zone,
 * no-go and wall tools a one-finger drag draws the shape; pinch still zooms.
 */
public class RoomsMapView extends SlamMapView {

  public enum Tool {
    SELECT,
    GO_TO,
    ZONE,
    NO_GO,
    WALL,
    HOME
  }

  public interface EditListener {
    /** A shape drawn with the ZONE, NO_GO or WALL tool, from corner (or end) A to B. */
    void onShapeDrawn(Tool tool, double ax, double ay, double bx, double by);
  }

  // Colours like Valetudo's map: soft room colours, blue zones, red restrictions.
  private static final int[] PALETTE = {
    0xFF19A1A1, 0xFF7AC037, 0xFFDF5618, 0xFFF7C841, 0xFF9966CC, 0xFF4A90E2, 0xFFE57373, 0xFF8D6E63,
  };

  public Tool tool = Tool.SELECT;
  private RoomMap rooms;
  private int roomsVersion = -1;
  private int drawnRoomsVersion = -2;
  private Set<Integer> selected = new HashSet<>();
  private int selectedHash;
  private int drawnSelectedHash = -1;
  private Bitmap roomBitmap;
  private final Map<Integer, double[]> centroids = new HashMap<>();
  private List<double[]> plan;
  private int[] selectedItem; // {kind, index} from RoomMap.itemAt
  private EditListener editListener;
  private double[] draft; // ax, ay, bx, by while dragging

  private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint stroke = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint bmp = new Paint();
  private final Matrix m = new Matrix();
  private final Path p = new Path();

  public RoomsMapView(Context c) {
    this(c, null);
  }

  public RoomsMapView(Context c, AttributeSet a) {
    super(c, a);
    stroke.setStyle(Paint.Style.STROKE);
    stroke.setStrokeCap(Paint.Cap.ROUND);
    text.setTextAlign(Paint.Align.CENTER);
    text.setFakeBoldText(true);
    bmp.setFilterBitmap(false);
    setContentDescription("Rooms and zones map");
  }

  public void setEditListener(EditListener l) {
    editListener = l;
  }

  /** Rooms, zones and restrictions; bump {@code version} whenever they change. */
  public void setRooms(RoomMap r, int version, Set<Integer> selectedRooms, int[] selectedItem, List<double[]> plan) {
    rooms = r;
    roomsVersion = version;
    selected = selectedRooms;
    selectedHash = selectedRooms.hashCode();
    this.selectedItem = selectedItem;
    this.plan = plan;
    invalidate();
  }

  @Override
  protected boolean onEditTouch(MotionEvent e) {
    if (tool != Tool.ZONE && tool != Tool.NO_GO && tool != Tool.WALL) return false;
    double x = toWorldX(e.getX()), y = toWorldY(e.getY());
    switch (e.getActionMasked()) {
      case MotionEvent.ACTION_DOWN:
        draft = new double[] {x, y, x, y};
        break;
      case MotionEvent.ACTION_MOVE:
        if (draft != null) {
          draft[2] = x;
          draft[3] = y;
        }
        break;
      case MotionEvent.ACTION_UP:
        if (draft != null) {
          double[] d = draft;
          draft = null;
          if (Math.hypot(d[2] - d[0], d[3] - d[1]) > 0.2 && editListener != null) {
            editListener.onShapeDrawn(tool, d[0], d[1], d[2], d[3]);
          }
        }
        break;
      default:
        draft = null; // cancel, or a second finger: zoom instead
        return false;
    }
    invalidate();
    return true;
  }

  @Override
  protected void drawOverlay(Canvas c, float dp) {
    RoomMap r = rooms;
    if (r == null) return;
    // Rooms: one colour each, the selected ones stronger.
    if (r.labels.length > 0 && r.w > 0) {
      if (roomBitmap == null || drawnRoomsVersion != roomsVersion || drawnSelectedHash != selectedHash) {
        drawnRoomsVersion = roomsVersion;
        drawnSelectedHash = selectedHash;
        if (roomBitmap == null || roomBitmap.getWidth() != r.w || roomBitmap.getHeight() != r.h) {
          roomBitmap = Bitmap.createBitmap(r.w, r.h, Bitmap.Config.ARGB_8888);
        }
        int[] px = new int[r.w * r.h];
        for (int y = 0; y < r.h; y++) {
          int row = (r.h - 1 - y) * r.w;
          for (int x = 0; x < r.w; x++) {
            int id = r.labels[y * r.w + x];
            if (id <= 0) continue;
            int col = PALETTE[(id - 1) % PALETTE.length];
            int alpha = selected.isEmpty() || selected.contains(id) ? 120 : 45;
            px[row + x] = (alpha << 24) | (col & 0x00FFFFFF);
          }
        }
        roomBitmap.setPixels(px, 0, r.w, 0, 0, r.w, r.h);
        centroids.clear();
        for (Integer id : r.names.keySet()) {
          double[] ce = r.centroid(id);
          if (ce != null) centroids.put(id, ce);
        }
      }
      float cell = (float) (r.res * getScale());
      m.reset();
      m.postScale(cell, cell);
      m.postTranslate(sx(r.originX), sy(r.originY + r.h * r.res));
      c.drawBitmap(roomBitmap, m, bmp);
    } else {
      roomBitmap = null;
      centroids.clear();
    }
    // Zones, no-go areas, walls.
    for (int i = 0; i < r.zones.size(); i++) polygon(c, r.zones.get(i), 0x332196F3, 0xFF1976D2, isSel(0, i), dp);
    for (int i = 0; i < r.noGo.size(); i++) polygon(c, r.noGo.get(i), 0x44E53935, 0xFFD32F2F, isSel(1, i), dp);
    for (int i = 0; i < r.walls.size(); i++) {
      double[] s = r.walls.get(i);
      stroke.setPathEffect(null);
      stroke.setColor(0xFFD32F2F);
      stroke.setStrokeWidth((isSel(2, i) ? 6 : 4) * dp);
      c.drawLine(sx(s[0]), sy(s[1]), sx(s[2]), sy(s[3]), stroke);
    }
    // Lanes still to drive.
    List<double[]> pl = plan;
    if (pl != null && pl.size() > 1) {
      stroke.setColor(0xCC6A1B9A);
      stroke.setStrokeWidth(1.5f * dp);
      stroke.setPathEffect(new DashPathEffect(new float[] {6 * dp, 4 * dp}, 0));
      p.reset();
      p.moveTo(sx(pl.get(0)[0]), sy(pl.get(0)[1]));
      for (int i = 1; i < pl.size(); i++) p.lineTo(sx(pl.get(i)[0]), sy(pl.get(i)[1]));
      c.drawPath(p, stroke);
      stroke.setPathEffect(null);
    }
    // Home.
    if (r.home != null) {
      float hx = sx(r.home[0]), hy = sy(r.home[1]);
      fill.setColor(0xFF2E7D32);
      c.drawCircle(hx, hy, 9 * dp, fill);
      text.setColor(Color.WHITE);
      text.setTextSize(11 * dp);
      c.drawText("H", hx, hy + 4 * dp, text);
    }
    // Room names.
    text.setTextSize(13 * dp);
    for (Map.Entry<Integer, double[]> e : centroids.entrySet()) {
      String name = r.name(e.getKey());
      float x = sx(e.getValue()[0]), y = sy(e.getValue()[1]);
      text.setColor(Color.WHITE);
      text.setStyle(Paint.Style.STROKE);
      text.setStrokeWidth(3 * dp);
      c.drawText(name, x, y, text);
      text.setStyle(Paint.Style.FILL);
      text.setColor(0xFF263238);
      c.drawText(name, x, y, text);
    }
    // The shape being drawn.
    double[] d = draft;
    if (d != null) {
      if (tool == Tool.WALL) {
        stroke.setColor(0xFFD32F2F);
        stroke.setStrokeWidth(4 * dp);
        c.drawLine(sx(d[0]), sy(d[1]), sx(d[2]), sy(d[3]), stroke);
      } else {
        double[] box = RoomMap.box(d[0], d[1], d[2], d[3]);
        if (tool == Tool.ZONE) polygon(c, box, 0x332196F3, 0xFF1976D2, true, dp);
        else polygon(c, box, 0x44E53935, 0xFFD32F2F, true, dp);
      }
    }
  }

  private boolean isSel(int kind, int i) {
    return selectedItem != null && selectedItem[0] == kind && selectedItem[1] == i;
  }

  private void polygon(Canvas c, double[] poly, int fillColor, int strokeColor, boolean sel, float dp) {
    p.reset();
    p.moveTo(sx(poly[0]), sy(poly[1]));
    for (int i = 1; i < poly.length / 2; i++) p.lineTo(sx(poly[2 * i]), sy(poly[2 * i + 1]));
    p.close();
    fill.setColor(fillColor);
    c.drawPath(p, fill);
    stroke.setPathEffect(sel ? new DashPathEffect(new float[] {8 * dp, 5 * dp}, 0) : null);
    stroke.setColor(strokeColor);
    stroke.setStrokeWidth((sel ? 3 : 2) * dp);
    c.drawPath(p, stroke);
    stroke.setPathEffect(null);
  }
}
