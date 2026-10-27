package org.openbot.app.robot.sensorlink;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Settings the browser page can read and change over the {@code /control} WebSocket.
 *
 * <p>Each setting describes itself (group, label, type, range), so the page draws whatever the
 * current screen offers: camera settings everywhere, driving settings on driving screens, the
 * tracking confidence only on Object Tracking. Providers add their settings while active.
 *
 * <pre>
 *   phone -> browser  {"screen":"GPS Mission","settings":[{"key":"camera.torch","group":"Camera",
 *                      "label":"Flashlight","type":"bool","value":false}, ...]}
 *   browser -> phone  {"set":"camera.torch","value":true}
 * </pre>
 *
 * Pure Java: getters and setters are called on whatever thread the caller is on; the Android side
 * calls them on the main thread.
 */
public final class RemoteSettings {

  public enum Type {
    BOOL("bool"),
    CHOICE("choice"),
    RANGE("range"),
    ACTION("action");

    final String json;

    Type(String json) {
      this.json = json;
    }
  }

  public interface Getter {
    Object get();
  }

  public interface Setter {
    void set(Object value);
  }

  /** One setting the page can show. */
  public static final class Setting {
    public final String key;
    public final String group;
    public final String label;
    public final Type type;
    final List<String> options;
    final double min;
    final double max;
    final double step;
    final Getter getter;
    final Setter setter;

    private Setting(
        String key, String group, String label, Type type, List<String> options,
        double min, double max, double step, Getter getter, Setter setter) {
      this.key = key;
      this.group = group;
      this.label = label;
      this.type = type;
      this.options = options;
      this.min = min;
      this.max = max;
      this.step = step;
      this.getter = getter;
      this.setter = setter;
    }

    public static Setting bool(String key, String group, String label, Getter g, Setter s) {
      return new Setting(key, group, label, Type.BOOL, Collections.emptyList(), 0, 0, 0, g, s);
    }

    public static Setting choice(
        String key, String group, String label, List<String> options, Getter g, Setter s) {
      return new Setting(key, group, label, Type.CHOICE, options, 0, 0, 0, g, s);
    }

    public static Setting range(
        String key, String group, String label, double min, double max, double step, Getter g, Setter s) {
      return new Setting(key, group, label, Type.RANGE, Collections.emptyList(), min, max, step, g, s);
    }

    /** A button; {@code options} are its two labels (value false, value true), e.g. STOP / RESUME. */
    public static Setting action(
        String key, String group, String label, String offLabel, String onLabel, Getter g, Setter s) {
      return new Setting(key, group, label, Type.ACTION, Arrays.asList(offLabel, onLabel), 0, 0, 0, g, s);
    }

    String toJson() {
      StringBuilder b = new StringBuilder(128);
      b.append("{\"key\":\"").append(SensorLinkProtocol.escapeJson(key));
      b.append("\",\"group\":\"").append(SensorLinkProtocol.escapeJson(group));
      b.append("\",\"label\":\"").append(SensorLinkProtocol.escapeJson(label));
      b.append("\",\"type\":\"").append(type.json).append('"');
      if (!options.isEmpty()) {
        b.append(",\"options\":[");
        for (int i = 0; i < options.size(); i++) {
          if (i > 0) b.append(',');
          b.append('"').append(SensorLinkProtocol.escapeJson(options.get(i))).append('"');
        }
        b.append(']');
      }
      if (type == Type.RANGE) {
        b.append(",\"min\":").append(num(min)).append(",\"max\":").append(num(max));
        b.append(",\"step\":").append(num(step));
      }
      b.append(",\"value\":").append(valueJson(getter.get())).append('}');
      return b.toString();
    }

    /** Converts what the page sent to this setting's type; null when it does not fit. */
    Object coerce(Object raw) {
      switch (type) {
        case BOOL:
        case ACTION:
          if (raw instanceof Boolean) return raw;
          if ("true".equals(raw)) return Boolean.TRUE;
          if ("false".equals(raw)) return Boolean.FALSE;
          return null;
        case CHOICE:
          String s = raw == null ? null : String.valueOf(raw);
          return options.contains(s) ? s : null;
        case RANGE:
          double d;
          if (raw instanceof Number) d = ((Number) raw).doubleValue();
          else {
            try {
              d = Double.parseDouble(String.valueOf(raw));
            } catch (NumberFormatException e) {
              return null;
            }
          }
          if (Double.isNaN(d)) return null;
          return Math.max(min, Math.min(max, d));
        default:
          return null;
      }
    }
  }

  /** Something that contributes settings while it is active (a screen, the camera...). */
  public interface Provider {
    void addSettings(List<Setting> out);
  }

  private final List<Provider> providers = new CopyOnWriteArrayList<>();

  public void addProvider(Provider p) {
    if (!providers.contains(p)) providers.add(p);
  }

  public void removeProvider(Provider p) {
    providers.remove(p);
  }

  public List<Setting> list() {
    List<Setting> out = new ArrayList<>();
    for (Provider p : providers) p.addSettings(out);
    return out;
  }

  public String stateJson(String screen) {
    StringBuilder b = new StringBuilder(1024);
    b.append("{\"screen\":\"").append(SensorLinkProtocol.escapeJson(screen)).append("\",\"settings\":[");
    List<Setting> all = list();
    for (int i = 0; i < all.size(); i++) {
      if (i > 0) b.append(',');
      b.append(all.get(i).toJson());
    }
    return b.append("]}").toString();
  }

  /** Applies a message from the page. Returns false if it was not a known, valid setting. */
  public boolean apply(String message) {
    String key = stringField(message, "set");
    if (key == null) return false;
    Object raw = valueField(message);
    for (Setting s : list()) {
      if (s.key.equals(key)) {
        Object v = s.coerce(raw);
        if (v == null) return false;
        s.setter.set(v);
        return true;
      }
    }
    return false;
  }

  // region tiny JSON helpers

  private static final Pattern VALUE =
      Pattern.compile("\"value\"\\s*:\\s*(\"((?:[^\"\\\\]|\\\\.)*)\"|true|false|-?[0-9][0-9.eE+-]*)");

  static String stringField(String json, String name) {
    Matcher m =
        Pattern.compile("\"" + Pattern.quote(name) + "\"\\s*:\\s*\"((?:[^\"\\\\]|\\\\.)*)\"").matcher(json);
    return m.find() ? m.group(1).replace("\\\"", "\"").replace("\\\\", "\\") : null;
  }

  static Object valueField(String json) {
    Matcher m = VALUE.matcher(json);
    if (!m.find()) return null;
    String v = m.group(1);
    if (m.group(2) != null) return m.group(2).replace("\\\"", "\"").replace("\\\\", "\\");
    if ("true".equals(v)) return Boolean.TRUE;
    if ("false".equals(v)) return Boolean.FALSE;
    try {
      return Double.parseDouble(v);
    } catch (NumberFormatException e) {
      return null;
    }
  }

  static String valueJson(Object v) {
    if (v == null) return "null";
    if (v instanceof Boolean) return v.toString();
    if (v instanceof Number) return num(((Number) v).doubleValue());
    return "\"" + SensorLinkProtocol.escapeJson(String.valueOf(v)) + "\"";
  }

  private static String num(double d) {
    if (Double.isNaN(d) || Double.isInfinite(d)) return "0";
    if (d == Math.rint(d) && Math.abs(d) < 1e15) return String.valueOf((long) d);
    return String.format(Locale.US, "%.4f", d).replaceAll("0+$", "");
  }

  // endregion
}
