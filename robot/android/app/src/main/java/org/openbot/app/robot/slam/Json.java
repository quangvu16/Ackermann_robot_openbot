package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A very small JSON reader and string quoting, so the Rooms &amp; Zones code (and its REST API)
 * stays plain Java and testable on the JVM without Android's org.json. Objects become {@code
 * Map<String, Object>}, arrays {@code List<Object>}, numbers {@code Double}.
 */
public final class Json {

  private Json() {}

  public static Object parse(String s) {
    if (s == null) return null;
    Json.Reader r = new Json.Reader(s);
    r.ws();
    if (r.i >= s.length()) return null;
    Object v = r.value();
    r.ws();
    if (r.i != s.length()) throw new IllegalArgumentException("Trailing characters at " + r.i);
    return v;
  }

  /** Parse, or null when the text is not valid JSON. */
  public static Object tryParse(String s) {
    try {
      return parse(s);
    } catch (RuntimeException e) {
      return null;
    }
  }

  @SuppressWarnings("unchecked")
  public static Map<String, Object> obj(Object o) {
    return o instanceof Map ? (Map<String, Object>) o : null;
  }

  @SuppressWarnings("unchecked")
  public static List<Object> arr(Object o) {
    return o instanceof List ? (List<Object>) o : null;
  }

  public static double num(Object o, double def) {
    return o instanceof Number ? ((Number) o).doubleValue() : def;
  }

  public static String str(Object o) {
    return o instanceof String ? (String) o : o == null ? null : String.valueOf(o);
  }

  /** Follow keys into nested objects: {@code get(body, "coordinates", "x")}. */
  public static Object get(Object o, String... keys) {
    for (String k : keys) {
      Map<String, Object> m = obj(o);
      if (m == null) return null;
      o = m.get(k);
    }
    return o;
  }

  public static String quote(String s) {
    if (s == null) return "null";
    StringBuilder b = new StringBuilder(s.length() + 2).append('"');
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      switch (c) {
        case '"':
          b.append("\\\"");
          break;
        case '\\':
          b.append("\\\\");
          break;
        case '\n':
          b.append("\\n");
          break;
        case '\r':
          b.append("\\r");
          break;
        case '\t':
          b.append("\\t");
          break;
        default:
          if (c < 0x20) b.append(String.format("\\u%04x", (int) c));
          else b.append(c);
      }
    }
    return b.append('"').toString();
  }

  private static final class Reader {
    final String s;
    int i;

    Reader(String s) {
      this.s = s;
    }

    void ws() {
      while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
    }

    Object value() {
      ws();
      if (i >= s.length()) throw new IllegalArgumentException("Unexpected end");
      char c = s.charAt(i);
      if (c == '{') return object();
      if (c == '[') return array();
      if (c == '"') return string();
      if (s.startsWith("true", i)) {
        i += 4;
        return Boolean.TRUE;
      }
      if (s.startsWith("false", i)) {
        i += 5;
        return Boolean.FALSE;
      }
      if (s.startsWith("null", i)) {
        i += 4;
        return null;
      }
      return number();
    }

    Map<String, Object> object() {
      Map<String, Object> m = new LinkedHashMap<>();
      i++; // {
      ws();
      if (peek() == '}') {
        i++;
        return m;
      }
      while (true) {
        ws();
        String k = string();
        ws();
        expect(':');
        m.put(k, value());
        ws();
        char c = next();
        if (c == '}') return m;
        if (c != ',') throw new IllegalArgumentException("Expected , or } at " + (i - 1));
      }
    }

    List<Object> array() {
      List<Object> l = new ArrayList<>();
      i++; // [
      ws();
      if (peek() == ']') {
        i++;
        return l;
      }
      while (true) {
        l.add(value());
        ws();
        char c = next();
        if (c == ']') return l;
        if (c != ',') throw new IllegalArgumentException("Expected , or ] at " + (i - 1));
      }
    }

    String string() {
      expect('"');
      StringBuilder b = new StringBuilder();
      while (true) {
        char c = next();
        if (c == '"') return b.toString();
        if (c != '\\') {
          b.append(c);
          continue;
        }
        char e = next();
        switch (e) {
          case 'n':
            b.append('\n');
            break;
          case 't':
            b.append('\t');
            break;
          case 'r':
            b.append('\r');
            break;
          case 'b':
            b.append('\b');
            break;
          case 'f':
            b.append('\f');
            break;
          case 'u':
            if (i + 4 > s.length()) throw new IllegalArgumentException("Bad \\u escape");
            b.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
            i += 4;
            break;
          default:
            b.append(e); // \" \\ \/
        }
      }
    }

    Double number() {
      int start = i;
      while (i < s.length() && "+-0123456789.eE".indexOf(s.charAt(i)) >= 0) i++;
      if (start == i) throw new IllegalArgumentException("Unexpected character at " + i);
      return Double.parseDouble(s.substring(start, i));
    }

    char peek() {
      return i < s.length() ? s.charAt(i) : '\0';
    }

    char next() {
      if (i >= s.length()) throw new IllegalArgumentException("Unexpected end");
      return s.charAt(i++);
    }

    void expect(char c) {
      if (next() != c) throw new IllegalArgumentException("Expected " + c + " at " + (i - 1));
    }
  }
}
