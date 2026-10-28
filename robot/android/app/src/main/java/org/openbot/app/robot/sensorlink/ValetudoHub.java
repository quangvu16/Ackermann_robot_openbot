package org.openbot.app.robot.sensorlink;

/**
 * Hands the Rooms &amp; Zones screen to the Sensor link web server while the screen is open: the
 * server passes every {@code /api/v2/...} request (Valetudo's REST API) to it. With the screen
 * closed the API answers 503, as a Valetudo robot that is not ready would.
 */
public final class ValetudoHub {

  /** What the Rooms & Zones screen offers. May be called from any thread. */
  public interface Source {
    /** {status, contentType, body}. */
    String[] handle(String method, String path, String body);
  }

  private static final ValetudoHub INSTANCE = new ValetudoHub();

  public static ValetudoHub get() {
    return INSTANCE;
  }

  private volatile Source source;

  private ValetudoHub() {}

  public void setSource(Source s) {
    source = s;
  }

  public void clearSource(Source s) {
    if (source == s) source = null;
  }

  MiniWebServer.HttpResponse handle(String method, String path, String body) {
    Source s = source;
    if (s == null) {
      return MiniWebServer.HttpResponse.text(503, "text/plain",
          "Open Rooms & Zones on the phone to use the Valetudo API\n");
    }
    String[] r = s.handle(method, path, body);
    return MiniWebServer.HttpResponse.text(Integer.parseInt(r[0]), r[1], r[2]);
  }
}
