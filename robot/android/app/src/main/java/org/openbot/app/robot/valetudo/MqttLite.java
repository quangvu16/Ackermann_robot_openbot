package org.openbot.app.robot.valetudo;

import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * A small MQTT 3.1.1 client (OASIS standard, docs.oasis-open.org/mqtt/mqtt/v3.1.1): plain TCP,
 * QoS 0 publish and subscribe, retained messages, a last-will message, keep-alive pings and
 * reconnecting every 5 s. Enough for Valetudo-style topics and Home Assistant discovery without a
 * library. Plain Java; the packet encoders are unit tested.
 */
public final class MqttLite {

  public interface Listener {
    void onConnected();

    void onMessage(String topic, String payload);

    void onStatus(String text);
  }

  public static final class Config {
    public String host = "";
    public int port = 1883;
    public String clientId = "openbot";
    public String username = "";
    public String password = "";
    public int keepAliveSeconds = 30;
    public String willTopic;
    public String willPayload;
    public final List<String> subscriptions = new ArrayList<>();
  }

  private final Config config;
  private final Listener listener;
  private volatile boolean running;
  private volatile Socket socket;
  private volatile OutputStream out;
  private volatile boolean connected;
  private Thread thread;
  private int packetId = 1;

  public MqttLite(Config config, Listener listener) {
    this.config = config;
    this.listener = listener;
  }

  public boolean isConnected() {
    return connected;
  }

  public synchronized void start() {
    if (running) return;
    running = true;
    thread = new Thread(this::loop, "MqttLite");
    thread.setDaemon(true);
    thread.start();
  }

  public void stop() {
    running = false;
    OutputStream o = out;
    if (o != null && connected) {
      try {
        synchronized (this) {
          o.write(new byte[] {(byte) 0xE0, 0}); // DISCONNECT: no last will
          o.flush();
        }
      } catch (IOException ignored) {
        // closing anyway
      }
    }
    closeSocket();
    if (thread != null) thread.interrupt();
  }

  /** QoS 0 publish; dropped quietly while disconnected. */
  public void publish(String topic, String payload, boolean retain) {
    OutputStream o = out;
    if (o == null || !connected) return;
    try {
      byte[] p = publishPacket(topic, payload, retain);
      synchronized (this) {
        o.write(p);
        o.flush();
      }
    } catch (IOException e) {
      closeSocket();
    }
  }

  private void loop() {
    while (running) {
      try {
        listener.onStatus("Connecting to " + config.host + ":" + config.port);
        Socket s = new Socket();
        s.connect(new InetSocketAddress(config.host, config.port), 5000);
        s.setSoTimeout(1000);
        socket = s;
        OutputStream o = s.getOutputStream();
        DataInputStream in = new DataInputStream(s.getInputStream());
        o.write(connectPacket(config));
        o.flush();
        int[] head = readHeader(in, true);
        byte[] ack = readFully(in, head[1]);
        if ((head[0] & 0xF0) != 0x20 || ack.length < 2) throw new IOException("No CONNACK");
        if (ack[1] != 0) throw new IOException(connackText(ack[1]));
        out = o;
        connected = true;
        for (String topic : config.subscriptions) {
          synchronized (this) {
            o.write(subscribePacket(packetId++ & 0xFFFF, topic));
          }
        }
        o.flush();
        listener.onStatus("Connected to " + config.host);
        listener.onConnected();
        long lastSent = System.currentTimeMillis();
        while (running) {
          int[] h;
          try {
            h = readHeader(in, false);
          } catch (SocketTimeoutException e) {
            h = null;
          }
          if (h != null) {
            byte[] body = readFully(in, h[1]);
            if ((h[0] & 0xF0) == 0x30) handlePublish(h[0], body);
          }
          long now = System.currentTimeMillis();
          if (now - lastSent > config.keepAliveSeconds * 500L) { // ping at half the keep-alive
            synchronized (this) {
              o.write(new byte[] {(byte) 0xC0, 0});
              o.flush();
            }
            lastSent = now;
          }
        }
      } catch (IOException e) {
        listener.onStatus("MQTT: " + (e.getMessage() == null ? e.toString() : e.getMessage()));
      } finally {
        connected = false;
        out = null;
        closeSocket();
      }
      if (!running) break;
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        break;
      }
    }
  }

  private void handlePublish(int flags, byte[] body) {
    int topicLen = ((body[0] & 0xFF) << 8) | (body[1] & 0xFF);
    String topic = new String(body, 2, topicLen, StandardCharsets.UTF_8);
    int at = 2 + topicLen;
    if (((flags >> 1) & 3) > 0) at += 2; // QoS 1/2 carry a packet id (we subscribe at QoS 0)
    String payload = new String(body, at, body.length - at, StandardCharsets.UTF_8);
    try {
      listener.onMessage(topic, payload);
    } catch (RuntimeException e) {
      listener.onStatus("MQTT message failed: " + e.getMessage());
    }
  }

  private void closeSocket() {
    Socket s = socket;
    socket = null;
    if (s != null) {
      try {
        s.close();
      } catch (IOException ignored) {
        // already closed
      }
    }
  }

  private static String connackText(int code) {
    switch (code) {
      case 1:
        return "Broker refused: protocol version";
      case 2:
        return "Broker refused: client id";
      case 3:
        return "Broker unavailable";
      case 4:
        return "Broker refused: bad user name or password";
      case 5:
        return "Broker refused: not authorised";
      default:
        return "Broker refused: code " + code;
    }
  }

  // region Packets

  /** {type byte, remaining length}; with {@code block} false a timeout before the first byte throws. */
  private static int[] readHeader(DataInputStream in, boolean block) throws IOException {
    int type;
    while (true) {
      try {
        type = in.readUnsignedByte();
        break;
      } catch (SocketTimeoutException e) {
        if (!block) throw e;
      }
    }
    int mult = 1, len = 0, b;
    do {
      b = readByte(in);
      len += (b & 0x7F) * mult;
      mult *= 128;
      if (mult > 128 * 128 * 128 * 128) throw new IOException("Bad length");
    } while ((b & 0x80) != 0);
    return new int[] {type, len};
  }

  private static int readByte(DataInputStream in) throws IOException {
    while (true) {
      try {
        return in.readUnsignedByte();
      } catch (SocketTimeoutException e) {
        // mid-packet: keep waiting
      }
    }
  }

  private static byte[] readFully(DataInputStream in, int n) throws IOException {
    byte[] b = new byte[n];
    int at = 0;
    while (at < n) {
      try {
        int r = in.read(b, at, n - at);
        if (r < 0) throw new IOException("Connection closed");
        at += r;
      } catch (SocketTimeoutException e) {
        // keep reading
      }
    }
    return b;
  }

  static byte[] remainingLength(int len) {
    ByteArrayOutputStream b = new ByteArrayOutputStream();
    do {
      int d = len % 128;
      len /= 128;
      if (len > 0) d |= 0x80;
      b.write(d);
    } while (len > 0);
    return b.toByteArray();
  }

  private static void str(ByteArrayOutputStream b, String s) {
    byte[] u = s.getBytes(StandardCharsets.UTF_8);
    b.write(u.length >> 8);
    b.write(u.length & 0xFF);
    b.write(u, 0, u.length);
  }

  private static byte[] packet(int type, ByteArrayOutputStream body) {
    byte[] v = body.toByteArray();
    byte[] len = remainingLength(v.length);
    byte[] p = new byte[1 + len.length + v.length];
    p[0] = (byte) type;
    System.arraycopy(len, 0, p, 1, len.length);
    System.arraycopy(v, 0, p, 1 + len.length, v.length);
    return p;
  }

  static byte[] connectPacket(Config c) {
    ByteArrayOutputStream b = new ByteArrayOutputStream();
    str(b, "MQTT");
    b.write(4); // protocol level 3.1.1
    int flags = 0x02; // clean session
    boolean will = c.willTopic != null && c.willPayload != null;
    if (will) flags |= 0x04 | 0x20; // will, retained, QoS 0
    boolean user = c.username != null && !c.username.isEmpty();
    boolean pass = user && c.password != null && !c.password.isEmpty();
    if (user) flags |= 0x80;
    if (pass) flags |= 0x40;
    b.write(flags);
    b.write(c.keepAliveSeconds >> 8);
    b.write(c.keepAliveSeconds & 0xFF);
    str(b, c.clientId);
    if (will) {
      str(b, c.willTopic);
      str(b, c.willPayload);
    }
    if (user) str(b, c.username);
    if (pass) str(b, c.password);
    return packet(0x10, b);
  }

  static byte[] publishPacket(String topic, String payload, boolean retain) {
    ByteArrayOutputStream b = new ByteArrayOutputStream();
    str(b, topic);
    byte[] p = payload.getBytes(StandardCharsets.UTF_8);
    b.write(p, 0, p.length);
    return packet(0x30 | (retain ? 1 : 0), b);
  }

  static byte[] subscribePacket(int id, String filter) {
    ByteArrayOutputStream b = new ByteArrayOutputStream();
    b.write(id >> 8);
    b.write(id & 0xFF);
    str(b, filter);
    b.write(0); // QoS 0
    return packet(0x82, b);
  }

  // endregion
}
