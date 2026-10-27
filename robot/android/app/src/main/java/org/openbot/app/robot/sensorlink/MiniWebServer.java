package org.openbot.app.robot.sensorlink;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;

/**
 * A small HTTP + WebSocket (RFC 6455) server on one port: plain GET requests get a page (the
 * browser dashboard), upgrade requests become WebSocket connections (the SensorServer API).
 *
 * <p>Pure Java (no Android classes), so it is tested on the desktop JVM. Each connection has a
 * reader thread and a writer thread with a bounded queue: a slow browser is dropped instead of
 * stalling the sensor thread that feeds every client.
 */
public abstract class MiniWebServer {

  /** Writes a long-running response body, e.g. an MJPEG camera stream. Return to end it. */
  public interface Streamer {
    void stream(OutputStream out) throws IOException;
  }

  /** A plain HTTP answer, or a streamed one when {@code streamer} is set. */
  public static final class HttpResponse {
    final int status;
    final String contentType;
    final byte[] body;
    final Streamer streamer;

    public HttpResponse(int status, String contentType, byte[] body) {
      this.status = status;
      this.contentType = contentType;
      this.body = body;
      this.streamer = null;
    }

    private HttpResponse(String contentType, Streamer streamer) {
      this.status = 200;
      this.contentType = contentType;
      this.body = new byte[0];
      this.streamer = streamer;
    }

    public static HttpResponse stream(String contentType, Streamer streamer) {
      return new HttpResponse(contentType, streamer);
    }

    public static HttpResponse text(int status, String contentType, String body) {
      return new HttpResponse(status, contentType, body.getBytes(StandardCharsets.UTF_8));
    }
  }

  private static final String GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  private static final int MAX_HEADER = 16 * 1024;
  private static final int MAX_FRAME = 1 << 20;
  private static final int SEND_QUEUE = 256;

  private final int port;
  private volatile ServerSocket serverSocket;
  private volatile boolean running;
  private final Set<Connection> connections = ConcurrentHashMap.newKeySet();
  private final Set<Socket> streams = ConcurrentHashMap.newKeySet();

  protected MiniWebServer(int port) {
    this.port = port;
  }

  /** A WebSocket client. {@link #send} may be called from any thread. */
  public final class Connection {
    private final Socket socket;
    private final BlockingQueue<byte[]> queue = new ArrayBlockingQueue<>(SEND_QUEUE);
    private volatile boolean open = true;
    private volatile boolean closeQueued;
    /** Free for the subclass to keep per-connection state in. */
    public volatile Object tag;

    Connection(Socket socket) {
      this.socket = socket;
    }

    public boolean isOpen() {
      return open && !closeQueued;
    }

    public void send(String text) {
      if (!isOpen()) return;
      if (!queue.offer(frame(0x1, text.getBytes(StandardCharsets.UTF_8)))) {
        // The client is not keeping up; drop it rather than buffer without end.
        close(1008, "client too slow");
      }
    }

    public void close(int code, String reason) {
      if (!open || closeQueued) return;
      closeQueued = true;
      byte[] r = reason == null ? new byte[0] : reason.getBytes(StandardCharsets.UTF_8);
      int n = Math.min(r.length, 120);
      byte[] payload = new byte[2 + n];
      payload[0] = (byte) (code >> 8);
      payload[1] = (byte) code;
      System.arraycopy(r, 0, payload, 2, n);
      queue.clear();
      queue.offer(frame(0x8, payload));
    }

    private void writeLoop(OutputStream out) {
      try {
        while (open) {
          byte[] f = queue.poll(500, TimeUnit.MILLISECONDS);
          if (f == null) continue;
          out.write(f);
          out.flush();
          if ((f[0] & 0x0F) == 0x8) break; // close frame sent: done
        }
      } catch (IOException | InterruptedException e) {
        // fall through to shutdown
      }
      shutdown();
    }

    private void shutdown() {
      boolean wasOpen;
      synchronized (this) {
        wasOpen = open;
        open = false;
      }
      try {
        socket.close();
      } catch (IOException ignored) {
      }
      if (wasOpen && connections.remove(this)) {
        onClose(this);
      }
    }
  }

  // region Lifecycle

  /** Starts listening on a background thread; reports through onStarted / onFailed. */
  public void start() {
    running = true;
    Thread t = new Thread(this::acceptLoop, "MiniWebServer:" + port);
    t.setDaemon(true);
    t.start();
  }

  /** Closes the port and every connection. */
  public void stop() {
    running = false;
    ServerSocket s = serverSocket;
    if (s != null) {
      try {
        s.close();
      } catch (IOException ignored) {
      }
    }
    for (Connection c : connections) {
      c.close(1001, "server stopped");
    }
    for (Socket st : streams) {
      try {
        st.close();
      } catch (IOException ignored) {
      }
    }
  }

  public boolean isRunning() {
    return running;
  }

  public int getConnectionCount() {
    return connections.size();
  }

  private void acceptLoop() {
    try (ServerSocket s = new ServerSocket()) {
      s.setReuseAddress(true);
      s.bind(new InetSocketAddress(port));
      serverSocket = s;
      onStarted();
      while (running) {
        Socket client = s.accept();
        Thread t = new Thread(() -> handle(client), "MiniWebServer-client");
        t.setDaemon(true);
        t.start();
      }
    } catch (IOException e) {
      if (running) onFailed(e);
    }
  }

  // endregion

  // region Request handling

  private void handle(Socket socket) {
    try {
      socket.setTcpNoDelay(true);
      InputStream in = socket.getInputStream();
      OutputStream out = socket.getOutputStream();
      String head = readHead(in);
      if (head == null) {
        socket.close();
        return;
      }
      String[] lines = head.split("\r\n");
      String[] requestLine = lines[0].split(" ");
      if (requestLine.length < 2 || !"GET".equals(requestLine[0])) {
        writeHttp(out, HttpResponse.text(405, "text/plain", "Only GET is supported\n"));
        socket.close();
        return;
      }
      String path = requestLine[1];
      Map<String, String> headers = new HashMap<>();
      for (int i = 1; i < lines.length; i++) {
        int colon = lines[i].indexOf(':');
        if (colon > 0) {
          headers.put(
              lines[i].substring(0, colon).trim().toLowerCase(Locale.ROOT),
              lines[i].substring(colon + 1).trim());
        }
      }
      String key = headers.get("sec-websocket-key");
      String upgrade = headers.get("upgrade");
      if (key == null || upgrade == null || !upgrade.equalsIgnoreCase("websocket")) {
        HttpResponse r = onHttpRequest(path);
        if (r == null) r = HttpResponse.text(404, "text/plain", "Not found\n");
        if (r.streamer != null) {
          streams.add(socket);
          try {
            out.write(
                ("HTTP/1.1 200 OK\r\nContent-Type: "
                        + r.contentType
                        + "\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *"
                        + "\r\nConnection: close\r\n\r\n")
                    .getBytes(StandardCharsets.US_ASCII));
            r.streamer.stream(out);
          } finally {
            streams.remove(socket);
          }
        } else {
          writeHttp(out, r);
        }
        socket.close();
        return;
      }
      String accept = acceptKey(key);
      out.write(
          ("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                  + "Sec-WebSocket-Accept: "
                  + accept
                  + "\r\n\r\n")
              .getBytes(StandardCharsets.US_ASCII));
      out.flush();

      Connection c = new Connection(socket);
      connections.add(c);
      Thread writer = new Thread(() -> c.writeLoop(out), "MiniWebServer-writer");
      writer.setDaemon(true);
      writer.start();
      onOpen(c, path);
      readFrames(c, in);
      c.shutdown();
    } catch (IOException e) {
      try {
        socket.close();
      } catch (IOException ignored) {
      }
    }
  }

  private static String readHead(InputStream in) throws IOException {
    ByteArrayOutputStream b = new ByteArrayOutputStream();
    int state = 0; // counts \r\n\r\n
    while (b.size() < MAX_HEADER) {
      int ch = in.read();
      if (ch < 0) return null;
      b.write(ch);
      if ((state % 2 == 0 && ch == '\r') || (state % 2 == 1 && ch == '\n')) {
        state++;
        if (state == 4) {
          String s = b.toString("ISO-8859-1");
          return s.substring(0, s.length() - 4);
        }
      } else {
        state = ch == '\r' ? 1 : 0;
      }
    }
    return null;
  }

  private void readFrames(Connection c, InputStream in) throws IOException {
    while (c.open) {
      int b0 = in.read();
      int b1 = in.read();
      if (b0 < 0 || b1 < 0) return;
      int opcode = b0 & 0x0F;
      boolean masked = (b1 & 0x80) != 0;
      long len = b1 & 0x7F;
      if (len == 126) {
        len = ((long) readByte(in) << 8) | readByte(in);
      } else if (len == 127) {
        len = 0;
        for (int i = 0; i < 8; i++) len = (len << 8) | readByte(in);
      }
      if (len > MAX_FRAME) return;
      byte[] mask = new byte[4];
      if (masked) readFully(in, mask);
      byte[] payload = new byte[(int) len];
      readFully(in, payload);
      if (masked) {
        for (int i = 0; i < payload.length; i++) payload[i] ^= mask[i & 3];
      }
      switch (opcode) {
        case 0x1:
          onMessage(c, new String(payload, StandardCharsets.UTF_8));
          break;
        case 0x8:
          c.close(payload.length >= 2 ? ((payload[0] & 0xFF) << 8) | (payload[1] & 0xFF) : 1000, null);
          return;
        case 0x9:
          c.queue.offer(frame(0xA, payload));
          break;
        default:
          break; // pong, binary, continuation: not used by SensorServer clients
      }
    }
  }

  private static int readByte(InputStream in) throws IOException {
    int b = in.read();
    if (b < 0) throw new SocketException("closed");
    return b;
  }

  private static void readFully(InputStream in, byte[] buf) throws IOException {
    int off = 0;
    while (off < buf.length) {
      int n = in.read(buf, off, buf.length - off);
      if (n < 0) throw new SocketException("closed");
      off += n;
    }
  }

  private static void writeHttp(OutputStream out, HttpResponse r) throws IOException {
    String reason = r.status == 200 ? "OK" : r.status == 404 ? "Not Found" : "Error";
    String head =
        "HTTP/1.1 "
            + r.status
            + " "
            + reason
            + "\r\nContent-Type: "
            + r.contentType
            + "\r\nContent-Length: "
            + r.body.length
            + "\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
    out.write(head.getBytes(StandardCharsets.US_ASCII));
    out.write(r.body);
    out.flush();
  }

  /** Server frames are never masked. */
  static byte[] frame(int opcode, byte[] payload) {
    int n = payload.length;
    int headerLen = n < 126 ? 2 : n <= 0xFFFF ? 4 : 10;
    byte[] f = new byte[headerLen + n];
    f[0] = (byte) (0x80 | opcode);
    if (n < 126) {
      f[1] = (byte) n;
    } else if (n <= 0xFFFF) {
      f[1] = 126;
      f[2] = (byte) (n >> 8);
      f[3] = (byte) n;
    } else {
      f[1] = 127;
      for (int i = 0; i < 8; i++) f[2 + i] = (byte) ((long) n >> (56 - 8 * i));
    }
    System.arraycopy(payload, 0, f, headerLen, n);
    return f;
  }

  static String acceptKey(String key) {
    try {
      MessageDigest sha1 = MessageDigest.getInstance("SHA-1");
      byte[] d = sha1.digest((key + GUID).getBytes(StandardCharsets.US_ASCII));
      return base64(d);
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  private static final char[] B64 =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".toCharArray();

  /** java.util.Base64 needs API 26; this app supports 24. */
  static String base64(byte[] d) {
    StringBuilder sb = new StringBuilder((d.length + 2) / 3 * 4);
    for (int i = 0; i < d.length; i += 3) {
      int b = (d[i] & 0xFF) << 16;
      if (i + 1 < d.length) b |= (d[i + 1] & 0xFF) << 8;
      if (i + 2 < d.length) b |= d[i + 2] & 0xFF;
      sb.append(B64[(b >> 18) & 63]).append(B64[(b >> 12) & 63]);
      sb.append(i + 1 < d.length ? B64[(b >> 6) & 63] : '=');
      sb.append(i + 2 < d.length ? B64[b & 63] : '=');
    }
    return sb.toString();
  }

  // endregion

  // region For subclasses

  protected void onStarted() {}

  protected void onFailed(Exception e) {}

  protected abstract void onOpen(Connection c, String path);

  protected abstract void onMessage(Connection c, String text);

  protected abstract void onClose(Connection c);

  /** Answer for a plain HTTP GET, or null for 404. */
  protected abstract HttpResponse onHttpRequest(String path);

  // endregion
}
