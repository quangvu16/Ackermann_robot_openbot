package org.openbot.app.robot.sensorlink;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.Test;

public class MiniWebServerTest {

  @Test
  public void acceptKeyMatchesRfc6455Example() {
    // RFC 6455 section 1.3.
    assertEquals("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", MiniWebServer.acceptKey("dGhlIHNhbXBsZSBub25jZQ=="));
  }

  @Test
  public void base64Padding() {
    assertEquals("", MiniWebServer.base64(new byte[0]));
    assertEquals("Zg==", MiniWebServer.base64("f".getBytes(StandardCharsets.US_ASCII)));
    assertEquals("Zm8=", MiniWebServer.base64("fo".getBytes(StandardCharsets.US_ASCII)));
    assertEquals("Zm9v", MiniWebServer.base64("foo".getBytes(StandardCharsets.US_ASCII)));
  }

  @Test
  public void framesUseTheRightLengthEncoding() {
    assertArrayEquals(new byte[] {(byte) 0x81, 2, 'h', 'i'}, MiniWebServer.frame(1, "hi".getBytes(StandardCharsets.US_ASCII)));
    byte[] mid = MiniWebServer.frame(1, new byte[300]);
    assertEquals(126, mid[1]);
    assertEquals(300, ((mid[2] & 0xFF) << 8) | (mid[3] & 0xFF));
    assertEquals(127, MiniWebServer.frame(1, new byte[70000])[1]);
  }

  /** A real socket: a page over HTTP, then a WebSocket echo with a masked client frame. */
  @Test
  public void servesHttpAndWebSocketOnOnePort() throws Exception {
    int port;
    try (ServerSocket probe = new ServerSocket(0)) {
      port = probe.getLocalPort();
    }
    CountDownLatch started = new CountDownLatch(1);
    MiniWebServer server =
        new MiniWebServer(port) {
          @Override
          protected void onStarted() {
            started.countDown();
          }

          @Override
          protected void onOpen(Connection c, String path) {
            c.send("open " + path);
          }

          @Override
          protected void onMessage(Connection c, String text) {
            c.send("echo " + text);
          }

          @Override
          protected void onClose(Connection c) {}

          @Override
          protected HttpResponse onHttpRequest(String path) {
            return "/".equals(path) ? HttpResponse.text(200, "text/html", "<p>hi</p>") : null;
          }
        };
    server.start();
    assertTrue(started.await(5, TimeUnit.SECONDS));
    try {
      try (Socket s = new Socket("127.0.0.1", port)) {
        s.getOutputStream().write("GET / HTTP/1.1\r\nHost: x\r\n\r\n".getBytes(StandardCharsets.US_ASCII));
        String reply = readAll(s.getInputStream());
        assertTrue(reply, reply.startsWith("HTTP/1.1 200"));
        assertTrue(reply.endsWith("<p>hi</p>"));
      }
      try (Socket s = new Socket("127.0.0.1", port)) {
        s.setSoTimeout(5000);
        OutputStream out = s.getOutputStream();
        InputStream in = s.getInputStream();
        out.write(
            ("GET /gps HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    + "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n")
                .getBytes(StandardCharsets.US_ASCII));
        String head = readHead(in);
        assertTrue(head, head.contains("101 Switching Protocols"));
        assertTrue(head.contains("s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));
        assertEquals("open /gps", readTextFrame(in));
        byte[] mask = {1, 2, 3, 4};
        byte[] text = "ping".getBytes(StandardCharsets.US_ASCII);
        byte[] f = new byte[2 + 4 + text.length];
        f[0] = (byte) 0x81;
        f[1] = (byte) (0x80 | text.length);
        System.arraycopy(mask, 0, f, 2, 4);
        for (int i = 0; i < text.length; i++) f[6 + i] = (byte) (text[i] ^ mask[i & 3]);
        out.write(f);
        assertEquals("echo ping", readTextFrame(in));
      }
    } finally {
      server.stop();
    }
  }

  private static String readAll(InputStream in) throws Exception {
    ByteArrayOutputStream b = new ByteArrayOutputStream();
    byte[] buf = new byte[1024];
    int n;
    while ((n = in.read(buf)) > 0) b.write(buf, 0, n);
    return b.toString("UTF-8");
  }

  private static String readHead(InputStream in) throws Exception {
    StringBuilder sb = new StringBuilder();
    while (!sb.toString().endsWith("\r\n\r\n")) sb.append((char) in.read());
    return sb.toString();
  }

  private static String readTextFrame(InputStream in) throws Exception {
    int b0 = in.read();
    int len = in.read() & 0x7F;
    assertEquals(0x81, b0);
    byte[] p = new byte[len];
    int off = 0;
    while (off < len) off += in.read(p, off, len - off);
    return new String(p, StandardCharsets.UTF_8);
  }
}
