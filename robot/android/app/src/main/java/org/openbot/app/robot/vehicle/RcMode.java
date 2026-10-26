package org.openbot.app.robot.vehicle;

/**
 * The RC mode switch on the car, reported by the firmware as {@code m<letter>} on every change and
 * once a second (see firmware/openbot-esp32s3/openbot-esp32s3.cpp, send_rc_mode()).
 *
 * <ul>
 *   <li>MANUAL: the RC transmitter drives; the app shows a smiling face.
 *   <li>AUTO: the app drives; no face.
 *   <li>HOLD: the car is stopped by the switch; the app shows a crying face.
 *   <li>LOST: the RC link dropped and the car stopped; also a crying face.
 * </ul>
 */
public enum RcMode {
  MANUAL,
  AUTO,
  HOLD,
  LOST;

  /** Which face the app shows for a mode. */
  public enum Face {
    NONE,
    SMILE,
    CRY
  }

  /**
   * Parses the body of an {@code m} message: "M", "A", "H" or "L", optionally with surrounding
   * whitespace. Returns null for anything else, so a garbled message never changes the screen.
   */
  public static RcMode parse(String body) {
    if (body == null) return null;
    String b = body.trim();
    if (b.length() != 1) return null;
    switch (b.charAt(0)) {
      case 'M':
        return MANUAL;
      case 'A':
        return AUTO;
      case 'H':
        return HOLD;
      case 'L':
        return LOST;
      default:
        return null;
    }
  }

  /** Smile for MANUAL, cry for HOLD and LOST (the car is stopped), nothing for AUTO. */
  public Face face() {
    switch (this) {
      case MANUAL:
        return Face.SMILE;
      case HOLD:
      case LOST:
        return Face.CRY;
      default:
        return Face.NONE;
    }
  }
}
