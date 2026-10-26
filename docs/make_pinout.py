# Builds docs/ESP32S3_PINOUT.svg (run: python3 docs/make_pinout.py docs/ESP32S3_PINOUT.svg). Every pin and value comes from
# firmware/openbot-esp32s3/openbot-esp32s3.cpp (SETTINGS sections 3-6) and
# lidar_ld14.h (wiring comment). Items marked "general practice" are not in
# the firmware.
import sys
from xml.sax.saxutils import escape as e

W, H = 1400, 1330
C = {  # (stroke, fill)
 "drive": ("#1b7f3b", "#e6f4ea"), "light": ("#b06000", "#fef3e2"),
 "status": ("#1a5fb4", "#e8f0fe"), "lidar": ("#7b3fb0", "#f3e8fd"),
 "phone": ("#00796b", "#e0f2f1"), "power": ("#3c4043", "#fef7e0"),
 "avoid": ("#b3261e", "#fce8e6"),
 "rc": ("#c2185b", "#fce4ec"),
}
o = []; A = o.append
A(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">')
A('<title>ESP32-S3 pinout and wiring map: OpenBot Ackermann robot</title>')
A('<desc>GPIO map of firmware/openbot-esp32s3. Left: pins the firmware drives and what they connect to. Right: phone link, power, LiDAR wiring and pins to avoid.</desc>')
A(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
A('<text x="700" y="54" text-anchor="middle" font-size="34" font-weight="700" fill="#202124">ESP32-S3 Pinout and Wiring: OpenBot Ackermann Robot</text>')
A('<text x="700" y="86" text-anchor="middle" font-size="17" fill="#5f6368">Firmware: firmware/openbot-esp32s3 (robot type RC_S3). Logical map by GPIO number, not the physical pad order of your board.</text>')

# board
BX, BY, BW, BH = 600, 130, 200, 830
A(f'<rect x="{BX}" y="{BY}" width="{BW}" height="{BH}" rx="16" fill="#263238"/>')
A(f'<text x="{BX+BW/2}" y="{BY+36}" text-anchor="middle" font-size="22" font-weight="700" fill="#fff">ESP32-S3</text>')
A(f'<text x="{BX+BW/2}" y="{BY+60}" text-anchor="middle" font-size="14" fill="#cfd8dc">N16R8</text>')

A('<text x="30" y="160" font-size="19" font-weight="700" fill="#202124">Pins the firmware uses</text>')
rows = [
 (1, "ESC signal (throttle)", "50 Hz pulse 1000-2000 us, 1500 us = stop", "drive"),
 (2, "Steering servo signal", "50 Hz pulse 1000-2000 us, 1500 us = straight", "drive"),
 (4, "Left turn indicator LED", "on/off, blinks every 500 ms", "light"),
 (5, "Right turn indicator LED", "on/off, blinks every 500 ms", "light"),
 (6, "Left front light LED", "brightness 0-255 (PWM)", "light"),
 (7, "Right front light LED", "brightness 0-255 (PWM)", "light"),
 (8, "Left rear light LED", "brightness 0-255 (PWM)", "light"),
 (9, "Right rear light LED", "brightness 0-255 (PWM)", "light"),
 (10, "Status LED yellow", "on/off, set by the app", "status"),
 (11, "Status LED green", "on/off, set by the app", "status"),
 (12, "Status LED blue", "on/off, set by the app", "status"),
 (13, "LiDAR TX (input)", "UART1 RX: 115200 baud LD14, 230400 LD14P", "lidar"),
 (14, "RC receiver iBUS (input)", "FS-iA6B SERVO port; or CH1 steering (PWM mode)", "rc"),
 (15, "RC CH2 throttle (input)", "PWM mode only (FS-iA6)", "rc"),
 (16, "RC CH5 mode switch (input)", "PWM mode only (FS-iA6)", "rc"),
]
y0, step = 200, 52
for i, (g, name, sub, k) in enumerate(rows):
    y = y0 + i * step; s, f = C[k]
    A(f'<rect x="30" y="{y-21}" width="450" height="43" rx="8" fill="{f}" stroke="{s}" stroke-width="1.5"/>')
    A(f'<text x="44" y="{y+6}" font-size="18" font-weight="700" fill="{s}">GPIO {g}</text>')
    A(f'<text x="140" y="{y-3}" font-size="16" font-weight="700" fill="#202124">{e(name)}</text>')
    A(f'<text x="140" y="{y+16}" font-size="13.5" fill="#3c4043">{e(sub)}</text>')
    A(f'<line x1="480" y1="{y}" x2="{BX-6}" y2="{y}" stroke="{s}" stroke-width="2.5"/>')
    A(f'<circle cx="{BX}" cy="{y}" r="7" fill="{s}"/>')
    A(f'<text x="{BX+14}" y="{y+5}" font-size="14" fill="#eceff1">IO{g}</text>')

# right-hand boxes
RX, RW = 880, 490
def box(y, h, k, title, lines, pins):
    s, f = C[k]
    A(f'<rect x="{RX}" y="{y}" width="{RW}" height="{h}" rx="10" fill="{f}" stroke="{s}" stroke-width="1.5"/>')
    A(f'<text x="{RX+16}" y="{y+28}" font-size="18" font-weight="700" fill="{s}">{e(title)}</text>')
    for j, t in enumerate(lines):
        A(f'<text x="{RX+16}" y="{y+54+j*21}" font-size="14.5" fill="#202124">{e(t)}</text>')
    py = y + 40
    A(f'<line x1="{BX+BW+6}" y1="{py}" x2="{RX}" y2="{py}" stroke="{s}" stroke-width="2.5"/>')
    A(f'<circle cx="{BX+BW}" cy="{py}" r="7" fill="{s}"/>')
    A(f'<text x="{BX+BW-14}" y="{py+5}" text-anchor="end" font-size="14" fill="#eceff1">{e(pins)}</text>')

box(175, 140, "phone", "Phone link", [
 "USB-C cable, 115200 baud (Android drives over USB):",
 "• COM/UART port: UART0 = GPIO 43 (TX), 44 (RX)",
 "• native USB port: GPIO 19 (D-), 20 (D+)",
 "Bluetooth LE: device name \"OpenBot\" (iPhone drives here)",
], "IO43/44")
box(330, 140, "power", "Power and ground", [
 "5 V rail: ESP32 5V pin and LiDAR VCC (about 200 mA).",
 "Join every GND: ESP32, ESC, servo, LiDAR and LEDs.",
 "Never power the LiDAR from the 3V3 pin.",
 "Servo power: from the ESC's BEC (general practice).",
], "5V/GND")
box(485, 140, "lidar", "LiDAR LD14 / LD14P wiring", [
 "LiDAR TX   ->  GPIO 13 (3.3 V logic, no level shifter)",
 "LiDAR RX   ->  not wired (LIDAR_TX_PIN = -1)",
 "LiDAR GND  ->  GND (shared with the ESP32)",
 "LiDAR VCC  ->  5 V rail",
], "IO13")
box(640, 160, "rc", "RC receiver (FlySky FS-i6)", [
 "FS-iA6B: iBUS SERVO port signal -> GPIO 14, + -> 5 V, - -> GND",
 "FS-iA6 (PWM): CH1 -> GPIO 14, CH2 -> GPIO 15, CH5 -> GPIO 16",
 "Switch SwC on CH5: up = MANUAL (RC drives),",
 "   middle = AUTO (phone drives), down = HOLD (stop)",
 "RC link lost for 250 ms -> stop.",
], "IO14")
box(815, 172, "avoid", "Do not use for anything else", [
 "• GPIO 26 to 37: in-package flash and octal PSRAM.",
 "   Wiring anything here makes the board unbootable.",
 "• GPIO 0, 3, 45, 46: strapping pins (boot mode, JTAG,",
 "   VDD_SPI voltage, ROM print).",
 "• GPIO 19, 20, 43, 44: the phone link above.",
 "Still free: GPIO 17, 18, 21, 38-42, 47, 48 (check your board).",
], "26-37")

# legend
ly = 1005
A(f'<text x="30" y="{ly}" font-size="19" font-weight="700" fill="#202124">Legend</text>')
leg = [("drive","Drive: throttle and steering"),("light","Lights: indicators, front, rear"),
       ("status","Status LEDs"),("lidar","LiDAR data"),("rc","RC receiver"),("phone","Phone link"),
       ("power","Power and ground"),("avoid","Reserved: do not use")]
for i,(k,t) in enumerate(leg):
    x = 30 + (i % 4) * 340; y = ly + 34 + (i // 4) * 36; s, f = C[k]
    A(f'<rect x="{x}" y="{y-16}" width="22" height="22" rx="4" fill="{s}"/>')
    A(f'<text x="{x+32}" y="{y+1}" font-size="16" fill="#202124">{e(t)}</text>')

ny = 1125
A(f'<text x="30" y="{ny}" font-size="19" font-weight="700" fill="#202124">Notes</text>')
notes = [
 "LEDs are active-high: GPIO -> resistor -> LED -> GND. A 220-330 ohm resistor suits most LEDs (general practice).",
 "At boot the left, then the right indicator blinks once: a quick check that GPIO 4 and 5 are wired.",
 "The ESC sees neutral (1500 us) at boot so it can arm. Calibrate with the wheels off the ground.",
 "Failsafe: no command for CONTROL_TIMEOUT_MS (1000 ms) -> throttle neutral, wheels straight.",
 "Pins are set in the SETTINGS block of openbot-esp32s3.cpp (or your openbot_settings.h). Change both if you rewire.",
]
for i,t in enumerate(notes):
    A(f'<text x="30" y="{ny+32+i*27}" font-size="15.5" fill="#202124">- {e(t)}</text>')
A('</svg>')
open(sys.argv[1], "w").write("\n".join(o))
print("ok")
