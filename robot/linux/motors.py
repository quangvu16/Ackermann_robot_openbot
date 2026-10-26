"""Motor output over the OpenBot Arduino serial protocol.

Extracted from :mod:`infer` so non-inference entry points (e.g. the GPS mission
runner) can drive the robot without pulling in the neural-network stack. The
wire format is unchanged: ``c<left>,<right>\\n`` with the wheel commands scaled
to integers in ``[-255, 255]``.
"""

import numpy as np
import serial


def get_arduino_serial(device_address="/dev/ttyUSB0", baudrate=115200):
    """Getting serial objet from a device address.

    Args:
        device_address (str, optional): Address to the serial communication.
            Defaults to '/dev/ttyUSB0'.
        baudrate (int, optional): Baudrate of communication. Defaults to 115200.

    Returns:
        serial.Serial: Returns serial object which can be interfaced with.
    """

    try:
        arduino_serial = serial.Serial(device_address, baudrate=baudrate)
    except Exception as e:
        print(e)
        print(f"Arduino not found at {device_address} with baudrate {baudrate}")
        arduino_serial = None
    return arduino_serial


def apply_action_serial(serial_obj, left, right, scale=255, gate=None):
    """Writes left and right motor command to serial interface (e.g.,
    Arduino board connected to motor drivers).

    Args:
        serial_obj (serial.Serial): Serial object that is retrieved via
            get_arduino_serial().
        left (float): Left motor command between -1, 1. This value is scaled
            by `scale`.
        right (float): Right motor command between -1, 1. This value is scaled
            by `scale`.
        scale (int): Scaling factor for left and right motor command.
            Default: 255
        gate (lidar.drive.DriveGate): Object-avoidance gate applied before the
            command reaches the wire, mirroring the Android app's
            ``Vehicle.sendControl()`` chokepoint. Optional; without one the
            command is sent untouched.
    Returns:
        (int, int): Scaled left and right motor command sent to serial object.
    """
    if gate is not None:
        decision = gate.apply(left, right)
        left, right = decision.left, decision.right

    left = np.clip(left, -1.0, 1)
    right = np.clip(right, -1.0, 1)

    command_string = f"c{int(left*scale)},{int(right*scale)}\n"

    serial_obj.write(bytes(command_string, encoding="utf-8"))
    return int(left * scale), int(right * scale)


def stop_robot(serial_obj):
    """Sends a zero command, stopping both motors. Safe to call with None.

    Bypasses the avoidance gate on purpose: an explicit stop is a safety
    command, not a driving request, and must never be vetoed or turned into a
    reverse.
    """
    if serial_obj is None:
        return 0, 0
    return apply_action_serial(serial_obj, 0, 0, gate=None)
