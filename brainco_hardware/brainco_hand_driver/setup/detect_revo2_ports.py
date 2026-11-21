#!/usr/bin/env python3
# Copyright (c) 2025 BrainCo
# Setup tool: Revo2 Modbus port scanner (slave_id 126=left, 127=right).
#
# SDK-free Revo2 port detector: speaks Modbus RTU directly over pyserial, so it
# does NOT depend on the Stark SDK and survives SDK updates.
# Invoked by setup/detect_revo2_ports_auto.sh (which bootstrap_revo2.sh calls).
#
# Detection principle: for each candidate serial port, send a Modbus RTU
# "read holding registers" request (FC 0x03) to slave 126 and 127 at the
# fixed Revo2 baudrate (460800). ANY CRC-valid framed reply from that
# slave -- normal data OR an exception response -- proves the device is on
# the bus at that slave_id. We never need to know a specific register map.
#
# Output (stdout), identical to the C++ detector so bootstrap parsing is unchanged:
#   REVO2_LEFT_PORT=/dev/ttyXXX
#   REVO2_LEFT_SLAVE=126
#   REVO2_RIGHT_PORT=/dev/ttyYYY
#   REVO2_RIGHT_SLAVE=127
# Exit code: 0 only when BOTH hands are found; 1 otherwise (matches C++).

import glob
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.stderr.write(
        "[ERROR] python module 'serial' (pyserial) not installed.\n"
        "  Install: sudo apt install -y python3-serial   (or: python3 -m pip install pyserial)\n"
    )
    # Exit 2 = environment problem (distinct from 1 = ran but hands not found).
    sys.exit(2)

LEFT_SLAVE_ID = 126
RIGHT_SLAVE_ID = 127
BAUDRATE = 460800  # fixed for Revo2 Modbus (see config/protocol_modbus_*.yaml)

PROBE_TIMEOUT = 0.30   # max wait for one slave's reply (covers slow firmware)
READ_CHUNK_TIMEOUT = 0.05  # per ser.read() poll inside the wait loop
OPEN_SETTLE = 0.02     # let the USB-serial chip settle after open/baud-set

# Glob patterns for USB-based serial ports, in probe-priority order.
# CH34* first (WCH ch343 driver -> /dev/ttyCH343USB*, plus ch341/ch9102/... variants),
# then CDC-ACM, then generic usb-serial. Globs (not a fixed index range) so any
# enumerated index is caught. ttyS*/ttyAMA* (on-board UARTs) are intentionally
# excluded -- they never host Revo2 hands.
PORT_GLOBS = ("/dev/ttyCH34*", "/dev/ttyACM*", "/dev/ttyUSB*")


def scan_serial_ports():
    ports = []
    seen = set()
    for pattern in PORT_GLOBS:
        for path in sorted(glob.glob(pattern)):
            if path not in seen:
                seen.add(path)
                ports.append(path)
    return ports


def modbus_crc16(data):
    """Standard Modbus RTU CRC16 (poly 0xA001), little-endian on the wire."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_request(slave_id, address=0, count=1):
    """FC 0x03 read-holding-registers request frame."""
    body = bytes(
        [
            slave_id & 0xFF,
            0x03,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ]
    )
    crc = modbus_crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _crc_ok(frame):
    if len(frame) < 4:
        return False
    payload, crc_bytes = frame[:-2], frame[-2:]
    crc = modbus_crc16(payload)
    return crc_bytes == bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _expected_frame_len(buf):
    """Total RTU frame length implied by the bytes seen so far.
    Returns None if not yet determinable, -1 if it is not a 0x03/0x83 reply."""
    if len(buf) < 2:
        return None
    func = buf[1]
    if func == 0x03:
        # Normal reply: [slave][0x03][byte_count][data...][crc_lo][crc_hi]
        if len(buf) < 3:
            return None
        return 3 + buf[2] + 2
    if func == 0x83:
        # Exception reply: [slave][0x83][exc_code][crc_lo][crc_hi]
        return 5
    return -1


def probe_slave(ser, slave_id):
    """Return True if a CRC-valid reply is received from this slave_id.

    Reads incrementally until a complete RTU frame arrives or PROBE_TIMEOUT
    elapses, so a slow firmware reply is not truncated by a single short read.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(build_read_request(slave_id))
    ser.flush()

    deadline = time.monotonic() + PROBE_TIMEOUT
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if not chunk:
            continue  # nothing yet; keep waiting until the deadline
        buf += chunk
        if buf[0] != slave_id:
            return False  # something other than the target slave answered
        expected = _expected_frame_len(buf)
        if expected == -1:
            return False  # not a read/exception reply
        if expected is not None and len(buf) >= expected:
            return _crc_ok(bytes(buf[:expected]))
    return False


def probe_port(port):
    """Return the slave_id (126/127) found on this port, or None."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=READ_CHUNK_TIMEOUT,
            write_timeout=0.5,
        )
    except (serial.SerialException, OSError) as exc:
        sys.stderr.write("[WARN] cannot open %s: %s\n" % (port, exc))
        return None

    time.sleep(OPEN_SETTLE)  # avoid losing the first frame right after open
    try:
        for slave_id in (LEFT_SLAVE_ID, RIGHT_SLAVE_ID):
            try:
                if probe_slave(ser, slave_id):
                    return slave_id
            except (serial.SerialException, OSError):
                return None
    finally:
        ser.close()
    return None


def main():
    ports = scan_serial_ports()
    if not ports:
        sys.stderr.write(
            "[ERROR] no serial ports found (ttyCH34*, ttyACM*, ttyUSB*)\n"
        )
        return 1

    sys.stdout.write(
        "[INFO] scanning %d serial port(s) for Revo2 Modbus...\n" % len(ports)
    )

    by_slave = {}
    for port in ports:
        slave_id = probe_port(port)
        if slave_id is None:
            continue
        sys.stdout.write("[OK] port=%s slave_id=%d\n" % (port, slave_id))
        if slave_id in by_slave and by_slave[slave_id] != port:
            sys.stderr.write(
                "[WARN] duplicate slave_id %d on %s and %s\n"
                % (slave_id, by_slave[slave_id], port)
            )
        by_slave[slave_id] = port
        if LEFT_SLAVE_ID in by_slave and RIGHT_SLAVE_ID in by_slave:
            sys.stdout.write("[INFO] both hands found, stopping scan.\n")
            break

    left_port = by_slave.get(LEFT_SLAVE_ID)
    right_port = by_slave.get(RIGHT_SLAVE_ID)

    if left_port:
        sys.stdout.write("REVO2_LEFT_PORT=%s\n" % left_port)
        sys.stdout.write("REVO2_LEFT_SLAVE=%d\n" % LEFT_SLAVE_ID)
    if right_port:
        sys.stdout.write("REVO2_RIGHT_PORT=%s\n" % right_port)
        sys.stdout.write("REVO2_RIGHT_SLAVE=%d\n" % RIGHT_SLAVE_ID)

    if not left_port or not right_port:
        sys.stderr.write(
            "[ERROR] need both slave_id %d (left) and %d (right). Found %d device(s).\n"
            % (LEFT_SLAVE_ID, RIGHT_SLAVE_ID, len(by_slave))
        )
        for slave_id, port in by_slave.items():
            sys.stderr.write("        slave_id=%d port=%s\n" % (slave_id, port))
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
