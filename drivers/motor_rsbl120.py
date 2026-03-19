"""Servo motor control for SMS/STS series servos using the FT-SCS protocol.

This module provides low-level packet building, serial communication helpers,
and a high-level API for controlling Smart Bus servos (Magnetic Encoding series).

Protocol summary (from Smart Bus Servo Communication Protocol Manual):
    Instruction packet:  FF FF | ID | Length | Instr  | Param... | Checksum
    Reply packet:        FF FF | ID | Length | Error  | Param... | Checksum

    - Length       = number_of_params + 2
    - Checksum     = ~(ID + Length + Instr/Error + Param1 + ... + ParamN) & 0xFF
    - Two-byte values are little-endian (LOW byte first, HIGH byte second)

EPROM write-lock semantics (register 0x37):
    - Write 0  -> lock closed  -> EPROM writes ARE saved on power-off
    - Write 1  -> lock open    -> EPROM writes are NOT saved (factory default)
    To persist a setting: unlock (0), write value, re-lock (1).
"""

import time

import numpy as np
import serial

from robot_arm import JointCal, rad_to_step

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_STEP_PER_RAD: float = 4095.0 / (300.0 * np.pi / 180.0)
"""Steps per radian based on a 300-degree range mapped to 4095 steps."""

BROADCAST_ID = 0xFE
"""Broadcast ID: all servos receive the packet; no reply is sent (except PING)."""

STEP_CENTER = 2048
"""Mid-point of the 0-4095 step range, corresponding to the physical centre position."""

# -- EPROM addresses (persist across power cycles when write-lock is closed) -
ADDR_ID = 0x05  # Servo ID              (1 byte,  0-253)
ADDR_MAX_TORQUE = 0x10  # Maximum torque        (2 bytes, 0-1000, unit 0.1 %)

# -- SRAM control addresses (volatile; reset on power-off unless lock = 0) ---
ADDR_TORQUE_SWITCH = 0x28  # Torque on/off/damping  (1 byte)
ADDR_ACCELERATION = (
    0x29  # Acceleration           (1 byte, 0-254, unit 8.7 deg/s^2)
)
ADDR_TARGET_POS = (
    0x2A  # Target position        (2 bytes, -32767-32767; BIT15 = direction)
)
ADDR_PWM_SPEED = (
    0x2C  # PWM / open-loop speed  (2 bytes; write 0 in position mode)
)
ADDR_RUNNING_SPEED = 0x2E  # Running speed limit    (2 bytes, unit 0.732 RPM)
ADDR_TORQUE_LIMIT = 0x30  # Torque limit           (2 bytes, 0-1000, unit 0.1 %)
ADDR_LOCK = 0x37  # EPROM write-lock flag  (1 byte)

# -- SRAM feedback addresses (read-only) -------------------------------------
ADDR_PRESENT_POS = 0x38  # Current position       (2 bytes, unit 0.087 deg)
ADDR_PRESENT_SPEED = 0x3A  # Current speed          (2 bytes, unit 0.732 RPM)
ADDR_PRESENT_LOAD = 0x3C  # Current load / PWM     (2 bytes, unit 0.1 %)
ADDR_PRESENT_VOLT = 0x3E  # Supply voltage         (1 byte,  unit 0.1 V)
ADDR_PRESENT_TEMP = 0x3F  # Temperature            (1 byte,  unit degC)
ADDR_SERVO_STATUS = 0x41  # Error status bits      (1 byte)
ADDR_PRESENT_AMP = 0x45  # Phase current          (2 bytes, unit 6.5 mA)

# -- Torque switch values (written to ADDR_TORQUE_SWITCH) --------------------
TORQUE_OFF = 0  # Servo output disabled (limp)
TORQUE_ON = 1  # Servo output enabled
TORQUE_DAMPING = 2  # Passive damping / resistance mode


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------


def rsbl120_open_comm(port: str) -> serial.Serial:
    """Open a serial connection to the servo bus at 1 Mbaud.

    Args:
        port: System serial port string (e.g. ``"/dev/ttyUSB0"`` or ``"COM3"``).

    Returns:
        An open :class:`serial.Serial` instance ready for communication.
    """
    print(f"DEBUG: rsbl120_open_comm {port}")
    comm = serial.Serial(port, 1_000_000, timeout=0.1)
    time.sleep(0.2)  # allow hardware to settle
    return comm


def rsbl120_close_comm(comm: serial.Serial) -> None:
    """Close the serial connection to the servo bus.

    Args:
        comm: The open :class:`serial.Serial` instance to close.
    """
    print("DEBUG: rsbl120_close_comm")
    comm.close()


def __read_exact(comm: serial.Serial, n: int) -> bytes:
    """Read exactly *n* bytes from *comm*, respecting the port timeout.

    Args:
        comm: Open serial port.
        n:    Number of bytes to read.

    Returns:
        A :class:`bytes` object of up to *n* bytes (may be shorter on timeout).
    """
    buf = bytearray()
    deadline = time.time() + comm.timeout
    while len(buf) < n and time.time() < deadline:
        chunk = comm.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


# ---------------------------------------------------------------------------
# Low-level packet builders
# ---------------------------------------------------------------------------


def __build_packet(servo_id: int, instr: int, params: bytes) -> bytes:
    """Build a complete FT-SCS instruction packet.

    Packet layout::

        FF FF | ID | Length | Instr | Params... | Checksum

    where ``Length = len(params) + 2`` and
    ``Checksum = ~(ID + Length + Instr + sum(params)) & 0xFF``.

    Args:
        servo_id: Target servo ID (0-253, or 0xFE for broadcast).
        instr:    Instruction byte (e.g. 0x03 for WRITE DATA).
        params:   Instruction-specific parameter bytes.

    Returns:
        Complete packet as a :class:`bytes` object.
    """
    length = len(params) + 2
    body = bytes([servo_id, length, instr]) + params
    checksum = (~sum(body)) & 0xFF
    return b"\xff\xff" + body + bytes([checksum])


def __write_u8(
    cal: JointCal, addr: int, value: int, packet_id: int | None = None
) -> None:
    """Write a single byte to a servo register.

    Args:
        cal:       :class:`JointCal` carrying the serial port and default servo ID.
        addr:      Register address to write.
        value:     Byte value to write (0-255).
        packet_id: Override servo ID for this packet; uses ``cal.servo_id`` if ``None``.
    """
    servo_id = cal.servo_id if packet_id is None else packet_id
    INSTR_WRITE = 0x03
    packet = __build_packet(
        servo_id, INSTR_WRITE, bytes([addr & 0xFF, value & 0xFF])
    )
    cal.comm.write(packet)


def __write_u16(
    cal: JointCal, addr: int, value: int, packet_id: int | None = None
) -> None:
    """Write a 16-bit value to a servo register (little-endian: low byte first).

    Args:
        cal:       :class:`JointCal` carrying the serial port and default servo ID.
        addr:      Register address to write (first of two consecutive bytes).
        value:     16-bit value to write (0-65535).
        packet_id: Override servo ID for this packet; uses ``cal.servo_id`` if ``None``.
    """
    servo_id = cal.servo_id if packet_id is None else packet_id
    INSTR_WRITE = 0x03
    v = int(value) & 0xFFFF
    params = bytes([addr & 0xFF, v & 0xFF, (v >> 8) & 0xFF])
    packet = __build_packet(servo_id, INSTR_WRITE, params)
    cal.comm.write(packet)


# ---------------------------------------------------------------------------
# Reply parser
# ---------------------------------------------------------------------------


def _read_reply(cal: JointCal) -> bytes:
    """Read and validate a reply packet from the servo.

    Reply layout::

        FF FF | ID | Length | Error | Param1..ParamN | Checksum

    where ``Length = N_params + 2`` (Error byte + Checksum byte).

    Args:
        cal: :class:`JointCal` whose ``servo_id`` is used for ID validation.

    Returns:
        Parameter bytes from the reply (excludes Error and Checksum).

    Raises:
        RuntimeError: On timeout, ID mismatch, servo error flag, or checksum failure.
    """
    comm = cal.comm

    # Resync to the FF FF preamble
    b = comm.read(1)
    while b:
        if b == b"\xff":
            b2 = comm.read(1)
            if b2 == b"\xff":
                break
            b = b2
        else:
            b = comm.read(1)
    else:
        raise RuntimeError("Timeout waiting for reply preamble (FF FF)")

    # ID, Length, Error
    header = __read_exact(comm, 3)
    if len(header) != 3:
        raise RuntimeError("Short reply: missing ID/Length/Error bytes")

    resp_id, length, error = header[0], header[1], header[2]

    if resp_id != cal.servo_id:
        raise RuntimeError(
            f"Reply ID mismatch: expected {cal.servo_id}, got {resp_id}"
        )
    if error:
        raise RuntimeError(f"Servo reported error status: 0x{error:02X}")

    # Parameters + Checksum
    # length covers: Error(1) + params(N) + Checksum(1) -> tail = length - 1 bytes
    tail = __read_exact(comm, length - 1)
    if len(tail) != length - 1:
        raise RuntimeError("Short reply: truncated parameter/checksum bytes")

    params = tail[:-1]  # everything except the trailing checksum byte
    rx_chksum = tail[-1]

    chk_input = bytes([resp_id, length, error]) + params
    calc_chksum = (~sum(chk_input)) & 0xFF
    if rx_chksum != calc_chksum:
        raise RuntimeError(
            f"Checksum mismatch: received 0x{rx_chksum:02X}, "
            f"calculated 0x{calc_chksum:02X}"
        )

    return params


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def rsbl120_set_servo_id_nvm(cal: JointCal, new_id: int) -> None:
    """Permanently change the servo ID and persist it to EPROM.

    Sequence:

    1. Close write-lock on the current ID (write 0 -> saves on power-off).
    2. Write the new ID via broadcast (safe when only one servo is on the bus).
    3. Update ``cal.servo_id`` so subsequent calls reach the servo on its new ID.
    4. Re-open write-lock on the new ID (write 1 -> factory default; prevents
       accidental NVM wear).

    Args:
        cal:    :class:`JointCal` with the *current* ``servo_id``.
        new_id: Target ID (0-253).

    Raises:
        ValueError: If *new_id* is outside the valid range.
    """
    if not (0 <= new_id <= 253):
        raise ValueError(f"Servo ID must be 0-253, got {new_id}")

    __write_u8(cal, ADDR_LOCK, 0)  # step 1 - close lock (persist writes)
    time.sleep(0.05)

    __write_u8(
        cal, ADDR_ID, new_id, packet_id=BROADCAST_ID
    )  # step 2 - set new ID
    time.sleep(0.1)

    cal.servo_id = new_id  # step 3 - update software state

    __write_u8(cal, ADDR_LOCK, 1)  # step 4 - re-open lock (factory default)
    time.sleep(0.05)

    print(f"DEBUG: set_servo_id_nvm -> new ID={new_id}")


def rsbl120_enable_actuator(cal: JointCal) -> None:
    """Enable torque output on the servo.

    Args:
        cal: :class:`JointCal` for the target servo.
    """
    __write_u8(cal, ADDR_TORQUE_SWITCH, TORQUE_ON)


def rsbl120_disable_actuator(cal: JointCal) -> None:
    """Disable torque output, leaving the servo free to rotate (limp).

    Args:
        cal: :class:`JointCal` for the target servo.
    """
    __write_u8(cal, ADDR_TORQUE_SWITCH, TORQUE_OFF)


def rsbl120_set_damping(cal: JointCal) -> None:
    """Switch the servo to damping mode (passive resistance, no active driving).

    Args:
        cal: :class:`JointCal` for the target servo.
    """
    __write_u8(cal, ADDR_TORQUE_SWITCH, TORQUE_DAMPING)


def rsbl120_scan_for_servo(
    comm: serial.Serial, id_min: int = 0, id_max: int = 253
) -> list[int]:
    """Scan the bus for responding servos by sending PING to each ID in range.

    Args:
        comm:    Open serial port to scan.
        id_min:  First ID to try (inclusive, default 0).
        id_max:  Last ID to try (inclusive, default 253).

    Returns:
        Sorted list of IDs that responded. Empty if none found.
    """
    INSTR_PING = 0x01
    found = []

    print(f"DEBUG: scan_for_servo scanning IDs {id_min}-{id_max} ...")

    for servo_id in range(id_min, id_max + 1):
        comm.reset_input_buffer()
        packet = __build_packet(servo_id, INSTR_PING, b"")
        comm.write(packet)

        # Collect bytes for up to 20 ms; most servos reply within a few ms.
        deadline = time.time() + 0.02
        buf = bytearray()
        while time.time() < deadline:
            chunk = comm.read(comm.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
            if len(buf) >= 6:
                break

        raw = bytes(buf)
        idx = raw.find(b"\xff\xff")
        if idx == -1:
            continue

        reply = raw[idx + 2 :]  # skip preamble
        if len(reply) < 4:
            continue

        resp_id, length, error, rx_chksum = (
            reply[0],
            reply[1],
            reply[2],
            reply[3],
        )

        calc_chksum = (~(resp_id + length + error)) & 0xFF
        if rx_chksum != calc_chksum:
            continue  # corrupted packet

        if resp_id == servo_id:
            print(
                f"DEBUG: scan_for_servo -> found servo at ID {servo_id} "
                f"(error=0x{error:02X})"
            )
            found.append(servo_id)

    if not found:
        print("DEBUG: scan_for_servo -> no servos found")

    return found


def rsbl120_reset_servo_id(cal: JointCal) -> None:
    """Reset the servo to factory defaults using the RESET instruction (0x06).

    This restores the entire control table, including setting the ID back to 1.
    All other EPROM settings (PID gains, angle limits, baud rate, etc.) are also
    reset - use with caution.

    ``cal.servo_id`` is updated to 1 after the call so subsequent commands work
    without reconstructing the :class:`JointCal`.

    Args:
        cal: :class:`JointCal` with the current (known) servo ID.
    """
    INSTR_RESET = 0x06
    packet = __build_packet(cal.servo_id, INSTR_RESET, b"")
    cal.comm.write(packet)
    time.sleep(0.5)  # allow the servo to reboot and apply factory defaults

    cal.servo_id = 1
    print(
        "DEBUG: reset_servo_id -> servo reset to factory defaults, ID is now 1"
    )


def rsbl120_read_position_step(cal: JointCal) -> int:
    """Read the current absolute position in raw steps.

    The nominal range is 0-4095. BIT15 encodes the rotation direction in
    multi-turn mode; mask it off for single-turn use: ``pos & 0x7FFF``.

    Args:
        cal: :class:`JointCal` for the target servo.

    Returns:
        Raw 16-bit step count from register ``ADDR_PRESENT_POS`` (0x38).

    Raises:
        RuntimeError: If the reply is too short or fails validation.
    """
    INSTR_READ = 0x02
    READ_LEN = 2

    params = bytes([ADDR_PRESENT_POS, READ_LEN])
    packet = __build_packet(cal.servo_id, INSTR_READ, params)
    cal.comm.write(packet)

    reply = _read_reply(cal)
    if len(reply) < 2:
        raise RuntimeError(f"Position reply too short: {len(reply)} bytes")

    # Low byte first, high byte second (little-endian per protocol spec)
    return reply[0] | (reply[1] << 8)


def rsbl120_read_current_ma(cal: JointCal) -> float:
    """Read the current phase current in milliamps.

    Reads register ``ADDR_PRESENT_AMP`` (0x45); resolution is 6.5 mA per LSB.

    Args:
        cal: :class:`JointCal` for the target servo.

    Returns:
        Phase current in milliamps.

    Raises:
        RuntimeError: If the reply is too short or fails validation.
    """
    INSTR_READ = 0x02
    READ_LEN = 2

    params = bytes([ADDR_PRESENT_AMP, READ_LEN])
    packet = __build_packet(cal.servo_id, INSTR_READ, params)
    cal.comm.write(packet)

    reply = _read_reply(cal)
    if len(reply) < 2:
        raise RuntimeError(f"Current reply too short: {len(reply)} bytes")

    raw = reply[0] | (reply[1] << 8)
    return raw * 6.5  # mA per LSB


def rsbl120_set_torque_limit(cal: JointCal, limit_pct: float) -> None:
    """Set the torque (locked-rotor) limit.

    Args:
        cal:       :class:`JointCal` for the target servo.
        limit_pct: Desired limit as a percentage of maximum torque (0.0-100.0).
                   Internally stored as 0-1000 (unit 0.1 %).
    """
    raw = int(np.clip(limit_pct * 10.0, 0, 1000))
    __write_u16(cal, ADDR_TORQUE_LIMIT, raw)


def rsbl120_zero_to_current_position(cal: JointCal) -> None:
    """Set the software zero to the servo's current physical position.

    Reads the present step position and stores the corresponding angular offset
    in ``cal.zero_position_rad`` so that the current position maps to 0 rad in
    all subsequent commands.

    Args:
        cal: :class:`JointCal` for the target servo.
    """
    s_cur = (
        rsbl120_read_position_step(cal) & 0x7FFF
    )  # mask direction bit (single-turn)
    cal.zero_position_rad = (
        -cal.sign * (float(s_cur) - STEP_CENTER) / DEFAULT_STEP_PER_RAD
    )
    print(
        f"DEBUG: zero_to_current_position -> "
        f"s_cur={s_cur}, zero_position_rad={cal.zero_position_rad:.6f} rad"
    )


def rsbl120_send_move(
    cal: JointCal, pos_rad: float, speed_rpm: float = 0.0
) -> None:
    """Command the servo to move to *pos_rad* at up to *speed_rpm*.

    Writes six consecutive bytes starting at ``ADDR_TARGET_POS`` (0x2A)::

        0x2A-0x2B  Target position  (2 bytes, little-endian)
        0x2C-0x2D  PWM open-loop    (2 bytes, written as 0 in position mode)
        0x2E-0x2F  Running speed    (2 bytes, unit 0.732 RPM; 0 = servo max)

    Args:
        cal:       :class:`JointCal` for the target servo.
        pos_rad:   Desired position in radians.
        speed_rpm: Maximum movement speed in RPM (0 = servo's own maximum).
                   Resolution is 0.732 RPM per LSB.
    """
    pos_step = int(rad_to_step(pos_rad, cal, DEFAULT_STEP_PER_RAD)) & 0xFFFF
    speed_raw = int(np.clip(speed_rpm / 0.732, 0, 32767))

    print(f"DEBUG: send_move -> pos_step={pos_step}, speed_raw={speed_raw}")

    INSTR_WRITE = 0x03
    params = bytes(
        [
            ADDR_TARGET_POS,
            pos_step & 0xFF,
            (pos_step >> 8) & 0xFF,  # 0x2A-0x2B: target position
            0x00,
            0x00,  # 0x2C-0x2D: PWM (unused)
            speed_raw & 0xFF,
            (speed_raw >> 8) & 0xFF,  # 0x2E-0x2F: running speed
        ]
    )
    packet = __build_packet(cal.servo_id, INSTR_WRITE, params)
    cal.comm.write(packet)
