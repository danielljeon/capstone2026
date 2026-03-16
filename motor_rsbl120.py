"""Python servo motor control (SMS/STS series, FT-SCS protocol).

Register map source: Magnetic Encoding SMS/STS Servo Memory Table
Protocol source: Smart Bus Servo Communication Protocol Manual

Key protocol facts:
  - Packet:  FF FF | ID | Length | Instr | Param... | Checksum
  - Reply:   FF FF | ID | Length | Error | Param... | Checksum
  - Length field = number_of_params + 2  (covers Error/Instr byte + Checksum byte)
  - Checksum = ~(ID + Length + Instr/Error + Param1 + ... + ParamN) & 0xFF
  - Two-byte values: LOW byte first, HIGH byte second (little-endian)

Lock flag semantics (addr 0x37):
  - Write 0  -> close lock  -> EPROM writes ARE saved on power-off
  - Write 1  -> open lock   -> EPROM writes are NOT saved (factory default)
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

BROADCAST_ID = 0xFE

# -- EPROM addresses (persist across power cycles when lock is closed) -------
ADDR_ID = 0x05  # Servo ID                (1 byte,  0-253)
ADDR_MAX_TORQUE = 0x10  # Maximum torque          (2 bytes, 0-1000, unit 0.1%)

# -- SRAM Control addresses (volatile, reset on power-off unless lock=0) ----
ADDR_TORQUE_SWITCH = 0x28  # Torque on/off/damping   (1 byte)
ADDR_ACCELERATION = (
    0x29  # Acceleration            (1 byte,  0-254, unit 8.7deg/s^2)
)
ADDR_TARGET_POS = (
    0x2A  # Target position         (2 bytes, -32767-32767, BIT15=dir)
)
ADDR_PWM_SPEED = 0x2C  # PWM / open-loop speed   (2 bytes) - write 0 in pos mode
ADDR_RUNNING_SPEED = 0x2E  # Running speed limit     (2 bytes, unit 0.732 RPM)
ADDR_TORQUE_LIMIT = 0x30  # Torque limit            (2 bytes, 0-1000, unit 0.1%)
ADDR_LOCK = 0x37  # EPROM write-lock flag   (1 byte)  <- was wrongly 0x30

# -- SRAM Feedback addresses (read-only) ------------------------------------
ADDR_PRESENT_POS = 0x38  # Current position        (2 bytes, unit 0.087deg)
ADDR_PRESENT_SPEED = 0x3A  # Current speed           (2 bytes, unit 0.732 RPM)
ADDR_PRESENT_LOAD = 0x3C  # Current load / PWM duty (2 bytes, unit 0.1%)
ADDR_PRESENT_VOLT = 0x3E  # Current voltage         (1 byte,  unit 0.1 V)
ADDR_PRESENT_TEMP = 0x3F  # Current temperature     (1 byte,  unit degC)
ADDR_SERVO_STATUS = 0x41  # Servo error status bits (1 byte)
ADDR_PRESENT_AMP = 0x45  # Current phase current   (2 bytes, unit 6.5 mA)

# Torque switch values
TORQUE_OFF = 0
TORQUE_ON = 1
TORQUE_DAMPING = 2

# Position centre step (mid-point of 0-4095 range)
STEP_CENTER = 2048


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------


def open_actuator_serial_comm(port: str) -> serial.Serial:
    print(f"DEBUG: open_actuator_serial_comm {port}")
    comm = serial.Serial(port, 1_000_000, timeout=0.1)
    time.sleep(0.2)  # allow hardware to settle
    return comm


def close_actuator_comm(comm: serial.Serial) -> None:
    print("DEBUG: close_actuator_comm")
    comm.close()


def _read_exact(comm: serial.Serial, n: int) -> bytes:
    """Read exactly *n* bytes, respecting the port timeout."""
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


def _build_packet(servo_id: int, instr: int, params: bytes) -> bytes:
    """Build a complete FT-SCS instruction packet.

    Structure: FF FF | ID | Length | Instr | Params... | Checksum
    Length = len(params) + 2
    """
    length = len(params) + 2
    body = bytes([servo_id, length, instr]) + params
    checksum = (~sum(body)) & 0xFF
    return b"\xff\xff" + body + bytes([checksum])


def write_u8(
        cal: JointCal, addr: int, value: int, packet_id: int | None = None
) -> None:
    """Write a single byte to *addr* on the servo."""
    servo_id = cal.servo_id if packet_id is None else packet_id
    INSTR_WRITE = 0x03
    packet = _build_packet(
        servo_id, INSTR_WRITE, bytes([addr & 0xFF, value & 0xFF])
    )
    cal.comm.write(packet)


def write_u16(
        cal: JointCal, addr: int, value: int, packet_id: int | None = None
) -> None:
    """Write a 16-bit value to *addr* (little-endian: low byte first)."""
    servo_id = cal.servo_id if packet_id is None else packet_id
    INSTR_WRITE = 0x03
    v = int(value) & 0xFFFF
    params = bytes([addr & 0xFF, v & 0xFF, (v >> 8) & 0xFF])
    packet = _build_packet(servo_id, INSTR_WRITE, params)
    cal.comm.write(packet)


# ---------------------------------------------------------------------------
# Reply parser
# ---------------------------------------------------------------------------


def _read_reply(cal: JointCal) -> bytes:
    """Read and validate a reply packet; return the parameter bytes.

    Reply format: FF FF | ID | Length | Error | Param1..ParamN | Checksum
    Length = N_params + 2  (Error byte + Checksum byte)
    So N_params = Length - 2.
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
    header = _read_exact(comm, 3)
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
    # length covers: Error(1) + params(N) + Checksum(1)  -> tail = length - 1 bytes
    tail = _read_exact(comm, length - 1)
    if len(tail) != length - 1:
        raise RuntimeError("Short reply: truncated parameter/checksum bytes")

    params = tail[:-1]  # everything except last byte
    rx_chksum = tail[-1]

    # Verify checksum
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


def set_servo_id_nvm(cal: JointCal, new_id: int) -> None:
    """Permanently change the servo ID and persist it to EPROM.

    Sequence:
      1. Close write-lock on current ID (write 0 -> saves on power-off).
      2. Write new ID using broadcast (safe if wiring only has one servo).
      3. Update cal.servo_id so subsequent calls reach the servo.
      4. Re-open write-lock on new ID (write 1 -> default, no accidental saves).

    Args:
        cal:    JointCal with the *current* servo_id.
        new_id: Target ID (0-253).
    """
    if not (0 <= new_id <= 253):
        raise ValueError(f"Servo ID must be 0-253, got {new_id}")

    # Step 1 - close lock so EPROM writes are retained
    write_u8(cal, ADDR_LOCK, 0)
    time.sleep(0.05)

    # Step 2 - broadcast new ID (0x05) so it works even if current ID is uncertain
    write_u8(cal, ADDR_ID, new_id, packet_id=BROADCAST_ID)
    time.sleep(0.1)

    # Step 3 - update software state to new ID
    cal.servo_id = new_id

    # Step 4 - re-open lock (factory default; prevents accidental NVM wear)
    write_u8(cal, ADDR_LOCK, 1)
    time.sleep(0.05)

    print(f"DEBUG: set_servo_id_nvm -> new ID={new_id}")


def enable_actuator(cal: JointCal) -> None:
    """Enable torque output on the servo."""
    write_u8(cal, ADDR_TORQUE_SWITCH, TORQUE_ON)


def disable_actuator(cal: JointCal) -> None:
    """Disable torque output (servo goes limp)."""
    write_u8(cal, ADDR_TORQUE_SWITCH, TORQUE_OFF)


def set_damping(cal: JointCal) -> None:
    """Set servo to damping output mode (passive resistance)."""
    write_u8(cal, ADDR_TORQUE_SWITCH, TORQUE_DAMPING)


def scan_for_servo(
        comm: serial.Serial, id_min: int = 0, id_max: int = 253
) -> list[int]:
    """Scan the bus for responding servos by sending PING to each ID in range.

    Args:
        comm:    Open serial port to scan on.
        id_min:  First ID to try (default 0).
        id_max:  Last ID to try inclusive (default 253).

    Returns:
        List of IDs that responded, in ascending order. Empty list if none found.
    """
    INSTR_PING = 0x01
    found = []

    print(f"DEBUG: scan_for_servo scanning IDs {id_min}–{id_max} ...")

    for servo_id in range(id_min, id_max + 1):
        comm.reset_input_buffer()
        packet = _build_packet(servo_id, INSTR_PING, b"")
        comm.write(packet)

        # Reply: FF FF | ID | 0x02 | Error | Checksum  (4 bytes after preamble)
        # Use a short timeout — most servos reply within a few ms
        deadline = time.time() + 0.02  # 20 ms per ID
        buf = bytearray()
        while time.time() < deadline:
            chunk = comm.read(comm.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
            if len(buf) >= 6:
                break

        # Look for FF FF preamble in whatever we received
        raw = bytes(buf)
        idx = raw.find(b"\xff\xff")
        if idx == -1:
            continue

        reply = raw[idx + 2:]  # skip preamble
        if len(reply) < 4:
            continue  # too short to be a valid reply

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
                f"DEBUG: scan_for_servo → found servo at ID {servo_id} (error=0x{error:02X})"
            )
            found.append(servo_id)

    if not found:
        print("DEBUG: scan_for_servo → no servos found")

    return found


def reset_servo_id(cal: JointCal) -> None:
    """Reset the servo's ID back to the factory default (1) using the RESET instruction.

    The RESET instruction (0x06) restores the entire control table to factory
    values, which includes setting ID back to 1. Use with caution — all other
    EPROM settings (PID gains, angle limits, baud rate, etc.) are also reset.

    After this call, cal.servo_id is updated to 1 so subsequent commands work
    without re-constructing the JointCal.

    Args:
        cal: JointCal with the current (known) servo_id.
    """
    INSTR_RESET = 0x06
    packet = _build_packet(cal.servo_id, INSTR_RESET, b"")
    cal.comm.write(packet)
    time.sleep(0.5)  # give the servo time to reboot and apply factory defaults

    cal.servo_id = 1
    print(
        f"DEBUG: reset_servo_id → servo reset to factory defaults, ID is now 1"
    )


def read_position_step(cal: JointCal) -> int:
    """Read the current absolute position in raw steps (0-4095 nominal).

    Returns:
        Integer step count. BIT15 encodes direction for multi-turn; mask it off
        for single-turn use: ``pos & 0x7FFF``.
    """
    INSTR_READ = 0x02
    READ_LEN = 2

    # cal.comm.reset_input_buffer()  # TODO

    params = bytes([ADDR_PRESENT_POS, READ_LEN])
    packet = _build_packet(cal.servo_id, INSTR_READ, params)
    cal.comm.write(packet)

    reply = _read_reply(cal)

    if len(reply) < 2:
        raise RuntimeError(f"Position reply too short: {len(reply)} bytes")

    # Low byte first, high byte second (little-endian per protocol spec)
    pos = reply[0] | (reply[1] << 8)
    return pos


def read_current_ma(cal: JointCal) -> float:
    """Read the current phase current in milliamps.

    Returns:
        Current in mA (unit per register: 6.5 mA / LSB).
    """
    INSTR_READ = 0x02
    READ_LEN = 2

    # cal.comm.reset_input_buffer()  # TODO

    params = bytes([ADDR_PRESENT_AMP, READ_LEN])
    packet = _build_packet(cal.servo_id, INSTR_READ, params)
    cal.comm.write(packet)

    reply = _read_reply(cal)

    if len(reply) < 2:
        raise RuntimeError(f"Current reply too short: {len(reply)} bytes")

    raw = reply[0] | (reply[1] << 8)
    return raw * 6.5  # mA


def set_torque_limit(cal: JointCal, limit_pct: float) -> None:
    """Set the torque (locked-rotor) limit.

    Args:
        limit_pct: Desired limit as a percentage of maximum torque (0.0-100.0).
                   Internally stored as 0-1000 (unit 0.1%).
    """
    raw = int(np.clip(limit_pct * 10.0, 0, 1000))
    write_u16(cal, ADDR_TORQUE_LIMIT, raw)


def zero_to_current_position(cal: JointCal) -> None:
    """Set the software zero to wherever the servo is right now.

    Reads the current step position and calculates the angular offset so that
    the current physical position is treated as 0 rad in subsequent commands.
    """
    s_cur = read_position_step(cal)
    # Mask direction bit for single-turn interpretation
    s_cur = s_cur & 0x7FFF
    cal.zero_position_rad = (
            -cal.sign * (float(s_cur) - STEP_CENTER) / DEFAULT_STEP_PER_RAD
    )
    print(
        f"DEBUG: zero_to_current_position -> "
        f"s_cur={s_cur}, zero_position_rad={cal.zero_position_rad:.6f} rad"
    )


def send_move(cal: JointCal, pos_rad: float, speed_rpm: float = 0.0) -> None:
    """Command the servo to move to *pos_rad* at up to *speed_rpm*.

    Writes six consecutive bytes starting at ADDR_TARGET_POS (0x2A):
      0x2A-0x2B  Target position  (2 bytes, little-endian)
      0x2C-0x2D  PWM open-loop    (2 bytes) - written as 0 in position mode
      0x2E-0x2F  Running speed    (2 bytes, unit 0.732 RPM; 0 = max speed)

    Args:
        cal:       JointCal for the target servo.
        pos_rad:   Desired position in radians.
        speed_rpm: Maximum movement speed in RPM (0 = servo's own maximum).
                   Resolution is 0.732 RPM per LSB.
    """
    pos_step = int(rad_to_step(pos_rad, cal, DEFAULT_STEP_PER_RAD)) & 0xFFFF
    speed_raw = int(np.clip(speed_rpm / 0.732, 0, 32767))  # 0.732 RPM per LSB

    print(f"DEBUG: send_move -> pos_step={pos_step}, speed_raw={speed_raw}")

    INSTR_WRITE = 0x03

    # Six data bytes written as a single contiguous block from 0x2A
    params = bytes(
        [
            ADDR_TARGET_POS,
            pos_step & 0xFF,
            (pos_step >> 8) & 0xFF,  # 0x2A-0x2B: target position
            0x00,
            0x00,  # 0x2C-0x2D: PWM (unused in pos mode)
            speed_raw & 0xFF,
            (speed_raw >> 8) & 0xFF,  # 0x2E-0x2F: running speed
        ]
    )
    packet = _build_packet(cal.servo_id, INSTR_WRITE, params)
    cal.comm.write(packet)
