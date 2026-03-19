"""CAN-based PWM servo driver using the can_pwm_node DBC definition.

This module provides a high-level API for commanding servos over CAN via the
PWM_NODE firmware. Commands are encoded using the project DBC file and sent with
python-can, using the shared JointCal from robot_arm to stay consistent with the
other servo drivers (motor_rsbl120.py / motor_sts3215.py).

JointCal field usage:
    comm          -> open can.BusABC instance
    servo_id      -> PWM_NODE channel number (1-4)
    sign          -> direction polarity (+1 or -1)
    hardware_zero -> angular offset so physical home = 0 rad

DBC summary (can_pwm_node.dbc):
    command_servo_1  (CAN ID  64) - signal command_servo_1_pwm  [500, 2500] us
    command_servo_2  (CAN ID  96) - signal command_servo_2_pwm  [500, 2500] us
    command_servo_3  (CAN ID 128) - signal command_servo_3_pwm  [500, 2500] us
    command_servo_4  (CAN ID 160) - signal command_servo_4_pwm  [500, 2500] us

    All signals are 16-bit unsigned, little-endian, factor=1, offset=0.

PWM -> radians convention (tunable via module-level constants):
    1500 us  ->   0.0 rad   (servo centre / neutral)
    1000 us  ->  -PI/2 rad
    2000 us  ->  +PI/2 rad
    -> RAD_PER_US = PI / 1000

    These values are standard RC-servo approximations. Adjust PWM_CENTER_US
    and RAD_PER_US to match your actual hardware limits.
"""

from __future__ import annotations

import time
from pathlib import Path

import can
import cantools
import numpy as np

from robot_arm import JointCal

# ---------------------------------------------------------------------------
# DBC path - update if the file is moved relative to this module
# ---------------------------------------------------------------------------

DBC_PATH: Path = Path(__file__).parent / "pwm_node_driver/can_pwm_node.dbc"
"""Absolute path to the CAN database file used for message encoding."""

# ---------------------------------------------------------------------------
# PWM <-> radians conversion constants (tune these for your hardware)
# ---------------------------------------------------------------------------

PWM_MIN_US: int = 500  # Minimum pulse width from DBC range
PWM_MAX_US: int = 2500  # Maximum pulse width from DBC range
PWM_CENTER_US: int = 1500  # Pulse width that maps to 0.0 rad (servo centre)
RAD_PER_US: float = np.pi / 1000.0
"""Radians per microsecond deviation from centre.

Standard approximation: 500 us deviation = PI/2 rad (90 deg).
Adjust together with PWM_CENTER_US to match physical servo limits.
"""

# ---------------------------------------------------------------------------
# CAN channel -> DBC message name mapping (channels 1-4 == servo_id 1-4)
# ---------------------------------------------------------------------------

_CHANNEL_TO_MSG: dict[int, str] = {
    1: "command_servo_1",
    2: "command_servo_2",
    3: "command_servo_3",
    4: "command_servo_4",
}

# ---------------------------------------------------------------------------
# Module-level DBC database (loaded once on import)
# ---------------------------------------------------------------------------

_db: cantools.database.Database = cantools.database.load_file(str(DBC_PATH))


# ---------------------------------------------------------------------------
# CAN bus open/close helpers
# ---------------------------------------------------------------------------


def pwm_node_servo_open_comm(
        interface: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 500_000,
) -> can.BusABC:
    """Open a CAN bus connection.

    The returned bus should be passed as `comm` when constructing a
    :class:`JointCal` for this driver.

    Args:
        interface: python-can interface name (e.g. `"socketcan"`,
                   `"pcan"`, `"kvaser"`).
        channel:   OS channel name (e.g. `"can0"`).
        bitrate:   Bus bitrate in bits/s (default 500 kbit/s).

    Returns:
        An open :class:`can.BusABC` instance ready for use.
    """
    print(
        f"DEBUG: pwm_node_servo_open_comm interface={interface} "
        f"channel={channel}"
    )
    bus = can.interface.Bus(interface=interface, channel=channel,
                            bitrate=bitrate)
    time.sleep(0.05)  # allow hardware to settle
    return bus


def pwm_node_servo_close_comm(bus: can.BusABC) -> None:
    """Shut down and release the CAN bus.

    Args:
        bus: The open :class:`can.BusABC` instance to close.
    """
    print("DEBUG: pwm_node_servo_close_comm")
    bus.shutdown()


# ---------------------------------------------------------------------------
# PWM <-> radians conversion helpers
# ---------------------------------------------------------------------------


def _rad_to_us(pos_rad: float, cal: JointCal) -> int:
    """Convert a position in radians to a PWM pulse width in microseconds.

    Applies `cal.sign` and `cal.hardware_zero` before converting, then
    clamps to the hardware-safe range [`PWM_MIN_US`, `PWM_MAX_US`].

    Args:
        pos_rad: Desired position in radians (software frame).
        cal:     :class:`JointCal` supplying sign and hardware zero offset.

    Returns:
        Pulse width in microseconds, clamped to [`PWM_MIN_US`, `PWM_MAX_US`].
    """
    physical_rad = cal.sign * (pos_rad + cal.hardware_zero)
    us = PWM_CENTER_US + physical_rad / RAD_PER_US
    return int(np.clip(round(us), PWM_MIN_US, PWM_MAX_US))


def _us_to_rad(pulse_us: int, cal: JointCal) -> float:
    """Convert a raw PWM pulse width in microseconds to a position in radians.

    Inverts `cal.sign` and `cal.hardware_zero` so the result is in the
    software frame (consistent with :func:`_rad_to_us`).

    Args:
        pulse_us: Pulse width in microseconds.
        cal:      :class:`JointCal` supplying sign and hardware zero offset.

    Returns:
        Position in radians in the software frame.
    """
    physical_rad = (pulse_us - PWM_CENTER_US) * RAD_PER_US
    return cal.sign * physical_rad - cal.hardware_zero


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def pwm_node_servo_send_move(
        cal: JointCal, pos_rad: float, move_time_ms: int = 0
) -> None:
    """Command the servo to move to *pos_rad*.

    Encodes the target angle as a PWM pulse width using the DBC definition and
    transmits the corresponding CAN frame on `cal.comm`.

    `cal.servo_id` selects the PWM channel (1-4).
    `move_time_ms` is accepted for signature compatibility with
    :func:`execute_q_frames` but is not used — the PWM_NODE has no
    timed-move concept.

    Args:
        cal:          :class:`JointCal` whose `comm` is the CAN bus and
                      `servo_id` is the channel number (1-4).
        pos_rad:      Desired position in radians (software frame).
        move_time_ms: Ignored; present for compatibility with
                      :func:`execute_q_frames`.

    Raises:
        KeyError: If `cal.servo_id` is not in 1-4.
    """
    pulse_us = _rad_to_us(pos_rad, cal)
    msg_name = _CHANNEL_TO_MSG[cal.servo_id]
    signal_name = f"{msg_name}_pwm"

    dbc_msg = _db.get_message_by_name(msg_name)
    data = dbc_msg.encode({signal_name: pulse_us})
    frame = can.Message(
        arbitration_id=dbc_msg.frame_id,
        data=data,
        is_extended_id=False,
    )

    print(
        f"DEBUG: pwm_node_servo_send_move channel={cal.servo_id} "
        f"pos_rad={pos_rad:.4f} -> pulse_us={pulse_us} us"
    )

    cal.comm.send(frame)


def pwm_node_servo_zero_to_current_position(
        cal: JointCal, current_pulse_us: int
) -> None:
    """Set the software zero to a known physical pulse width.

    The PWM_NODE has no position-feedback messages in the DBC, so the current
    position must be supplied by the caller (e.g. read from hardware at startup
    or assumed to be at mechanical centre). The result is stored in
    `cal.hardware_zero`.

    Args:
        cal:              :class:`JointCal` for the target channel.
        current_pulse_us: Current pulse width reported by the hardware (us).
    """
    physical_rad = (current_pulse_us - PWM_CENTER_US) * RAD_PER_US
    cal.hardware_zero = -cal.sign * physical_rad
    print(
        f"DEBUG: pwm_node_servo_zero_to_current_position channel={cal.servo_id} "
        f"current_pulse_us={current_pulse_us} "
        f"-> hardware_zero={cal.hardware_zero:.6f} rad"
    )


def pwm_node_servo_zero_to_centre(cal: JointCal) -> None:
    """Convenience wrapper: treat the servo centre (1500 us) as 0 rad.

    Equivalent to calling
    `pwm_node_servo_zero_to_current_position(cal, PWM_CENTER_US)`.

    Args:
        cal: :class:`JointCal` for the target channel.
    """
    pwm_node_servo_zero_to_current_position(cal, PWM_CENTER_US)
