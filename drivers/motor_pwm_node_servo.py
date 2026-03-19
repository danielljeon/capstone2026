"""CAN-based PWM servo driver using the can_pwm_node DBC definition.

This module provides a high-level API for commanding servos over CAN via the
PWM_NODE firmware.  Commands are encoded using the project DBC file and sent
with python-can, mirroring the structure of the serial servo drivers
(motor_rsbl120.py / motor_sts3215.py).

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

    These values are standard RC-servo approximations.  Adjust PWM_CENTER_US
    and RAD_PER_US to match your actual hardware limits.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import can
import cantools
import numpy as np

# ---------------------------------------------------------------------------
# DBC path - update if the file is moved relative to this module
# ---------------------------------------------------------------------------

DBC_PATH: Path = Path(__file__).parent / "pwm_node_driver/can_pwm_node.dbc"
"""Absolute path to the CAN database file used for message encoding."""

# ---------------------------------------------------------------------------
# PWM <-> radians conversion constants  (tune these for your hardware)
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
# CAN channel -> DBC message name mapping  (channels 1-4)
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
# Configuration dataclass
# ---------------------------------------------------------------------------


@dataclass
class CanJointCal:
    """Runtime configuration for a single CAN-connected servo channel.

    Attributes:
        bus:              Open :class:`can.BusABC` instance for this servo.
        channel:          PWM_NODE servo channel number (1-4).
        sign:             Direction polarity (+1 or -1).  Multiply the
                          commanded angle by this before converting to PWM.
        zero_position_rad: Angular offset (rad) applied so that the physical
                          home position corresponds to 0 rad in commands.
    """

    bus: can.BusABC
    channel: int
    sign: int = 1
    zero_position_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.channel not in _CHANNEL_TO_MSG:
            raise ValueError(
                f"channel must be 1-4, got {self.channel}"
            )
        if self.sign not in (1, -1):
            raise ValueError(f"sign must be +1 or -1, got {self.sign}")


# ---------------------------------------------------------------------------
# Serial-port-style open/close helpers (CAN bus)
# ---------------------------------------------------------------------------


def pwm_node_servo_open_comm(
        interface: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 500_000,
) -> can.BusABC:
    """Open a CAN bus connection.

    Args:
        interface: python-can interface name (e.g. `"socketcan"`,
                   `"pcan"`, `"kvaser"`).
        channel:   OS channel name (e.g. `"can0"`).
        bitrate:   Bus bitrate in bits/s (default 1 Mbit/s).

    Returns:
        An open :class:`can.BusABC` instance ready for use.
    """
    print(
        f"DEBUG: pwm_node_servo_open_comm interface={interface} "
        f"channel={channel}")
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


def __rad_to_us(pos_rad: float, cal: CanJointCal) -> int:
    """Convert a position in radians to a PWM pulse width in microseconds.

    Applies `cal.sign` and `cal.zero_position_rad` before converting, then
    clamps to the hardware-safe range `[PWM_MIN_US, PWM_MAX_US]`.

    Args:
        pos_rad: Desired position in radians (in the software frame).
        cal:     :class:`CanJointCal` supplying sign and zero offset.

    Returns:
        Pulse width in microseconds, clamped to [`PWM_MIN_US`, `PWM_MAX_US`].
    """
    # Apply zero-offset and direction polarity
    physical_rad = cal.sign * (pos_rad + cal.zero_position_rad)
    us = PWM_CENTER_US + physical_rad / RAD_PER_US
    return int(np.clip(round(us), PWM_MIN_US, PWM_MAX_US))


def __us_to_rad(pulse_us: int, cal: CanJointCal) -> float:
    """Convert a raw PWM pulse width to a position in radians.

    Inverts `cal.sign` and `cal.zero_position_rad` so the result is in the
    software frame (consistent with :func:`__rad_to_us`).

    Args:
        pulse_us: Pulse width in microseconds.
        cal:      :class:`CanJointCal` supplying sign and zero offset.

    Returns:
        Position in radians in the software frame.
    """
    physical_rad = (pulse_us - PWM_CENTER_US) * RAD_PER_US
    return cal.sign * physical_rad - cal.zero_position_rad


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def pwm_node_servo_send_move(cal: CanJointCal, pos_rad: float) -> None:
    """Command the servo to move to *pos_rad*.

    Encodes the target angle as a PWM pulse width using the DBC definition and
    transmits the corresponding CAN frame.

    Args:
        cal:     :class:`CanJointCal` for the target channel.
        pos_rad: Desired position in radians (software frame).
    """
    pulse_us = __rad_to_us(pos_rad, cal)
    msg_name = _CHANNEL_TO_MSG[cal.channel]
    signal_name = f"{msg_name}_pwm"

    dbc_msg = _db.get_message_by_name(msg_name)
    data = dbc_msg.encode({signal_name: pulse_us})
    frame = can.Message(
        arbitration_id=dbc_msg.frame_id,
        data=data,
        is_extended_id=False,
    )

    print(
        f"DEBUG: send_move channel={cal.channel} "
        f"pos_rad={pos_rad:.4f} -> pulse_us={pulse_us} us"
    )

    cal.bus.send(frame)


def pwm_node_servo_zero_to_current_position(cal: CanJointCal,
                                            current_pulse_us: int) -> None:
    """Set the software zero to a known physical pulse width.

    The PWM_NODE has no position-feedback messages in the DBC, so the current
    position must be supplied by the caller (e.g. read from hardware at startup
    or assumed to be at the mechanical centre).

    Args:
        cal:              :class:`CanJointCal` for the target channel.
        current_pulse_us: Current pulse width reported by the hardware (us).
    """
    physical_rad = (current_pulse_us - PWM_CENTER_US) * RAD_PER_US
    cal.zero_position_rad = -cal.sign * physical_rad
    print(
        f"DEBUG: zero_to_current_position channel={cal.channel} "
        f"current_pulse_us={current_pulse_us} "
        f"-> zero_position_rad={cal.zero_position_rad:.6f} rad"
    )


def pwm_node_servo_zero_to_centre(cal: CanJointCal) -> None:
    """Convenience wrapper: treat the servo centre (1500 us) as 0 rad.

    Equivalent to calling `zero_to_current_position(cal, PWM_CENTER_US)`.

    Args:
        cal: :class:`CanJointCal` for the target channel.
    """
    pwm_node_servo_zero_to_current_position(cal, PWM_CENTER_US)
