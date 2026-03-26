"""CAN-based PWM servo driver using the can_pwm_node DBC definition."""

from __future__ import annotations

import time

import can
import numpy as np

from drivers.motor_pwm_node_constants import db, CHANNEL_TO_MSG, CHANNEL_TO_SIG
from robot_arm import JointCal

# Using a variant of the MG90s hobby servos:
PWM_MIN_US: int = 500  # Minimum pulse width from DBC range
PWM_MAX_US: int = 2500  # Maximum pulse width from DBC range
PWM_CENTER_US: int = 500  # Pulse width that maps to 0.0 rad (servo centre)
RAD_PER_US: float = (np.pi) / 2600  # Radians per microsecond from centre


def pwm_node_servo_open_comm(
    interface: str = "socketcan",
    channel: str | int = "can0",
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
    bus = can.interface.Bus(
        interface=interface, channel=channel, bitrate=bitrate
    )
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

    Applies `cal.sign` and `cal.hardware_zero`, then clamps to the hardware-safe
    range [`PWM_MIN_US`, `PWM_MAX_US`].

    Args:
        pos_rad: Desired position in radians (software frame).
        cal:     :class:`JointCal` supplying sign and hardware zero offset.

    Returns:
        Pulse width in microseconds, clamped to [`PWM_MIN_US`, `PWM_MAX_US`].
    """
    physical_rad = cal.sign * pos_rad
    us = physical_rad / RAD_PER_US + PWM_CENTER_US + cal.hardware_zero
    return int(np.clip(round(us), PWM_MIN_US, PWM_MAX_US))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def pwm_node_servo_send_move(
    cal: JointCal, pos_rad: float, move_time_ms: int = 0
) -> None:
    """Command the servo to move to *pos_rad*.

    Encodes the target angle as a PWM pulse width using the DBC definition and
    transmits the corresponding CAN frame on `cal.comm`.

    `cal.servo_id` selects the PWM channel (1-8).
    `move_time_ms` is accepted for signature compatibility with
    :func:`execute_q_frames` but is not used - the PWM_NODE has no
    timed-move concept.

    Args:
        cal:          :class:`JointCal` whose `comm` is the CAN bus and
                      `servo_id` is the channel number (1-8).
        pos_rad:      Desired position in radians (software frame).
        move_time_ms: Ignored; present for compatibility with
                      :func:`execute_q_frames`.

    Raises:
        KeyError: If `cal.servo_id` is not in 1-8.
    """
    pulse_us = _rad_to_us(pos_rad, cal)

    dbc_msg = db.get_message_by_name(CHANNEL_TO_MSG[cal.servo_id])
    data = dbc_msg.encode({CHANNEL_TO_SIG[cal.servo_id]: pulse_us})
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
