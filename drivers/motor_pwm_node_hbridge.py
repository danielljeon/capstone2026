"""L298N H-bridge driver over the CAN PWM node.

Channels are hardcoded to match the physical wiring:
    CH2 -> L298N IN1  (direction)
    CH3 -> L298N IN2  (direction)
    CH4 -> L298N EN   (speed / PWM duty)

Speed is a normalised float in [0.0, 1.0] mapping to the EN PWM pulse width.
Direction is set by the sign passed to :func:`hbridge_drive`.

Digital HIGH/LOW pulse widths and the EN PWM range can be tuned via the
constants at the top of this file to match your firmware's thresholds.
"""

from __future__ import annotations

import time
from pathlib import Path

import can
import cantools
import numpy as np

# ---------------------------------------------------------------------------
# DBC
# ---------------------------------------------------------------------------

DBC_PATH: Path = Path(__file__).parent / "pwm_node_driver/can_pwm_node.dbc"
_db: cantools.database.Database = cantools.database.load_file(str(DBC_PATH))

# ---------------------------------------------------------------------------
# Hardcoded channel assignments
# ---------------------------------------------------------------------------

_CH_IN1 = 2
_CH_IN2 = 3
_CH_EN = 4

# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------

PWM_DIGITAL_HIGH_US: int = 20000  # pulse width the firmware reads as logic HIGH
PWM_DIGITAL_LOW_US: int = 0  # pulse width the firmware reads as logic LOW
PWM_EN_MIN_US: int = 0  # EN pulse width at speed = 0.0
PWM_EN_MAX_US: int = 20000  # EN pulse width at speed = 1.0


# ---------------------------------------------------------------------------
# Internal helper
# ---------------------------------------------------------------------------


def _send(bus: can.BusABC, channel: int, pulse_us: int) -> None:
    msg_name = f"command_servo_{channel}"
    dbc_msg = _db.get_message_by_name(msg_name)
    data = dbc_msg.encode({f"{msg_name}_pwm": int(pulse_us)})
    bus.send(
        can.Message(
            arbitration_id=dbc_msg.frame_id,
            data=data,
            is_extended_id=False,
        )
    )


def _en_us(speed: float) -> int:
    """Map normalised speed [0.0, 1.0] to an EN pulse width in microseconds."""
    return int(
        round(
            PWM_EN_MIN_US
            + np.clip(speed, 0.0, 1.0) * (PWM_EN_MAX_US - PWM_EN_MIN_US)
        )
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def hbridge_drive(
    bus: can.BusABC,
    speed: float,
    duration_s: float,
    *,
    reverse: bool = False,
) -> None:
    """Drive the H-bridge at *speed* for *duration_s* seconds, then brake.

    Args:
        bus:        Open CAN bus.
        speed:      Normalised speed in [0.0, 1.0].
        duration_s: How long to run before braking, in seconds.
        reverse:    If True, reverses the motor direction.
    """
    in1_us = PWM_DIGITAL_LOW_US if reverse else PWM_DIGITAL_HIGH_US
    in2_us = PWM_DIGITAL_HIGH_US if reverse else PWM_DIGITAL_LOW_US
    direction_str = "reverse" if reverse else "forward"

    print(
        f"DEBUG: hbridge_drive {direction_str} "
        f"speed={speed:.2f} duration={duration_s}s"
    )

    _send(bus, _CH_IN1, in1_us)
    _send(bus, _CH_IN2, in2_us)
    _send(bus, _CH_EN, _en_us(speed))

    time.sleep(duration_s)
    hbridge_brake(bus)


def hbridge_brake(bus: can.BusABC) -> None:
    """Active brake: IN1=HIGH, IN2=HIGH, EN=min.

    Args:
        bus: Open CAN bus.
    """
    print("DEBUG: hbridge_brake")
    _send(bus, _CH_IN1, PWM_DIGITAL_HIGH_US)
    _send(bus, _CH_IN2, PWM_DIGITAL_HIGH_US)
    _send(bus, _CH_EN, PWM_EN_MIN_US)


def hbridge_coast(bus: can.BusABC) -> None:
    """Coast / free-wheel: IN1=LOW, IN2=LOW, EN=min.

    Args:
        bus: Open CAN bus.
    """
    print("DEBUG: hbridge_coast")
    _send(bus, _CH_IN1, PWM_DIGITAL_LOW_US)
    _send(bus, _CH_IN2, PWM_DIGITAL_LOW_US)
    _send(bus, _CH_EN, PWM_EN_MIN_US)
