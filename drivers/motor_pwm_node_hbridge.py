"""L298N H-bridge driver over the CAN PWM node.

Speed is a normalised float in [0.0, 1.0] mapping to the EN PWM pulse width.
Direction is set by the sign passed to :func:`hbridge_drive`.

Digital HIGH/LOW pulse widths and the EN PWM range can be tuned via the
constants at the top of this file to match your firmware's thresholds.
"""

from __future__ import annotations
from cantools.database import Message
import time
from dataclasses import dataclass

import can
import numpy as np

from drivers.motor_pwm_node_constants import pwm_node_send


@dataclass
class HBridge:
    channel_in1: int
    channel_in2: int
    channel_enable: int
    current_message: Message


PWM_DIGITAL_HIGH_US: int = 20000  # pulse width the firmware reads as logic HIGH
PWM_DIGITAL_LOW_US: int = 0  # pulse width the firmware reads as logic LOW
PWM_EN_MIN_US: int = 0  # EN pulse width at speed = 0.0
PWM_EN_MAX_US: int = 20000  # EN pulse width at speed = 1.0


def _en_us(speed: float) -> int:
    """Map normalised speed [0.0, 1.0] to an EN pulse width in microseconds."""
    return int(
        round(
            PWM_EN_MIN_US
            + np.clip(speed, 0.0, 1.0) * (PWM_EN_MAX_US - PWM_EN_MIN_US)
        )
    )


def hbridge_drive(
    bus: can.BusABC,
    hbridge: HBridge,
    speed: float,
    duration_s: float,
    *,
    brake: bool = False,
    coast: bool = False,
    reverse: bool = False,
) -> None:
    """Drive the H-bridge at *speed* for *duration_s* seconds, then brake.

    Args:
        bus: Open CAN bus.
        hbridge: HBridge object to identify PWM Node channels.
        speed: Normalised speed in [0.0, 1.0].
        duration_s: How long to run before braking, in seconds.
        brake: If True, brakes the motor after moving, default False.
        coast: If True, coast the motor after moving, brake overrides this!
        reverse: If True, reverses the motor direction, default False.
    """
    in1_us = PWM_DIGITAL_LOW_US if reverse else PWM_DIGITAL_HIGH_US
    in2_us = PWM_DIGITAL_HIGH_US if reverse else PWM_DIGITAL_LOW_US
    direction_str = "reverse" if reverse else "forward"

    print(
        f"DEBUG: hbridge_drive {direction_str} "
        f"speed={speed:.2f} duration={duration_s}s"
    )

    pwm_node_send(bus, hbridge.channel_in1, in1_us)
    pwm_node_send(bus, hbridge.channel_in2, in2_us)
    pwm_node_send(bus, hbridge.channel_enable, _en_us(speed))

    time.sleep(duration_s)

    if coast and not brake:
        hbridge_coast(bus, hbridge)
    if brake:
        hbridge_brake(bus, hbridge)


def hbridge_brake(bus: can.BusABC, hbridge: HBridge) -> None:
    """Active brake: IN1=HIGH, IN2=HIGH, EN=HIGH.

    Args:
        bus: Open CAN bus.
        hbridge: HBridge object to identify PWM Node channels.
    """
    print("DEBUG: hbridge_brake")
    pwm_node_send(bus, hbridge.channel_in1, PWM_DIGITAL_HIGH_US)
    pwm_node_send(bus, hbridge.channel_in2, PWM_DIGITAL_HIGH_US)
    pwm_node_send(bus, hbridge.channel_enable, PWM_DIGITAL_HIGH_US)


def hbridge_coast(bus: can.BusABC, hbridge: HBridge) -> None:
    """Coast / free-wheel: IN1=LOW, IN2=LOW, EN=LOW.

    Args:
        bus: Open CAN bus.
        hbridge: HBridge object to identify PWM Node channels.
    """
    print("DEBUG: hbridge_coast")
    pwm_node_send(bus, hbridge.channel_in1, PWM_DIGITAL_LOW_US)
    pwm_node_send(bus, hbridge.channel_in2, PWM_DIGITAL_LOW_US)
    pwm_node_send(bus, hbridge.channel_enable, PWM_DIGITAL_LOW_US)
