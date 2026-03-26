from pathlib import Path

import can
import cantools

DBC_PATH: Path = Path(__file__).parent / "can_pwm_node.dbc"
db: cantools.database.Database = cantools.database.load_file(str(DBC_PATH))


CHANNEL_TO_MSG: dict[int, str] = {
    1: "command_servo_1_01",
    2: "command_servo_2_01",
    3: "command_servo_3_01",
    4: "command_servo_4_01",
    5: "command_servo_1_02",
    6: "command_servo_2_02",
    7: "command_servo_3_02",
    8: "command_servo_4_02",
}

CHANNEL_TO_SIG: dict[int, str] = {
    1: "command_servo_1_pwm",
    2: "command_servo_2_pwm",
    3: "command_servo_3_pwm",
    4: "command_servo_4_pwm",
    5: "command_servo_1_pwm",
    6: "command_servo_2_pwm",
    7: "command_servo_3_pwm",
    8: "command_servo_4_pwm",
}


def pwm_node_send(bus: can.BusABC, channel: int, pulse_us: int) -> None:
    dbc_msg = db.get_message_by_name(CHANNEL_TO_MSG[channel])
    data = dbc_msg.encode({CHANNEL_TO_SIG[channel]: int(pulse_us)})
    bus.send(
        can.Message(
            arbitration_id=dbc_msg.frame_id,
            data=data,
            is_extended_id=False,
        )
    )
