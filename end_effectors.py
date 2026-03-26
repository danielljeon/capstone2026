from constants import *
from drivers.motor_pwm_node_hbridge import HBridge, hbridge_drive
from drivers.motor_pwm_node_servo import (
    pwm_node_servo_send_move,
    pwm_node_servo_open_comm,
    pwm_node_servo_close_comm,
)
from robot_arm import *

EE1_TC = JointCal(
    name="ee1_tool_changer",
    comm=None,
    servo_id=1,
    sign=1,
    hardware_zero=0,
)
EE1_TOOL = HBridge(channel_in1=2, channel_in2=3, channel_enable=4)
EE2_TC = JointCal(
    name="ee2_tool_changer",
    comm=None,
    servo_id=5,
    sign=1,
    hardware_zero=0,
)
EE2_TOOL = HBridge(channel_in1=6, channel_in2=7, channel_enable=8)


def release_tool_changer(cal: JointCal):
    pwm_node_servo_comm = pwm_node_servo_open_comm(
        PWM_NODE_SERVO_INTERFACE, PWM_NODE_SERVO_CHANNEL, PWM_NODE_SERVO_BITRATE
    )
    cal.comm = pwm_node_servo_comm
    pwm_node_servo_send_move(cal, 0)
    pwm_node_servo_close_comm(pwm_node_servo_comm)


def lock_tool_changer(cal: JointCal):
    pwm_node_servo_comm = pwm_node_servo_open_comm(
        PWM_NODE_SERVO_INTERFACE, PWM_NODE_SERVO_CHANNEL, PWM_NODE_SERVO_BITRATE
    )
    cal.comm = pwm_node_servo_comm
    pwm_node_servo_send_move(cal, 1.5707)  # 90 deg.
    pwm_node_servo_close_comm(pwm_node_servo_comm)


def open_claw(hbridge: HBridge):
    pwm_node_hbridge_comm = pwm_node_servo_open_comm(
        PWM_NODE_SERVO_INTERFACE, PWM_NODE_SERVO_CHANNEL, PWM_NODE_SERVO_BITRATE
    )
    hbridge_drive(pwm_node_hbridge_comm, hbridge, 1, 3, reverse=False)
    pwm_node_servo_close_comm(pwm_node_hbridge_comm)


def close_claw(hbridge: HBridge):
    pwm_node_hbridge_comm = pwm_node_servo_open_comm(
        PWM_NODE_SERVO_INTERFACE, PWM_NODE_SERVO_CHANNEL, PWM_NODE_SERVO_BITRATE
    )
    hbridge_drive(pwm_node_hbridge_comm, hbridge, 1, 3, reverse=True)
    pwm_node_servo_close_comm(pwm_node_hbridge_comm)
