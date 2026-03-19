from robot_arm import *
from constants import *
from drivers.motor_pwm_node_servo import (
    pwm_node_servo_send_move,
    pwm_node_servo_open_comm,
    pwm_node_servo_close_comm,
)

BASE_TC = JointCal(
    name="base_tool_changer",
    comm=None,
    servo_id=1,
    sign=1,
    hardware_zero=0,
)


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
