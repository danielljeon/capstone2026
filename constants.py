# constants.py

IK_FILE_CSV = ".csv"

# CAN bus.
PWM_NODE_SERVO_INTERFACE = "socketcan"
PWM_NODE_SERVO_CHANNEL = "can0"
PWM_NODE_SERVO_BITRATE = 500_000

# Serial ports.
RSBL120_PORT = "COM4"  # "/dev/cu.usbmodem5ABA0052171"
ST3215_PORT = "COM3"  # "/dev/cu.usbmodem5A460829321"

# IK calculation and execution frame time step (seconds).
IK_DT_S = 0.1
