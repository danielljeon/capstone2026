# capstone2026

![black_formatter](https://github.com/danielljeon/capstone2026/actions/workflows/black_formatter.yaml/badge.svg)

Robot arm Mechatronics undergraduate capstone project.

<details markdown="1">
  <summary>Table of Contents</summary>

---

<!-- TOC -->
* [capstone2026](#capstone2026)
  * [1 Overview](#1-overview)
    * [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    * [1.2 Block Diagram](#12-block-diagram)
  * [2 Setup and Calibration Scripts](#2-setup-and-calibration-scripts)
    * [2.1 General Setup](#21-general-setup)
    * [2.2 Inverse Kinematics URDF](#22-inverse-kinematics-urdf)
    * [2.3 Actuators Calibration](#23-actuators-calibration)
    * [2.4 Computer Vision Calibration](#24-computer-vision-calibration)
  * [3 Runnables](#3-runnables)
  * [4 Post Runtime Recording](#4-post-runtime-recording)
<!-- TOC -->

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer | Description       | Quantity | Notes |
|--------------------------|--------------|-------------------|---------:|-------|
| RSBL120-24               | Waveshare    | RS-485 24 V Motor |        4 |       |
| ST3215-12V               | Waveshare    | UART 12 V Motor   |        2 |       |
| CAN Controller           |              | CAN Controller    |        2 |       |
| Intel RealSense D455f    | RealSense    | Depth Camera      |        1 |       |

### 1.2 Block Diagram

![capstone2026.drawio.png](docs/capstone2026.drawio.png)

> Drawio file here: [capstone2026.drawio](docs/capstone2026.drawio).

---

## 2 Setup and Calibration Scripts

### 2.1 General Setup

[`constants.py`](constants.py).

### 2.2 Inverse Kinematics URDF

- The [`robot.urdf`](urdf/robot.urdf) and assets within [urdf](urdf) define the
  robot geometry for inverse kinematics calculations, including:
    - DOF and link lengths.
    - Rotation axis and their joint limits.
    - Zero position.

### 2.3 Actuators Calibration

- [`end_effectors.py`](robot/end_effectors.py) and [
  `motor_joints.py`](robot/motor_joints.py) contain hardware (actuator)
  calibrations such as software joint limits, communication interfaces, control
  and more.

- Waveshare RSBL120-24 and ST3215-12V actuators use NVM assigned motor IDs, set
  with [`motor_id.py`](drivers/motor_id.py).

### 2.4 Computer Vision Calibration

[computer_vision_cals](computer_vision_cals) contains zero position transform
calibrations for known April Tags and targets.

- Generated with [`calibrate.py`](computer_vision/calibrate.py).

---

## 3 Runnables

[`main.py`](main.py).

Quick demo and test scripts:

- [`demo_april_tag.py`](demo_april_tag.py).
- [`demo_run_tool.py`](demo_run_tool.py).
- [`demo_tool_changer.py`](demo_tool_changer.py).

---

## 4 Post Runtime Recording

[recorder.py](recorder.py).
