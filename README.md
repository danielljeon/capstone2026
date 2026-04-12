# capstone2026

![black_formatter](https://github.com/danielljeon/capstone2026/actions/workflows/black_formatter.yaml/badge.svg)

Robot arm Mechatronics undergraduate capstone project.

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [capstone2026](#capstone2026)
  * [1 Overview](#1-overview)
    * [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    * [1.2 Block Diagram](#12-block-diagram)
    * [1.3 Operating System](#13-operating-system)
    * [1.4 Python Version](#14-python-version)
  * [2 SDK Implementation](#2-sdk-implementation)
  * [3 Setup and Calibration Scripts](#3-setup-and-calibration-scripts)
    * [3.1 General Setup](#31-general-setup)
    * [3.2 Inverse Kinematics URDF](#32-inverse-kinematics-urdf)
    * [3.3 Actuators Calibration](#33-actuators-calibration)
    * [3.4 Computer Vision Calibration](#34-computer-vision-calibration)
  * [4 Runnables](#4-runnables)
  * [5 Parallel Modules](#5-parallel-modules)
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

### 1.3 Operating System

The core code was developed on an x86 Windows machine. Implementations in Linux
and macOS will require some additional install and setup, especially the
RealSense SDK.

### 1.4 Python Version

As of time of writing Python `3.12` is used due to conflicting versions of
python package compatibility.

---

## 2 SDK Implementation

The [drivers](drivers) directory contains individual motor and actuator
implementations.

Extended logic implementation:

- [`vision.py`](vision.py).
    - Lower vision implementation in [
      `april_tag_realsense.py`](computer_vision/april_tag_realsense.py).
- [`abstracted.py`](main_tasks/abstracted.py).

---

## 3 Setup and Calibration Scripts

### 3.1 General Setup

[`constants.py`](constants.py).

### 3.2 Inverse Kinematics URDF

- The [`robot.urdf`](urdf/robot.urdf) and assets within [urdf](urdf) define the
  robot geometry for inverse kinematics calculations, including:
    - DOF and link lengths.
    - Rotation axis and their joint limits.
    - Zero position.

### 3.3 Actuators Calibration

- [`end_effectors.py`](robot/end_effectors.py) and [
  `motor_joints.py`](robot/motor_joints.py) contain hardware (actuator)
  calibrations such as software joint limits, communication interfaces, control
  and more.

- Waveshare RSBL120-24 and ST3215-12V actuators use NVM assigned motor IDs, set
  with [`motor_id.py`](drivers/motor_id.py).

### 3.4 Computer Vision Calibration

[computer_vision_cals](computer_vision_cals) contains zero position transform
calibrations for known April Tags and targets.

- Generated with [`calibrate.py`](computer_vision/calibrate.py).

---

## 4 Runnables

[`main.py`](main.py).

Planned tasks (located in [main_tasks](main_tasks)):

1. [`task_bolt_tighten.py`](main_tasks/task_bolt_tighten.py).
2. [`task_inchworm.py`](main_tasks/task_inchworm.py).
3. [`task_tool_change_claw.py`](main_tasks/task_tool_change_claw.py).
4. [
   `task_tool_change_screwdriver.py`](main_tasks/task_tool_change_screwdriver.py).
5. [`task_wire.py`](main_tasks/task_wire.py).

Quick demo and test scripts:

1. [`demo_april_tag.py`](demo_april_tag.py).
2. [`demo_read_q.py`](demo_read_q.py).
3. [`demo_run_tool.py`](demo_run_tool.py).
4. [`demo_tool_changer.py`](demo_tool_changer.py).

---

## 5 Parallel Modules

[`virtualizer.py`](virtualizer.py).

- Handles joint angle tracking for virtual only mode.

[`recorder.py`](recorder.py).

- Records joint angle and target related commands throughout the entire system
  operation.
