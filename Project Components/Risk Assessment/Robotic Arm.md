
# Overview
[Link](https://docs.google.com/spreadsheets/d/1xtqAxHWYK6aWHqMlSB8oQumTtSej7oC2NRcMuzSayRY/edit?usp=sharing)
<iframe src="https://docs.google.com/spreadsheets/d/e/2PACX-1vTneMCXInTGZOsa8GAoILBNCtg1X9S1CXvVHoZUdKxvcH7xxjbaZcXa3l_vTJdwRmwCAk6-ZF5Cuyu0/pubhtml?widget=true&amp;headers=false" height="1000px" width="100%"></iframe>

# Manipulator Subsystem: Risk Assessment Report

**Project:** Modular Omni-Wheel Mobile Manipulator (Spring 2026 Mechatronics Omni-Challenge)

**Subsystem:** Arm / Manipulator Module

## 1. Introduction

This document outlines the risk assessment process and results for the Manipulator Subsystem of the mobile robot. The goal of this Failure Mode and Effects Analysis (FMEA) style report is to proactively identify, analyze, and evaluate potential mechanical, electrical, software, and perception risks to minimize their negative impact on the project's performance, safety, and competition constraints.

## 2. Risk Assessment Matrix

The risk score is calculated by multiplying the **Likelihood (L)** and **Impact (I)** ratings. The resulting Risk Score (R) determines the overall Risk Level, which dictates the priority of the mitigation strategies.

|   |   |   |   |   |   |
|---|---|---|---|---|---|
|**Likelihood (L)**|**Description**|**Impact (I)**|**Description**|**Risk Score (R) = L × I**|**Risk Level**|
|**1**|Rare|**1**|Insignificant|**1 - 4**|🟢 **Low**|
|**2**|Unlikely|**2**|Minor|**5 - 9**|🟡 **Medium**|
|**3**|Moderate|**3**|Moderate|**10 - 14**|🟠 **High**|
|**4**|Likely|**4**|Major|**15 - 20**|🔴 **Extreme**|
|**5**|Almost Certain|**5**|Catastrophic|**21 - 25**|🔴 **Extreme**|

## 3. Identified Risks and Mitigation Strategies

### 3.1 Mechanical Risks

Mechanical risks primarily concern the physical structure of the arm, its compliance with spatial constraints, and the physical interaction between the gripper and the target objects.

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|**Risk ID**|**Risk Description**|**L**|**I**|**Score (R)**|**Level**|**Mitigation Strategy**|
|**M-001**|Arm tip deflection and oscillation during fast movements.|4|3|**12**|🟠 High|Use lightweight, stiff materials (e.g., carbon fiber tubes, ribbed 3D prints); implement S-curve motion profiling to reduce jerk.|
|**M-002**|Base joint mechanical failure under high moment load.|2|5|**10**|🟠 High|Use thrust bearings to support the vertical load; reinforce the base bracket with metal rather than pure 3D printed plastics.|
|**M-003**|Robot fails the 50x50x70cm folded dimension constraint.|2|5|**10**|🟠 High|Design a foldable kinematic chain (e.g., SCARA or nested links); physically prototype and measure the folded state in Week 3.|
|**M-004**|Gripper drops the 5x5x5cm cube during base translation.|3|4|**12**|🟠 High|Add high-friction silicone pads to fingers; design V-shaped or interlocking jaws to enclose the cube completely.|
|**M-005**|Wire tangling or snapping during complex 3D arm maneuvers.|4|4|**16**|🔴 Extreme|Route cables internally through hollow joints if possible; use drag chains; leave ample service loops and use flexible silicone wire.|

### 3.2 Electromechanical Risks

These risks exist at the intersection of electrical control and mechanical output, primarily focusing on actuation, torque, and thermal management.

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|**Risk ID**|**Risk Description**|**L**|**I**|**Score (R)**|**Level**|**Mitigation Strategy**|
|**E-001**|Motors undersized for peak reach (stall torque exceeded).|3|5|**15**|🔴 Extreme|Perform worst-case static torque calculations before ordering; add physical counterweights or tension springs to offset gravity.|
|**E-002**|Joint motors overheat while holding the arm in a static position.|3|4|**12**|🟠 High|Select actuators with sufficient thermal mass; implement holding-current reduction in firmware; use worm gears for self-locking joints.|
|**E-003**|Backlash in gearboxes causes major positioning error at the end-effector.|4|3|**12**|🟠 High|Use high-quality planetary or harmonic gearboxes; mount the encoder directly on the output shaft (absolute encoder) rather than the motor.|

### 3.3 Electrical & Power Risks

Electrical risks revolve around power distribution, signal integrity, and the safety of the required plug-and-play interfaces.

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|**Risk ID**|**Risk Description**|**L**|**I**|**Score (R)**|**Level**|**Mitigation Strategy**|
|**P-001**|MCU Brownout/Reset when multiple joints move simultaneously.|4|5|**20**|🔴 Extreme|Isolate logic (3.3V/5V) and motor power rails; add large bypass capacitors near motor drivers; stagger motor startup sequences.|
|**P-002**|Plug-and-play module connector short circuit or reverse polarity.|2|5|**10**|🟠 High|Use keyed, polarized connectors (e.g., XT60/Aviation plugs); integrate a reverse polarity protection circuit (MOSFET/Diode) on the arm PCB.|
|**P-003**|Signal noise corrupts encoder data (I2C/SPI) or UART comms.|4|4|**16**|🔴 Extreme|Use twisted pair and shielded cables for data; route signal wires far from high-current motor phase wires; prefer CAN bus for long runs.|

### 3.4 Software & Control Risks

These risks cover the logic layer, including kinematic solvers, state machines, and intra-module communication.

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|**Risk ID**|**Risk Description**|**L**|**I**|**Score (R)**|**Level**|**Mitigation Strategy**|
|**S-001**|Arm self-collision or driving into mechanical hard stops.|3|5|**15**|🔴 Extreme|Implement software joint limits in the kinematic solver; install physical hardware limit switches on every joint as a fail-safe.|
|**S-002**|Inverse Kinematics (IK) singularity causes erratic/infinite velocity commands.|2|4|**8**|🟡 Medium|Restrict the operational workspace in software to avoid singular configurations; use Damped Least Squares (DLS) IK algorithms.|
|**S-003**|Communication timeout between Base Integration Layer and Arm MCU.|3|4|**12**|🟠 High|Implement a software heartbeat (e.g., 10Hz ping); program a safe-state fallback that immediately freezes all motors if the ping is lost.|

### 3.5 Perception Risks

Perception risks involve the integration of sensors and cameras required for the autonomous QR-code reading segment of the competition.

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|**Risk ID**|**Risk Description**|**L**|**I**|**Score (R)**|**Level**|**Mitigation Strategy**|
|**C-001**|End-effector camera fails to read QR code due to shadows/lighting.|3|5|**15**|🔴 Extreme|Mount an active LED ring light directly around the camera lens; implement multi-frame retry logic and auto-exposure in the vision pipeline.|
|**C-002**|Gripper fingers physically obstruct the camera's view of the QR code.|4|4|**16**|🔴 Extreme|Model the camera's Field of View (FOV) cone in CAD early; mount the camera on an offset bracket or use transparent/thin gripper fingers.|
