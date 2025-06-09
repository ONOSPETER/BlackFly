Abstract

BlackFly is a lightweight, real-time autonomous navigation system tailored for UAVs in cluttered or dynamic environments. It integrates two core modules: the Offset Algorithm for obstacle detection and avoidance, and the Pilot Module for trajectory planning employing Bézier curve smoothing. By leveraging processed LiDAR data for fast reactive behavior and smooth goal-aligned path correction, BlackFly is suitable for embedded UAVs operating at 20–50 Hz with limited computational resources.


1. Introduction

Unmanned Aerial Vehicles (UAVs) require fast, reliable navigation systems to operate safely in cluttered environments such as forests, warehouses, and urban areas. Classical mapping and planning methods (e.g., SLAM) often prove computationally demanding for small embedded systems. Additionally, pre-mapping isn’t always feasible in dynamic or unknown environments.
BlackFly addresses these challenges with a reactive, planning-on-the-fly framework combining real-time obstacle avoidance and smooth path planning. The Offset Algorithm interprets LiDAR data into a binary obstacle representation and selects a safe lateral offset, while the Pilot Module converts that offset into a Bézier-smoothed trajectory, ensuring alignment with the intended forward path.


2. System Architecture:

The architecture of BlackFly consists of three integrated modules:

1. LiDAR Sensor Input – provides 2D scan data (angle, distance).


2. Offset Algorithm – evaluates safe direction and deviation.


3. Pilot Module – computes smooth Bézier curves for navigation.



The main loop connects these: sensor → offset → pilot → actuator, supporting real-time flight at 20–50 Hz. The modular design allows the Data Handler to interface with platforms like PX4, ROS, or ArduPilot.



3. Offset Algorithm: Fast Obstacle Avoidance

3.1 System Overview (200 words)

The Offset Algorithm is designed for lightweight UAV platforms requiring fast, reliable navigation in obstacle-rich settings. It processes streaming LiDAR data to identify safe paths without mapping. The input LiDAR beams are aggregated into multiple Frames, each representing obstacle proximity at increasing distances. Each frame is a binary array—nanoboxes indicate presence (1) or absence (0) of obstacles within the frame’s scan radius.

A sliding scan window traverses the frame’s nanobox array left and right from center, sampling contiguous scanboxes (usually 4 beams). Each scanbox is scored, with nearer frames receiving exponentially higher weight via the formula:

w_i = \frac{2^{n-i}}{2^n - 1}.

The algorithm aggregates scores across frames:

A = \sum_i w_i \cdot S_i,
\quad S_i = \sum \left(\frac{\text{nanobox}_i \times 100}{N}\right).

A path direction is chosen if its aggregate score falls below a safety threshold T. This decision yields a lateral deviation direction and magnitude, passed on to the Pilot module. The method is computationally inexpensive and robust, making it ideal for embedded UAVs.


---

3.2 Subfunctions & Concepts

LiDAR Data Input – (angle, distance) scan readings.

Frame – binary representation per scan radius.

Nanobox – obstacle presence (1/0) per beam.

Scanbox – group of nanoboxes (~4).

Scan Window – sliding mechanism left/right.

Score Window – computes blockage percentage per scanbox.

Frame Weighting – bias toward near-video obstacles.

Aggregate Score – sum of weighted window scores.

Threshold Comparison – evaluates safe direction.

Offset Decision – chosen deviation and direction.



---

3.3 Process Flow & Mathematics

1. LiDAR Scan Input: obtain .


2. Generate Frames: for radii , build:



\text{nanobox}_{i, j} = \begin{cases}
1 & d_i \le R_j, \\
0 & \text{otherwise}.
\end{cases}

3. Scanboxes and Windows: For each frame, sample scanboxes of size N=4:



S_j = \sum_{i=1}^{N} \frac{\text{nanobox}_{i, j} \times 100}{N}.

4. Frame Weighting: For total frames n:



w_j = \frac{2^{n-j}}{2^n - 1}, \quad Ws_j = w_j \cdot S_j.

5. Aggregate Score: sum .


6. Decision Logic: if , direction = safe, return (direction, deviation).


7. Output: Deviation value and direction passed to Pilot.



This yields low-latency, robust obstacle avoidance in real time.


---

4. Pilot Module: Adaptive Bézier Path Planning

4.1 System Overview (200 words)

The Pilot module translates lateral deviations into smooth, goal-aligned paths using quadratic Bézier curves. Its aim is to ensure obstacle avoidance (via offset) is temporary, with the UAV returning to its original trajectory.

Every cycle, Pilot computes:

P1 (Offset Point): lateral shift to avoid obstacle.

P (Projected Point): forward continuation of offset.

P2 (Return Anchor): guiding the UAV back to the central path.


Using user-tunable parameters—scan range, maximum deviation, goal distance—the module ensures constraints are respected. Bézier smoothing ensures continuous transitions, and return heuristics (e.g., maximum deviation limit, snap-back, goal funnel) prevent long-term drift. The result is a reactive, smooth path suitable for embedded control.


---

4.2 Subfunctions & Concepts

Function	Purpose

update_path(deviation)	Computes control points  based on deviation
bezier_curve(p0, p1, p2)	Generates smooth quadratic Bézier curve between points


Key Concepts:

True Waypoint Path – goal-aligned central line.

Deviation – lateral offset required.

Scan Range – lookahead distance.

Bézier Curves – smoothing control points into trajectories.

Heuristics – limit/max deviation, snap-back, funnel enforcement.



---

4.3 Mathematical Formulation

1. Compute Points: with deviation , scan range , goal overhead :



P1 = (d, l), \quad P = (d, g), \quad P2 = (0, g + \Delta) \quad \text{where } g = 0.6 \times \text{goal\_distance}

2. Bézier Curve: parameterized by :



B(t) = (1 - t)^2 P_0 + 2(1 - t)t P_1 + t^2 P_2.

3. Deviation Heuristics: penalize:



C(d) \propto |d|^2

Returns: concatenated Bézier curve defines trajectory away and back to the main path.


---

5. System Integration & Real-Time Loop

The BlackFly class integrates the Data Handler, Offset Algorithm, and Pilot modules:

Data Handler: interfaces with the UAV platform to get LiDAR scans and send path commands—designed for user overloading.

Main Loop (20–50 Hz):

1. Acquire LiDAR data


2. Compute safe deviation (offset algorithm)


3. Generate smooth Bézier path (Pilot)


4. Send path to flight controller



Checksum: ensures all components are initialized, functional, and communicating.


The modular structure eases integration into autopilot systems (PX4, ROS) and supports extensions like 3D planning or sensor fusion.


---

6. Applications & Future Work

Deployment Scenarios:

Forest surveillance drones

Indoor warehouse logistics

Urban delivery in constrained corridors


Extensions:

3D obstacle fusion via improved LiDAR

Probabilistic scoring for sensor uncertainty

Vision–LiDAR integration

Long-range planning via hybrid SLAM or learning-based systems


Future Research:

Predictive avoidance with dynamic obstacles

Reinforcement learning blends for path priority trade-offs



---

7. Conclusion

BlackFly represents an effective bridge between reactive avoidance and goal-aligned planning. Its combined simplicity, modularity, and reactive path refinement suit embedded UAV systems in real-world, cluttered environments. The system’s principles—fast multiscale scanning, weighted scoring, adaptive Bézier planning—offer a foundation for future UAV navigation research and development.

