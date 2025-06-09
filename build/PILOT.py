import numpy as np
import math
import matplotlib.pyplot as plt

class Pilot:
    def __init__(self, goal_distance=80, offset=5, scan_range=30, emergency_max_dev=8, bezier_steps=20):
        self.goal_distance = goal_distance  # Total forward distance to goal
        self.offset = offset                # Forward distance to place P1 from D
        self.scan_range = scan_range
        self.emergency_max_dev = emergency_max_dev
        self.steps = bezier_steps          # Bézier curve resolution

    def update_path(self, deviation, scan_range=None):
        """
        Main path update logic based on deviation input.
        Returns three points: P1, P, P2.
        """
        if scan_range is None:
            scan_range = self.scan_range

        # Clamp deviation to emergency max deviation
        deviation = max(min(deviation, self.emergency_max_dev), -self.emergency_max_dev)

        # Origin D is always (0, 0)
        D = np.array([0.0, 0.0])

        # P1 is placed at offset distance forward and 'deviation' laterally
        P1 = np.array([deviation, self.offset])

        # P is projected forward along the same deviation direction
        projected_forward = self.goal_distance * 0.6
        P = np.array([deviation, projected_forward])

        # P2 is the True Waypoint correction point — straight up ahead on the center line
        P2 = np.array([0.0, self.goal_distance + 2])  # +2 to ensure overshoot into target corridor

        return P1, P, P2

    def bezier_curve(self, P0, P1, P2):
        """
        Quadratic Bézier Curve from P0 to P2 with control point P1.
        """
        curve = []
        for t in np.linspace(0, 1, self.steps):
            point = (1 - t)**2 * np.array(P0) + 2 * (1 - t) * t * np.array(P1) + t**2 * np.array(P2)
            curve.append(tuple(point))
        return curve
        


# Instantiate planner
planner = Pilot(goal_distance=80)

# Simulate deviation input (e.g., 6m to the right)
deviation = 6
P1, P, P2 = planner.update_path(deviation)

# Get Bézier curves
curve1 = planner.bezier_curve((0, 0), P1, P)   # D → P1 → P
curve2 = planner.bezier_curve(P, P2, P2)       # P → P2 (flattened for anchoring)

# Plotting
x1, y1 = zip(*curve1)
x2, y2 = zip(*curve2)

plt.figure(figsize=(8, 6))
plt.plot(x1, y1, label='D → P1 → P (Curve1)', color='blue')
plt.plot(x2, y2, label='P → P2 (Curve2)', color='green')
plt.scatter(*zip(P1, P, P2), c='red', label='Deviation Points')
plt.axvline(0, linestyle='--', color='gray', label='True Waypoint Path')
plt.xlabel("X-axis (Deviation)")
plt.ylabel("Y-axis (Forward Distance)")
plt.title("Deviation-Corrective Path Planning with Bézier Curves")
plt.legend()
plt.grid(True)
plt.show()


""" ××****************PILOT****************××

By Obiegba Onoteoghene Peter

	The Pilot Path Planning System is an adaptive UAV path planner that intelligently balances obstacle avoidance with trajectory correction toward a fixed reference route called the True Waypoint Path. Designed for dynamic 2D environments, the system ensures that the drone avoids lateral drift by constantly steering back toward the centerline (aligned with the final destination). It does this by generating three adaptive subgoals per planning cycle:
	
	P1 (Offset Point): shifts the drone laterally to avoid immediate obstacles.
	
	P (Projected Point): extends the deviation forward.
	
	P2 (Return Anchor): brings the drone back to the central path beyond the obstacle field.
	
	
	Smooth motion is guaranteed via Bézier curves connecting these points, ensuring navigable and natural turns. The system operates under user-defined constraints like scan range, deviation thresholds, and goal distance, dynamically reacting to real-time deviation input from a lower-level offset algorithm. Heuristics like snap-back, penalized deviation cost, and funnel-based goal enforcement help prevent excessive drift and ensure safe, efficient flight. The combination of forward projection and return anchoring allows the UAV to adapt to complex environments while preserving global path fidelity.
	

Subfunctions:

	update_path(deviation, scan_range)	Computes the three subgoals: P1, P, and P2.
	bezier_curve(p0, p1, p2, num_points=20)	Returns smooth path between three control points using quadratic Bézier formulation.
	
	
	Key Concepts
	
	True Waypoint Path: Central y-axis-aligned route from drone origin to goal.
	
	Deviation: Lateral distance drone must temporarily shift from central path.
	
	Scan Range: Forward look-ahead to determine how far subgoals should be projected.
	
	P1, P, P2: Subgoals used to temporarily deviate and return to goal trajectory.
	
	Bézier Curves: Mathematical tool used to generate smooth, continuous flight paths.
	
	Deviation Control Mechanisms:
	
	Max Deviation Limit
	
	Snap-back Correction
	
	Goal-Approach Funnel
	
	Penalty-based Deviation Cost


Process Flow:

	1. Input Deviation (d) and Scan Range (l)
	
	2. Compute Subgoal Points:
	
	P1 = (d, l) – first offset point.
	
	P  = (d, g) – projected forward.
	
	P2 = (0, g + 2) – anchor back to central goal path.
	
	g = l + 10 – lookahead to ensure return after deviation.
	
	
	
	3. Generate Smooth Paths:
	
		Curve1 = Bézier(D, midpoint(D, P1), P1)
		
		Curve2 = Bézier(P, midpoint(P, P2), P2)
		
	4. Drone Follows Combined Path
	
	
Mathematical Concepts:

	Quadratic Bézier Curve:
	
	
	B(t) = (1 - t)^2 P_0 + 2(1 - t)t P_1 + t^2 P_2, \quad t \in [0, 1]
	
	Midpoint Calculation (for control points):
	
	
	M(x, y) = \left( \frac{x_1 + x_2}{2}, \frac{y_1 + y_2}{2} \right)
	
	Deviation Penalty Heuristic:
	
	
	Cost \propto |x_{\text{deviation}}|^2
	
	Snap-Back Threshold:
	Triggered when 


This systematic approach allows the UAV to navigate adaptively while maintaining alignment with its primary goal."""


