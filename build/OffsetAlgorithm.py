import random

class Offset_Algorithm:
    class Frame:
        def __init__(self, lidar_data, scan_radius):
            self.R = scan_radius
            self.data = self.create_nanobox_array(lidar_data)

        def create_nanobox_array(self, lidar_data):
            return [
                1 if distance <= self.R else 0
                for _, distance in lidar_data
            ]

        def __str__(self):
            return f"Frame(R={self.R}): {self.data}"

    def __init__(self, lidar_data, scan_radii, threshold=5.0):
        self.lidar_data = lidar_data
        self.scan_radii = sorted(scan_radii)
        self.threshold = threshold
        self.frames = [self.Frame(lidar_data, r) for r in self.scan_radii]
        self.mid = len(self.frames[0].data) // 2
        self.last_pos_L = self.mid + 2
        self.last_pos_R = self.mid - 1

    def score_window(self, slice_data):
        N = len(slice_data) or 1
        return sum((value * 100) // N for value in slice_data)

    def aggregate_scores(self, scores):
        return sum(scores)

    def scan_box(self, array, current_pos, direction="left"):
        if direction == "left":
            start = max(0, current_pos - 3)
            end = current_pos + 1
        else:  # "right"
            start = current_pos
            end = min(len(array), current_pos + 4)
        return array[start:end]

    def run(self):
        k = 0
        while k < self.mid:
            print(f"\n--- Cycle {k} ---")
            Wl_score, Wr_score = [], []

            for idx, frame in enumerate(self.frames):
                scanbox_L = self.scan_box(frame.data, self.last_pos_L, "left")
                scanbox_R = self.scan_box(frame.data, self.last_pos_R, "right")

                frame_weight = 2 ** (len(self.frames) - idx - 1) / (2 ** len(self.frames) - 1)
                Fl_score = self.score_window(scanbox_L) * frame_weight
                Fr_score = self.score_window(scanbox_R) * frame_weight

                Wl_score.append(Fl_score)
                Wr_score.append(Fr_score)

                print(f"Frame R={frame.R:.1f} | Left Window: {scanbox_L}, Score: {int(Fl_score)}")
                print(f"Frame R={frame.R:.1f} | Right Window: {scanbox_R}, Score: {int(Fr_score)}")
                print(f"Frame{idx + 1} weight: {frame_weight:.4f}")

            Wl_aggregate = self.aggregate_scores(Wl_score)
            Wr_aggregate = self.aggregate_scores(Wr_score)

            print(f"\nAggregate Left Score: {Wl_aggregate}")
            print(f"Aggregate Right Score: {Wr_aggregate}")

            # === Decision Section ===
            if k == 0 and Wl_aggregate <= self.threshold and Wr_aggregate <= self.threshold:
                print("=> Preferred Direction: LEFT (first cycle tie, both meet threshold)")
                return "LEFT"

            if Wl_aggregate <= self.threshold and Wr_aggregate <= self.threshold:
                direction = "LEFT" if Wl_aggregate < Wr_aggregate else "RIGHT"
                print(f"=> Preferred Direction: {direction} (both below threshold, {direction.lower()} is better)")
                return direction
            elif Wl_aggregate <= self.threshold:
                print("=> Preferred Direction: LEFT (only left meets threshold)")
                return "LEFT"
            elif Wr_aggregate <= self.threshold:
                print("=> Preferred Direction: RIGHT (only right meets threshold)")
                return "RIGHT"
            else:
                print("=> No direction found — both exceed threshold")

            self.last_pos_L -= 1
            self.last_pos_R += 1
            k += 1

        print("=> Exhausted all scan positions without a clear path")
        return None
        
        
        
#Generate Lidar data
def generate_lidar_data(radius=7.0, angle_range=(-60, 60), step=2):
    lidar_data = []
    angle = angle_range[0]
    while angle <= angle_range[1]:
        distance = round(random.uniform(0.5, radius), 2)
        lidar_data.append((angle, distance))
        angle += step
    return lidar_data
                    
# Example usage
lidar_data = generate_lidar_data()  # Assume this is defined
scan_radii = [1.0, 2.0, 3.0, 0.5, 5.0, 15.0, 20.0]
threshold = 5.0

oa = Offset_Algorithm(lidar_data, scan_radii, threshold)
decision = oa.run()
print(f"Final Decision: {decision}")



"""××××**********OFFSET ALGORITHM**********××××
By Obiegba Onoteoghene Peter

	The Offset Algorithm is a real-time obstacle avoidance and path planning strategy designed for autonomous UAVs navigating cluttered environments. It leverages multi-layered LiDAR perception and efficient spatial reasoning to find clear paths with minimal computational overhead. The system transforms raw LiDAR distance data into binary obstacle maps called Frames, each representing a different scan radius (i.e., range from the drone). Each frame is composed of nanoboxes, representing whether a LiDAR beam hits an obstacle (1) or free space (0). The algorithm deploys scan windows that slide left and right from the drone’s current forward-facing center, examining small groups of nanoboxes (called scanboxes) to locate a safe path.

Each scanbox in each frame is scored based on how clear it is. Crucially, nearer frames (closer obstacles) are given greater weight in the decision-making process using an exponential weighting formula. These scores are aggregated and compared against a threshold to determine if a path is safe. If so, the best-scoring direction is chosen. The system interfaces with a PILOT module, which generates a smooth Bézier curve trajectory for the drone to follow. The Offset Algorithm is fast, modular, and ideal for embedded systems where latency and power are limited.



2. Subfunctions and Concepts

	LiDAR Data Input: Raw (angle, distance) pairs from laser scans.
	
	Frame: Binary arrays constructed from LiDAR data per scan radius.
	
	Nanobox: A single beam result (0 = free, 1 = blocked).
	
	Scanbox: A group of adjacent nanoboxes (usually 4) analyzed for obstacle-free passage.
	
	Scan Window: A moving window that samples scanboxes to the left and right from center.
	
	Score Window: Computes a percentage score for a scanbox (100 = fully blocked).
	
	Frame Weighting: Assigns higher priority to nearer frames (closer obstacles).
	
	Aggregate Score: Weighted sum of scan scores per window.
	
	Threshold Comparison: Validates whether a direction is safe.
	
	Offset Decision: Chooses direction and lateral shift (offset).
	
	PILOT System: Converts offset and heading into a Bézier path.
	


3. Process Flow & Mathematical Concepts

	Step-by-step Flow:

		1. LiDAR scan generates (angle, distance) readings.
		
		
		2. Create multiple Frames from scan radii (R₁, R₂...Rn).
		
		
		3. For each frame:
		
		Convert beam distances into nanoboxes:
		



		\text{nanobox}_i = 
		     \begin{cases}
		     1, & \text{if } d_i \leq R \\
		     0, & \text{otherwise}
		     \end{cases}
		
		Slide across and sample a Scanbox of size N (typically N=4).
		
		Score each scanbox:
		
		
		S = \sum_{i=1}^{N} \frac{\text{nanobox}_i \times 100}{N}
		
		w_i = \frac{2^{n - i}}{2^n - 1}
		
		Ws_i = S_i \times w_i
		
		A = \sum_{i=1}^{n} Ws_i
		
		If either , it's a valid path.
		
		Select path with smaller aggregate score.
		
		
		6. Return direction, score, offset → feed to PILOT.
		
		
		7. PILOT:
		
		Uses last free scan radius as deviation range.
		
		Generates Bézier path from current → offset point → goal.

"""
