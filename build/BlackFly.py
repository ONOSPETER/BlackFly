import numpy as np

class BlackFly:
		 # === Path Planner Pilot === 
		 class Pilot: def init(self, goal_distance=80, offset=5, scan_range=30, emergency_max_dev=8, bezier_steps=20): 
		 self.goal_distance = goal_distance 
		 self.offset = offset 
		 self.scan_range = scan_range 
		 self.emergency_max_dev = emergency_max_dev 
		 self.steps = bezier_steps
	
		def update_path(self, deviation, scan_range=None):
		        if scan_range is None:
		            scan_range = self.scan_range
		        deviation = max(min(deviation, self.emergency_max_dev), -self.emergency_max_dev)
		        D = np.array([0.0, 0.0])
		        P1 = np.array([deviation, self.offset])
		        projected_forward = self.goal_distance * 0.6
		        P = np.array([deviation, projected_forward])
		        P2 = np.array([0.0, self.goal_distance + 2])
		        return P1, P, P2
		
		def bezier_curve(self, P0, P1, P2):
		        curve = []
		        for t in np.linspace(0, 1, self.steps):
		            point = (1 - t)**2 * np.array(P0) + 2 * (1 - t) * t * np.array(P1) + t**2 * np.array(P2)
		            curve.append(tuple(point))
		        return curve
	
	# === Offset Algorithm ===
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
	
	# === Data Handler ===
	class Data_Handler:
	    def __init__(self):
	        self.latest_lidar_data = []
	
	    def receive_data(self):
	        # Placeholder: Replace with real data acquisition method
	        # Return list of (angle, distance)
	        return self.latest_lidar_data
	
	    def send_deviation_path(self, path):
	        # Placeholder: Implement platform-specific path sending logic
	        print("Sending path to drone:", path)
	
	# === Main Controller ===
	def __init__(self):
	    self.handler = self.Data_Handler()
	    self.pilot = self.Pilot()
	
	def main(self):
	    lidar_data = self.handler.receive_data()
	    offset_algorithm = self.Offset_Algorithm(lidar_data, scan_radii=[10, 15, 20])
	    direction, deviation = offset_algorithm.run()
	
	    P1, P, P2 = self.pilot.update_path(deviation)
	    bezier_path = self.pilot.bezier_curve((0, 0), P1, P)
	    bezier_path += self.pilot.bezier_curve(P, P, P2)
	
	    self.handler.send_deviation_path(bezier_path)
	
	def checksum(self):
	    print("Running diagnostics...")
	    assert hasattr(self.pilot, 'update_path')
	    assert hasattr(self.pilot, 'bezier_curve')
	    assert hasattr(self.handler, 'receive_data')
	    assert hasattr(self.handler, 'send_deviation_path')
	    print("All systems functional.")
	
	
