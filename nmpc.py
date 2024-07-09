
from casadi import *

# NMPC for 2D Turtle bot. Constraints on forward speed and angular rate.

class NMPC():
    
    # Needs to take a path as an argument
    def __init__(self) -> None:
        pass
        # Set up the variables
        
        
        self.sampling_time = 1 # T_s
        self.prediction_horizon = 3 # T_p
    
    # Where I am now    
    def update_vehicle_state(self):
        pass
        
        
# NMPC
# 1. Initialize the beginning of the path
# For every time t do:
#   2. Compute the path following errors in x1, y1 (rotation transpose)*(robot - desired), yaw error (robot - desired)
#   3. Solve FOCP-1 for r and speed of the point
#   4. Compute the desired forward speed - Just set it up to some constant!!!!
#   5. Set vehicle's anguar rate to r
#   6. Update the point of the path with calculated speed of the point.