
from casadi import *
import numpy as np

# NMPC for 2D Turtle bot. Constraints on forward speed and angular rate.

# class NMPC():
    
#     # Needs to take a path as an argument
#     def __init__(self) -> None:
#         pass
#         vehicle_x1 = MX.sym('vehicle_x1')
#         vehicle_y1 = MX.sym('vehicle_y1')
#         vehicle_yaw = MX.sym('vehicle_yaw')
#         vehicle_x = [[vehicle_x1], [vehicle_y1]]
        
#         point_x1 = MX.sym('point_x1') # This is a function of the path function somehow...
#         point_y1 = MX.sym('point_y1') # This is a function of the path function somehow...
#         point_yaw = MX.sym('point_yaw')
#         point_x = [[point_x1], [point_y1]]
        
        
#         e_p   = dot([[cos(point_yaw), sin(point_yaw)],[-sin(point_yaw), cos(point_yaw)]], (vehicle_x - point_x))
#         psi_e = vehicle_yaw - point_yaw
        
        
        
#         U_d = 0.1 # Desired forward speed of the vehicle
#         r = MX.sym('r') # angular rate of the vehicle
#         v_gamma = MX.sym('v_gamma') # Linear velocity of the point on the path
        
#         curvature_gamma = 0.0 # ???? How to calculate this. Norm of a vector
        
#         u_a = SX([[U_d * cos(psi_e) - v_gamma], [r - curvature_gamma *  v_gamma]])
        
        
        
#         weights_Q = SX([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
#         weights_R = SX([[1.0, 0.0], [0.0, 1.0]])

#         # time interval
#         t_start = 0.0
#         t_end   = t_start + 4.0
        
#         # Function part
        
#         # Integral
#         # integral_part = 
        
        
        
#         # Define optimization problem
#         opti = Opti()
                
        
        
        
        
        
        
        
        
#         pass
        
#         ####### Using the normal notation first, then the casadi later
        
#         # point on the bath between its beginning and end that gets updated
#         self.gamma = 0.0 # Start from 0 and update internally - 1.
        
#         # Point Pose
#         point_x       = 0.0 # given
#         point_y       = 0.0 # given
#         point_theta   = 0.0 # given        
        
#         # Vehicle Pose
#         vehicle_x     = 0.0 # given
#         vehicle_y     = 0.0 # given
#         vehicle_theta = 0.0 # given 
        
#         # Errors, calculated given the point and vehicle poses. - 2.
#         e_p   = np.dot(np.array([[np.cos(point_theta), -np.sin(point_theta)],[np.sin(point_theta), np.cos(point_theta)]]).T, np.array([vehicle_x-point_x, vehicle_y-point_y]).T) # Calculated
#         psi_e = vehicle_theta - point_theta # Calculated
        
        
#         # Constraints
#         # ...
        
#         U_d = 0.1 # Desired forward speed of the vehicle
#         # u_gamma = ? # optimized for
#         # r = ? # optimized for
#         curvature_gamma = 0.0 # Calculated given pose at point gamma on a path
        
#         # Uncertainties - Covariances passed as an argument
#         state_uncertainty_Q = np.eye(3)                  # given   # These may be weights for the position errors...
#         forward_angular_speeds_uncertainty_R = np.eye(2) # given   # These may be weight for the control inputs...
        
#         # Current time - given?
#         time_t = 0.0
        
#         # Sampling time - Set up, design part
#         self.sampling_time = 1 # T_s
#         self.prediction_horizon = 3 # T_p
    

        
#     # Where I am now    
#     def update_vehicle_state(self):
#         pass
        
        
# # NMPC
# # 1. Initialize the beginning of the path
# # For every time t do:
# #   2. Compute the path following errors in x1, y1 (rotation transpose)*(robot - desired), yaw error (robot - desired)
# #   3. Solve FOCP-1 for r and speed of the point
# #   4. Compute the desired forward speed - Just set it up to some constant!!!!
# #   5. Set vehicle's anguar rate to r
# #   6. Update the point of the path with calculated speed of the point.

def main(args=None):

    # Define constants
    DT = 0.1  # Time step
    N = 20    # Horizon length

    # Define symbolic variables
    x = MX.sym('x')      # Robot position x
    y = MX.sym('y')      # Robot position y
    theta = MX.sym('theta')  # Robot orientation
    v = 2.0                  # Constant forward speed (adjust as needed)


    # Set up the optimization problem
    opti = Opti()
    
    
    # Reference point (desired position to follow)
    ref_x = opti.parameter()  # Reference x position
    ref_y = opti.parameter()  # Reference y position

    
    # Decision variables (to be optimized)
    X = opti.variable(3, N+1)  # State trajectory variables (x, y, theta)
    U = opti.variable(1, N)    # Control trajectory variable (omega)

    # Initial state
    opti.subject_to(X[:, 0] == [2.0, 0.0, 0.0])  # Initial position and orientation of the robot

    # Dynamics constraints
    for k in range(N):
        opti.subject_to(X[0, k+1] == X[0, k] + v * cos(X[2, k]) * DT)
        opti.subject_to(X[1, k+1] == X[1, k] + v * sin(X[2, k]) * DT)
        opti.subject_to(X[2, k+1] == X[2, k] + U[0, k] * DT)

    # Example reference point (replace with your actual reference point)
    # Set parameter values
    opti.set_value(ref_x, 3.0)  # Example value for reference x position
    opti.set_value(ref_y, 3.0)  # Example value for reference y position
    
    # Objective (minimize the distance to the reference point)
    cost = sqrt((X[0, N] - ref_x)**2 + (X[1, N] - ref_y)**2)
    opti.minimize(cost)

    # Bounds on control (angular velocity limit)
    for k in range(N):
        opti.subject_to(-1.0 <= U[0, k])
        opti.subject_to(U[0, k] <= 1.0)

    # Solve the optimization problem
    opti.solver('ipopt')
    
    
    
    # Set initial guess for state and control
    opti.set_initial(X, np.zeros((3, N+1)))  # Initialize state trajectory (zeros)
    opti.set_initial(U, np.zeros((1, N)))    # Initialize control trajectory (zeros)

    # Solve the optimization problem
    sol = opti.solve()

    # Extract optimal state and control
    state_trajectory = sol.value(X)
    control_trajectory = sol.value(U)

    # Print the optimal state and control
    print("Optimal State Trajectory:")
    print(state_trajectory)

    print("\nOptimal Control (Angular Velocity) Trajectory:")
    print(control_trajectory)



if __name__ == '__main__':
    main()