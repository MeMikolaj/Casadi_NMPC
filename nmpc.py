
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
    N = 20    # Horizon length # 2s


    # Set up the optimization problem
    opti = Opti()
    
    # Gamma - point on the path quantifying where on the path we want to be at the moment. 0-start.
    gamma = opti.parameter(); opti.set_value(gamma, 0) # Desired forward velocity

    
    # Vehicle
    X = opti.variable(3, N+1)  # State trajectory variables (x, y, theta)
    r = opti.variable(1, N)    # Control trajectory variable (omega)
    v = opti.parameter(); opti.set_value(v, 1.0) # Desired forward velocity
    
    # Point
    X_p = opti.variable(3, N+1)  # State trajectory variables (x, y, theta)
    v_p = opti.variable(1, N)    # Linear speed of the path
    
    # Position and rotation errors
    e_p   = opti.variable(2, N+1)
    psi_e = opti.variable(1, N+1)
    
    # Whole Error
    error_tp = opti.variable(3, N+1)

    # Curvature
    curvature = opti.variable(1, N)
    
    # Weights
    weights_Q = opti.parameter(3, 3)
    opti.set_value(weights_Q, DM([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
    weights_R = opti.parameter(2, 2)
    opti.set_value(weights_R, DM([[1.0, 0.0], [0.0, 1.0]]))

    
    # Cost associated with the input
    u_a = opti.variable(2, N)
    

    # Initial state
    opti.subject_to(X[:, 0] == [1.1, 0, 0])  # Initial position and orientation of the robot
    opti.subject_to(X_p[:, 0] == [3.4, 2.4, 0.785])  # Initial position and orientation of the robot
    
    opti.subject_to(e_p[:, 0] == vertcat(horzcat(cos(X_p[2, 0]), sin(X_p[2, 0])), horzcat(-sin(X_p[2, 0]), cos(X_p[2, 0]))) @ (X[0:2, 0] - X_p[0:2, 0]))
    opti.subject_to(psi_e[0, 0] == X[2, 0] - X_p[2, 0])
    opti.subject_to(error_tp[:,0] == vertcat(e_p[:,0], psi_e[:,0]))


    # Dynamics constraints
    for k in range(N):
        
        # Set Up Vehicle States
        opti.subject_to(X[0, k+1] == X[0, k] + v * cos(X[2, k]) * DT)
        opti.subject_to(X[1, k+1] == X[1, k] + v * sin(X[2, k]) * DT)
        opti.subject_to(X[2, k+1] == X[2, k] + r[0, k] * DT)
        
        # Set Up Point States
        opti.subject_to(X_p[0, k+1] == X_p[0, k])
        opti.subject_to(X_p[1, k+1] == X_p[1, k])
        opti.subject_to(X_p[2, k+1] == X_p[2, k])
        
        # Set Up Errors
        opti.subject_to(e_p[:, k+1] == vertcat(horzcat(cos(X_p[2, k+1]), sin(X_p[2, k+1])), horzcat(-sin(X_p[2, k+1]), cos(X_p[2, k+1]))) @ (X[0:2, k+1] - X_p[0:2, k+1]))
        opti.subject_to(psi_e[0, k+1] == X[2, k+1] - X_p[2, k+1])
        opti.subject_to(error_tp[:, k+1] == vertcat(e_p[:, k+1], psi_e[:, k+1]))
        
        
    
        
    # Objective Function to minimise
    
    # sqrt((sum1(e_p[:, N]) + psi_e[0, N])**2
    cost = sqrt((sum1(e_p[:, N]) + psi_e[0, N])**2) # (X[0, N] - X_p[0, N])**2 + (X[1, N] - X_p[1, N])**2 + (X[2, N] - X_p[2, N])**2
    opti.minimize(cost)

    # Bounds on control (angular velocity of the vehicle and speed of the reference point limits)
    for k in range(N):
        opti.subject_to(opti.bounded(-1.0, r[0,k], 1.0))
        opti.subject_to(opti.bounded(0.0, v_p[0,k], 1.5)) # has to be able to be '<= 0' and '> v'.

    # Solve the optimization problem
    opti.solver('ipopt')
    
    
    
    # Set initial guess for state and control
    opti.set_initial(X, np.zeros((3, N+1)))  # Initialize state trajectory (zeros)
    opti.set_initial(r, np.zeros((1, N)))    # Initialize control trajectory (zeros)
    opti.set_initial(X_p, np.zeros((3, N+1)))
    opti.set_initial(v_p, np.zeros((1, N)))

    # Solve the optimization problem
    sol = opti.solve()

    # Extract optimal state and control
    state_trajectory = sol.value(X)
    control_trajectory = sol.value(r)

    # Print the optimal state and control
    print("Optimal State Trajectory:")
    print(state_trajectory)

    print("\nOptimal Control (Angular Velocity) Trajectory:")
    print(control_trajectory)

    # Integral of first and seconds terms
    # First term
    # error_tp[:,0].T @ weights_Q @ error_tp[:,0] 
    
    # Second term
    # u_a[:,0].T @ weights_R @ u_a[:,0]
    u_a = vertcat(v * cos(psi_e) - pi*v_p[0, 0], r - curvature[0, 0]*pi*v_p[0, 0])
    
    # Third term
    # sqrt(sum1(error_tp[:,N])**2) # Can be whatever cost I design, should probably be smaller
    
    
    # integral_cost = integral((x_desired(t) - x(t))**2, t, t_start, t_end)

if __name__ == '__main__':
    main()