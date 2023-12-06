import numpy as np
#############################################
# local path planning in frenet frame params
#############################################

# Simulation parameter
DT = 0.2                # time tick [s]
L_W = 0.3302            # [m] Wheel base of vehicle
W_V = 0.2032            # [m] width of vehicle
L_V = 0.4               # [m] total length of the vehicle
TARGET_SPEED = 3        # target speed [m/s]
ROBOT_RADIUS = 1.2      # robot radius [m]
MAX_ROAD_WIDTH = 1.5    # maximum road width [m]
MAX_SPEED = 8           # maximum speed [m/s]
MAX_ACCEL = 7.51        # maximum acceleration [m/ss]
MAX_STEER = 0.4189      # [rad] max steering angle
MAX_SIMULATION_TIME = 100


# Local path planning parameter
D_ROAD_W = 0.2  # road width sampling length [m]
MAX_T = 0.5     # max prediction time [m]
MIN_T = 0.49     # min prediction time [m]
D_T_S = 0.1       # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed

# Cost function parameters
K_J = 0.01
K_T = 0.1
K_D = 100
K_LAT = 1.0
K_LON = 1.0
K_OBS = 5.0

# Stanley controller parameter
K_S = 0.5  # control gain
KP_S = 1.0  # speed proportional gain
