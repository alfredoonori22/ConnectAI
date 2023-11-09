import sys
import coppeliasim.api.sim as sim
import numpy as np

# Initialization
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)


def connectionMessage(clientID):
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Connection unsuccessful :(. Ensure simulation is already running + correct files in directory.")
        sys.exit()


PI = np.pi
velocity = 0.5

# Obtaining appropriate handles
errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking) # target dummy
errorCode, j1 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1', sim.simx_opmode_blocking) #gripper joint 1
errorCode, j2 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2', sim.simx_opmode_blocking) #gripper joint 2
errorCode, connector = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_attachPoint', sim.simx_opmode_blocking) # gripper connect point

# Obtaining joint positions for the gripper to close & open
errorCode, p1 = sim.simxGetJointPosition(clientID, j1, sim.simx_opmode_streaming)
errorCode, p2 = sim.simxGetJointPosition(clientID, j2, sim.simx_opmode_streaming)

returnCode, pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
returnCode, orient = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_streaming)

# Coordinates
initial_pos = [0, 0.1, 1.1, 0, -PI / 2, 0]

# Cuboid initial position ([A0, A1...], [B0, B1...], ...)
position = [
        [[-0.6, -0.1, 1.0, 0, -PI/2, 0], [-0.4, -0.1, 1.0, 0, -PI/2, 0], [-0.2, -0.1, 1.0, 0, -PI/2, 0], [0, -0.1, 1.0, 0, -PI/2, 0], [0.2, -0.1, 1.0, 0, -PI/2, 0], [0.4, -0.1, 1.0, 0, -PI/2, 0], [0.6, -0.1, 1.0, 0, -PI/2, 0]],
        [[-0.6, 0.1, 1.0, 0, -PI/2, 0], [-0.4, 0.1, 1.0, 0, -PI/2, 0], [-0.2, 0.1, 1.0, 0, -PI/2, 0], [0, 0.1, 1.0, 0, -PI/2, 0], [0.2, 0.1, 1.0, 0, -PI/2, 0], [0.4, 0.1, 1.0, 0, -PI/2, 0], [0.6, 0.1, 1.0, 0, -PI/2, 0]],
        [[-0.6, 0.3, 1.0, 0, -PI/2, 0], [-0.4, 0.3, 1.0, 0, -PI/2, 0], [-0.2, 0.3, 1.0, 0, -PI/2, 0], [0, 0.3, 1.0, 0, -PI/2, 0], [0.2, 0.3, 1.0, 0, -PI/2, 0], [0.4, 0.3, 1.0, 0, -PI/2, 0], [0.6, 0.3, 1.0, 0, -PI/2, 0]],
        [[-0.6, 0.5, 1.0, 0, -PI/2, 0], [-0.4, 0.5, 1.0, 0, -PI/2, 0], [-0.2, 0.5, 1.0, 0, -PI/2, 0], [0, 0.5, 1.0, 0, -PI/2, 0], [0.2, 0.5, 1.0, 0, -PI/2, 0], [0.4, 0.5, 1.0, 0, -PI/2, 0], [0.6, 0.5, 1.0, 0, -PI/2, 0]],
        [[-0.6, 0.7, 1.0, 0, -PI/2, 0], [-0.4, 0.7, 1.0, 0, -PI/2, 0], [-0.2, 0.7, 1.0, 0, -PI/2, 0], [0, 0.7, 1.0, 0, -PI/2, 0], [0.2, 0.7, 1.0, 0, -PI/2, 0], [0.4, 0.7, 1.0, 0, -PI/2, 0], [0.6, 0.7, 1.0, 0, -PI/2, 0]],
        [[-0.6, 0.9, 1.0, 0, -PI/2, 0], [-0.4, 0.9, 1.0, 0, -PI/2, 0], [-0.2, 0.9, 1.0, 0, -PI/2, 0], [0, 0.9, 1.0, 0, -PI/2, 0], [0.2, 0.9, 1.0, 0, -PI/2, 0], [0.4, 0.9, 1.0, 0, -PI/2, 0], [0.6, 0.9, 1.0, 0, -PI/2, 0]]]