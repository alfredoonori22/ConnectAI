import time
import coppeliasim.api.sim as sim
import coppeliasim.globalvariables as g
import numpy as np
from coppeliasim.gripper import Gripper


def move_L(clientid, target, target_pos, speed):
    returnCode, pos = sim.simxGetObjectPosition(clientid, target, -1, sim.simx_opmode_buffer)
    returnCode, orient = sim.simxGetObjectOrientation(clientid, target, -1, sim.simx_opmode_buffer)

    for i in range(3):
        pos[i] = float(pos[i])
        orient[i] = float(orient[i])

    old_pos = []
    old_orient = []
    delta_pos = []
    delta_orient = []
    intermediate_pos = [0, 0, 0]
    intermediate_orient = [0, 0, 0]

    # Accounting for orientation
    for i in range(3):
        if abs(target_pos[i + 3]) - orient[i] > g.PI and orient[i] < 0:
            orient[i] = orient[i] + 2 * g.PI
        elif abs(target_pos[i + 3]) - orient[i] > g.PI and orient[i] > 0:
            orient[i] = orient[i] - 2 * g.PI

    for i in range(3):
        old_pos.append(pos[i])
        delta_pos.append(target_pos[i] - old_pos[i])
        old_orient.append(orient[i])
        delta_orient.append(target_pos[i + 3] - old_orient[i])

    distance = np.linalg.norm(delta_pos)
    samples_number = round(distance * 50)

    for i in range(samples_number):
        for j in range(3):
            intermediate_pos[j] = (old_pos[j] + (delta_pos[j] / samples_number))
            intermediate_orient[j] = (old_orient[j] + (delta_orient[j] / samples_number))

        # Accounting for speed (?)
        startTime = time.time()
        while (time.time() - startTime) < (distance / (speed * samples_number)):
            time.sleep(0.01)

        sim.simxSetObjectPosition(clientid, target, -1, intermediate_pos, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(clientid, target, -1, intermediate_orient, sim.simx_opmode_oneshot)

        for k in range(3):
            old_pos[k] = intermediate_pos[k]
            old_orient[k] = intermediate_orient[k]

        for k in range(3):
            intermediate_pos[k] = 0
            intermediate_orient[k] = 0

    old_pos.clear()
    delta_pos.clear()
    intermediate_pos.clear()


def BlueMove(clientid, cuboidH):
    move_L(clientid, g.target, [0.6, -0.7, 1.0, -g.PI, 0, 0], 0.4)
    time.sleep(0.5)
    move_L(clientid, g.target, [0.6, -0.7, 0.547, g.PI, 0, 0], 0.5)
    time.sleep(0.5)
    Gripper(clientid, 0)
    sim.simxSetObjectParent(clientid, cuboidH, -1, True, sim.simx_opmode_blocking)
    time.sleep(0.5)
    move_L(clientid, g.target, [0.6, -0.7, 0.7, g.PI, 0, 0], 0.4)
    time.sleep(0.5)
    move_L(clientid, g.target, [0.428, -0.7, 1.0, 0, -g.PI / 2, 0], 0.4)
    time.sleep(0.5)
    move_L(clientid, g.target, [0.428, -0.7, 0.72, 0, -g.PI / 2, 0], 0.5)
    time.sleep(0.5)


def OrangeMove(clientid, cuboidH):
    move_L(clientid, g.target, [-0.5, -0.5, 1.0, 0, 0, g.PI / 2], 0.4)
    time.sleep(0.5)
    move_L(clientid, g.target, [-0.5, -0.5, 0.547, 0, 0, g.PI / 2], 0.5)
    time.sleep(0.5)
    Gripper(clientid, 0)
    sim.simxSetObjectParent(clientid, cuboidH, -1, True, sim.simx_opmode_blocking)
    time.sleep(0.5)
    move_L(clientid, g.target, [-0.5, -0.5, 0.7, 0, 0, g.PI / 2], 0.5)
    time.sleep(0.5)
    move_L(clientid, g.target, [-0.49963, -0.675, 1.0, 0, -g.PI / 2, 0], 0.4)
    time.sleep(0.5)
    move_L(clientid, g.target, [-0.49963, -0.675, 0.72, 0, -g.PI / 2, 0], 0.5)
    time.sleep(0.5)


# Moves the block
def moveBlockFunc(clientid, cuboid, position, color):
    errorCode, cuboidH = sim.simxGetObjectHandle(g.clientID, f'/Cuboid_{cuboid}', sim.simx_opmode_blocking)

    move_L(clientid, g.target, position, 0.5)
    time.sleep(0.5)
    move_L(clientid, g.target, [position[0], position[1], 0.72, 0, -g.PI/2, 0], 0.5)
    time.sleep(0.5)
    Gripper(clientid, 1)
    sim.simxSetObjectParent(clientid, cuboidH, g.connector, True, sim.simx_opmode_blocking)
    time.sleep(0.5)
    move_L(clientid, g.target, position, 0.5)
    time.sleep(0.5)

    if color == 'orange':
        OrangeMove(clientid, cuboidH)
    else:
        BlueMove(clientid, cuboidH)

    Gripper(clientid, 1)
    sim.simxSetObjectParent(clientid, cuboidH, g.connector, True, sim.simx_opmode_blocking)
    time.sleep(0.5)
    move_L(clientid, g.target, g.initial_pos, 0.5)
    time.sleep(0.5)
    move_L(clientid, g.target, position, 0.5)
    time.sleep(0.5)
    move_L(clientid, g.target, [position[0], position[1], 0.72, 0, -g.PI / 2, 0], 0.5)
    time.sleep(0.5)
    Gripper(clientid, 0)
    sim.simxSetObjectParent(clientid, cuboidH, -1, True, sim.simx_opmode_blocking)
    time.sleep(0.5)
    move_L(clientid, g.target, position, 0.5)
    time.sleep(0.5)
    move_L(clientid, g.target, g.initial_pos, 0.5)
    time.sleep(0.5)
