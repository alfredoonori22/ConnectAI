import coppeliasim.api.sim as sim
import coppeliasim.globalvariables as g


# Gripper function that opens/closes the gripper
def gripperFunction(clientid, closing, j1, j2, p1, p2):
    if closing:
        if p1 < (p2 - 0.008):
            sim.simxSetJointTargetVelocity(clientid, j1, -0.005, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientid, j2, -0.02, sim.simx_opmode_streaming)
        else:
            sim.simxSetJointTargetVelocity(clientid, j1, -0.02, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientid, j2, -0.02, sim.simx_opmode_streaming)
    else:
        if p1 < p2:
            sim.simxSetJointTargetVelocity(clientid, j1, 0.2, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientid, j2, 0.1, sim.simx_opmode_streaming)
        else:
            sim.simxSetJointTargetVelocity(clientid, j1, 0.1, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientid, j2, 0.2, sim.simx_opmode_streaming)


# Calls upon the gripper function to start closing, then pauses the closing to align with the geometry of the
# cuboid. This achieves the 'fake gripping' effect without actually gripping the object.
def Gripper(clientid, closing):
    gripperFunction(clientid, closing, g.j1, g.j2, g.p1, g.p2)
