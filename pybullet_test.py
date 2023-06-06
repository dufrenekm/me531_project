import pybullet as p
import time
import pybullet_data
import numpy as np
from numpy import sin, cos


class LittleDog:
    """
    A class to 

    lf_hip is 0
    lf_leg is 1
    rf_hip is 2
    rf_hip is 3
    ...
    Attributes
    ----------
    Methods
    -------
    """


    def __init__(self):
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        startPos = [0,0,1]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.bodyId = p.loadURDF("table.urdf", [0,0,3])
        #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)


        # For ease of parsing data, use a dictionary to store information
        self.lf_joint_state = {'x': 0, 'y': 0, 'z': 0, 'theta': 0, 'theta.': 0, 'extension': 0, 'extension.': 0}
        self.lf_hip_state = {'x': 0, 'y': 0, 'z': 0, 'theta': 0, 'theta.': 0}
        self.rf_joint_state = {'x': 0, 'y': 0, 'z': 0, 'theta': 0, 'theta.': 0, 'extension': 0, 'extension.': 0}
        self.rf_hip_state = {'x': 0, 'y': 0, 'z': 0, 'theta': 0, 'theta.': 0}

        theta = 0
        print(p.getNumJoints(self.bodyId))
        for i in range (10000):
            # Update our recorded joint states
            self.get_joint_states()


            # x = [qf(font angle) q1f(front angle velocity) qh qh1 qb qb1]

            #print(p.getJointState(self.bodyId, 0))
            p.stepSimulation()
            #p.setJointMotorControl2(self.bodyId, 2, p.POSITION_CONTROL, theta)
            #p.setJointMotorControl2(self.bodyId, 1, p.POSITION_CONTROL, theta)
            #theta += .5
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
            print(cubePos,cubeOrn)
            time.sleep(.025)
            

        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
        print(cubePos,cubeOrn)
        p.disconnect()



        time.sleep(2)#1./240.)
        p.setJointMotorControl2(self.bodyId[0], 0, p.TORQUE_CONTROL, 5)

        p.stepSimulation()
        time.sleep(1)#1./240.)


   

    def get_joint_states(self):
        # Left front leg
        temp = p.getJointState(self.bodyId, 0)
        self.lf_hip_state['theta'] = temp[0]
        self.lf_hip_state['theta.'] = temp[1]
        temp = p.getJointState(self.bodyId, 1)
        self.lf_joint_state['theta'] = temp[0]
        self.lf_joint_state['theta.'] = temp[1]
        temp = p.getJointState(self.bodyId, 2)
        self.lf_joint_state['extension'] = temp[0]
        self.lf_joint_state['extension.'] = temp[1]

        # Right back leg
        temp = p.getJointState(self.bodyId, 3)
        self.rf_hip_state['theta'] = temp[0]
        self.rf_hip_state['theta.'] = temp[1]
        temp = p.getJointState(self.bodyId, 4)
        self.rf_joint_state['theta'] = temp[0]
        self.rf_joint_state['theta.'] = temp[1]
        temp = p.getJointState(self.bodyId, 5)
        self.rf_joint_state['extension'] = temp[0]
        self.rf_joint_state['extension.'] = temp[1]


    







if __name__ == "__main__":
    ld = LittleDog()
