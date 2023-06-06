import pybullet as p
import time
import pybullet_data
import numpy as np
from numpy import sin, cos
from scipy.spatial.transform import Rotation


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


        rot = Rotation.from_euler('xyz', [0, 2, 0], degrees=True)

        # Convert to quaternions and print
        rot_quat = rot.as_quat()

        self.bodyId = p.loadURDF("table_simple.urdf", [0,0,.5], rot_quat)
        #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)


        # For ease of parsing data, use a dictionary to store information
        self.lf_joint_state = {'x': 0, 'y': 0, 'z': 0, 'theta': 0, 'theta.': 0}
        self.rb_joint_state = {'x': 0, 'y': 0, 'z': 0, 'theta': 0, 'theta.': 0}
        self.base_state = {'theta': 0, 'theta.': 0}

        referenceVector = np.array([[0],
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0]])
        gain = np.array([[556.2026, 101.3257, 41.7006, 32.6506, 0.7071, 2.4191],
                        [41.7006, 32.6506,556.2026,101.3257,.7071,.24191]])

        theta = 0
        print(p.getNumJoints(self.bodyId))
        for i in range (10000):

            p.stepSimulation()
            # Update our recorded joint states
            self.get_base_state()
            self.get_joint_states()
            
            state_vector = np.array([[self.lf_joint_state["theta"]-self.base_state["theta"]],
                                    [self.lf_joint_state["theta."]],
                                    [self.rb_joint_state["theta"]-self.base_state["theta"]],
                                    [self.rb_joint_state["theta."]],
                                    [self.base_state["theta"]],
                                    [self.base_state["theta."]]])
            
            delta_x = np.subtract(state_vector, referenceVector)
            control = -gain@delta_x
            #print(control)
            p.setJointMotorControl2(self.bodyId, 0, p.TORQUE_CONTROL, force=control[0][0])#targetVelocity=control[0][0]/200)
            p.setJointMotorControl2(self.bodyId, 1, p.TORQUE_CONTROL, force=control[1][0])#targetVelocity=control[1][0]/200)

            if i==500:
                print("FROCE")
                #Apply external force
                p.applyExternalForce(self.bodyId, 0, [-10000, 0, 0], [0,0,0], p.LINK_FRAME)

            # x = [qf(font angle) q1f(front angle velocity) qh qh1 qb qb1]

            #print(p.getJointState(self.bodyId, 0))
            
            #p.setJointMotorControl2(self.bodyId, 2, p.POSITION_CONTROL, theta)
            #p.setJointMotorControl2(self.bodyId, 1, p.POSITION_CONTROL, theta)
            #theta += .5
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
            #print(cubePos,cubeOrn)
            time.sleep(.002)
            

        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
        #print(cubePos,cubeOrn)
        p.disconnect()



        time.sleep(2)#1./240.)
        p.setJointMotorControl2(self.bodyId[0], 0, p.TORQUE_CONTROL, 5)

        p.stepSimulation()
        time.sleep(1)#1./240.)

    def get_base_state(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
        rot = Rotation.from_quat(cubeOrn)
        rot_euler = rot.as_euler('xyz', degrees=False)
        self.base_state["theta"] = rot_euler[1]
        vel, thetaVel = p.getBaseVelocity(self.bodyId)
        self.base_state["theta."] = thetaVel[1]


   

    def get_joint_states(self):
        # Left front leg
        temp = p.getJointState(self.bodyId, 0)
        self.lf_joint_state['theta'] = temp[0]
        self.lf_joint_state['theta.'] = temp[1]
        
        temp = p.getJointState(self.bodyId, 1)
        self.rb_joint_state['theta'] = temp[0]
        self.rb_joint_state['theta.'] = temp[1]
        

    







if __name__ == "__main__":
    ld = LittleDog()
