import pybullet as p
import time
import pybullet_data
import numpy as np
from numpy import sin, cos
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

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


        rot = Rotation.from_euler('xyz', [.2, .1, 0], degrees=False)

        # Convert to quaternions and print
        rot_quat = rot.as_quat()

        self.bodyId = p.loadURDF("table_simple.urdf", [0,0,.55], rot_quat)
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
        gain = np.array([[723.2640,  139.2210,  208.5697,   69.8089,    7.0711,   12.8798],[208.5697,69.8089,723.2640,139.2210,7.0711,12.8798]])

        gain = np.array([[100, 50, 100, 50, 100, 50], [100, 50, 100, 50, 100, 50]])
        gain = np.array([[592.701787108467, 109.504556339578, 78.2017676650497, 40.8295887145679, 2.23606797706993, 4.74054233998768],
[78.2017677007310, 40.8295887214923, 592.701787120507, 109.504556343346, 2.23606797738994, 4.74054234059246]])
        theta = 0
        print(p.getNumJoints(self.bodyId))

        save_array_lf_theta = []
        save_array_base_theta = []
        save_actuator_1 = []
        save_actuator_2 = []

        delta_x = np.array([[0],[0],[0],[0],[0],[0]])
        control = np.array([[0],[0]])
        input()
        for i in range (30000):
           
            p.stepSimulation()
            # Update our recorded joint states
            self.get_base_state()
            self.get_joint_states()
            
            state_vector = np.array([[self.lf_joint_state["theta"]+self.base_state["theta"]-.1],
                                    [self.lf_joint_state["theta."]],
                                    [self.rb_joint_state["theta"]+self.base_state["theta"]-.1],
                                    [self.rb_joint_state["theta."]],
                                    [self.base_state["theta"]],
                                    [self.base_state["theta."]]])
            
            
            delta_x = np.subtract(state_vector, referenceVector)
            control = -gain@delta_x
            #print(delta_x)
            #print(control)
            max_vel = 2
            if control[0][0] > 100:
                control[0][0] = max_vel
            if control[0][0] < -100:
                control[0][0] = -max_vel
            if control[1][0] > 100:
                control[1][0] = max_vel
            if control[1][0] < -100:
                control[1][0] = -max_vel
            p.setJointMotorControl2(self.bodyId, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-control[0][0]/200)
            p.setJointMotorControl2(self.bodyId, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=-control[1][0]/200)
            save_array_lf_theta.append(self.lf_joint_state["theta"]+self.base_state["theta"]-.1)
            save_array_base_theta.append(self.base_state["theta"])
            save_actuator_1.append(-control[0][0])
            save_actuator_2.append(-control[0][0])
            
            if i==10000:
                print("FROCE")
                input()
                #Apply external force
                p.applyExternalForce(self.bodyId, 0, [-10000, 0, 10000], [0,0,0], p.LINK_FRAME)
            time.sleep(.001)
            # x = [qf(font angle) q1f(front angle velocity) qh qh1 qb qb1]

            #print(p.getJointState(self.bodyId, 0))
            
            #p.setJointMotorControl2(self.bodyId, 2, p.POSITION_CONTROL, theta)
            #p.setJointMotorControl2(self.bodyId, 1, p.POSITION_CONTROL, theta)
            #theta += .5
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
            #print(cubePos,cubeOrn)
            
            #time.sleep(.02)
            

        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.bodyId)
        #print(cubePos,cubeOrn)
        p.disconnect()

        
        #plt.plot(save_array_lf_theta)
        t = np.linspace(0,5,len(save_array_base_theta))
        plt.subplot(211)
        plt.title('Body Pitch')
        plt.plot(t, save_array_base_theta)
        plt.xlabel('Time (s)')
        plt.ylabel('Radians')
        plt.subplot(212)
        plt.title('Control Input')
        plt.ylabel('Torque (Nm)')
        plt.xlabel('Time (s)')
        plt.plot(t, save_actuator_1)
        plt.plot(t, save_actuator_2)
        plt.legend(["Front","Rear"])
        plt.suptitle("Fall w/ Pitch and Roll Disturbance")
        plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.4,
                    hspace=0.4)
        plt.show()



        time.sleep(2)#1./240.)
        p.setJointMotorControl2(self.bodyId[0], 0, p.TORQUE_CONTROL, 5)

        p.stepSimulation()
        time.sleep(.002)#1./240.)

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
