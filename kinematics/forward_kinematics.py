'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint 
       in function ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h25/joints_h25.html
       http://doc.aldebaran.com/2-1/family/nao_h25/links_h25.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for differnt joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..',
                             'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip,
                                                     simspark_port,
                                                     teamname,
                                                     player_id,
                                                     sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch']
                       'LArm': ['LShoulderPitch','LShoulderRoll', 
                                'LElbowYaw', 'LElbowRoll']
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 
                                'RElbowYaw', 'RElbowRoll']
                       'LLeg': ['LHipYawPitch', 'LHipRoll',
                                'LHipPitch', 'LKneePitch',
                                'LAnklePitch', 'LAnkleRoll']
                       'RLeg': ['RHipYawPitch', 'RHipRoll'
                                'RHipPitch', 'RKneePitch',
                                'RAnklePitch', 'RAnkleRoll']}
        self.lengths = {'Head': [0.0, 0.0]
                        'LArm': ['LShoulderPitch','LShoulderRoll', 
                                 'LElbowYaw', 'LElbowRoll']
                        'RArm': ['RShoulderPitch', 'RShoulderRoll', 
                                 'RElbowYaw', 'RElbowRoll']
                        'LLeg': ['LHipYawPitch', 'LHipRoll',
                                 'LHipPitch', 'LKneePitch',
                                 'LAnklePitch', 'LAnkleRoll']
                        'RLeg': ['RHipYawPitch', 'RHipRoll'
                                 'RHipPitch', 'RKneePitch',
                                 'RAnklePitch', 'RAnkleRoll']}

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        Rx = identity(4)
        Ry = identity(4)
        Rz = identity(4)
        # Update matrices if necessary
        if ("Roll" in joint_name):
            Rx = matrix([[1., 0., 0.],
                         [0., cos(joint_angle), -sin(joint_angle)],
                         [0., sin(joint_angle), cos(joint_angle)]])

        if ("Pitch" in joint_name):
            Ry = matrix([[cos(joint_angle), 0., sin(joint_angle)], 
                         [0., 1., 0.],
                         [-sin(joint_angle), 0., cos(joint_angle)]])

        if ("Yaw" in joint_name):
            Rz = matrix([[cos(joint_angle), sin(joint_angle), 0.],
                         [-sin(joint_angle), cos(joint_angle), 0.],
                         [0., 0., 1.]])

        T = Rx*Ry*Rz
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = local_trans(joint, angle)
                # YOUR CODE HERE

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
