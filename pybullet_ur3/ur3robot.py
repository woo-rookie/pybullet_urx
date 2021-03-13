import pybullet as p
import numpy as np
import math
import time
import urx
from collections import namedtuple
from attrdict import AttrDict
from urx.gripper import OnRobotGripperRG2

ur3NumDofs = 6

#lower limits for null space
ll = [-7]*ur3NumDofs
#upper limits for null space
ul = [7]*ur3NumDofs
#joint ranges for null space
jr = [7]*ur3NumDofs
#restposes for null space
rp = [0, -0.5 * math.pi, 0, -math.pi, -0.5 * math.pi, math.pi]

gripper_main_control_joint_name = "gripper_finger1_joint"
EndEffector_joint_name = "ur_grasptarget_hand"
mimic_joint_name = ["gripper_finger2_joint",
                    "gripper_finger1_inner_knuckle_joint",
                    "gripper_finger2_inner_knuckle_joint",
                    "gripper_finger1_finger_tip_joint",
                    "gripper_finger2_finger_tip_joint"]
mimic_multiplier = [-1, 1, -1, -1, 1]
mimic_offset = [0, 0, 0, 0, 0]
position_control_joint_name = ["shoulder_pan_joint",
                               "shoulder_lift_joint",
                               "elbow_joint",
                               "wrist_1_joint",
                               "wrist_2_joint",
                               "wrist_3_joint"]

class ur3Sim(object):
    def __init__(self, bullet_client, offset, ip):
        self.rob = urx.Robot(ip)
        self.trans = self.rob.get_pose()
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)

        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        robotUrdfPath = "ur3_pybullet_data/ur3_visual_grabbing_with_rg2.urdf"
        robotStartPos = [0, 0, 0]
        robotStartOrn = self.bullet_client.getQuaternionFromEuler([0, 0, 0])
        self.ur3 = self.bullet_client.loadURDF(robotUrdfPath, np.array(robotStartPos) + self.offset, robotStartOrn,
                                                 useFixedBase=True, flags=flags)

        self.finger_target = 0
        # create a constraint to keep the fingers centered
        jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        numJoints = self.bullet_client.getNumJoints(self.ur3)
        jointInfo = namedtuple("jointInfo",
                               ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity"])
        self.joints = AttrDict()
        # get jointInfo
        for i in range(numJoints):
            self.bullet_client.changeDynamics(self.ur3, i, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.ur3, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = jointTypeList[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce,
                                   jointMaxVelocity)
            self.joints[singleInfo.name] = singleInfo

    def reset(self, a=0.1, v=0.1, reset_gripper_length=0, reset_joint_position=None):
        if reset_joint_position is None:
            reset_joint_position = [0, -0.5 * math.pi, 0, -math.pi, -0.5 * math.pi, math.pi]
        self.move_velocity_control = p.addUserDebugParameter("move_velocity", 0, 1, v)
        self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 101, reset_gripper_length)
        self.position_control_group = []
        self.position_control_group.append(p.addUserDebugParameter("shoulder_pan_joint", -math.pi, math.pi, reset_joint_position[0]))
        self.position_control_group.append(p.addUserDebugParameter("shoulder_lift_joint", -math.pi, math.pi, reset_joint_position[1]))
        self.position_control_group.append(p.addUserDebugParameter("elbow_joint", -math.pi, math.pi, reset_joint_position[2]))
        self.position_control_group.append(p.addUserDebugParameter("wrist_1_joint", -math.pi, math.pi, reset_joint_position[3]))
        self.position_control_group.append(p.addUserDebugParameter("wrist_2_joint", -math.pi, math.pi, reset_joint_position[4]))
        self.position_control_group.append(p.addUserDebugParameter("wrist_3_joint", -math.pi, math.pi, reset_joint_position[5]))
        i = 0
        for jointName in self.joints:
            if jointName in position_control_joint_name:
                self.bullet_client.setJointMotorControl2(self.ur3, i, self.bullet_client.POSITION_CONTROL,rp[i])
                i += 1
            if jointName == EndEffector_joint_name:
                self.virtualGripperControl()
        self.rob.movej((rp), a, v)
        gripper = OnRobotGripperRG2(self.rob)
        gripper.open_gripper(
            target_width=reset_gripper_length,  # Width in mm, 110 is fully open
            target_force=40,  # Maximum force applied in N, 40 is maximum
            payload=0.5,  # Payload in kg
            set_payload=False,  # If any payload is attached
            depth_compensation=False,  # Whether to compensate for finger depth
            slave=False,  # Is this gripper the master or slave gpper?
            wait=1  # Wait up to 1s for movement
        )

    def moveSpeed(self):
        self.move_velocity = p.readUserDebugParameter(self.move_velocity_control)

    def virtualGripperControl(self):
        self.gripper_opening_length = p.readUserDebugParameter(self.gripper_opening_length_control)
        self.gripper_opening_angle = self.gripper_opening_length * 0.8/101
        self.bullet_client.setJointMotorControl2(self.ur3,
                                                 self.joints[gripper_main_control_joint_name].id,
                                                 self.bullet_client.POSITION_CONTROL,
                                                 targetPosition=self.gripper_opening_angle,
                                                 force=self.joints[gripper_main_control_joint_name].maxForce,
                                                 maxVelocity=self.joints[
                                                     gripper_main_control_joint_name].maxVelocity)
        for i in range(len(mimic_joint_name)):
            joint = self.joints[mimic_joint_name[i]]
            self.bullet_client.setJointMotorControl2(self.ur3, joint.id, self.bullet_client.POSITION_CONTROL,
                                                     targetPosition=self.gripper_opening_angle * mimic_multiplier[i] + mimic_offset[i],
                                                     force=10,
                                                     maxVelocity=joint.maxVelocity)

    def realGripperControl(self):
        gripper = OnRobotGripperRG2(self.rob)
        gripper.open_gripper(
            target_width=self.gripper_opening_length,  # Width in mm, 110 is fully open
            target_force=40,  # Maximum force applied in N, 40 is maximum
            payload=0.5,  # Payload in kg
            set_payload=False,  # If any payload is attached
            depth_compensation=False,  # Whether to compensate for finger depth
            slave=False,  # Is this gripper the master or slave gripper?
            wait=1  # Wait up to 1s for movement
        )


    def virtualRobotControl(self):
        self.parameter = []
        for i in range(6):
            self.parameter.append(p.readUserDebugParameter(self.position_control_group[i]))
        num = 0
        for jointName in self.joints:
            if jointName in position_control_joint_name:
                joint = self.joints[jointName]
                p.setJointMotorControl2(self.ur3, joint.id, p.POSITION_CONTROL,
                                        targetPosition=self.parameter[num],
                                        force=joint.maxForce,
                                        maxVelocity=joint.maxVelocity)
                num += 1

    def realRobotControl(self, a=0.1):
        time.sleep(1) # Wait up to 1s for movement
        self.rob.movej((self.parameter), a, self.move_velocity)
