import pybullet as p
import pybullet_data as pd
from collections import namedtuple
from attrdict import AttrDict
import time
import numpy as np
import math

useNullSpace = 1
ikSolver = 0
ur3NumDofs = 6

#lower limits for null space
ll = [-7]*ur3NumDofs
#upper limits for null space
ul = [7]*ur3NumDofs
#joint ranges for null space
jr = [7]*ur3NumDofs
#restposes for null space
# rp = [0, -0.5 * math.pi, 0, 0, 0.5 * math.pi, 0]
rp = [0, -0.5 * math.pi, 0, -math.pi, -0.5 * math.pi, math.pi]

gripper_main_control_joint_name = "gripper_finger1_joint"
EndEffector_joint_name = "ur_grasptarget_hand"
# EndEffector_joint_name = "ee_fixed_joint"
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
    def __init__(self, bullet_client, offset):
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)

        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.legos = []
        self.sphereId = []
        self.bullet_client.loadURDF("tray/traybox.urdf", [-0.4 + offset[0], 0 + offset[1], -0.1 + offset[2]], flags=flags)
        self.legos = self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.3, -0.1, 0]) + self.offset, flags=flags)
        # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.3, -0.1, 0]) + self.offset, flags=flags))
        # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.4, 0.1, 0]) + self.offset, flags=flags))
        # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.5, -0.1, 0]) + self.offset, flags=flags))
        # self.bullet_client.changeVisualShape(self.legos[0], -1, rgbaColor=[1, 0, 0, 1])
        # self.sphereId.append(self.bullet_client.loadURDF("sphere_small.urdf", np.array([-0.3, 0.1, 0]) + self.offset, flags=flags))
        # self.sphereId.append(self.bullet_client.loadURDF("sphere_small.urdf", np.array([-0.4, -0.1, 0]) + self.offset, flags=flags))
        # self.sphereId.append(self.bullet_client.loadURDF("sphere_small.urdf", np.array([-0.5, 0.1, 0]) + self.offset, flags=flags))
        robotUrdfPath = "../ur3_pybullet_data/urdf/ur3_visual_grabbing_with_rg2.urdf"
        robotStartPos = [0, 0, 0]
        robotStartOrn = self.bullet_client.getQuaternionFromEuler([0, 0, math.pi])
        self.ur3 = self.bullet_client.loadURDF(robotUrdfPath, np.array(robotStartPos) + self.offset, robotStartOrn,
                                                 useFixedBase=True, flags=flags)

        self.state = 0
        self.theta = 0
        self.control_dt = 1. / 120.
        self.finger_target = 100
        self.gripper_opening_angle = 0
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

    def gripper_control(self, finger_target):
        # angle calculation
        # opening_length = 110.46 * math.sin(math.radians(2.08) + theta) - 4.0091318997
        # theta = math.asin((opening_length + 4.0091318997) / 110.46) - math.radians(2.08)
        self.gripper_opening_angle = math.asin((finger_target + 4.0091318997) / 110.46) - math.radians(2.08)  # angle calculation
        self.bullet_client.setJointMotorControl2(self.ur3,
                                                 self.joints[gripper_main_control_joint_name].id,
                                                 self.bullet_client.POSITION_CONTROL,
                                                 targetPosition=self.gripper_opening_angle,
                                                 force=100,
                                                 maxVelocity=self.joints[gripper_main_control_joint_name].maxVelocity)
        for i in range(len(mimic_joint_name)):
            joint = self.joints[mimic_joint_name[i]]
            self.bullet_client.setJointMotorControl2(self.ur3, joint.id, self.bullet_client.POSITION_CONTROL,
                                                     targetPosition=self.gripper_opening_angle * mimic_multiplier[i] + mimic_offset[i],
                                                     force=100,
                                                     maxVelocity=joint.maxVelocity)
    def reset(self):
        i = 0
        for jointName in self.joints:
            if jointName in position_control_joint_name:
                self.bullet_client.setJointMotorControl2(self.ur3, i, self.bullet_client.POSITION_CONTROL,rp[i])
                i += 1
            if jointName == EndEffector_joint_name:
                self.gripper_control(self.finger_target)
        self.t = 0.

    def update_state(self):
        keys = self.bullet_client.getKeyboardEvents()
        if len(keys) > 0:
            for k, v in keys.items():
                if v & self.bullet_client.KEY_WAS_TRIGGERED:
                    if (k == ord('1')):
                        self.state = 1
                    if (k == ord('2')):
                        self.state = 2
                    if (k == ord('3')):
                        self.state = 3
                    if (k == ord('4')):
                        self.state = 4
                    if (k == ord('5')):
                        self.state = 5
                    if (k == ord('6')):
                        self.state = 6
                if v & self.bullet_client.KEY_WAS_RELEASED:
                    self.state = 0

    def step(self):
        if self.state == 6:
            self.finger_target = 80
        if self.state == 5:
            self.finger_target = 20
        self.bullet_client.submitProfileTiming("step")
        self.update_state()

        if self.state == 1 or self.state == 2 or self.state == 3 or self.state == 4 or self.state == 7:
            # self.gripper_height = 0.001*(math.s(self.gripper_opening_angle + math.radians(2.08))+26.2)
            if self.state == 2 or self.state == 3 or self.state == 7:
                self.gripper_height = 0.001*(math.cos(self.gripper_opening_angle + math.radians(2.08)) + 0.02)
            self.t += self.control_dt
            global pos
            if self.state == 3 or self.state == 4:
                pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos)
                pos = [pos[0], pos[1], self.gripper_height+0.04]
                self.prev_pos = pos
            if self.state == 7:
                pos = self.prev_pos
                diffX = pos[0] + self.offset[0]
                diffY = pos[1] + self.offset[1]
                # diffZ = pos[2] + self.offset[2] + 0.1
                self.prev_pos = [diffX, diffY, self.prev_pos[2]+0.02]

            orn = self.bullet_client.getQuaternionFromEuler([0., math.pi/2., 0.])
            self.bullet_client.submitProfileTiming("IK")

            jointPoses = self.bullet_client.calculateInverseKinematics(self.ur3, self.joints[EndEffector_joint_name].id, pos, orn, ll, ul,
                                                                       jr, rp, maxNumIterations=20)
            self.bullet_client.submitProfileTiming()
            for i in range(ur3NumDofs):
                self.bullet_client.setJointMotorControl2(self.ur3, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],
                                                         force=5 * 240.)

        # target for fingers
        self.gripper_control(self.finger_target)
        self.bullet_client.submitProfileTiming()


class ur3SimAuto(ur3Sim):
    def __init__(self, bullet_client, offset):
        ur3Sim.__init__(self, bullet_client, offset)
        self.state_t = 0
        self.cur_state = 0
        self.states = [0, 3, 5, 4, 6, 3, 7]
        self.state_durations = [1, 1, 1, 2, 1, 1, 10]
        ur3Sim.reset(self)

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]

#video requires ffmpeg available in path
createVideo=False

# connect to engine servers
if createVideo:
    p.connect(p.GUI, options="--mp4=\"pybullet_grasp.mp4\", --mp4fps=240")
else:
    p.connect(p.GUI)

# p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

# add search path for loadURDF
p.setAdditionalSearchPath(pd.getDataPath())
# define world
timeStep=1./120.#240.
p.setTimeStep(timeStep)
p.setGravity(0,0,-9.8)
p.resetDebugVisualizerCamera(cameraDistance=0.7,cameraYaw=0,cameraPitch=0,cameraTargetPosition=[0.5, -0.5, 0.5])

ur3 = ur3SimAuto(p,[0,0,0])
logId = ur3.bullet_client.startStateLogging(ur3.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
ur3.bullet_client.submitProfileTiming("start")
for i in range (100000):
    # allow the manipulator to render slowly
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

    ur3.bullet_client.submitProfileTiming("full_step")
    ur3.step()
    p.stepSimulation()
    if createVideo:
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    time.sleep(timeStep)
    ur3.bullet_client.submitProfileTiming()
ur3.bullet_client.submitProfileTiming()
ur3.bullet_client.stopStateLogging(logId)
