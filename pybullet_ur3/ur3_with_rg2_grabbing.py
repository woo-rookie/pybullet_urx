import pybullet as p
import pybullet_data as pd
from collections import namedtuple
from attrdict import AttrDict
from datetime import datetime
import math
import time

# video requires ffmpeg available in path
createVideo = False

# connect to engine servers
if createVideo:
    p.connect(p.GUI, options="--mp4=\"pybullet_grasp.mp4\", --mp4fps=240")
else:
    p.connect(p.GUI)

# add search path for loadURDF
p.setAdditionalSearchPath(pd.getDataPath())
# define world
timeStep = 1. / 120.  # 240.
p.setTimeStep(timeStep)
p.setGravity(0, 0, -9.8)
p.resetDebugVisualizerCamera(cameraDistance=0.7, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[-0.5, -0.5, 0.5])

#######################################
###    define and setup robot       ###
#######################################
robotUrdfPath = "../ur3_pybullet_data/urdf/ur3_visual_grabbing_with_rg2.urdf"
robotStartPos = [0, 0, 0]
robotStartOrn = p.getQuaternionFromEuler([0, 0, 0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, useFixedBase=True,
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
# load Object
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
boxUid = p.loadURDF("tray/traybox.urdf", basePosition=[-0.5, 0, -0.1], flags=flags)
kinect_v2Uid = p.loadURDF("../ur3_pybullet_data/urdf/kinect_v2.urdf", basePosition=[-0.4, 0, 1.5])
# objectUid = p.loadURDF("cube_small.urdf",basePosition=[0.3,0,0])
legos = []
sphereId = []
legos.append(p.loadURDF("lego/lego.urdf", [-0.4, 0, 0.01], flags=flags))
p.changeVisualShape(legos[0], -1, rgbaColor=[1, 0, 0, 1])
legos.append(p.loadURDF("lego/lego.urdf", [-0.5, 0, 0.01], flags=flags))
legos.append(p.loadURDF("lego/lego.urdf", [-0.6, 0, 0.01], flags=flags))
sphereId.append(p.loadURDF("sphere_small.urdf", [-0.4, -0.1, 0.01], flags=flags))
sphereId.append(p.loadURDF("sphere_small.urdf", [-0.5, 0.1, 0.01], flags=flags))
sphereId.append(p.loadURDF("sphere_small.urdf", [-0.6, -0.1, 0.01], flags=flags))

# sphere = p.loadURDF("sphere_small.urdf", basePosition=[0.3, 0.2, 0.1])

jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotID)
jointInfo = namedtuple("jointInfo",
                       ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity"])

joints = AttrDict()
# get jointInfo
for i in range(numJoints):
    info = p.getJointInfo(robotID, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = jointTypeList[info[2]]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce,
                           jointMaxVelocity)
    joints[singleInfo.name] = singleInfo

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

gripper_opening_angle_control = p.addUserDebugParameter("gripper_opening_angle", 0, 0.8, 0.8)


def gripper_control(state):
    p.setJointMotorControl2(robotID,
                            joints[gripper_main_control_joint_name].id,
                            p.POSITION_CONTROL,
                            targetPosition=state,
                            force=joints[gripper_main_control_joint_name].maxForce,
                            maxVelocity=joints[gripper_main_control_joint_name].maxVelocity)
    for i in range(len(mimic_joint_name)):
        joint = joints[mimic_joint_name[i]]
        p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                targetPosition=state * mimic_multiplier[i] + mimic_offset[i],
                                force=joint.maxForce,
                                maxVelocity=joint.maxVelocity)


ikSolver = 0
ur3NumDofs = 6
# lower limits for null space
ll = [-5] * ur3NumDofs
# upper limits forull space
ul = [5] * ur3NumDofs
# joint ranges for null space
jr = [5] * ur3NumDofs
# restposes for null space
# rp = [0, -0.5 * math.pi, 0, 0, 0.5 * math.pi, 0]
rp = [0, -0.5 * math.pi, 0, -math.pi, -0.5 * math.pi, math.pi]
# rp = [0] * ur3NumDofs
# joint damping coefficents
jd = [0] * ur3NumDofs

def reset():
    i = 0
    for jointName in joints:
        if jointName in position_control_joint_name:
            # p.resetJointState(robotID, i, rp[i])
            p.setJointMotorControl2(robotID, i, p.POSITION_CONTROL, rp[i])
            i += 1
        if jointName == EndEffector_joint_name:
            gripper_control(0.8)

reset()
while True:
    # allow the manipulator to render slowly
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    p.stepSimulation()
    time.sleep(timeStep)
    # pos = [0.4, 0.3, 0.3]
    pos, o = p.getBasePositionAndOrientation(legos[0])
    pos = [-pos[0], pos[1], pos[2]]
    orn = p.getQuaternionFromEuler([math.pi, 0, 0])

    jointPoses = p.calculateInverseKinematics(robotID, joints[EndEffector_joint_name].id, pos, orn, ll, ul,
                                              jr, rp, maxNumIterations=10)
    for i in range(ur3NumDofs):
        p.setJointMotorControl2(robotID, i, p.POSITION_CONTROL, jointPoses[i],
                                                 force=5 * 240.)
    gripper_control(0.8)