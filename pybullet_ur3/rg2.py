import pybullet as p
import pybullet_data as pd
from collections import namedtuple
from attrdict import AttrDict
import math
import time

# connect to engine servers
physicsClient = p.connect(p.GUI)
# add search path for loadURDF
p.setAdditionalSearchPath(pd.getDataPath())
# define world
p.setGravity(0,0,-9.8)
p.resetDebugVisualizerCamera(cameraDistance=0.7,cameraYaw=90,cameraPitch=-40,cameraTargetPosition=[0,0,0])

#######################################
###    define and setup robot       ###
#######################################
robotUrdfPath = "../ur3_pybullet_data/urdf/onrobot_rg2.urdf"
controlJoints = ["gripper_finger1_joint",
                 "gripper_finger2_joint",
                 "gripper_finger1_inner_knuckle_joint",
                 "gripper_finger2_inner_knuckle_joint",
                 "gripper_finger1_finger_tip_joint",
                 "gripper_finger2_finger_tip_joint"]
robotStartPos = [0, 0, 0.3]
robotStartOrn = p.getQuaternionFromEuler([0, 3.14, 0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn,
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
# load Object
tableUid = p.loadURDF("table/table.urdf",basePosition=[0.5,0,-0.65])
ObjectID = p.loadURDF("../ur3_pybullet_data/urdf/object_demo.urdf", [0, 0, 0], globalScaling=0.0030)

jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotID)
jointInfo = namedtuple("jointInfo",
                       ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity"])

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
    singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity)
    joints[singleInfo.name] = singleInfo

gripper_main_control_joint_name = "gripper_finger1_joint"
mimic_joint_name = ["gripper_finger2_joint",
                    "gripper_finger1_inner_knuckle_joint",
                    "gripper_finger2_inner_knuckle_joint",
                    "gripper_finger1_finger_tip_joint",
                    "gripper_finger2_finger_tip_joint"]
mimic_multiplier = [-1, 1, -1, -1, 1]
mimic_offset = [0, 0, 0, 0, 0]

gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 100, 0)

while True:
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    # gripper control
    gripper_opening_length = p.readUserDebugParameter(gripper_opening_length_control)
    # angle calculation
    # opening_length = 110.46 * math.sin(math.radians(2.08) + theta) - 4.0091318997
    # theta = math.asin((opening_length + 4.0091318997) / 110.46) - math.radians(2.08)
    gripper_opening_angle = math.asin((gripper_opening_length + 4.0091318997) / 110.46) - math.radians(2.08)  # angle calculation

    p.setJointMotorControl2(robotID,
                            joints[gripper_main_control_joint_name].id,
                            p.POSITION_CONTROL,
                            targetPosition=gripper_opening_angle,
                            force=joints[gripper_main_control_joint_name].maxForce,
                            maxVelocity=joints[gripper_main_control_joint_name].maxVelocity)
    for i in range(len(mimic_joint_name)):
        joint = joints[mimic_joint_name[i]]
        p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                targetPosition=gripper_opening_angle * mimic_multiplier[i],
                                force=joint.maxForce,
                                maxVelocity=joint.maxVelocity)
    p.stepSimulation()
