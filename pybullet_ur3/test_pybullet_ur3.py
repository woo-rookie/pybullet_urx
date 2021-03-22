
import pybullet as p
import pybullet_data as pd
from collections import namedtuple
from attrdict import AttrDict
import math
import time

ur3NumDofs = 6

#lower limits for null space
ll = [-7]*ur3NumDofs
#upper limits for null space
ul = [7]*ur3NumDofs
#joint ranges for null space
jr = [7]*ur3NumDofs
#restposes for null space
# rp = [0, -0.5 * math.pi, 0, 0, 0.5 * math.pi, 0]
# rp = [0, -0.5 * math.pi, 0, -math.pi, -0.5 * math.pi, math.pi]
rp = [0, -0.5 * math.pi, 0, 0, 0.5 * math.pi, math.pi]

# connect to engine servers
physicsClient = p.connect(p.GUI)
# add search path for loadURDF
p.setAdditionalSearchPath(pd.getDataPath())
# define world
timeStep=1./120.#240.
p.setTimeStep(timeStep)
p.setGravity(0,0,-9.8)
p.resetDebugVisualizerCamera(cameraDistance=0.7,cameraYaw=90,cameraPitch=-40,cameraTargetPosition=[0.5,0,0.5])

#######################################
###    define and setup robot       ###
#######################################
robotUrdfPath = "./ur3_pybullet_data/urdf/ur3_visual_grabbing_with_rg2.urdf"
robotStartPos = [0, 0, 0]
robotStartOrn = p.getQuaternionFromEuler([0, 0, math.pi])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, useFixedBase=True,
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
# load Object
tableUid = p.loadURDF("table/table.urdf",basePosition=[0.5,0,-0.65])
kinect_v2Uid = p.loadURDF("./ur3_pybullet_data/urdf/kinect_v2.urdf",basePosition=[0.4,0,1.57])
objectUid = p.loadURDF("cube_small.urdf",basePosition=[0.7,0,0])

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
    # print('{}'.format(info))

EndEffector_joint_name = "ur_grasptarget_hand"
gripper_main_control_joint_name = "gripper_finger1_joint"
mimic_joint_name = ["gripper_finger2_joint",
                    "gripper_finger1_inner_knuckle_joint",
                    "gripper_finger2_inner_knuckle_joint",
                    "gripper_finger1_finger_tip_joint",
                    "gripper_finger2_finger_tip_joint"]
mimic_multiplier = [-1, 1, -1, -1, 1]
mimic_offset = [0, 0, 0, 0, 0]

gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 100, 0)

# joint position control
position_control_group = []
position_control_group.append(p.addUserDebugParameter("shoulder_pan_joint", -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter("shoulder_lift_joint", -math.pi, math.pi, -math.pi / 2))
position_control_group.append(p.addUserDebugParameter("elbow_joint", -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter("wrist_1_joint", -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter("wrist_2_joint", -math.pi, math.pi, math.pi / 2))
position_control_group.append(p.addUserDebugParameter("wrist_3_joint", -math.pi, math.pi, 0))

# TCP control
TCP_control_group = []
TCP_control_group.append(p.addUserDebugParameter("X", -0.5, 0.5, 0.02))
TCP_control_group.append(p.addUserDebugParameter("Y", -0.5, 0.5, 0))
TCP_control_group.append(p.addUserDebugParameter("Z", 0, 1, 1))
TCP_control_group.append(p.addUserDebugParameter("RX", -math.pi, math.pi, 0))
TCP_control_group.append(p.addUserDebugParameter("RY", -math.pi, math.pi, 0))
TCP_control_group.append(p.addUserDebugParameter("RZ", -math.pi, math.pi, 0))

position_control_joint_name = ["shoulder_pan_joint",
                               "shoulder_lift_joint",
                               "elbow_joint",
                               "wrist_1_joint",
                               "wrist_2_joint",
                               "wrist_3_joint"]

while True:
    p.stepSimulation()
    # allow the manipulator to render slowly
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    # ur3 control
    parameter = []
    for i in range(6):
        parameter.append(p.readUserDebugParameter(position_control_group[i]))
    num = 0
    for jointName in joints:
        if jointName in position_control_joint_name:
            joint = joints[jointName]
            p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                    targetPosition=parameter[num],
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity)
            num += 1

    # pos = [TCP_control_group[0], TCP_control_group[1], TCP_control_group[2]]
    # orn = [TCP_control_group[3], TCP_control_group[4], TCP_control_group[5]]
    # jointPoses = p.calculateInverseKinematics(robotID, joints[EndEffector_joint_name].id, pos,
    #                                                            orn, ll, ul, jr, rp, maxNumIterations=20)
    # num = 0
    # for jointName in joints:
    #     if jointName in position_control_joint_name:
    #         joint = joints[jointName]
    #         p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
    #                                 targetPosition=jointPoses[num],
    #                                 force=joint.maxForce,
    #                                 maxVelocity=joint.maxVelocity)
    #         num += 1

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
                                targetPosition=gripper_opening_angle * mimic_multiplier[i]+mimic_offset[i],
                                force=joint.maxForce,
                                maxVelocity=joint.maxVelocity)
    time.sleep(timeStep)