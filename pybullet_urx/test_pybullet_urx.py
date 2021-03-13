import pybullet as p
import pybullet_data as pd
from pybullet_ur3.ur3robot import ur3Sim

#video requires ffmpeg available in path
createVideo = False

# connect to engine servers
if createVideo:
    p.connect(p.GUI, options="--mp4=\"pybullet_grasp.mp4\", --mp4fps=240")
else:
    p.connect(p.GUI)

# add search path for loadURDF
p.setAdditionalSearchPath(pd.getDataPath())
# define world
timeStep = 1./120. #240.
p.setTimeStep(timeStep)
p.setGravity(0, 0, -9.8)
p.resetDebugVisualizerCamera(cameraDistance=0.7, cameraYaw=70, cameraPitch=-40, cameraTargetPosition=[0.3, 0, 0.5])
p.setRealTimeSimulation(1)

ur3 = ur3Sim(p, [0, 0, 0], "192.168.56.1")
ur3.reset()

while True:
    # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    if createVideo:
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    # speed control
    ur3.moveSpeed()
    # robot control
    ur3.virtualRobotControl()
    ur3.realRobotControl()
    # gripper control
    ur3.virtualGripperControl()
    ur3.realGripperControl()

