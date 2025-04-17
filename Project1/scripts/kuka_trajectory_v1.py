import pybullet as p
import pybullet_data
import numpy as np
import time

# === Setup PyBullet GUI ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# === Load environment and robot ===
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1  # End-effector link

# === Define goal ===
goal_pos = [0.4, -0.4, 0.3]  # X, Y, Z (world coordinates)
# goal_ori = p.getQuaternionFromEuler([np.pi, np.pi/2, 0])  # EE facing downward


# === Visualize goal and EE orientation ===
p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 1])
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 1]),
    basePosition=goal_pos
)

# === Inverse Kinematics ===
ik_solution = p.calculateInverseKinematics(
    bodyUniqueId=robot_id,
    endEffectorLinkIndex=ee_link_index,
    targetPosition=goal_pos,
)

# === Send joint commands ===
for joint_idx in range(num_joints):
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=joint_idx,
        controlMode=p.POSITION_CONTROL,
        targetPosition=ik_solution[joint_idx],
        force=1000
    )

# === Simulate to move robot ===
for _ in range(240):
    p.stepSimulation()
    time.sleep(1. / 100.)

# === Keep simulation running ===
while True:
    p.stepSimulation()
    time.sleep(1. / 100.)
