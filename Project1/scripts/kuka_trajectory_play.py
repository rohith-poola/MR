import pybullet as p
import pybullet_data
import numpy as np
import time
import csv

# === Path to your joint angle CSV ===
csv_file = "/Users/rohith/MS/MR/Project1/data/mppi_kuka_20250415_123937.csv"

# === Load joint angle sequence ===
joint_trajectory = []
with open(csv_file, 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if any(cell.strip().lower().startswith("joint") for cell in row):
            continue
        joint_angles = [float(x) for x in row]
        joint_trajectory.append(joint_angles)

# === Launch PyBullet GUI ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)

# === Replay each joint configuration ===
for step_angles in joint_trajectory:
    for i in range(len(step_angles)):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=step_angles[i],
            force=200
        )
    for _ in range(10): 
        p.stepSimulation()
        time.sleep(1. / 240.)

while True:
    p.stepSimulation()
    time.sleep(1. / 240.)
