import pybullet as p
import pybullet_data
import numpy as np
import time

# Connect to PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and KUKA iiwa arm
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

# Load KUKA iiwa with fixed base
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)

# Define nominal control sequence for 7 joints and 5 timesteps
nominal_control = np.array([
    [0.0, 0.1, 0.2, -0.1, 0.0, 0.05, -0.05],
    [0.1, 0.2, 0.1, -0.2, 0.1, 0.1, -0.1],
    [0.2, 0.3, 0.0, -0.3, 0.2, 0.15, -0.15],
    [0.3, 0.4, -0.1, -0.4, 0.3, 0.2, -0.2],
    [0.4, 0.5, -0.2, -0.5, 0.4, 0.25, -0.25]
])

# End-effector link index (KUKA iiwa end-effector is link 6 or 7 in most URDFs)
ee_link_index = num_joints - 1  # Assuming end-effector is the last link

# Array to store end-effector positions
ee_positions = []

# Move the robot through the control sequence
for t in range(5):
    # Apply target joint positions from nominal control
    for joint_idx in range(num_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=nominal_control[t, joint_idx],
            force=500
        )
    
    # Simulate until the arm reaches the target position
    for _ in range(240):  # Simulate for 1 second (240 steps at 1/240 sec per step)
        p.stepSimulation()
        time.sleep(1./10000.)

    # Get and store the end-effector position
    link_state = p.getLinkState(robot_id, ee_link_index)
    ee_position = link_state[0]  # Extract (x, y, z) position
    ee_positions.append(ee_position)

# Print the stored end-effector positions
for i, pos in enumerate(ee_positions):
    print(f"End-effector position after timestep {i+1}: {pos}")

# Disconnect after execution
p.disconnect()
