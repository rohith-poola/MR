import pybullet as p
import pybullet_data
import time

# Connect to GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf, kuka_iiwa etc.

# Load environment
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

# Load KUKA iiwa
start_pos = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("kuka_iiwa/model.urdf", start_pos, start_orientation, useFixedBase=True)

# Get number of joints
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints: {num_joints}")

# Optional: print joint info
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}")

# Set joint positions (example pose)
target_positions = [0.5, -1.5, 0.5, -1.0, 0.5, 1.2, -0.8]  # 7 DoF

# Run simulation loop
for _ in range(1000):
    for joint_idx in range(num_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[joint_idx],
            force=300
        )
    p.stepSimulation()
    time.sleep(1./1000.)
