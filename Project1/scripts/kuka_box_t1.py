import pybullet as p
import pybullet_data
import time

# Connect to GUI
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane, etc.

# Load ground
plane_id = p.loadURDF("plane.urdf")

# Load robot arm (e.g., KUKA iiwa or UR5 from PyBullet examples)
robot_start_pos = [0, 0, 0]
robot_start_ori = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("kuka_iiwa/model.urdf", robot_start_pos, robot_start_ori, useFixedBase=True)

# Set home configuration
home_joint_positions = [0, 0.5, 0, -1.2, 0, 1.0, 0.8]
for joint_idx in range(p.getNumJoints(robot_id)):
    p.resetJointState(robot_id, joint_idx, home_joint_positions[joint_idx])

# Create the box
box_size = [0.15, 0.15, 0.1]
box_mass = 0.2
box_start_pos = [0.5, 0, 0.1]
box_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_size)
box_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=box_size, rgbaColor=[1, 0, 0, 1])
box_id = p.createMultiBody(baseMass=box_mass, baseCollisionShapeIndex=box_col_id,
                           baseVisualShapeIndex=box_vis_id, basePosition=box_start_pos)


# Creating a Goal position marker
# goal_pos = [0.5, -0.5, 0.22]
goal_pos = [0.4, -0.4, 0.3]
# Create a purely visual sphere (no collision, no physics)
goal_marker_vis_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.03,
    rgbaColor=[0, 1, 0, 1]  # Red
)

goal_marker_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,         # No collision
    baseVisualShapeIndex=goal_marker_vis_id,
    basePosition=goal_pos
)

# Run the simulation
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1. / 240.)
