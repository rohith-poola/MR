import pybullet as p
import pybullet_data
import numpy as np
import time

# === Helper Functions ===
def generate_linear_waypoints(start, end, num_points=10):
    start = np.array(start)
    end = np.array(end)
    waypoints = np.linspace(start, end, num=num_points)
    return waypoints

def get_ee_pos():
    return np.array(p.getLinkState(robot_id, ee_link_index)[0])


# === Setup PyBullet GUI ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# === Load environment and robot ===
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1  # End-effector link

# === Set Dynamics (Friction, Damping) ===
# Set ground and EE friction
p.changeDynamics(plane_id, -1, lateralFriction=1.0)
p.changeDynamics(robot_id, ee_link_index, lateralFriction=1.0)

# === Create Box ===
box_size = [0.15, 0.15, 0.1]
box_mass = 4
box_start_pos = [0.4, 0, 0.1]
box_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_size)
box_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=box_size, rgbaColor=[1, 0, 0, 1])
box_id = p.createMultiBody(baseMass=box_mass, baseCollisionShapeIndex=box_col_id,
                           baseVisualShapeIndex=box_vis_id, basePosition=box_start_pos)

# box dynamics (friction + damping)
p.changeDynamics(box_id, -1, lateralFriction=1.1, linearDamping=0.9, angularDamping=0.7, restitution=0.0)

goal_pos = [0.4, 0.4, 0.22]
# === Visual goal marker ===
goal_marker_vis_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.03,
    rgbaColor=[0, 1, 0, 1]  # Green
)
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=goal_marker_vis_id,
    basePosition=goal_pos
)

# ======= Waypoints =======
home = p.getLinkState(robot_id, ee_link_index)[0]
waypoints = [
    [0.4, -0.4, 0.1],   # near side of box
    [0.4, 0.2, 0.1]   # push target
]

# Generate smooth trajectory
array1 = generate_linear_waypoints(home, waypoints[0], num_points=10)
array2 = generate_linear_waypoints(waypoints[0], waypoints[1], num_points=10)
goal_array = np.concatenate((array1, array2), axis=0)

# === Move to each waypoint ===
for goal_pos in goal_array:
    ik_solution = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ee_link_index,
        targetPosition=goal_pos,
        maxNumIterations=200,
        residualThreshold=1e-4 # Affects the convergence speed
    )

    for joint_idx in range(num_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=ik_solution[joint_idx],
            force=250,
            positionGain=0.08,  # joint aggression control
            velocityGain=1.0
        )

    # Wait until EE reaches the target within a small threshold
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)
        ee_pos = get_ee_pos()
        if np.linalg.norm(ee_pos - goal_pos) < 0.05:
            break

# === Keep simulation running after all waypoints ===
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)
