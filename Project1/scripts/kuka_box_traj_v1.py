import pybullet as p
import pybullet_data
import numpy as np
import time

# Helper Functions
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
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1  # End-effector link

# === Define multiple waypoints ===
home = p.getLinkState(robot_id, ee_link_index)[0]
waypoints = [
    [0.4, -0.4, 0.1],
    [0.4, 0.4, 0.1]
]

array1 = generate_linear_waypoints(home, waypoints[0])
array2 = generate_linear_waypoints(waypoints[0], waypoints[1])
goal_array = np.concatenate((array1, array2), axis=0)

# === Move to each waypoint one by one ===
for goal_pos in waypoints:
    print(f"Moving to waypoint: {goal_pos}")
    
    ik_solution = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ee_link_index,
        targetPosition=goal_pos,
        maxNumIterations=200,
        residualThreshold=1e-5
    )

    for joint_idx in range(num_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=ik_solution[joint_idx],
            force=1000
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
