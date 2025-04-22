# ========= Description =================
# Objective is to move the end effector of the KUKA IIWA robot to a specified goal position using the MPPI algorithm.
# =======================================

import numpy as np
import pybullet as p
import pybullet_data
import time

# Simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)

goal_pos = [0.4, -0.4, 0.3]  # Desired EE goal position

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

# MPPI parameters
N = 100  # number of rollouts
H = 15   # time horizon
lambda_ = 1.0  # temperature
sigma = 0.05   # std for delta theta


def get_joint_angles():
    return np.array([p.getJointState(robot_id, i)[0] for i in range(num_joints)])

def set_joint_angles(joint_angles):
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=200)

def get_end_effector_position():
    state = p.getLinkState(robot_id, num_joints - 1)
    return np.array(state[0])  # Position

def is_self_collision():
    contact_points = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)
    if contact_points is None:
        return 0
    return len(contact_points) > 0

def is_ground_collision():
    contact_points = p.getContactPoints(bodyA=robot_id, bodyB=-1)
    if contact_points is None:
        return 0
    return len(contact_points) > 0

def compute_cost(ee_pos, goal, collision_penalty):
    dist_cost = np.linalg.norm(ee_pos - goal)
    return dist_cost + collision_penalty

time.sleep(10)
dist = np.linalg.norm(get_end_effector_position() - goal_pos)
# Main MPPI loop
while dist > 0.05:
    state_id = p.saveState()
    costs = []
    all_action_seqs = []

    for _ in range(N):
        p.restoreState(state_id)
        joint_traj = []
        curr_joint_angles = get_joint_angles()
        total_cost = 0

        for t in range(H):
            delta = np.random.normal(0, sigma, num_joints)
            new_angles = curr_joint_angles + delta
            set_joint_angles(new_angles)
            p.stepSimulation()
            curr_joint_angles = new_angles
            joint_traj.append(delta)

            ee_pos = get_end_effector_position()
            collision_penalty = 10.0 if is_self_collision() or is_ground_collision() else 0.0
            total_cost += compute_cost(ee_pos, goal_pos, collision_penalty)

        costs.append(total_cost)
        all_action_seqs.append(joint_traj)

    # Softmin over costs (MPPI)
    costs = np.array(costs)
    action_weights = np.exp(-1.0 / lambda_ * (costs - np.min(costs)))
    action_weights /= np.sum(action_weights)

    # Weighted average of first action in each sequence
    avg_action = np.zeros(num_joints)
    for i in range(N):
        avg_action += action_weights[i] * all_action_seqs[i][0]

    # Apply averaged action
    final_angles = get_joint_angles() + avg_action
    set_joint_angles(final_angles)
    p.stepSimulation()
    time.sleep(1. / 240.)
    print(f'End effector position: {get_end_effector_position()}')
    dist = np.linalg.norm(get_end_effector_position() - goal_pos)



# # Draw the trajectory
# for i in range(len(ee_traj) - 1):
#     p.addUserDebugLine(
#         ee_traj[i],
#         ee_traj[i + 1],
#         lineColorRGB=[1, 0, 1],  # Magenta
#         lineWidth=2,
#         lifeTime=0
#     )

# # Mark start and goal
# p.addUserDebugText("START", ee_traj[0], textColorRGB=[0, 1, 0], textSize=1.5)
# p.addUserDebugText("GOAL", goal_pos, textColorRGB=[1, 0, 0], textSize=1.5)

# # Optional: Keep simulation running
# while True:
#     p.stepSimulation()
#     time.sleep(1. / 240.)