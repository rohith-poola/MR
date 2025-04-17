# ========= Description & Logs =================
# Taken the optimal sequence from previous iteration as nominal sequence for the MPPI algorithm.
# Working - 300 steps required to reach the goal position
# =======================================

import numpy as np
import pybullet as p
import pybullet_data
import time

# Setup PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1
goal_pos = [0.4, -0.4, 0.3]

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


# MPPI Functions
def get_joint_angles():
    return np.array([p.getJointState(robot_id, i)[0] for i in range(num_joints)])

def set_joint_angles(joint_angles):
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=300)

def get_ee_position():
    return np.array(p.getLinkState(robot_id, ee_link_index)[0])

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
    return np.linalg.norm(ee_pos - goal) + collision_penalty


# MPPI Hyperparameters
N = 100            # Number of sampled sequences
H = 10             # Time horizon
sigma = 0.05       # Exploration noise std
lambda_ = 1.0      # Temperature for importance sampling
nominal_sequence = np.zeros((H, num_joints))

# === MPPI Control Loop ===
dist = np.linalg.norm(get_ee_position() - goal_pos)
step_count = 0

while dist > 0.05:
    state_id = p.saveState()

    # === Step 2: Sample N sequences (N, H, num_joints) around nominal ===
    noise = np.random.normal(0, sigma, size=(N, H, num_joints))
    samples = nominal_sequence[np.newaxis, :, :] + noise  # (N, H, num_joints)

    total_costs = np.zeros(N)

    # === Step 3: Evaluate all N sequences ===
    for i in range(N):
        p.restoreState(state_id)
        joint_angles = get_joint_angles()
        sequence = samples[i]

        for t in range(H):
            action = sequence[t]
            joint_angles += action
            set_joint_angles(joint_angles)
            p.stepSimulation()

            ee_pos = get_ee_position()
            collision_penalty = 10.0 if is_self_collision() or is_ground_collision() else 0.0
            total_costs[i] += compute_cost(ee_pos, goal_pos, collision_penalty)

    # === Step 5: Importance Sampling to get optimal sequence ===
    weights = np.exp(-1.0 / lambda_ * (total_costs - np.min(total_costs)))
    weights /= np.sum(weights)
    optimal_sequence = np.tensordot(weights, samples, axes=(0, 0))  # shape: (H, num_joints)

    # === Step 6: Apply first control command ===
    first_action = optimal_sequence[0]
    new_joint_angles = get_joint_angles() + first_action
    set_joint_angles(new_joint_angles)
    p.stepSimulation()
    time.sleep(1. / 240.)

    # === Step 7: Update nominal sequence ===
    nominal_sequence = np.vstack([optimal_sequence[1:], np.zeros((1, num_joints))])

    # === Step 8: Print and update distance ===
    ee_pos = get_ee_position()
    dist = np.linalg.norm(ee_pos - goal_pos)
    print(f"[{step_count}] EE: {ee_pos.round(3)} | Distance: {dist:.4f}")
    step_count += 1

    p.removeState(state_id)