# ========= Description & Logs =================
# Taken the optimal sequence from previous iteration as nominal sequence for the MPPI algorithm.
# Use inverse kinematics to kick start the MPPI algorithm. This time interpolate joint space
# It's Working ...... Need to tune some parameters but went close to 10cm to the goal position
# =======================================

import enum
from math import e
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
    for joint_idx in range(num_joints):
        p.setJointMotorControl2(
                        bodyIndex=robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=joint_angles[joint_idx],
                        force=300
                    )

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

def get_nominal_sequence(start_pos, goal_pos, H):
    start_joint_angles = np.array([p.getJointState(robot_id, i)[0] for i in range(num_joints)])
    goal_joint_angles = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ee_link_index,
        targetPosition=goal_pos,
        maxNumIterations=200,
        residualThreshold=1e-5
    )
    goal_joint_angles = np.array(goal_joint_angles[:num_joints])

    # Interpolate in joint space
    joint_trajectory = np.linspace(start_joint_angles, goal_joint_angles, num=H)

    return joint_trajectory

# MPPI Hyperparameters
N = 20            # Number of sampled sequences
H = 10             # Time horizon
sigma = 0.05       # Exploration noise std
lambda_ = 1.0      # Temperature for importance sampling
# nominal_sequence = np.zeros((H, num_joints))
home = p.getLinkState(robot_id, ee_link_index)[0]
nominal_sequence = get_nominal_sequence(start_pos=home, goal_pos=goal_pos, H=H)


# # === MPPI Control Loop ===
dist = np.linalg.norm(get_ee_position() - goal_pos)
step_count = 0

while dist > 0.05:
    state_id = p.saveState()

    noise = np.random.normal(0, sigma, size=(N, H, num_joints))
    samples = nominal_sequence[np.newaxis, :, :] + noise  # (N, H, num_joints)
    total_costs = np.zeros(N)

    # === Step 3: Evaluate all N sequences ===
    for i, sequence in enumerate(samples):
        p.restoreState(state_id)
        for _ in range(int(2.0 * 50)):  # Assuming simulation runs at 240Hz
            p.stepSimulation()
            time.sleep(1. / 1000.)

        for t in range(H):
            joint_angles = sequence[t]
            set_joint_angles(joint_angles)
            p.stepSimulation()
            time.sleep(1. / 240.)

            ee_pos = get_ee_position()
            collision_penalty = 100.0 if is_self_collision() or is_ground_collision() else 0.0
            total_costs[i] += compute_cost(ee_pos, goal_pos, collision_penalty)

        print(f'Step {step_count+1} | Sequence {i + 1}/{N} |')

    # === Step 5: Importance Sampling to get optimal sequence ===
    weights = np.exp(-1.0 / lambda_ * (total_costs - np.min(total_costs)))
    weights /= np.sum(weights)
    nominal_sequence = np.tensordot(weights, samples, axes=(0, 0))  # shape: (H, num_joints)

    # === Step 6: Apply first control command ===
    # first_action = nominal_sequence[0]
    new_joint_angles = nominal_sequence[0]
    set_joint_angles(new_joint_angles)
    p.stepSimulation()
    time.sleep(1. / 240.)

    # === Step 8: Print and update distance ===
    ee_pos = get_ee_position()
    dist = np.linalg.norm(ee_pos - goal_pos)
    print(f"{step_count}] EE: {ee_pos.round(3)} | Distance: {dist:.4f} \n Nominal Sequence: {nominal_sequence}")
    step_count += 1

    p.removeState(state_id)




# === Save the initial state before the sequence starts ===
state_id = p.saveState()

# for idx, sequence in enumerate(samples):
#     print(f"\n=== Simulating Sequence {idx + 1}/{N} ===")
    
#     p.restoreState(state_id)
    
#     # Wait for 2 seconds before starting the sequence
#     for _ in range(int(2.0 * 50)):  # Assuming simulation runs at 240Hz
#         p.stepSimulation()
#         time.sleep(1. / 1000.)

#     # === Simulate the sequence of joint angles ===
#     for t in range(sequence.shape[0]):  # H timesteps
#         joint_angles = sequence[t]

#         for joint_idx in range(num_joints):
#             p.setJointMotorControl2(
#                 bodyIndex=robot_id,
#                 jointIndex=joint_idx,
#                 controlMode=p.POSITION_CONTROL,
#                 targetPosition=joint_angles[joint_idx],
#                 force=300
#             )

#         p.stepSimulation()
#         time.sleep(1. / 240.)
    

# === Keep simulation running after sequence is done ===
# while True:
#     p.stepSimulation()
#     time.sleep(1. / 240.)

