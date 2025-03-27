import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

# Connect to GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground and Husky robot
p.loadURDF("plane.urdf")
husky_id = p.loadURDF("husky/husky.urdf", [0, 0, 0.2])

# Set gravity
p.setGravity(0, 0, -9.8)

# Identify wheel joints (manually confirmed from URDF or inspection)
wheel_joints = [2, 3, 4, 5]  # front_left, front_right, rear_left, rear_right

positions = []

def set_wheel_velocity(left_vel, right_vel, force=100):
    """Applies velocity to Husky's wheels."""
    for i, joint in enumerate(wheel_joints):
        if i % 2 == 0:  # left wheels
            p.setJointMotorControl2(husky_id, joint, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=force)
        else:  # right wheels
            p.setJointMotorControl2(husky_id, joint, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=force)

def move_husky(left_vel, right_vel, duration):
    """Moves Husky with given wheel velocities for `duration` seconds."""
    set_wheel_velocity(left_vel, right_vel)
    # steps = int(duration / (1/240.0))
    for i in range(duration * 240):
        p.stepSimulation()
        pos, _ = p.getBasePositionAndOrientation(husky_id)
        positions.append(pos[:2])
        time.sleep(1/10000)

# 1. Move straight for 5 sec
move_husky(5, 1, 10)
move_husky(1, 5, 10)
move_husky(0, 0, 1)

# Disconnect
p.disconnect()

# Plot trajectory
x_vals, y_vals = zip(*positions)
plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, marker='o', linewidth=2)
plt.title("Husky Trajectory (X-Y)")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid(True)
plt.axis("equal")
plt.show()