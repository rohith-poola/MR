import pybullet as p
import pybullet_data
import time

# Connect to PyBullet with GUI
physicsClient = p.connect(p.GUI)

# Optional: add built-in PyBullet data like URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and robot
planeId = p.loadURDF("plane.urdf")

start_pos = [0, 0, 0]
racecar_id = p.loadURDF("kuka_iiwa/model.urdf", start_pos)
# racecar_id = p.loadURDF("husky/husky.urdf", start_pos)

# Set gravity
p.setGravity(0, 0, -9.8)

# Run the simulation for a few seconds
for i in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect
p.disconnect()