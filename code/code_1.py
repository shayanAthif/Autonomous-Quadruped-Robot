import pybullet as p
import pybullet_data
import time

# Connect to PyBullet and load the environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the ground plane
plane_id = p.loadURDF("plane.urdf")

# Load a simple car URDF model
car_id = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.2])

# Set gravity for the simulation
p.setGravity(0, 0, -9.8)

# Set the camera to view the car
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

# Simulation loop
for i in range(10000):
    # Apply forces to move the car forward by controlling the rear wheels
    p.setJointMotorControl2(car_id, 2, p.VELOCITY_CONTROL, targetVelocity=10)  # Rear left wheel
    p.setJointMotorControl2(car_id, 3, p.VELOCITY_CONTROL, targetVelocity=10)  # Rear right wheel
    
    # Step the simulation
    p.stepSimulation()

    # Add a small delay to slow down the simulation
    time.sleep(1./240.)
