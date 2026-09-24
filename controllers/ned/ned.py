from controller import Robot

# Create robot
robot = Robot()

# Get simulation timestep
timestep = int(robot.getBasicTimeStep())

print("Timestep:", timestep)

# Get Ned joint 1
ned_joint_1 = robot.getDevice("joint_1")
ned_joint_2 = robot.getDevice("joint_2")
ned_joint_3 = robot.getDevice("joint_3")

# Set movement speed
ned_joint_1.setVelocity(0.5)
ned_joint_2.setVelocity(0.5)
ned_joint_3.setVelocity(0.5)

# Move joint to 0.5 radians
ned_joint_1.setPosition(0.0)
ned_joint_2.setPosition(0.0)
ned_joint_3.setPosition(0.0)

# Keep simulation running
while robot.step(timestep) != -1:
    pass