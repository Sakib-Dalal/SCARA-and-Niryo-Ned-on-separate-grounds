from controller import Robot

# Create robot
robot = Robot()

# Get simulation timestep
timestep = int(robot.getBasicTimeStep())

print("Timestep:", timestep)

# Get SCARA joint 1
scara_joint_1 = robot.getDevice("base_arm_motor")
scara_joint_2 = robot.getDevice("arm_motor")
scara_joint_3 = robot.getDevice("shaft_linear_motor")

# Set movement speed
scara_joint_1.setVelocity(0.5)
scara_joint_2.setVelocity(0.5)
scara_joint_3.setVelocity(0.5)

# Move joint to 0.5 radians
scara_joint_1.setPosition(0.3)
scara_joint_2.setPosition(0.2)
scara_joint_3.setPosition(-0.2)

# Keep simulation running
while robot.step(timestep) != -1:
    pass
