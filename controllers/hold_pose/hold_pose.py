"""Keep both floor-mounted robots at their initial joint positions.

The optional --exercise argument is used only by the mounting validation world.
No external Python packages or missing factory-demo objects are needed.
"""
import math
import sys

from controller import Motor, Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
motors = []
for index in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(index)
    if isinstance(device, Motor):
        device.setVelocity(min(0.35, device.getMaxVelocity()))
        device.setPosition(0.0)
        motors.append(device)

exercise = "--exercise" in sys.argv
while robot.step(timestep) != -1:
    if exercise:
        t = robot.getTime()
        for motor in motors:
            if motor.getName() in ("A motor", "base_arm_motor", "arm_motor"):
                motor.setPosition(0.2 * math.sin(t) if t < 8 else 0.0)
