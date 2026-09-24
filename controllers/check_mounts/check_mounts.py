"""Exercise the robot joints, measure base drift, and export scene views."""
import json
import math
import sys
from pathlib import Path

from controller import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
output = Path(__file__).resolve().parents[2] / "docs"
output.mkdir(exist_ok=True)

bases = {name: robot.getFromDef(name) for name in ("SCARA", "NED")}
grounds = {name: robot.getFromDef(name + "_GROUND") for name in bases}
assert all(bases.values()) and all(grounds.values()), "Both robots and grounds must exist"
initial = {name: node.getPose() for name, node in bases.items()}
max_drift = {name: 0.0 for name in bases}
ranges = {name: [0.0, 0.0] for name in bases}
motor_counts = {}

while robot.getTime() < 12 and robot.step(step) != -1:
    for name, node in bases.items():
        pose = node.getPose()
        assert all(math.isfinite(v) for v in pose), name
        max_drift[name] = max(max_drift[name], max(abs(a - b) for a, b in zip(pose, initial[name])))
        # The stock SCARA exposes customData but does not bind the PROTO field.
        data = node.getBaseNodeField("customData").getSFString()
        if data:
            feedback = json.loads(data)
            angle = feedback["base_joint"]
            assert math.isfinite(angle), name
            ranges[name][0] = min(ranges[name][0], angle)
            ranges[name][1] = max(ranges[name][1], angle)
            motor_counts[name] = feedback["motor_count"]

passed = all(drift < 1e-9 for drift in max_drift.values())
passed &= all(high - low > 0.2 for low, high in ranges.values())
sizes = {name: node.getField("size").getSFFloat() for name, node in grounds.items()}
gap = (grounds["NED"].getPosition()[0] - sizes["NED"] / 2
       - grounds["SCARA"].getPosition()[0] - sizes["SCARA"] / 2)
fixed_bases = {
    "SCARA": bases["SCARA"].getField("staticBase").getSFBool(),
    "NED": bases["NED"].getBaseNodeField("physics").getSFNode() is None,
}
fixed_grounds = {
    name: node.getBaseNodeField("physics").getSFNode() is None
    and node.getBaseNodeField("boundingObject").getSFNode() is not None
    for name, node in grounds.items()
}
passed &= math.isclose(gap, 0.4, abs_tol=1e-9)
passed &= all(fixed_bases.values()) and all(fixed_grounds.values())
passed &= motor_counts == {"SCARA": 4, "NED": 8}
report = {
    "passed": passed,
    "simulated_seconds": robot.getTime(),
    "ground_size_metres": sizes,
    "ground_gap_metres": gap,
    "separate_static_collision_grounds": fixed_grounds,
    "maximum_base_pose_change": max_drift,
    "measured_base_joint_range_radians": ranges,
    "static_base": fixed_bases,
    "motor_count": motor_counts,
}
(output / "mount-check.json").write_text(json.dumps(report, indent=2) + "\n")
print("MOUNT CHECK " + ("PASS" if passed else "FAIL") + ": " + json.dumps(report), flush=True)

view = robot.getFromDef("OVERVIEW")
original_position = view.getField("position").getSFVec3f()
original_orientation = view.getField("orientation").getSFRotation()


def camera(position, target):
    delta = [b - a for a, b in zip(position, target)]
    yaw = math.atan2(delta[1], delta[0])
    pitch = math.atan2(-delta[2], math.hypot(delta[0], delta[1]))
    q = [-math.sin(pitch/2) * math.sin(yaw/2),
         math.sin(pitch/2) * math.cos(yaw/2),
         math.cos(pitch/2) * math.sin(yaw/2),
         math.cos(pitch/2) * math.cos(yaw/2)]
    length = math.sqrt(1 - q[3] ** 2)
    view.getField("position").setSFVec3f(position)
    view.getField("orientation").setSFRotation([v / length for v in q[:3]] + [2 * math.acos(q[3])])


def capture(filename, title, subtitle):
    robot.setLabel(0, title, 0.025, 0.025, 0.075, 0xFFFFFF)
    robot.setLabel(1, subtitle, 0.025, 0.095, 0.05, 0xFFFFFF)
    for _ in range(8):
        robot.step(step)
    robot.exportImage(str(output / filename), 100)


camera([1.05, -4.5, 3.05], [1.05, 0, 0.15])
capture("robot-world.png", "SCARA + NIRYO NED", "Native metre scale  |  Separate 1.6 m grounds  |  200 mm grid")
camera([0.55, -0.85, 0.58], [0.13, 0, 0.20])
capture("scara-mount.png", "EPSON T6 SCARA - FLOOR MOUNT", "Four M8 flange bolts  |  Four floor anchors  |  12 mm plate")
camera([2.6, -0.85, 0.65], [2, 0, 0.18])
capture("ned-mount.png", "NIRYO NED - FIXED BASE", "Native 6-axis model  |  Four floor anchors  |  12 mm plate")
camera([1, -1.8, 4.6], [1, 0, 0])
capture("world-dimensions.png", "TWO SEPARATE GROUNDS", "Each 1.6 m x 1.6 m  |  400 mm gap  |  Every grid square is 200 mm")
view.getField("position").setSFVec3f(original_position)
view.getField("orientation").setSFRotation(original_orientation)
robot.setLabel(0, "MOUNT CHECK PASSED" if passed else "MOUNT CHECK FAILED", 0.025, 0.025, 0.07, 0xFFFFFF)
robot.setLabel(1, "12 seconds of joint motion: both bases stayed fixed", 0.025, 0.095, 0.05, 0xFFFFFF)
robot.step(step)
if "--quit" in sys.argv:
    robot.simulationQuit(0 if passed else 1)
robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
while robot.step(step) != -1:
    pass
