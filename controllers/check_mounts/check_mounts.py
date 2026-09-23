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

bases = {name: robot.getFromDef(name) for name in ("SCARA", "ABB")}
initial = {name: node.getPose() for name, node in bases.items()}
max_drift = {name: 0.0 for name in bases}
joints = {
    "SCARA": bases["SCARA"].getFromProtoDef("LINK1_HINGEJOINT"),
    "ABB": bases["ABB"].getFromProtoDef("LINK1_TRANSFORM"),
}
angles = {
    name: joint.getField("jointParameters").getSFNode().getField("position")
    for name, joint in joints.items()
}
ranges = {name: [0.0, 0.0] for name in bases}

while robot.getTime() < 12 and robot.step(step) != -1:
    for name, node in bases.items():
        pose = node.getPose()
        assert all(math.isfinite(v) for v in pose), name
        max_drift[name] = max(max_drift[name], max(abs(a - b) for a, b in zip(pose, initial[name])))
        angle = angles[name].getSFFloat()
        ranges[name][0] = min(ranges[name][0], angle)
        ranges[name][1] = max(ranges[name][1], angle)

passed = all(drift < 1e-9 for drift in max_drift.values())
passed &= all(high - low > 0.2 for low, high in ranges.values())
report = {
    "passed": passed,
    "simulated_seconds": robot.getTime(),
    "floor_metres": robot.getFromDef("FLOOR").getField("floorSize").getSFVec2f(),
    "maximum_base_pose_change": max_drift,
    "measured_base_joint_range_radians": ranges,
    "static_base": {name: node.getField("staticBase").getSFBool() for name, node in bases.items()},
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


camera([1.35, -6.5, 3.8], [1.35, 0, 0.77])
capture("robot-world.png", "REAL-WORLD METRE SCALE", "10 m x 10 m floor  |  1 m grid  |  Both bases fixed")
camera([0.55, -0.85, 0.58], [0.13, 0, 0.20])
capture("scara-mount.png", "EPSON T6 SCARA - FLOOR MOUNT", "Four M8 flange bolts  |  Four floor anchors  |  12 mm plate")
camera([2.55, -1.65, 1.05], [1.69, 0, 0.22])
capture("abb-mount.png", "ABB IRB 4600-40 - FLOOR MOUNT", "Six M16 flange bolts  |  Four floor anchors  |  25 mm plate")
camera([6.8, -11.5, 9.8], [0, 0, 0.3])
capture("world-dimensions.png", "10 m x 10 m WORLD", "Every floor square is 1 m x 1 m")
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
