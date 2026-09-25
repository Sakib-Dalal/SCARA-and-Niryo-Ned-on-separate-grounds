"""Generate a validation world from the current scene, without changing it."""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
source = root / "worlds" / "scara_robot.wbt"
target = root / "worlds" / ".mount_check.wbt"
world = source.read_text()
for controller in ("scara", "ned"):
    world = world.replace(f'controller "{controller}"', 'controller "hold_pose"')
world = world.replace('controller "hold_pose"', 'controller "hold_pose"\n  controllerArgs [ "--exercise" ]')
world += '\nRobot {\n  name "mount validation"\n  supervisor TRUE\n  controller "check_mounts"\n}\n'
if "--quit" in sys.argv:
    world = world.replace('controller "check_mounts"', 'controller "check_mounts"\n  controllerArgs [ "--quit" ]')
target.write_text(world)
print(target)
