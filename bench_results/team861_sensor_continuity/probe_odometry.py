"""Compare uninterrupted and interrupted public odometry sequences."""
import importlib.util
import math
from pathlib import Path
import sys

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[1]
sys.path.insert(0, str(REPO / "src"))


def estimator(path):
    spec = importlib.util.spec_from_file_location("pose_probe", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pose = module.PoseEstimator()
    pose.initialized = True
    return pose


def update(pose, odom):
    pose.update(None, None, odom, None, None, None, None)


def pair():
    base = OUT.parent / "team861_generic_recovery/confirmed_all_maps/runtime/src/swarm_rescue/solutions"
    return estimator(base / "localization.py"), estimator(OUT / "odometry_gap/localization.py")


old, new = pair()
for pose in (old, new):
    update(pose, (2.0, 0.0, 0.0))
    for _ in range(9):
        update(pose, None)
    update(pose, (30.0, 0.0, 0.5))
assert (old.x, old.y, old.heading) == (new.x, new.y, new.heading)
assert math.isclose(new.vx, 3.0) and math.isclose(new.omega, 0.05)
print(f"10 steps, 30 px: old vx={old.vx:.3f}, new vx={new.vx:.3f}, same final x={new.x:.1f}")
print(f"0.5 radians over the same interval: old omega={old.omega}, new omega={new.omega}")
old, new = pair()
for i in range(60):
    sample = (3.0 + 0.2 * math.sin(i), 0.2 * math.cos(i), 0.03)
    for pose in (old, new):
        update(pose, sample)
    assert (old.x, old.y, old.heading, old.vx, old.vy, old.omega) == (
        new.x, new.y, new.heading, new.vx, new.vy, new.omega)
print("60 consecutive valid samples: pose and velocity identical at every step.")
