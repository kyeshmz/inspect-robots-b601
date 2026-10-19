"""Own one official Seeed mock launch and run both measured joint benchmarks."""

import os
import signal
import subprocess
from pathlib import Path

from inspect_robots import Scene
from inspect_robots_b601 import B601Embodiment
from inspect_robots_b601.evaluate import main as evaluate


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "61" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Use isolated localhost-only ROS domain 61, with no existing mock")
    if os.environ.get("ROS_DISCOVERY_SERVER") or os.environ.get("ROS_STATIC_PEERS"):
        raise RuntimeError("Remote discovery overrides are forbidden")
    directory = Path("logs/official-mock")
    directory.mkdir(parents=True, exist_ok=True)
    robot = B601Embodiment(simulated=True, timeout_s=90, max_step=0.002, position_tolerance=0.0006)
    with (directory / "launch.log").open("w") as output:
        process = subprocess.Popen(
            ["ros2", "launch", "rebotarm_moveit_config", "demo.launch.py", "model:=rs", "use_rviz:=false"],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            robot.reset(Scene(id="mock-preflight", instruction="Verify official GenericSystem mock"))
            robot.close()
            for task in ("hold", "joint1-nudge"):
                if evaluate(["ros-mock", "--task", task, "--log-dir", str(directory / task)]):
                    raise RuntimeError(f"Official mock benchmark failed: {task}")
            print("OFFICIAL_MOCK: PASS; hold and joint1-nudge measured scores both passed")
        finally:
            try:
                robot.close()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=10)


if __name__ == "__main__":
    main()
