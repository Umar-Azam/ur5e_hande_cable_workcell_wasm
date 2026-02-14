#!/usr/bin/env python3
"""Run the UR5e Hand-E cable workcell in native MuJoCo GUI (non-browser)."""

from __future__ import annotations

import argparse
import json
import pathlib
import time
from dataclasses import dataclass

import mujoco
import mujoco.viewer


ARM_ACTUATOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
]
GRIPPER_ACTUATOR_NAME = "hande_fingers_actuator"


@dataclass
class ReplayTrajectory:
    time_s: list[float]
    joint_target_rad: list[list[float]]
    gripper_command: list[float]
    duration_s: float


class ReplaySampler:
    def __init__(self, trajectory: ReplayTrajectory):
        self.trajectory = trajectory
        self.index = 0
        self.prev_time = 0.0

    def reset(self) -> None:
        self.index = 0
        self.prev_time = 0.0

    def sample(self, t: float, loop: bool = True) -> tuple[list[float], float]:
        traj = self.trajectory
        if loop and traj.duration_s > 0.0:
            t = t % traj.duration_s
            if t < self.prev_time:
                self.index = 0
        else:
            t = max(0.0, min(t, traj.duration_s))

        while self.index + 1 < len(traj.time_s) and traj.time_s[self.index + 1] <= t:
            self.index += 1

        self.prev_time = t
        return traj.joint_target_rad[self.index], traj.gripper_command[self.index]


def load_trajectory(path: pathlib.Path) -> ReplayTrajectory:
    payload = json.loads(path.read_text(encoding="utf-8"))
    ts = payload["time_s"]
    joints = payload["joint_target_rad"]
    gripper = payload["gripper_command"]
    if len(ts) < 2 or len(joints) != len(ts) or len(gripper) != len(ts):
        raise ValueError(f"Malformed replay trajectory at {path}")
    return ReplayTrajectory(ts, joints, gripper, ts[-1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        default="assets/scenes/ur5e_hande_cable_workcell.xml",
        help="Path to MuJoCo XML scene.",
    )
    parser.add_argument(
        "--trajectory",
        default="assets/trajectories/ur5e_hande_policy_replay.json",
        help="Replay trajectory JSON (same one used by browser demo).",
    )
    parser.add_argument("--manual", action="store_true", help="Disable trajectory replay and use current ctrl values only.")
    parser.add_argument("--replay-speed", type=float, default=1.0)
    parser.add_argument("--no-loop", action="store_true")
    args = parser.parse_args()

    scene_path = pathlib.Path(args.scene).resolve()
    traj_path = pathlib.Path(args.trajectory).resolve()

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    arm_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ARM_ACTUATOR_NAMES]
    gripper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_ACTUATOR_NAME)
    if any(i < 0 for i in arm_ids) or gripper_id < 0:
        raise RuntimeError("Failed to resolve UR5e/Hand-E actuator names in model")

    sampler = None
    if not args.manual:
        sampler = ReplaySampler(load_trajectory(traj_path))

    print("Native MuJoCo GUI controls:")
    print("- Space: pause/resume simulation")
    print("- Mouse: orbit/pan/zoom free camera")
    print("- Use native viewer side panels to inspect/override joints, actuators, sensors, and camera views")

    sim_time_acc = 0.0
    real_start = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        while viewer.is_running():
            step_wall_start = time.perf_counter()

            if sampler is not None:
                playback_time = sim_time_acc * args.replay_speed
                joint_cmd, gripper_cmd = sampler.sample(playback_time, loop=not args.no_loop)
                for i, actuator_id in enumerate(arm_ids):
                    data.ctrl[actuator_id] = float(joint_cmd[i])
                data.ctrl[gripper_id] = float(gripper_cmd)

            mujoco.mj_step(model, data)
            sim_time_acc += model.opt.timestep
            viewer.sync()

            step_elapsed = time.perf_counter() - step_wall_start
            sleep_s = model.opt.timestep - step_elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    elapsed = time.perf_counter() - real_start
    print(f"Exited native GUI. Wall time: {elapsed:.2f}s, simulated: {sim_time_acc:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
