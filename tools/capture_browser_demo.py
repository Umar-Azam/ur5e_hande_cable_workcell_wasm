#!/usr/bin/env python3
"""Capture browser demo video + screenshots and summarize physical behavior metrics."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import shutil
import subprocess
import sys
import time
from typing import Any

from playwright.sync_api import sync_playwright


def norm3(v: list[float]) -> float:
    if len(v) < 3:
        return 0.0
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def wait_until_ready(page, timeout_s: float = 60.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ready = page.evaluate(
            """() => {
              const api = typeof window.__UR5E_WORKCELL__ === 'object';
              const text = document.body ? document.body.innerText : '';
              return api && text.includes('UR5e Workcell Controls');
            }"""
        )
        if ready:
            return
        time.sleep(0.2)
    raise TimeoutError("Demo did not reach ready state within timeout")


def get_policy_io(page) -> dict[str, Any]:
    return page.evaluate("() => window.__UR5E_WORKCELL__.getPolicyIO()")


def maybe_convert_to_mp4(src_webm: pathlib.Path, dst_mp4: pathlib.Path) -> bool:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        return False
    cmd = [ffmpeg, "-y", "-i", str(src_webm), "-c:v", "libx264", "-pix_fmt", "yuv420p", str(dst_mp4)]
    run = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    return run.returncode == 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8877/index.html")
    parser.add_argument("--out-dir", default="artifacts/browser_validation")
    parser.add_argument("--duration-s", type=float, default=16.0)
    parser.add_argument("--headless", action="store_true", default=True)
    args = parser.parse_args()

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    video_tmp_dir = out_dir / "_video_tmp"
    video_tmp_dir.mkdir(parents=True, exist_ok=True)

    report: dict[str, Any] = {
        "url": args.url,
        "duration_s": args.duration_s,
        "screenshots": {},
        "metrics": {},
        "video": {},
        "actions": [],
    }

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=args.headless)
        context = browser.new_context(
            viewport={"width": 1600, "height": 900},
            record_video_dir=str(video_tmp_dir),
            record_video_size={"width": 1600, "height": 900},
        )
        page = context.new_page()

        page.goto(args.url, wait_until="domcontentloaded", timeout=120_000)
        wait_until_ready(page, timeout_s=90.0)
        time.sleep(1.0)

        path_init = out_dir / "demo_t00_ready.png"
        page.screenshot(path=str(path_init), full_page=True)
        report["screenshots"]["t00_ready"] = str(path_init)

        panel1 = out_dir / "demo_basler_1.png"
        panel2 = out_dir / "demo_basler_2.png"
        panel3 = out_dir / "demo_basler_3.png"
        page.locator("#basler-panels .basler-panel").nth(0).screenshot(path=str(panel1))
        page.locator("#basler-panels .basler-panel").nth(1).screenshot(path=str(panel2))
        page.locator("#basler-panels .basler-panel").nth(2).screenshot(path=str(panel3))
        report["screenshots"]["basler_wrist_cam_1"] = str(panel1)
        report["screenshots"]["basler_wrist_cam_2"] = str(panel2)
        report["screenshots"]["basler_wrist_cam_3"] = str(panel3)

        initial = get_policy_io(page)
        initial_connector = initial["geometry"]["cable_main_connector_pose_world"]["position_xyz"]

        page.evaluate("() => window.__UR5E_WORKCELL__.setPaused(true)")
        report["actions"].append("pause_simulation")
        time.sleep(0.4)

        page.evaluate("() => window.__UR5E_WORKCELL__.setJointTarget('wrist_3_joint', 0.25)")
        page.evaluate("() => window.__UR5E_WORKCELL__.setJointTarget('wrist_2_joint', -1.05)")
        page.evaluate("() => window.__UR5E_WORKCELL__.setJointTarget('elbow_joint', 1.65)")
        report["actions"].append("manual_joint_override_while_paused")

        page.evaluate(
            """() => {
                window.__UR5E_WORKCELL__.setCameraFrame('basler_wrist_cam_1', {
                    x:-0.028, y:0.118, z:-0.088, rollDeg:82.0, pitchDeg:27.0, yawDeg:101.0, fovDeg:74.0
                });
                window.__UR5E_WORKCELL__.setCameraFrame('basler_wrist_cam_2', {
                    x:-0.022, y:0.122, z:0.095, rollDeg:79.0, pitchDeg:24.0, yawDeg:-98.0, fovDeg:74.0
                });
                window.__UR5E_WORKCELL__.setCameraFrame('basler_wrist_cam_3', {
                    x:-0.016, y:0.118, z:-0.052, rollDeg:81.0, pitchDeg:26.0, yawDeg:88.0, fovDeg:70.0
                });
            }"""
        )
        report["actions"].append("manual_wrist_camera_frame_adjust")
        time.sleep(0.25)

        path_manual = out_dir / "demo_t01_paused_manual_adjust.png"
        page.screenshot(path=str(path_manual), full_page=True)
        report["screenshots"]["t01_paused_manual_adjust"] = str(path_manual)

        page.evaluate("() => window.__UR5E_WORKCELL__.setPaused(false)")
        page.evaluate("() => window.__UR5E_WORKCELL__.setReplayEnabled(true)")
        page.evaluate("() => window.__UR5E_WORKCELL__.setReplaySpeed(2.0)")
        report["actions"].append("resume_replay")

        sample_period = 0.2
        steps = max(1, int(args.duration_s / sample_period))
        advance_steps = max(1, int(sample_period / 0.002))
        connector_displacement = 0.0
        connector_lift = 0.0
        peak_force = 0.0

        t_mid = steps // 2
        for step in range(steps):
            page.evaluate(f"() => window.__UR5E_WORKCELL__.advanceSimulationSteps({advance_steps})")
            time.sleep(0.06)
            snap = get_policy_io(page)
            pos = snap["geometry"]["cable_main_connector_pose_world"]["position_xyz"]
            force = snap["inputs"]["force_torque"]["axia80_force_n"]

            disp = math.sqrt(
                (pos[0] - initial_connector[0]) ** 2
                + (pos[1] - initial_connector[1]) ** 2
                + (pos[2] - initial_connector[2]) ** 2
            )
            lift = max(0.0, pos[2] - initial_connector[2])
            connector_displacement = max(connector_displacement, disp)
            connector_lift = max(connector_lift, lift)
            peak_force = max(peak_force, norm3(force))

            if step == t_mid:
                mid_path = out_dir / "demo_t02_mid_replay.png"
                page.screenshot(path=str(mid_path), full_page=True)
                report["screenshots"]["t02_mid_replay"] = str(mid_path)

        end_path = out_dir / "demo_t03_end_replay.png"
        page.screenshot(path=str(end_path), full_page=True)
        report["screenshots"]["t03_end_replay"] = str(end_path)

        final_snap = get_policy_io(page)
        report["metrics"] = {
            "connector_displacement_m": connector_displacement,
            "connector_lift_m": connector_lift,
            "axia80_peak_force_n": peak_force,
            "final_sim_time_s": final_snap["sim_time_s"],
            "final_arm_command_rad": final_snap["outputs"]["arm_command_joint_target_rad"],
            "final_gripper_command": final_snap["outputs"]["gripper_command"],
        }

        video_obj = page.video
        context.close()
        raw_video = pathlib.Path(video_obj.path())
        browser.close()

    webm_path = out_dir / "ur5e_hande_browser_demo.webm"
    shutil.move(str(raw_video), str(webm_path))
    report["video"]["webm"] = str(webm_path)

    mp4_path = out_dir / "ur5e_hande_browser_demo.mp4"
    if maybe_convert_to_mp4(webm_path, mp4_path):
        report["video"]["mp4"] = str(mp4_path)

    motion_checks = {
        "connector_displacement_gt_0p02m": connector_displacement > 0.02,
        "connector_lift_gt_0p004m": connector_lift > 0.004,
        "axia80_force_gt_1n": peak_force > 1.0,
    }
    report["checks"] = motion_checks
    report["all_checks_passed"] = all(motion_checks.values())

    report_path = out_dir / "browser_demo_capture_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    # Cleanup temporary video directory after move.
    try:
        if video_tmp_dir.exists():
            for stale in video_tmp_dir.glob("*.webm"):
                stale.unlink(missing_ok=True)
        if video_tmp_dir.exists() and not any(video_tmp_dir.iterdir()):
            video_tmp_dir.rmdir()
    except OSError:
        pass

    print(json.dumps(report, indent=2))
    print(f"Demo capture report written to: {report_path}")

    return 0 if report["all_checks_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
