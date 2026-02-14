#!/usr/bin/env python3
"""Validate browser UI and runtime behavior for UR5e Hand-E cable workcell demo."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time
from dataclasses import dataclass
from typing import Any

from PIL import Image, ImageStat
from playwright.sync_api import sync_playwright


REQUIRED_GUI_LABELS = [
    "UR5e Workcell Controls",
    "Simulation",
    "Pause Simulation",
    "Actuators",
    "Joint Override (Paused)",
    "Wrist Cameras (Tool Frame)",
    "basler_wrist_cam_1",
    "basler_wrist_cam_2",
    "basler_wrist_cam_3",
]


@dataclass
class MotionMetrics:
    connector_displacement_m: float
    connector_lift_m: float
    axia80_peak_force_n: float
    sim_time_delta_s: float


def norm3(values: list[float]) -> float:
    if len(values) < 3:
        return 0.0
    return math.sqrt((values[0] ** 2) + (values[1] ** 2) + (values[2] ** 2))


def wait_for_ready(page, timeout_s: float = 60.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ready = page.evaluate(
            """() => {
                const apiReady = typeof window.__UR5E_WORKCELL__ === 'object';
                const text = document.body ? document.body.innerText : '';
                return apiReady && text.includes('UR5e Workcell Controls');
            }"""
        )
        if ready:
            return
        time.sleep(0.2)
    raise TimeoutError("Timed out waiting for browser demo to become ready")


def read_policy_io(page) -> dict[str, Any]:
    value = page.evaluate("() => window.__UR5E_WORKCELL__.getPolicyIO()")
    if not isinstance(value, dict):
        raise RuntimeError("Policy I/O snapshot not available")
    return value


def collect_motion_metrics(page, duration_s: float, sample_period_s: float) -> MotionMetrics:
    first = read_policy_io(page)
    first_pos = first["geometry"]["cable_main_connector_pose_world"]["position_xyz"]
    first_height = float(first_pos[2])
    first_time = float(first["sim_time_s"])

    peak_force = 0.0
    max_disp = 0.0
    max_lift = 0.0

    steps = max(1, int(duration_s / sample_period_s))
    # Advance simulation deterministically through debug API to avoid headless RAF throttling.
    advance_steps_per_sample = max(1, int(sample_period_s / 0.002))
    for _ in range(steps):
        page.evaluate(f"() => window.__UR5E_WORKCELL__.advanceSimulationSteps({advance_steps_per_sample})")
        snap = read_policy_io(page)
        pos = snap["geometry"]["cable_main_connector_pose_world"]["position_xyz"]
        force = snap["inputs"]["force_torque"]["axia80_force_n"]
        disp = math.sqrt(
            ((pos[0] - first_pos[0]) ** 2)
            + ((pos[1] - first_pos[1]) ** 2)
            + ((pos[2] - first_pos[2]) ** 2)
        )
        lift = max(0.0, float(pos[2]) - first_height)
        max_disp = max(max_disp, disp)
        max_lift = max(max_lift, lift)
        peak_force = max(peak_force, norm3(force))

    final = read_policy_io(page)
    final_time = float(final["sim_time_s"])

    return MotionMetrics(
        connector_displacement_m=max_disp,
        connector_lift_m=max_lift,
        axia80_peak_force_n=peak_force,
        sim_time_delta_s=max(0.0, final_time - first_time),
    )


def panel_texture_stddev(panel_image_path: pathlib.Path) -> float:
    image = Image.open(panel_image_path).convert("RGB")
    # Skip panel label/header region and score visible RGB feed only.
    image = image.crop((0, 24, image.width, image.height))
    stat = ImageStat.Stat(image)
    return float(sum(stat.stddev) / 3.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8877/index.html")
    parser.add_argument("--out-dir", default="artifacts/browser_validation")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--sample-period-s", type=float, default=0.2)
    args = parser.parse_args()

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    result: dict[str, Any] = {
        "url": args.url,
        "checks": {},
        "motion_metrics": {},
        "artifacts": {},
        "all_checks_passed": False,
    }

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=args.headless)
        context = browser.new_context(viewport={"width": 1600, "height": 900})
        page = context.new_page()

        console_errors: list[str] = []
        page_errors: list[str] = []

        page.on(
            "console",
            lambda msg: console_errors.append(msg.text)
            if msg.type == "error"
            else None,
        )
        page.on("pageerror", lambda err: page_errors.append(str(err)))

        page.goto(args.url, wait_until="domcontentloaded", timeout=120_000)
        wait_for_ready(page, timeout_s=90.0)
        time.sleep(1.5)

        overview_path = out_dir / "browser_overview.png"
        page.screenshot(path=str(overview_path), full_page=True)
        result["artifacts"]["overview_screenshot"] = str(overview_path)

        cam_paths: dict[str, str] = {}
        for idx, name in enumerate(["basler_wrist_cam_1", "basler_wrist_cam_2", "basler_wrist_cam_3"], start=1):
            panel = page.locator("#basler-panels .basler-panel").nth(idx - 1)
            cam_path = out_dir / f"basler_panel_{idx}_{name}.png"
            panel.screenshot(path=str(cam_path))
            cam_paths[name] = str(cam_path)
        result["artifacts"]["basler_panel_screenshots"] = cam_paths
        panel_texture = {
            name: panel_texture_stddev(pathlib.Path(path))
            for name, path in cam_paths.items()
        }

        ui_info = page.evaluate(
            """() => ({
                text: document.body ? document.body.innerText : '',
                allCanvasCount: document.querySelectorAll('canvas').length,
                baslerCanvasCount: document.querySelectorAll('.basler-canvas').length,
                hasApi: typeof window.__UR5E_WORKCELL__ === 'object'
            })"""
        )

        missing_labels = [label for label in REQUIRED_GUI_LABELS if label not in ui_info["text"]]

        result["checks"]["api_exposed"] = bool(ui_info["hasApi"])
        result["checks"]["three_basler_panels_rendered"] = int(ui_info["baslerCanvasCount"]) == 3
        result["checks"]["main_plus_basler_canvases"] = int(ui_info["allCanvasCount"]) >= 4
        result["checks"]["required_gui_labels_present"] = len(missing_labels) == 0
        result["checks"]["required_gui_labels_missing"] = missing_labels
        result["checks"]["basler_panel_texture_stddev"] = panel_texture
        result["checks"]["basler_panel_texture_nonflat"] = all(value > 5.0 for value in panel_texture.values())

        state_before = page.evaluate("() => window.__UR5E_WORKCELL__.getState()")
        page.evaluate("() => window.__UR5E_WORKCELL__.setPaused(true)")
        time.sleep(0.3)
        page.evaluate("() => window.__UR5E_WORKCELL__.setJointTarget('wrist_3_joint', 0.42)")
        time.sleep(0.3)
        state_after_joint = page.evaluate("() => window.__UR5E_WORKCELL__.getState()")
        result["checks"]["pause_toggle_works"] = bool(state_after_joint.get("paused"))
        result["checks"]["manual_joint_override_api_works"] = abs(
            float(state_after_joint["jointOverrideState"]["wrist_3_joint"]) - 0.42
        ) < 5e-3

        page.evaluate(
            """() => window.__UR5E_WORKCELL__.setCameraFrame('basler_wrist_cam_1', {
                x: -0.028,
                y: 0.118,
                z: -0.088,
                yawDeg: 101.0
            })"""
        )
        time.sleep(0.2)
        state_after_camera = page.evaluate("() => window.__UR5E_WORKCELL__.getState()")
        cam1 = state_after_camera["wristCameraFrames"]["basler_wrist_cam_1"]
        result["checks"]["camera_frame_adjust_api_works"] = (
            abs(float(cam1["x"]) - (-0.028)) < 5e-3 and abs(float(cam1["yawDeg"]) - 101.0) < 0.6
        )

        page.evaluate("() => window.__UR5E_WORKCELL__.setPaused(false)")
        page.evaluate("() => window.__UR5E_WORKCELL__.setReplayEnabled(true)")
        page.evaluate("() => window.__UR5E_WORKCELL__.setReplaySpeed(2.0)")
        metrics = collect_motion_metrics(page, args.duration_s, args.sample_period_s)

        result["motion_metrics"] = {
            "connector_displacement_m": metrics.connector_displacement_m,
            "connector_lift_m": metrics.connector_lift_m,
            "axia80_peak_force_n": metrics.axia80_peak_force_n,
            "sim_time_delta_s": metrics.sim_time_delta_s,
        }

        # Conservative thresholds to indicate physically active behavior in browser replay.
        result["checks"]["sim_time_advances"] = metrics.sim_time_delta_s > 1.0
        result["checks"]["connector_moves_in_workspace"] = metrics.connector_displacement_m > 0.02
        result["checks"]["connector_lifts_off_table"] = metrics.connector_lift_m > 0.004
        result["checks"]["nontrivial_axia80_force"] = metrics.axia80_peak_force_n > 1.0
        result["checks"]["no_runtime_console_errors"] = len(console_errors) == 0
        result["checks"]["no_page_exceptions"] = len(page_errors) == 0

        result["runtime"] = {
            "console_error_count": len(console_errors),
            "page_error_count": len(page_errors),
        }

        result["all_checks_passed"] = all(
            bool(v)
            for k, v in result["checks"].items()
            if k != "required_gui_labels_missing"
        )

        context.close()
        browser.close()

    report_path = out_dir / "validation_report_browser.json"
    report_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    result["artifacts"]["validation_report"] = str(report_path)

    print(json.dumps(result, indent=2))
    print(f"Validation report written to: {report_path}")

    return 0 if result["all_checks_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
