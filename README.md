# UR5e Hand-E Cable Workcell (MuJoCo WASM)

Self-contained browser and native-MuJoCo simulation repo for a cable-manipulation workcell aligned to the Intrinsic AI for Industry challenge hardware stack.

## Online Demo

After publishing this repository and enabling Pages from `gh-pages`, the browser demo can be hosted at:

- `https://<github-user>.github.io/<repo-name>/`
- Example URL shape used in this workspace: `https://umar-azam.github.io/<repo-name>/`

## Scope

This repo reconstructs the published stack and policy I/O envelope:

- UR5e arm
- Robotiq Hand-E gripper
- Axia80-like wrist force/torque channel
- Three wrist-mounted Basler RGB cameras
- Cable/connector workcell with contact-rich manipulation dynamics

Reference challenge page:
- https://www.intrinsic.ai/events/ai-for-industry-challenge

Additional context reviewed in this workspace:
- `deep-research-report.md`
- `AGENT_README.md`
- `docs/AGENT_MCP_SETUP.md`
- `third_party/example_repos/mujoco_wasm` (branches + live demo)
- `third_party/example_repos/evobot`

## Browser Features

- MuJoCo physics scene running fully in browser via `mujoco-js` WebAssembly.
- Control GUI modeled after `mujoco_wasm` live demo style:
  - `Simulation` controls (`Pause`, reset, replay toggles)
  - `Actuators` sliders for UR5e + Hand-E command channels
- Paused manual joint override for UR5e joints.
- Wrist camera extrinsic tuning relative to end-effector/tool frame:
  - `x/y/z` translation
  - `roll/pitch/yaw`
  - `fov`
- Three simultaneous Basler RGB view panels rendered on-screen.
- Debug API exposed on `window.__UR5E_WORKCELL__` for validation/automation.

## Quickstart (Browser)

```bash
cd third_party/example_repos/ur5e_hande_cable_workcell_wasm
npm install
npm run sync:vendor
python -m http.server 8877
```

Open:
- http://127.0.0.1:8877/index.html

## Bootstrap As Standalone Repo

If this directory is copied without git history:

```bash
cd third_party/example_repos/ur5e_hande_cable_workcell_wasm
git init -b main
git add .
git commit -m "Initial commit: UR5e Hand-E cable workcell browser simulation"
```

## Validation and Demo Capture

Run UI + behavior validation:

```bash
python tools/validate_browser_ui.py --url http://127.0.0.1:8877/index.html
```

Record screenshots + demo video:

```bash
python tools/capture_browser_demo.py --url http://127.0.0.1:8877/index.html --duration-s 16
```

Generated artifacts:
- `artifacts/browser_validation/validation_report_browser.json`
- `artifacts/browser_validation/browser_demo_capture_report.json`
- `artifacts/browser_validation/ur5e_hande_browser_demo.webm`
- `artifacts/browser_validation/ur5e_hande_browser_demo.mp4` (if `ffmpeg` available)
- browser and panel screenshots in `artifacts/browser_validation/`

## Native MuJoCo GUI Workflow (Non-browser)

This repo also supports standard desktop MuJoCo iteration with native UI:

```bash
python tools/run_native_mujoco_gui.py
```

Manual-only mode:

```bash
python tools/run_native_mujoco_gui.py --manual
```

This is intended for rapid scene/control tuning with the usual MuJoCo desktop interaction model.

## GitHub Pages Branch

A `gh-pages` branch is expected for static hosting (same style as `evobot`).

Prepare/update branch locally:

```bash
bash tools/setup_gh_pages_branch.sh
```

Then push and enable Pages:

```bash
git push -u origin gh-pages
```

Expected Pages URL pattern:
- `https://<github-user>.github.io/<repo-name>/`

## Camera Panel Troubleshooting

- If Basler panels look blank/flat, click `Wrist Cameras (Tool Frame) -> Reset Camera Frames`.
- If panels are still not informative, move `x/y/z` and `yaw/pitch` slightly in the camera controls, then verify each panel shows fixture + cable geometry.
- `tools/validate_browser_ui.py` includes a non-flat image-content check for all three Basler panels.

## Repository Layout

- `index.html`: browser entrypoint
- `src/main.js`: simulation loop, GUI, policy replay, wrist camera controls, debug API
- `assets/scenes/`: self-contained MuJoCo scene + meshes
- `assets/trajectories/`: replay trajectory
- `tools/`: vendor sync, validation, demo capture, native GUI runner, gh-pages helper
- `docs/`: hardware stack details, policy I/O mapping, validation notes, reference review

## Notes on Fidelity

The model follows publicly stated hardware components and I/O channels. Exact toolkit-specific calibration constants and full official scoring protocol are not public in this workspace; placeholders/assumptions are documented in `docs/HARDWARE_AND_POLICY_SPEC.md`.
