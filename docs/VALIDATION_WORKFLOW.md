# Validation Workflow

## 1) Start Browser Demo

```bash
npm install
npm run sync:vendor
python -m http.server 8877
```

## 2) Validate GUI + Runtime Requirements

```bash
python tools/validate_browser_ui.py --url http://127.0.0.1:8877/index.html
```

Checks include:
- MuJoCo scene loads with no page exceptions
- GUI control groups present
- pause/manual joint API works
- wrist camera frame adjustment API works
- 3 Basler panel canvases rendered
- connector movement/lift over time
- non-trivial Axia80 force magnitude

Output:
- `artifacts/browser_validation/validation_report_browser.json`
- screenshots of overview and per-camera panels

## 3) Record Browser Demonstration Video

```bash
python tools/capture_browser_demo.py --url http://127.0.0.1:8877/index.html --duration-s 16
```

Output:
- `artifacts/browser_validation/ur5e_hande_browser_demo.webm`
- `artifacts/browser_validation/ur5e_hande_browser_demo.mp4` (if ffmpeg installed)
- `artifacts/browser_validation/browser_demo_capture_report.json`
- staged screenshots from paused/manual and replay phases

## 4) Native Desktop Verification (Optional)

```bash
python tools/run_native_mujoco_gui.py
```

Use this for direct MuJoCo desktop debugging and scene iteration outside browser runtime.
