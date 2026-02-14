# Hardware and Policy Spec Mapping

## Hardware Stack Reconstruction

This simulation maps published challenge hardware into MuJoCo components.

| Stack element | Simulation mapping |
|---|---|
| UR5e arm | 6-DoF UR5e kinematic/dynamic chain with joint-level actuators (`shoulder_pan`..`wrist_3`) |
| Robotiq Hand-E | Two-finger gripper with coupled tendon actuation via `hande_fingers_actuator` (`0..255`) |
| Axia80 wrist F/T | `axia80_site` + force/torque sensors (`axia80/force`, `axia80/torque`) |
| Basler wrist cameras | Three tool-mounted RGB cameras: `basler_wrist_cam_1/2/3` |
| Cable task geometry | Two flexible cable chains with connector bodies + fixture workcell |

Primary scene file:
- `assets/scenes/ur5e_hande_cable_workcell.xml`

## Policy Inputs

Policy input channels exposed in-scene and through browser debug API (`getPolicyIO()`):

- Joint position [rad]
  - `ur5e/joint1_pos` ... `ur5e/joint6_pos`
- Joint velocity [rad/s]
  - `ur5e/joint1_vel` ... `ur5e/joint6_vel`
- Joint actuator torque estimate [Nm]
  - `ur5e/joint1_actuator_torque` ... `ur5e/joint6_actuator_torque`
- Wrist force/torque [N, Nm]
  - `ur5e/wrist_force`, `ur5e/wrist_torque`
- Axia80-like force/torque [N, Nm]
  - `axia80/force`, `axia80/torque`
- Camera streams (RGB)
  - `basler_wrist_cam_1`, `basler_wrist_cam_2`, `basler_wrist_cam_3`

## Policy Outputs

Command channels represented in simulation and replay:

- Arm command: 6 joint targets [rad]
  - actuators `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`
- Gripper command:
  - `hande_fingers_actuator` in `[0,255]`

Replay source:
- `assets/trajectories/ur5e_hande_policy_replay.json`

## Camera Frame Calibration Controls

Browser GUI supports camera-frame adjustment relative to tool frame:

- Translation: `x`, `y`, `z` [m]
- Rotation: `rollDeg`, `pitchDeg`, `yawDeg`
- Optics: `fovDeg`

This is intended to support calibration sensitivity studies and synthetic camera-domain randomization workflows.

## Assumptions and Known Gaps

- Exact official Basler intrinsics/extrinsics from the private toolkit are not available in this workspace.
- Scene and contacts are tuned to be physically coherent and policy-training useful, but not guaranteed numerically identical to private qualification environments.
- Official scoring thresholds and hidden evaluation distributions are not encoded here.
