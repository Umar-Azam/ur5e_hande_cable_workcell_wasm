import * as THREE from "three";
import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import loadMujoco from "../vendor/mujoco-js/mujoco_wasm.js";
import { loadSceneFromURL, drawTendonsAndFlex, getPosition, getQuaternion } from "./mujocoUtils.js";
import { mountSceneAssetsToWorkingFs } from "./sim/assets.js";
import { loadReplayTrajectory, ReplaySampler } from "./sim/trajectory.js";
import { createAppLayout } from "./ui/layout.js";

const WORKING_SCENE_PATH = "assets/scenes/ur5e_hande_cable_workcell.xml";
const SCENE_MANIFEST_URL = "./assets/scenes/manifest.json";
const REPLAY_TRAJECTORY_URL = "./assets/trajectories/ur5e_hande_policy_replay.json";

const ARM_JOINT_NAMES = [
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint",
];

const ARM_ACTUATOR_NAMES = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow",
  "wrist_1",
  "wrist_2",
  "wrist_3",
];

const BASLER_CAMERA_NAMES = [
  "basler_wrist_cam_1",
  "basler_wrist_cam_2",
  "basler_wrist_cam_3",
];

// Browser-calibrated wrist-camera defaults that keep cable/connector geometry
// visible for immediate policy-debugging and manual frame tuning.
const PREFERRED_WRIST_CAMERA_BASELINE = {
  basler_wrist_cam_1: {
    x: -0.02,
    y: 0.12,
    z: -0.10,
    rollDeg: 80.0,
    pitchDeg: 25.0,
    yawDeg: 95.0,
    fovDeg: 72.0,
  },
  basler_wrist_cam_2: {
    x: -0.02,
    y: 0.12,
    z: 0.10,
    rollDeg: 80.0,
    pitchDeg: 25.0,
    yawDeg: -95.0,
    fovDeg: 72.0,
  },
  basler_wrist_cam_3: {
    x: -0.02,
    y: 0.12,
    z: -0.06,
    rollDeg: 80.0,
    pitchDeg: 25.0,
    yawDeg: 84.0,
    fovDeg: 70.0,
  },
};

const UR5E_JOINT_POS_SENSOR_NAMES = [
  "ur5e/joint1_pos",
  "ur5e/joint2_pos",
  "ur5e/joint3_pos",
  "ur5e/joint4_pos",
  "ur5e/joint5_pos",
  "ur5e/joint6_pos",
];

const UR5E_JOINT_VEL_SENSOR_NAMES = [
  "ur5e/joint1_vel",
  "ur5e/joint2_vel",
  "ur5e/joint3_vel",
  "ur5e/joint4_vel",
  "ur5e/joint5_vel",
  "ur5e/joint6_vel",
];

const UR5E_ACTUATOR_TORQUE_SENSOR_NAMES = [
  "ur5e/joint1_actuator_torque",
  "ur5e/joint2_actuator_torque",
  "ur5e/joint3_actuator_torque",
  "ur5e/joint4_actuator_torque",
  "ur5e/joint5_actuator_torque",
  "ur5e/joint6_actuator_torque",
];

function swizzleMujocoPos(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

function swizzleMujocoQuat(qw, qx, qy, qz) {
  return new THREE.Quaternion(-qx, -qz, qy, -qw);
}

function clamp(value, minValue, maxValue) {
  return Math.max(minValue, Math.min(maxValue, value));
}

function deepCopy(obj) {
  return JSON.parse(JSON.stringify(obj));
}

class UR5eCableWorkcellBrowserDemo {
  constructor() {
    this.layout = createAppLayout(BASLER_CAMERA_NAMES);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0.11, 0.16, 0.22);
    this.scene.fog = new THREE.Fog(this.scene.background, 10.0, 28.0);

    // Add fill lighting so workcell geometry remains readable in-browser even when
    // scene-authored lights are highly directional.
    const hemiFill = new THREE.HemisphereLight(0xa9c7e4, 0x1f2f3f, 0.95);
    this.scene.add(hemiFill);
    const keyFill = new THREE.DirectionalLight(0xffffff, 0.55);
    keyFill.position.set(1.8, -1.2, 2.4);
    this.scene.add(keyFill);

    this.camera = new THREE.PerspectiveCamera(44, window.innerWidth / window.innerHeight, 0.001, 100.0);
    this.camera.position.set(1.25, 0.72, 0.92);
    this.scene.add(this.camera);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1.0, 1.5));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.useLegacyLights = true;
    this.layout.mainViewport.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.54, 0.04, 0.0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.panSpeed = 1.2;
    this.controls.zoomSpeed = 1.0;
    this.controls.update();

    this.disableReflector = true;
    this.enableShadows = true;

    this.tmpVec = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.tmpWorldPos = new THREE.Vector3();
    this.tmpWorldQuat = new THREE.Quaternion();

    this.wristCameraObjects = new Map();
    this.wristCameraRenderers = new Map();
    for (const name of BASLER_CAMERA_NAMES) {
      const wristCam = new THREE.PerspectiveCamera(58, 320 / 240, 0.01, 8.0);
      wristCam.name = name;
      this.scene.add(wristCam);
      this.wristCameraObjects.set(name, wristCam);

      const panelInfo = this.layout.cameraPanels.get(name);
      const panelRenderer = new THREE.WebGLRenderer({ antialias: false, alpha: false, powerPreference: "high-performance" });
      panelRenderer.setPixelRatio(1.0);
      panelRenderer.setSize(320, 240);
      panelRenderer.domElement.className = "basler-canvas";
      panelInfo.canvasHost.appendChild(panelRenderer.domElement);
      this.wristCameraRenderers.set(name, panelRenderer);
    }

    this.params = {
      paused: false,
      replayEnabled: true,
      replayLoop: true,
      replaySpeed: 1.0,
      manualActuatorOverride: false,
      applyPausedJointOverride: true,
      showBaslerPanels: true,
    };

    this.mujoco = null;
    this.model = null;
    this.data = null;
    this.mujocoRoot = null;
    this.bodies = {};
    this.lights = {};

    this.armActuatorIds = [];
    this.armJointIds = [];
    this.armJointQposAdr = [];
    this.gripperActuatorId = -1;
    this.tool0BodyId = -1;
    this.mainConnectorBodyId = -1;
    this.auxConnectorBodyId = -1;

    this.sensorSlices = {
      jointPos: [],
      jointVel: [],
      actuatorTorque: [],
      axia80Force: null,
      axia80Torque: null,
      wristForce: null,
      wristTorque: null,
    };

    this.actuatorGuiState = {};
    this.actuatorControllers = [];
    this.jointOverrideState = {};
    this.jointOverrideControllers = [];
    this.wristCameraFrames = {};
    this.defaultWristCameraFrames = {};

    this.trajectory = null;
    this.replaySampler = null;

    this.accumulatorSec = 0.0;
    this.simTimeSec = 0.0;
    this.lastFrameWallMs = null;

    this.gui = null;
    this.pauseController = null;

    window.addEventListener("resize", () => this.onResize());
    this.renderer.setAnimationLoop((timeMs) => this.render(timeMs));

    this.installDebugApi();
  }

  setStatus(text) {
    this.layout.statusOverlay.textContent = text;
  }

  getPolicyIoSnapshot() {
    if (!this.model || !this.data) {
      return null;
    }
    return {
      sim_time_s: this.simTimeSec,
      paused: this.params.paused,
      replay_policy_enabled: this.params.replayEnabled,
      inputs: {
        joint_state: {
          position_rad: this.sensorSlices.jointPos.map((slice) => this.readSensorSliceAsArray(slice)[0] || 0.0),
          velocity_rad_s: this.sensorSlices.jointVel.map((slice) => this.readSensorSliceAsArray(slice)[0] || 0.0),
          actuator_torque_nm: this.sensorSlices.actuatorTorque.map((slice) => this.readSensorSliceAsArray(slice)[0] || 0.0),
        },
        force_torque: {
          axia80_force_n: this.readSensorSliceAsArray(this.sensorSlices.axia80Force),
          axia80_torque_nm: this.readSensorSliceAsArray(this.sensorSlices.axia80Torque),
          wrist_force_n: this.readSensorSliceAsArray(this.sensorSlices.wristForce),
          wrist_torque_nm: this.readSensorSliceAsArray(this.sensorSlices.wristTorque),
        },
      },
      outputs: {
        arm_command_joint_target_rad: this.armActuatorIds.map((actId) => this.data.ctrl[actId]),
        gripper_command: this.data.ctrl[this.gripperActuatorId],
      },
      geometry: {
        tool0_pose_world: this.readBodyPoseById(this.tool0BodyId),
        cable_main_connector_pose_world: this.readBodyPoseById(this.mainConnectorBodyId),
        cable_aux_connector_pose_world: this.readBodyPoseById(this.auxConnectorBodyId),
      },
    };
  }

  installDebugApi() {
    const api = {
      getState: () => ({
        paused: this.params.paused,
        replayEnabled: this.params.replayEnabled,
        simTimeSec: this.simTimeSec,
        jointOverrideState: { ...this.jointOverrideState },
        wristCameraFrames: deepCopy(this.wristCameraFrames),
        tool0BodyId: this.tool0BodyId,
      }),
      setPaused: (value) => {
        this.params.paused = Boolean(value);
        if (this.pauseController) {
          this.pauseController.updateDisplay();
        }
        return this.params.paused;
      },
      setJointTarget: (jointName, value) => {
        if (!(jointName in this.jointOverrideState)) {
          throw new Error(`Unknown joint target: ${jointName}`);
        }
        this.jointOverrideState[jointName] = Number(value);
        return this.jointOverrideState[jointName];
      },
      setReplayEnabled: (value) => {
        this.params.replayEnabled = Boolean(value);
        return this.params.replayEnabled;
      },
      setReplaySpeed: (value) => {
        const numeric = Number(value);
        this.params.replaySpeed = clamp(Number.isFinite(numeric) ? numeric : 1.0, 0.25, 4.0);
        return this.params.replaySpeed;
      },
      setManualActuatorOverride: (value) => {
        this.params.manualActuatorOverride = Boolean(value);
        return this.params.manualActuatorOverride;
      },
      setCameraFrame: (cameraName, patch) => {
        if (!(cameraName in this.wristCameraFrames)) {
          throw new Error(`Unknown wrist camera: ${cameraName}`);
        }
        Object.assign(this.wristCameraFrames[cameraName], patch || {});
        return deepCopy(this.wristCameraFrames[cameraName]);
      },
      getPolicyIO: () => this.getPolicyIoSnapshot(),
      advanceSimulationSteps: (stepCount) => {
        this.advanceSimulationSteps(stepCount);
        return this.simTimeSec;
      },
      advanceSimulationSeconds: (durationSec) => {
        const timestep = this.model?.opt?.timestep || 0.002;
        const steps = Math.max(0, Math.floor(Number(durationSec || 0) / timestep));
        this.advanceSimulationSteps(steps);
        return this.simTimeSec;
      },
      resetSimulation: () => {
        this.resetSimulation();
      },
    };
    window.__UR5E_WORKCELL__ = api;
  }

  async init() {
    this.setStatus("Loading MuJoCo WASM runtime...");
    this.mujoco = await loadMujoco();

    this.setStatus("Loading UR5e workcell scene assets into /working...");
    await mountSceneAssetsToWorkingFs(this.mujoco, {
      manifestUrl: SCENE_MANIFEST_URL,
      assetsRootUrl: "./assets/scenes",
      workingRoot: "/working/assets/scenes",
    });

    this.setStatus("Building MuJoCo model and Three.js scene...");
    [this.model, this.data, this.bodies, this.lights] = await loadSceneFromURL(
      this.mujoco,
      WORKING_SCENE_PATH,
      this
    );
    this.mujoco.mj_forward(this.model, this.data);

    this.resolveModelHandles();
    this.cacheCameraDefaultsFromModel();

    this.setStatus("Loading replay trajectory...");
    this.trajectory = await loadReplayTrajectory(REPLAY_TRAJECTORY_URL);
    this.replaySampler = new ReplaySampler(this.trajectory);

    this.syncActuatorStateFromModel();
    this.syncJointOverrideFromData();
    this.buildGui();
    this.setBaslerVisibility(this.params.showBaslerPanels);

    this.setStatus("Ready\nSpace: pause/play\nUse GUI for manual joints + wrist-camera frame tuning.");
    this.installKeyboardShortcuts();
  }

  resolveSensorSliceByName(sensorName) {
    const sensorType = this.mujoco.mjtObj.mjOBJ_SENSOR.value;
    const sensorId = this.mujoco.mj_name2id(this.model, sensorType, sensorName);
    if (sensorId < 0) {
      return null;
    }
    return {
      id: sensorId,
      adr: this.model.sensor_adr[sensorId],
      dim: this.model.sensor_dim[sensorId],
    };
  }

  readSensorSliceAsArray(sliceInfo) {
    if (!sliceInfo) {
      return [];
    }
    const { adr, dim } = sliceInfo;
    return Array.from(this.data.sensordata.slice(adr, adr + dim));
  }

  readBodyPoseById(bodyId) {
    if (bodyId < 0) {
      return null;
    }
    const basePos = bodyId * 3;
    const baseQuat = bodyId * 4;
    return {
      position_xyz: [
        this.data.xpos[basePos + 0],
        this.data.xpos[basePos + 1],
        this.data.xpos[basePos + 2],
      ],
      quaternion_wxyz: [
        this.data.xquat[baseQuat + 0],
        this.data.xquat[baseQuat + 1],
        this.data.xquat[baseQuat + 2],
        this.data.xquat[baseQuat + 3],
      ],
    };
  }

  resolveModelHandles() {
    const obj = this.mujoco.mjtObj;
    const bodyType = obj.mjOBJ_BODY.value;
    const actuatorType = obj.mjOBJ_ACTUATOR.value;
    const jointType = obj.mjOBJ_JOINT.value;

    this.tool0BodyId = this.mujoco.mj_name2id(this.model, bodyType, "tool0_link");
    this.armActuatorIds = ARM_ACTUATOR_NAMES.map((name) => this.mujoco.mj_name2id(this.model, actuatorType, name));
    this.gripperActuatorId = this.mujoco.mj_name2id(this.model, actuatorType, "hande_fingers_actuator");
    this.armJointIds = ARM_JOINT_NAMES.map((name) => this.mujoco.mj_name2id(this.model, jointType, name));
    this.armJointQposAdr = this.armJointIds.map((jointId) => this.model.jnt_qposadr[jointId]);
    this.mainConnectorBodyId = this.mujoco.mj_name2id(this.model, bodyType, "cable_main_connector");
    this.auxConnectorBodyId = this.mujoco.mj_name2id(this.model, bodyType, "cable_aux_connector");

    this.sensorSlices.jointPos = UR5E_JOINT_POS_SENSOR_NAMES.map((name) => this.resolveSensorSliceByName(name));
    this.sensorSlices.jointVel = UR5E_JOINT_VEL_SENSOR_NAMES.map((name) => this.resolveSensorSliceByName(name));
    this.sensorSlices.actuatorTorque = UR5E_ACTUATOR_TORQUE_SENSOR_NAMES.map((name) => this.resolveSensorSliceByName(name));
    this.sensorSlices.axia80Force = this.resolveSensorSliceByName("axia80/force");
    this.sensorSlices.axia80Torque = this.resolveSensorSliceByName("axia80/torque");
    this.sensorSlices.wristForce = this.resolveSensorSliceByName("ur5e/wrist_force");
    this.sensorSlices.wristTorque = this.resolveSensorSliceByName("ur5e/wrist_torque");

    if (this.tool0BodyId < 0 || this.gripperActuatorId < 0 || this.armActuatorIds.some((x) => x < 0)) {
      throw new Error("Failed to resolve required model handles for UR5e workcell controls.");
    }

    if (
      this.sensorSlices.jointPos.some((v) => !v) ||
      this.sensorSlices.jointVel.some((v) => !v) ||
      this.sensorSlices.actuatorTorque.some((v) => !v) ||
      !this.sensorSlices.axia80Force ||
      !this.sensorSlices.axia80Torque
    ) {
      throw new Error("Failed to resolve one or more required policy I/O sensors from the MuJoCo model.");
    }
  }

  cacheCameraDefaultsFromModel() {
    const cameraType = this.mujoco.mjtObj.mjOBJ_CAMERA.value;
    for (const name of BASLER_CAMERA_NAMES) {
      const cameraId = this.mujoco.mj_name2id(this.model, cameraType, name);
      if (cameraId < 0) {
        throw new Error(`Camera not found in model: ${name}`);
      }
      const pos = swizzleMujocoPos(
        this.model.cam_pos[(cameraId * 3) + 0],
        this.model.cam_pos[(cameraId * 3) + 1],
        this.model.cam_pos[(cameraId * 3) + 2]
      );
      const quat = swizzleMujocoQuat(
        this.model.cam_quat[(cameraId * 4) + 0],
        this.model.cam_quat[(cameraId * 4) + 1],
        this.model.cam_quat[(cameraId * 4) + 2],
        this.model.cam_quat[(cameraId * 4) + 3]
      );
      const euler = new THREE.Euler().setFromQuaternion(quat, "XYZ");
      this.wristCameraFrames[name] = {
        x: pos.x,
        y: pos.y,
        z: pos.z,
        rollDeg: THREE.MathUtils.radToDeg(euler.x),
        pitchDeg: THREE.MathUtils.radToDeg(euler.y),
        yawDeg: THREE.MathUtils.radToDeg(euler.z),
        fovDeg: this.model.cam_fovy[cameraId],
      };
    }

    for (const name of BASLER_CAMERA_NAMES) {
      const preferred = PREFERRED_WRIST_CAMERA_BASELINE[name];
      if (!preferred) {
        continue;
      }
      Object.assign(this.wristCameraFrames[name], preferred);
    }

    this.defaultWristCameraFrames = deepCopy(this.wristCameraFrames);
  }

  syncActuatorStateFromModel() {
    for (let i = 0; i < ARM_ACTUATOR_NAMES.length; i += 1) {
      const name = ARM_ACTUATOR_NAMES[i];
      const actuatorId = this.armActuatorIds[i];
      this.actuatorGuiState[name] = this.data.ctrl[actuatorId] || 0.0;
    }
    this.actuatorGuiState.hande_fingers_actuator = this.data.ctrl[this.gripperActuatorId] || 0.0;
  }

  syncJointOverrideFromData() {
    for (let i = 0; i < ARM_JOINT_NAMES.length; i += 1) {
      const jointName = ARM_JOINT_NAMES[i];
      const qposAdr = this.armJointQposAdr[i];
      this.jointOverrideState[jointName] = this.data.qpos[qposAdr];
    }
    this.jointOverrideState.hande_fingers_actuator = this.data.ctrl[this.gripperActuatorId] || 0.0;
  }

  buildGui() {
    this.gui = new GUI({ title: "UR5e Workcell Controls" });

    const simulationFolder = this.gui.addFolder("Simulation");
    this.pauseController = simulationFolder.add(this.params, "paused").name("Pause Simulation");
    simulationFolder.add(this.params, "replayEnabled").name("Replay Policy Trajectory");
    simulationFolder.add(this.params, "replayLoop").name("Replay Loop");
    simulationFolder.add(this.params, "replaySpeed", 0.25, 4.0, 0.01).name("Replay Speed");
    simulationFolder.add(this.params, "showBaslerPanels").name("Show Basler RGB Panels").onChange((value) => {
      this.setBaslerVisibility(Boolean(value));
    });
    simulationFolder.add({ reset: () => this.resetSimulation() }, "reset").name("Reset Simulation State");

    const actuatorFolder = simulationFolder.addFolder("Actuators");
    actuatorFolder.add(this.params, "manualActuatorOverride").name("Enable Manual Actuators");
    for (let i = 0; i < ARM_ACTUATOR_NAMES.length; i += 1) {
      const name = ARM_ACTUATOR_NAMES[i];
      const actuatorId = this.armActuatorIds[i];
      const min = this.model.actuator_ctrlrange[(actuatorId * 2) + 0];
      const max = this.model.actuator_ctrlrange[(actuatorId * 2) + 1];
      const controller = actuatorFolder
        .add(this.actuatorGuiState, name, min, max, 0.001)
        .name(name)
        .listen();
      controller.onChange((value) => {
        this.data.ctrl[actuatorId] = value;
      });
      this.actuatorControllers.push({ name, actuatorId, controller });
    }

    {
      const actuatorId = this.gripperActuatorId;
      const min = this.model.actuator_ctrlrange[(actuatorId * 2) + 0];
      const max = this.model.actuator_ctrlrange[(actuatorId * 2) + 1];
      const controller = actuatorFolder
        .add(this.actuatorGuiState, "hande_fingers_actuator", min, max, 1.0)
        .name("hande_fingers_actuator")
        .listen();
      controller.onChange((value) => {
        this.data.ctrl[actuatorId] = value;
      });
      this.actuatorControllers.push({ name: "hande_fingers_actuator", actuatorId, controller });
    }

    const jointFolder = this.gui.addFolder("Joint Override (Paused)");
    jointFolder.add(this.params, "applyPausedJointOverride").name("Apply While Paused");
    jointFolder.add({ sync: () => this.syncJointOverrideFromData() }, "sync").name("Sync From Current State");

    for (let i = 0; i < ARM_JOINT_NAMES.length; i += 1) {
      const jointName = ARM_JOINT_NAMES[i];
      const jointId = this.armJointIds[i];
      const min = this.model.jnt_range[(jointId * 2) + 0];
      const max = this.model.jnt_range[(jointId * 2) + 1];
      const controller = jointFolder
        .add(this.jointOverrideState, jointName, min, max, 0.001)
        .name(jointName)
        .listen();
      controller.onChange(() => {
        if (this.params.paused && this.params.applyPausedJointOverride) {
          this.applyPausedJointOverrides();
        }
      });
      this.jointOverrideControllers.push(controller);
    }

    jointFolder
      .add(this.jointOverrideState, "hande_fingers_actuator", 0.0, 255.0, 1.0)
      .name("hande_fingers_actuator")
      .listen()
      .onChange(() => {
        if (this.params.paused && this.params.applyPausedJointOverride) {
          this.applyPausedJointOverrides();
        }
      });

    const cameraFolder = this.gui.addFolder("Wrist Cameras (Tool Frame)");
    cameraFolder.add({ reset: () => this.resetWristCameraFrames() }, "reset").name("Reset Camera Frames");

    for (const name of BASLER_CAMERA_NAMES) {
      const frame = this.wristCameraFrames[name];
      const folder = cameraFolder.addFolder(name);
      folder.add(frame, "x", -0.20, 0.20, 0.001).name("x [m]");
      folder.add(frame, "y", -0.20, 0.20, 0.001).name("y [m]");
      folder.add(frame, "z", -0.20, 0.20, 0.001).name("z [m]");
      folder.add(frame, "rollDeg", -180.0, 180.0, 0.5).name("roll [deg]");
      folder.add(frame, "pitchDeg", -180.0, 180.0, 0.5).name("pitch [deg]");
      folder.add(frame, "yawDeg", -180.0, 180.0, 0.5).name("yaw [deg]");
      folder.add(frame, "fovDeg", 20.0, 110.0, 0.1).name("fov [deg]");
      folder.close();
    }

    simulationFolder.open();
    cameraFolder.open();
  }

  installKeyboardShortcuts() {
    document.addEventListener("keydown", (event) => {
      if (event.code === "Space") {
        this.params.paused = !this.params.paused;
        if (this.pauseController) {
          this.pauseController.updateDisplay();
        }
        event.preventDefault();
      }
      if (event.code === "KeyR") {
        this.resetSimulation();
      }
    });
  }

  resetWristCameraFrames() {
    this.wristCameraFrames = deepCopy(this.defaultWristCameraFrames);
  }

  setBaslerVisibility(visible) {
    this.layout.baslerPanels.style.display = visible ? "grid" : "none";
  }

  applyReplayControlsForStep() {
    if (!this.params.replayEnabled || !this.replaySampler) {
      return;
    }

    const sampled = this.replaySampler.sample(this.simTimeSec * this.params.replaySpeed, this.params.replayLoop);
    if (!sampled) {
      return;
    }

    for (let i = 0; i < this.armActuatorIds.length; i += 1) {
      this.data.ctrl[this.armActuatorIds[i]] = sampled.jointTargets[i];
      this.actuatorGuiState[ARM_ACTUATOR_NAMES[i]] = sampled.jointTargets[i];
    }
    this.data.ctrl[this.gripperActuatorId] = sampled.gripperCommand;
    this.actuatorGuiState.hande_fingers_actuator = sampled.gripperCommand;
  }

  applyManualActuatorControls() {
    for (const { name, actuatorId } of this.actuatorControllers) {
      this.data.ctrl[actuatorId] = this.actuatorGuiState[name];
    }
  }

  applyPausedJointOverrides() {
    for (let i = 0; i < ARM_JOINT_NAMES.length; i += 1) {
      const jointName = ARM_JOINT_NAMES[i];
      const qposAdr = this.armJointQposAdr[i];
      const actuatorId = this.armActuatorIds[i];
      const value = this.jointOverrideState[jointName];
      this.data.qpos[qposAdr] = value;
      this.data.ctrl[actuatorId] = value;
      this.actuatorGuiState[ARM_ACTUATOR_NAMES[i]] = value;
    }

    const gripperValue = clamp(this.jointOverrideState.hande_fingers_actuator, 0.0, 255.0);
    this.data.ctrl[this.gripperActuatorId] = gripperValue;
    this.actuatorGuiState.hande_fingers_actuator = gripperValue;
    this.mujoco.mj_forward(this.model, this.data);
  }

  updateSceneTransformsFromData() {
    for (let bodyId = 0; bodyId < this.model.nbody; bodyId += 1) {
      const bodyObject = this.bodies[bodyId];
      if (!bodyObject) {
        continue;
      }
      getPosition(this.data.xpos, bodyId, bodyObject.position);
      getQuaternion(this.data.xquat, bodyId, bodyObject.quaternion);
      bodyObject.updateWorldMatrix();
    }

    for (let lightId = 0; lightId < this.model.nlight; lightId += 1) {
      const lightObject = this.lights[lightId];
      if (!lightObject) {
        continue;
      }
      getPosition(this.data.light_xpos, lightId, lightObject.position);
      getPosition(this.data.light_xdir, lightId, this.tmpVec);
      lightObject.lookAt(this.tmpVec.add(lightObject.position));
    }

    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);
  }

  updateWristCameraTransforms() {
    const tool0Object = this.bodies[this.tool0BodyId];
    if (!tool0Object) {
      return;
    }

    tool0Object.getWorldPosition(this.tmpWorldPos);
    tool0Object.getWorldQuaternion(this.tmpWorldQuat);

    for (const name of BASLER_CAMERA_NAMES) {
      const frame = this.wristCameraFrames[name];
      const camera = this.wristCameraObjects.get(name);
      if (!frame || !camera) {
        continue;
      }

      const localPos = this.tmpVec.set(frame.x, frame.y, frame.z);
      const localQuat = this.tmpQuat.setFromEuler(
        new THREE.Euler(
          THREE.MathUtils.degToRad(frame.rollDeg),
          THREE.MathUtils.degToRad(frame.pitchDeg),
          THREE.MathUtils.degToRad(frame.yawDeg),
          "XYZ"
        )
      );

      camera.position.copy(localPos.applyQuaternion(this.tmpWorldQuat).add(this.tmpWorldPos));
      camera.quaternion.copy(this.tmpWorldQuat).multiply(localQuat);
      camera.fov = frame.fovDeg;
      camera.updateProjectionMatrix();
    }
  }

  updateStatusOverlay() {
    const replayStatus = this.params.replayEnabled ? "on" : "off";
    const pausedStatus = this.params.paused ? "yes" : "no";
    const manualActuator = this.params.manualActuatorOverride ? "yes" : "no";
    this.setStatus(
      [
        `sim_time_s: ${this.simTimeSec.toFixed(3)}`,
        `paused: ${pausedStatus}`,
        `replay_policy: ${replayStatus}`,
        `manual_actuators: ${manualActuator}`,
        `basler_panels: ${this.params.showBaslerPanels ? "on" : "off"}`,
      ].join("\n")
    );
  }

  stepSimulation(frameDeltaSec) {
    const timestep = this.model.opt.timestep;
    this.accumulatorSec += Math.min(frameDeltaSec, 0.05);

    let substeps = 0;
    while (this.accumulatorSec >= timestep && substeps < 64) {
      if (this.params.manualActuatorOverride) {
        this.applyManualActuatorControls();
      } else {
        this.applyReplayControlsForStep();
      }

      this.mujoco.mj_step(this.model, this.data);
      this.simTimeSec += timestep;
      this.accumulatorSec -= timestep;
      substeps += 1;
    }
  }

  advanceSimulationSteps(stepCount) {
    if (!this.model || !this.data) {
      return;
    }
    const timestep = this.model.opt.timestep;
    const steps = Math.max(0, Math.floor(Number(stepCount) || 0));

    for (let i = 0; i < steps; i += 1) {
      if (this.params.manualActuatorOverride) {
        this.applyManualActuatorControls();
      } else {
        this.applyReplayControlsForStep();
      }
      this.mujoco.mj_step(this.model, this.data);
      this.simTimeSec += timestep;
    }

    this.syncJointOverrideFromData();
    this.updateSceneTransformsFromData();
    this.updateWristCameraTransforms();
  }

  render(timeMs) {
    if (!this.model || !this.data) {
      return;
    }

    if (this.lastFrameWallMs === null) {
      this.lastFrameWallMs = timeMs;
    }

    const frameDeltaSec = Math.max(0.0, (timeMs - this.lastFrameWallMs) * 1e-3);
    this.lastFrameWallMs = timeMs;

    this.controls.update();

    if (this.params.paused) {
      if (this.params.applyPausedJointOverride) {
        this.applyPausedJointOverrides();
      } else {
        this.mujoco.mj_forward(this.model, this.data);
      }
    } else {
      this.stepSimulation(frameDeltaSec);
      this.syncJointOverrideFromData();
    }

    this.updateSceneTransformsFromData();
    this.updateWristCameraTransforms();

    this.renderer.render(this.scene, this.camera);

    if (this.params.showBaslerPanels) {
      for (const name of BASLER_CAMERA_NAMES) {
        const renderer = this.wristCameraRenderers.get(name);
        const camera = this.wristCameraObjects.get(name);
        if (renderer && camera) {
          renderer.render(this.scene, camera);
        }
      }
    }

    this.updateStatusOverlay();
  }

  resetSimulation() {
    if (!this.model || !this.data) {
      return;
    }
    this.mujoco.mj_resetData(this.model, this.data);
    this.mujoco.mj_forward(this.model, this.data);
    this.accumulatorSec = 0.0;
    this.simTimeSec = 0.0;
    if (this.replaySampler) {
      this.replaySampler.reset();
    }
    this.syncActuatorStateFromModel();
    this.syncJointOverrideFromData();
  }

  onResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }
}

(async () => {
  const demo = new UR5eCableWorkcellBrowserDemo();
  try {
    await demo.init();
  } catch (error) {
    console.error(error);
    demo.setStatus(`Initialization failed:\n${error?.message || error}`);
    throw error;
  }
})();
