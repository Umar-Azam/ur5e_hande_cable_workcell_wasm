export async function loadReplayTrajectory(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to load replay trajectory: ${response.status} ${response.statusText}`);
  }
  const payload = await response.json();
  const time = Array.isArray(payload.time_s) ? payload.time_s : [];
  const joints = Array.isArray(payload.joint_target_rad) ? payload.joint_target_rad : [];
  const gripper = Array.isArray(payload.gripper_command) ? payload.gripper_command : [];
  if (time.length < 2 || joints.length !== time.length || gripper.length !== time.length) {
    throw new Error("Replay trajectory file is malformed.");
  }
  return {
    time,
    joints,
    gripper,
    duration: time[time.length - 1],
  };
}

export class ReplaySampler {
  constructor(trajectory) {
    this.trajectory = trajectory;
    this.index = 0;
    this.prevSampleTime = 0.0;
  }

  reset() {
    this.index = 0;
    this.prevSampleTime = 0.0;
  }

  sample(playbackTimeSec, loop = true) {
    const traj = this.trajectory;
    if (!traj) {
      return null;
    }

    let t = playbackTimeSec;
    if (loop && traj.duration > 0.0) {
      t = t % traj.duration;
      if (t < this.prevSampleTime) {
        this.index = 0;
      }
    } else {
      t = Math.max(0.0, Math.min(playbackTimeSec, traj.duration));
    }

    while (this.index + 1 < traj.time.length && traj.time[this.index + 1] <= t) {
      this.index += 1;
    }

    this.prevSampleTime = t;
    return {
      index: this.index,
      t,
      jointTargets: traj.joints[this.index],
      gripperCommand: traj.gripper[this.index],
    };
  }
}
