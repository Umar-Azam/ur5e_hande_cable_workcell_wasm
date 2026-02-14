function ensureFsDirectory(mujoco, targetDir) {
  const clean = targetDir.replace(/^\/+/, "");
  if (!clean) {
    return;
  }
  const parts = clean.split("/");
  let current = "";
  for (const part of parts) {
    current += `/${part}`;
    if (!mujoco.FS.analyzePath(current).exists) {
      mujoco.FS.mkdir(current);
    }
  }
}

export async function mountSceneAssetsToWorkingFs(
  mujoco,
  {
    manifestUrl = "./assets/scenes/manifest.json",
    assetsRootUrl = "./assets/scenes",
    workingRoot = "/working/assets/scenes",
  } = {}
) {
  ensureFsDirectory(mujoco, "/working");
  ensureFsDirectory(mujoco, workingRoot);

  const manifestResp = await fetch(manifestUrl);
  if (!manifestResp.ok) {
    throw new Error(`Failed to fetch scene manifest: ${manifestResp.status} ${manifestResp.statusText}`);
  }

  const manifest = await manifestResp.json();
  const files = Array.isArray(manifest.files) ? manifest.files : [];
  for (const rel of files) {
    const sourceUrl = `${assetsRootUrl}/${rel}`;
    const targetPath = `${workingRoot}/${rel}`;
    const lastSlash = targetPath.lastIndexOf("/");
    if (lastSlash > 0) {
      ensureFsDirectory(mujoco, targetPath.slice(0, lastSlash));
    }

    const response = await fetch(sourceUrl);
    if (!response.ok) {
      throw new Error(`Failed to fetch scene asset ${sourceUrl}: ${response.status}`);
    }

    const bytes = new Uint8Array(await response.arrayBuffer());
    mujoco.FS.writeFile(targetPath, bytes);
  }

  return files;
}
