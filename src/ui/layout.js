export function createAppLayout(cameraNames) {
  const root = document.createElement("div");
  root.id = "workcell-root";

  const mainViewport = document.createElement("div");
  mainViewport.id = "main-viewport";
  root.appendChild(mainViewport);

  const statusOverlay = document.createElement("div");
  statusOverlay.id = "status-overlay";
  statusOverlay.textContent = "Initializing...";
  root.appendChild(statusOverlay);

  const baslerPanels = document.createElement("div");
  baslerPanels.id = "basler-panels";
  root.appendChild(baslerPanels);

  const cameraPanels = new Map();
  for (const cameraName of cameraNames) {
    const panel = document.createElement("div");
    panel.className = "basler-panel";

    const label = document.createElement("div");
    label.className = "basler-label";
    label.textContent = `${cameraName} (RGB 20fps)`;
    panel.appendChild(label);

    const canvasHost = document.createElement("div");
    panel.appendChild(canvasHost);
    baslerPanels.appendChild(panel);

    cameraPanels.set(cameraName, { panel, canvasHost });
  }

  document.body.appendChild(root);

  return {
    root,
    mainViewport,
    statusOverlay,
    baslerPanels,
    cameraPanels,
  };
}
