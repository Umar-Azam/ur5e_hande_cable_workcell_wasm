# MuJoCo WASM Reference Review Notes

## Reviewed Repositories

- `third_party/example_repos/mujoco_wasm`
- `third_party/example_repos/evobot`

## Branch Review (`mujoco_wasm`)

Branches inspected:
- `main`
- `normal_swizzle`
- `piano-hands`
- `refactor`
- `upgrade_mujoco`

Observed pattern:
- `refactor` / `upgrade_mujoco` / `piano-hands` retain CMake + dist artifacts (`mujoco_wasm.wasm`) and earlier build pipeline.
- `main` is simplified around browser demo repurposability with scene loader and utility modules.

## Live Demo Review

Live demo reviewed:
- https://zalo.github.io/mujoco_wasm/

Key UI behavior replicated in this repo:
- `Simulation` folder with pause/reset style controls.
- `Actuators` folder with manual slider inputs.
- Runtime-reloadable parameter control through a right-side GUI.

## Evobot Hosting Pattern

`evobot` includes a `gh-pages` branch and static-hosting-friendly structure. This repo follows the same pattern via:
- static root `index.html`
- no backend requirement
- `tools/setup_gh_pages_branch.sh` helper
- `.nojekyll` marker for Pages compatibility
