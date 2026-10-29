# Project Data Layout Implementation Plan

> **For agentic workers:** Implement task by task in this workspace. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Move the project toward session-centered data storage while preserving existing recordings, command entry points, and reproducible links between raw inputs and processing outputs.

**Architecture:** Add one path/manifest layer and one explicit migration utility. New captures and runs use the session tree; legacy documents and generated HDF5 remain readable through a compatibility adapter. Migrate by copy, hash comparison, then switch readers; keep legacy sources in place for rollback.

**Tech Stack:** Python 3.10+, pathlib, JSON, HDF5/h5py, existing CLI and local HTTP viewer.

**Spec:** `docs/superpowers/specs/2026-10-29-project-layout-design.md`

## Global Constraints

- Preserve raw files and their contents byte-for-byte.
- Keep session IDs and existing `./mocap record`, `./mocap visualize`, and `--output-root` behavior.
- Preserve current Python package names and all historical source status and time/coordinate semantics.
- Reject collisions; never overwrite a session, run, or existing artifact.
- Keep legacy source directories during this migration for rollback.
- Do not claim accuracy or successful stages based only on file presence.
- The supplied workspace is not a Git checkout; make changes in the requested project directory without Git operations.

## Review Focus

- Partial or failed capture manifests retain their exact original status and errors during migration.
- Multiple unproven historical outputs are not silently grouped into one processing run.
- HDF5 absolute source paths still resolve after new files are introduced.
- Web manifest paths cannot escape the approved session data root, including through symlinks.
- Re-running migration cannot replace files or merge conflicting session identities.

---

### Task 1: Add central session paths and manifest compatibility

**Files:**
- Create: `src/joint_capture/paths.py`
- Create: `src/joint_capture/manifests.py`
- Modify: `src/joint_capture/session.py`
- Modify: `src/joint_capture/cli.py`
- Modify: `src/joint_capture/alignment.py`

**Interfaces:**
- `ProjectPaths(root: Path)` exposes `session(session_id)`, `raw_ego(session_id)`, `raw_mocap(session_id)`, `clock_probes(session_id)`, `capture_records(session_id)`, and `run(session_id, run_id)` paths.
- `load_session_manifest(path: Path) -> dict` reads both the current flat manifest and the new versioned manifest without dropping source fields.
- `resolve_session_asset(root: Path, manifest: dict, asset_id: str) -> Path` resolves local assets inside the session tree and rejects path escape.

- [x] Inventory every path consumer before editing and record legacy aliases.
- [x] Add the path and manifest interfaces, with session and run ID validation that rejects separators and traversal.
- [x] Change joint capture to create `data/sessions/<session_id>/{raw/ego,raw/mocap,raw/clocks,capture}` and a versioned session manifest, while preserving current CLI options.
- [x] Change offline alignment to read both legacy and new session manifests and resolve source files through the shared path layer.
- [x] Review the resulting call sites to ensure direct construction of `capture_sessions`, `ego_data`, and `mocap_data` is limited to compatibility code.

### Task 2: Make processing outputs session and run scoped

**Files:**
- Create: `src/joint_capture/runs.py`
- Modify: `tools/export_aligned_h5.py`
- Modify: `tools/okvis_pose_to_h5.py`
- Modify: offline alignment output handling in `src/joint_capture/alignment.py`

**Interfaces:**
- `create_run(paths: ProjectPaths, session_id: str, run_id: str, stages: list[str], config: dict) -> Path` creates a non-existing run directory and versioned `run.json` in `running` state.
- `finish_stage(run_dir: Path, stage: str, status: str, outputs: list[dict])` records stage state and output assets without changing prior runs.

- [x] Implement explicit run creation and completion records; do not infer that skipped or unimplemented stages completed.
- [x] Replace fixed `results/alignment_20261029` paths with explicit session/run paths, retaining a read-only legacy lookup when arguments omit the run selector.
- [x] Write aligned HDF5 and OKVIS pose exports under their owning run without changing numerical arrays or timestamp semantics.
- [x] Add an inventory command/report describing how each legacy result set maps to sessions and candidate runs; mark uncertain provenance instead of guessing.

### Task 3: Add a safe, resumable data migration command

**Files:**
- Create: `tools/migrate_project_layout.py`
- Create: `src/joint_capture/migration.py`
- Create on execution: `reports/migrations/<migration_id>/inventory.json`, `plan.json`, and `results.json`

**Interfaces:**
- `build_migration_plan(root: Path, migration_id: str) -> dict` inventories files and proposes old-to-new paths, hashes, provenance, and conflicts without writing into target session directories.
- `apply_migration_plan(root: Path, plan: dict) -> dict` copies to temporary sibling paths, verifies hashes, atomically promotes completed sessions, and skips only identical completed entries.

- [x] Inventory sizes, SHA-256 hashes, manifest links, HDF5 source references, available disk space, and collisions.
- [x] Map capture manifests, EGO data, NatNet HDF5, probes, capture logs, aligned HDF5, OKVIS outputs, calibration references, C3D samples, and unassigned files according to the spec.
- [x] Keep C3D files with no demonstrated session association under `data/samples/` or `data/unassigned/`.
- [x] Apply the plan by copying; preserve every old source; stop on insufficient space, hash mismatch, and conflicting session identity.
- [x] Write a migration result entry for every source file with source/destination, hashes, action, and result.

### Task 4: Update local visualization to discover the session tree

**Files:**
- Modify: `src/motion_web/server.py`
- Modify: `src/motion_web/data.py` if the session asset loader needs an adapter
- Modify: `src/motion_web/static/*` for session/run/type selection

**Interfaces:**
- File listing returns session ID, optional run ID, asset type, and project-relative asset reference.
- Loading an asset resolves through `ProjectPaths`/manifest validation and retains the approved data-root boundary.

- [x] Discover supported raw and aligned assets from the new tree and preserve explicit legacy preview support during transition.
- [x] Present session, run, and asset type in the selector where the asset exists.
- [x] Keep unsupported EGO raw HDF5 and C3D labeled as unsupported inputs; do not present them as hand-joint motion.
- [x] Keep simulation clearly marked and preserve local-only server behavior.

### Task 5: Consolidate deployment/config/docs paths

**Files:**
- Move or consolidate: `config/`, `configs/`, `tools/start_router.sh`, `Third_Repo/`
- Modify: `mocap`, `README.md`, `pyproject.toml`, `pixi.toml`, project docs

- [x] Consolidate active config under `configs/{network,capture,pipelines,visualization}`; preserve current ACL semantics and provide compatibility launchers for changed entry points.
- [x] Move the external repository to `third_party/` only after checking for embedded repositories, local modifications, absolute paths, and build assumptions; preserve its internal tree.
- [x] Keep `.pixi/`, manifest snapshots, licenses, and generated files in place unless the spec explicitly classifies and maps them.
- [x] Update current-state documentation and replace stale data path examples; retain historical reports with their original paths as historical evidence.

### Task 6: Reconcile and document the migrated project state

**Files:**
- Modify: `README.md`, `docs/联合数据采集.md`, `docs/项目数据流.md`, `docs/motion_web.md`, relevant migration notes
- Create: `reports/migrations/<migration_id>/final.json`

- [x] Confirm every session has a readable versioned manifest and every run reference resolves.
- [x] Compare source and destination raw SHA-256 values and confirm legacy sources remain present.
- [x] Compare existing aligned/pose HDF5 source references and numeric datasets with pre-migration values where applicable; do not rerun OKVIS.
- [x] Record unmigrated, ambiguous, or unavailable assets without fabricating session/run associations.
- [x] Document the new path conventions, migration ID, retained compatibility, and any unresolved data provenance.

## Execution notes

- Migration ID: `layout-20261029-v1`; 2,309 files (893,435,140 bytes) copied, zero conflicts, all destination hashes matched. A repeated apply found all 2,309 destinations identical.
- Eleven session manifests and historical run records are present. Capture status and errors remain in the session manifest and its preserved source copy.
- Historical processing runs are marked `historical_unknown` where files alone cannot prove execution success; their output availability is separately indexed.
- Old directories remain intact for rollback. `Third_Repo/` remains in place because generated CMake files refer to its absolute path; this was documented in `docs/数据目录重组_20261029.md`.
- No automated test suite was run. Python source syntax, the browser module syntax, web API discovery/loading/path traversal behavior, and migration integrity were checked.
- Independent review findings about Web symlink boundaries, atomic session promotion, historical status inference, run provenance, unassigned-file discovery, and HDF5 source references were addressed in one fix pass.
