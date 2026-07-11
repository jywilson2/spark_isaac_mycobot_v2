# V2 Bootstrap Plan — MyCobot Isaac Lab Reach

Portable plan for starting a fresh repository. Copy this file (and linked artifacts) into the new repo on day one.

**Old repo (read-only reference):** `spark_isaac_mycobot_demo`  
**Suggested new path:** `/workspaces/isaac_ros-dev/src/spark_mycobot_reach/`  
**Suggested git remote:** new GitHub repo (do not force-push over the old one)

---

## Why restart

The v1 repo accumulated:

- Two competing training recipes (`run_two_phase_training.sh` vs `run_staged_training.py`) with conflicting README/spec claims
- Unverified 1 mm precision targets vs a **verified 25 mm** checkpoint (`assets/checkpoints/verified_demo_25mm/`)
- Isaac Sim **stage-transition hangs** during multi-stage reload
- Phase 5, pick-and-place env, and mock ROS layers that dilute the core EE-reach goal

V2 should be **narrow, honest, and reproducible** before expanding scope.

---

## V2 scope (MVP)

| In scope (v2.0) | Out of scope (defer) |
|-----------------|----------------------|
| EE reach-to-target PPO in Isaac Lab | Phase 5 red-block vision |
| Host ↔ container delegation (`nsenter`) | Sub-8 mm precision until recipe is proven |
| Single staged training orchestrator | `mycobot_pick_place_env.py` |
| Verified demo @ **25 mm** (match v1 best result) | `last_prompt.md` append-only log |
| ROS 2 smoke tests (minimal) | Full Phase 1–4 mock ecosystem unless needed |
| ONNX export hook (stub or thin wrapper) | Edge deployment on RPi (Phase 4) |
| Tutorial-quality docs on MDP + motion | `commands/initial_project_generation*.md` |

**Success criterion for v2.0:** Reproduce **≥ 95%** reach success @ **25 mm** on 20/30/40 s demo horizons from a **single documented recipe**, with **no manual stage babysitting**.

---

## Minimal repo layout

```
spark_mycobot_reach/
├── .cursorrules                 # Copy from v1 (still valid)
├── .gitignore
├── spec.md                      # Slim v2 requirements (~80 lines)
├── README.md                    # Daily workflow only; no phase history essay
├── REFERENCES.md                # Copy from v1 (hardware + Isaac links)
│
├── docs/
│   ├── project_status.md        # Fresh; link v1 lessons in legacy/
│   ├── isaac_sim_host_scripts.md
│   └── legacy/
│       ├── v1_lessons_learned.md
│       └── v1_port_map.md       # Old path → new path mapping
│
├── scripts/
│   ├── source_container_env.sh
│   ├── monitor_training.sh
│   ├── verify_demo_policy.sh
│   └── host/
│       ├── spark_host_exec.sh   # PORT FIRST — non-negotiable
│       ├── env.isaac_host.sh
│       ├── check_prereqs.sh
│       ├── install_isaac_lab.sh
│       ├── verify_isaac_lab.sh
│       ├── run_isaac_lab_training.sh
│       ├── iter_urdf_import.sh
│       └── iter_build_isaac_scene.sh
│
├── isaac_sim/
│   ├── build_mycobot_limo_cobot_scene.py
│   ├── urdf_import.py
│   ├── urdf_utils.py
│   ├── run_mycobot_live_sim.py
│   ├── ros2_bridge_config.py
│   └── test/
│
├── isaac_lab/
│   ├── mdp_core.py              # Constants: tolerances, joint names, obs/action dims
│   ├── mycobot_reach_env.py     # Single env — no pick_place
│   ├── training_defaults.py
│   ├── training_recipe.py       # ONE recipe module
│   ├── run_training.py          # Single entry (replaces train + staged runners)
│   ├── play_ppo.py
│   ├── rsl_rl_ppo_cfg.py
│   ├── detect_isaac_lab.py
│   ├── warning_filters.py
│   ├── verify_install.py
│   └── test/
│       ├── test_mdp_contract.py
│       ├── test_training_recipe.py
│       └── test_isaac_lab_integration.py
│
├── spark_verify_pkg/            # Optional: rename to mycobot_reach_bringup
│   └── …                        # Minimal ROS smoke only (see port list)
│
└── assets/
    ├── scenes/                  # USD scene (copy or rebuild)
    └── checkpoints/
        └── seed_25mm/           # Copy verified policy from v1 OR retrain
            ├── policy.pt
            └── demo_verify_results.json
```

**Deliberately omitted from v2 tree:**

- `isaac_lab/phase5_red_block/`
- `isaac_lab/mycobot_pick_place_env.py`
- `isaac_lab/run_staged_training.py` (merge into `run_training.py`)
- `scripts/run_two_phase_training.sh` (one orchestrator)
- `commands/` agent playbooks
- `last_prompt.md`

---

## Port vs leave checklist

### PORT — copy or adapt (high confidence)

| Artifact | v1 path | v2 notes |
|----------|---------|----------|
| Host exec bridge | `scripts/host/spark_host_exec.sh` | Copy verbatim; update `SPARK_HOST_REPO_ROOT` default |
| Host env | `scripts/host/env.isaac_host.sh` | Copy; verify Isaac Sim paths on Spark |
| Training launcher | `scripts/host/run_isaac_lab_training.sh` | Copy; trim subcommands to `check`, `train`, `staged`, `play`, `demo`, `verify-demo`, `monitor` |
| Container env | `scripts/source_container_env.sh` | Copy; update repo path |
| MDP constants | `isaac_lab/mdp_core.py` | Copy; document 25 mm as **initial** success tolerance |
| Reach env | `isaac_lab/mycobot_reach_env.py` | Copy; drop Phase 5 hooks |
| PPO config | `isaac_lab/rsl_rl_ppo_cfg.py` | Copy |
| Play / demo | `isaac_lab/play_ppo.py` | Copy |
| Training defaults | `isaac_lab/training_defaults.py` | Copy |
| Warning filters | `isaac_lab/warning_filters.py` | Copy |
| Verify scripts | `scripts/host/verify_isaac_lab.sh`, `install_isaac_lab.sh` | Copy |
| Scene builder | `isaac_sim/build_mycobot_limo_cobot_scene.py` + URDF utils | Copy |
| Demo verify | `scripts/verify_demo_policy.sh`, `scripts/monitor_training.sh` | Copy |
| **Verified checkpoint** | `assets/checkpoints/verified_demo_25mm/` | Copy to `assets/checkpoints/seed_25mm/` |
| References | `REFERENCES.md` | Copy |
| Cursor rules | `.cursorrules` | Copy |
| Core tests | `isaac_lab/test/test_mdp_contract.py`, `test_training_recipe.py` | Copy; rewrite recipe tests for v2 single recipe |

### PORT — cherry-pick patterns (rewrite, don't copy wholesale)

| Concept | v1 source | v2 approach |
|---------|-----------|-------------|
| Two-phase recipe (works) | `scripts/run_two_phase_training.sh` | Encode as stages 1–2 in **one** `training_recipe.py` |
| Staged precision (broken) | `isaac_lab/training_recipe.py` 7-stage | **Do not port** until stage-reload hang is fixed |
| Stage orchestration | `isaac_lab/run_staged_training.py` | New `run_training.py` with **timeout + kill Isaac between stages** |
| Train CLI | `isaac_lab/train_ppo.py` | Merge train + verbose glossary into `run_training.py` |
| Host script docs | `docs/isaac_sim_host_scripts.md` | Copy; update paths |

### LEAVE — do not port

| Artifact | Reason |
|----------|--------|
| `isaac_lab/phase5_red_block/` | Deferred; adds cameras, contact logic, test surface |
| `isaac_lab/mycobot_pick_place_env.py` | Wrong task; README still says pick-and-place |
| `scripts/run_two_phase_training.sh` + `run_staged_training.sh` | Competing orchestrators; consolidate |
| `commands/initial_project_generation*.md` | One-shot agent prompts; not runtime code |
| `last_prompt.md` + retention policy | Agent-session artifact; use git commits instead |
| `docs/isaac_lab_warnings_audit.md` | Regenerate when warnings change in v2 |
| Most `spark_verify_pkg/` mock ecosystem | Phase 1–4 mock stack (~30 nodes) unless you need ROS live gate day one |
| `wip_live_testing` branch history | Tag v1 `archive/pre-v2-rewrite` (already exists); start v2 `main` clean |
| README phase-history tables | Replace with current-state-only README |
| Spec 1 mm requirement (day one) | **Lesson:** claim only what is verified; target 25 mm first |

### PORT LATER (v2.1+)

| Feature | When |
|---------|------|
| Gradual tolerance ramp (25 → 15 → 10 mm) | After stage-reload reliability fix |
| ROS live sim gate (`run_live_sim.sh`) | When Isaac Sim ↔ container bridge needed |
| ONNX export + pymycobot driver | When sim policy is stable |
| Phase 5 vision | Separate milestone |

---

## V2 architectural fixes (learned from v1)

1. **One training orchestrator** — single Python module + one shell wrapper; no parallel recipes.
2. **Stage transitions must not hang** — wrap Isaac Sim reload with watchdog timeout, process kill, and health check before next stage.
3. **Honest tolerances in spec** — spec matches verified metrics; tighten only after `verify-demo` passes.
4. **Checkpoint in git** — seed policy committed so CI/smoke can replay without retraining.
5. **Smaller ROS package** — if keeping `spark_verify_pkg`, strip to joint-state bridge + one integration test; add nodes back when needed.
6. **No IK solvers** — retain v1 prohibition (policy learns joint deltas).

---

## Day-zero bootstrap commands

```bash
# 1. Create repo (host or container as admin)
mkdir -p ~/workspaces/isaac_ros-dev/src/spark_mycobot_reach
cd ~/workspaces/isaac_ros-dev/src/spark_mycobot_reach
git init
git remote add origin git@github.com:YOUR_USER/spark_mycobot_reach.git

# 2. Copy bootstrap doc + rules from v1
V1=~/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_demo
cp "$V1/docs/v2_bootstrap_plan.md" docs/
cp "$V1/.cursorrules" .
cp "$V1/REFERENCES.md" .

# 3. Copy host delegation (critical path)
mkdir -p scripts/host
cp "$V1/scripts/host/spark_host_exec.sh" scripts/host/
cp "$V1/scripts/host/env.isaac_host.sh" scripts/host/
cp "$V1/scripts/source_container_env.sh" scripts/

# 4. Copy verified checkpoint
mkdir -p assets/checkpoints/seed_25mm
cp -r "$V1/assets/checkpoints/verified_demo_25mm/"* assets/checkpoints/seed_25mm/

# 5. Open in Cursor (new window)
# File → New Window → Open Folder → spark_mycobot_reach
```

---

## Multi-root workspace (bootstrap week)

Save `mycobot-bootstrap.code-workspace`:

```json
{
  "folders": [
    { "path": "spark_mycobot_reach", "name": "v2 (active)" },
    { "path": "spark_isaac_mycobot_demo", "name": "v1 (reference)" }
  ],
  "settings": {}
}
```

Open this workspace in Cursor while porting. Remove the v1 folder once v2 passes `verify-demo`.

---

## Suggested v2 `spec.md` outline (~80 lines)

1. Objective: EE reach PPO, tutorial-quality code
2. Host vs container table (copy from v1)
3. MDP definition (11-dim obs, 6-dim Δq actions, EMA + jerk)
4. **Initial success: 25 mm**; precision tiers are v2.1 backlog
5. Single recipe: curriculum 90 min → demo 30 min @ 25 mm
6. Demo verify: ≥ 95% @ 20/30/40 s
7. Prohibited: analytic/differential IK
8. Doc maintenance: `spec.md`, `README.md`, `docs/project_status.md` only

---

## v2.0 task order

| # | Task | Done when |
|---|------|-----------|
| 1 | Scaffold tree + host delegation | `./scripts/host/run_isaac_lab_training.sh check` passes |
| 2 | Port reach env + mdp_core + tests | `pytest isaac_lab/test/test_mdp_contract.py` green |
| 3 | Port scene + verify Isaac Lab | `./scripts/host/verify_isaac_lab.sh` green |
| 4 | Seed checkpoint replay | `play --checkpoint assets/checkpoints/seed_25mm/policy.pt` ≥ 95% |
| 5 | Single recipe from scratch | Staged train reproduces ≥ 95% @ 25 mm |
| 6 | Fix stage-reload watchdog | Stages run unattended without hang |
| 7 | Slim README + project_status | New contributor can run in < 15 min |

---

## v1 lessons learned (paste into `docs/legacy/v1_lessons_learned.md`)

- **Best verified result:** 99/100 demo success @ **25 mm**, not 1 mm (`verified_demo_25mm/`, commit `d92a3cc`).
- **Two-phase recipe works;** 7-stage precision recipe fails below 8 mm (0% reach) and Stage 2 hung on Isaac reload.
- **Do not** treat container missing `python.sh` as training failure — delegate to host.
- **Do not** use `--from-scratch` casually — wipes active checkpoints.
- **Plateau abort** should stay opt-in; default off.
- Pick-and-place naming in README/env was misleading; v2 is reach-only.

---

## Old → new path map

| v1 | v2 |
|----|-----|
| `assets/checkpoints/verified_demo_25mm/` | `assets/checkpoints/seed_25mm/` |
| `isaac_lab/train_ppo.py` + `run_staged_training.py` | `isaac_lab/run_training.py` |
| `scripts/run_two_phase_training.sh` | (merged into `training_recipe.py`) |
| `spark_verify_pkg/` | `mycobot_reach_bringup/` (optional rename) |
| `spec.md` (201 lines, 1 mm claim) | `spec.md` (~80 lines, 25 mm verified target) |
