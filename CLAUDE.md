# CLAUDE.md — guide for Claude Code on ArmForge

ArmForge is a 6-DOF robotic arm digital twin: a layered C++23 / Qt 6 desktop
app (Qt Quick 3D scene, real-time control loop, IK/FK, RRT-Connect motion
planning with optional CUDA acceleration). Full feature overview: [README.md](README.md).

## Layout & architecture

```
core/        AF_Core — pure domain logic (Eigen only, ZERO Qt)
             kinematics/ math/ model/ planning/ (incl. planning/cuda)
app/         Qt layer: control/ (ControlLoop, QThread @ 50 Hz)
             controllers/ (RobotController, PlanningController — QML singletons)
ui/          QML: Main.qml, scene/ (View3D, RobotView, gizmo, planning view),
             panels/ (JogPanel, ChartPanel, PlanningPanel)
```

**Dependency rule**: arrows point only downward — `ui` → `app` → `core`.
`AF_Core` must stay buildable/testable without Qt; never add a Qt include there.

## Stack

- C++23, MSVC 2022, CMake ≥ 3.21 + vcpkg (Eigen 3.4 via manifest)
- Qt 6.11 (Qt Quick 3D, Qt Quick Controls 2), QThread + QueuedConnection for
  cross-thread signal/slot (no mutexes on the 50 Hz hot path)
- Optional CUDA backend (`ARMFORGE_HAS_CUDA`) behind the `IMotionPlanner`
  interface — falls back to the CPU `RrtConnectPlanner` transparently when the
  toolkit/driver isn't available

## Build

This project is configured through Qt Creator ("Desktop Qt 6.11.0 MSVC2022
64bit" kit, NMake Makefiles JOM generator). Build from a terminal with the
provided script, which reproduces that exact kit — don't reach for a generic
`cmake -B build && cmake --build build`:

```powershell
.\build.ps1               # incremental build
.\build.ps1 -Reconfigure  # re-run CMake configure (after editing CMakeLists.txt)
.\build.ps1 -Run          # build then launch ArmForge.exe
```

Don't switch generators or reconfigure with a different toolchain — the CUDA
custom-command step (`core/CMakeLists.txt`) is hand-tuned for NMake/JOM quoting.

## Conventions

- **No comments explaining *what* the code does** — name things well instead.
  Comments are reserved for non-obvious *why* (hidden constraints, workarounds,
  invariants). Match the existing terse style in `core/` and `app/`.
- **SonarCloud is wired to this repo** (project `abdelmonamaz_ArmForge`,
  organization `abdelmonamaz`). When asked to fix Sonar issues, query the
  public API directly rather than guessing:
  `https://sonarcloud.io/api/issues/search?componentKeys=abdelmonamaz_ArmForge&resolved=false`
  When a rule's suggestion would actually hurt (e.g. suggesting `std::print`
  inside a SEH crash handler, or templating an already type-erased
  `std::function` boundary), suppress with an inline `// NOSONAR (SXXXX: why)`
  comment **on the exact reported line** — Sonar's C++ analyzer ignores
  suppression comments placed on preceding lines.
- Prefer modern STL idioms already used in the codebase: CTAD, `auto` for
  obviously-redundant types, `std::ranges::*` algorithms, `emplace_back`,
  in-class member initializers over constructor init lists (Sonar's current
  preference here — verify against the live ruleset, it has flipped before).
- `IMotionPlanner` is the strategy interface for path planning
  (`RrtConnectPlanner` CPU / `CudaRrtPlanner` GPU) — keep both implementations
  in lockstep; only the segment-validity check should differ between them.
