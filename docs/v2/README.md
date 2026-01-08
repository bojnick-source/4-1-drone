# v2 — Computational Engineering Engine + Desktop Studio

v2 is a precision-first computational engineering system built alongside the existing v1 codebase. v2 is architecturally independent and does not depend on v1 components.

## Directory Structure

```
v2/
├── engine/              # C++ computational engine
│   ├── include/v2/      # Public headers
│   ├── src/             # Implementation files
│   ├── tests/           # Unit and integration tests
│   ├── tools/           # CLI harnesses and utilities
│   └── CMakeLists.txt   # Engine build configuration
│
└── studio/              # Qt desktop application
    ├── app/             # Application entry and main window
    ├── ui/              # UI components and widgets
    ├── render/          # Viewport and 3D rendering
    ├── cad/             # CAD geometry and visualization
    ├── models/          # Data models and state management
    └── CMakeLists.txt   # Studio build configuration (requires Qt6)

docs/v2/                 # v2 documentation
schemas/v2/              # JSON schemas for v2 artifacts
```

## Build System

v2 uses CMake and builds independently of v1:

- **v2_engine**: INTERFACE library target for the computational engine
- **v2_studio**: Qt6 application (requires Qt6 Core, Gui, Widgets, OpenGL)

The root `CMakeLists.txt` includes both v1 and v2 targets without dependencies between them.

## Design Principles

1. **Fail-closed**: No silent failures, explicit error labels for all violations
2. **Deterministic**: All computations must be reproducible and traceable
3. **Fragment-based**: Development proceeds in labeled, testable increments
4. **Scope-locked**: Each fragment operates only within its designated directories

## Development Workflow

All v2 development follows a fragment-based discipline:

- Each fragment is labeled: `FRAGMENT <CATEGORY>.<MAJOR>.<MINOR>`
- Fragments are independently buildable and testable
- Fragment categories: AERO, PROP, ELEC, ENERGY, STRUCT, CTRL, SYS, IO, UI
- Progress is tracked fragment-by-fragment

## Current Status

**FRAGMENT SYS.0.1** — v2 scaffolding (COMPLETED)
- Directory structure created
- CMake build system configured
- Documentation initialized

## Next Fragments

- **FRAGMENT CORE.0.2** — Fail labels + error taxonomy
- **FRAGMENT IO.0.3** — Artifact schemas and hashing
- **FRAGMENT UI.0.1** — Qt studio skeleton

## Building v2

```bash
cmake -S . -B build
cmake --build build --target v2_engine
cmake --build build --target v2_studio  # Requires Qt6
```

## Requirements

- CMake 3.20+
- C++20 compiler
- Qt6 (for v2_studio only)
