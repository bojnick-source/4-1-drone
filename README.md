# DARPA LIFT - Drone Optimization Engine

This repository implements the computational engine for the DARPA LIFT (Learning, Improving, and Flying Together) drone optimization system.

## Project Structure

```
4-1-drone/
├── cpp/
│   ├── cli/                      # Command-line applications
│   │   ├── main.cpp              # Main CLI entrypoint
│   │   ├── closeout_demo.cpp     # Closeout pipeline demo
│   │   └── optimizer_demo.cpp    # Batch optimization demo
│   └── engine/
│       ├── core/                 # Core utilities
│       │   ├── design.hpp        # Design schema
│       │   ├── mission_spec.hpp  # Mission specification
│       │   ├── settings.hpp      # Evaluation settings
│       │   ├── hashing.hpp       # Deterministic hashing
│       │   └── ...
│       ├── physics/              # Physics models
│       │   ├── disk_area.*       # Disk area calculation
│       │   ├── hover_momentum.*  # Hover power model
│       │   └── cfd_results.*     # CFD integration container
│       ├── analysis/             # Analysis and closeout
│       │   ├── closeout_types.hpp    # Closeout data model
│       │   ├── closeout_eval.*       # Gate evaluation
│       │   └── closeout_json.*       # JSON serialization
│       ├── optimization/         # Optimization components
│       │   ├── candidate_generator.* # Design space sampling
│       │   └── objective.*           # Objective function
│       └── exports/              # Export utilities
│           └── stats_report_csv.*    # CSV export
├── scripts/
│   └── run_closeout.py          # Python CI wrapper
└── CMakeLists.txt               # Build configuration
```

## Features

### Fragment 1: Computational Engine

#### 1. Design Schema & Validation
- Canonical `Design` representation supporting multiple architectures
- Explicit validation with `validate_or_throw()`
- Support for quad, hex, octo, coaxial, tandem, shrouded configurations

#### 2. Candidate Generation
- Multiple sampling strategies:
  - Random (Monte Carlo)
  - Latin Hypercube Sampling (LHS)
  - Grid sampling
  - Sobol sequences (quasi-random)
- Design space bounds with validation
- Constraint pre-filtering
- Deterministic seeded generation

#### 3. Objective Function & Scoring
- Primary objective: Maximize payload ratio
- Constraint penalties:
  - Aircraft mass limit
  - Payload ratio minimum
  - Tip speed (Mach limit)
  - Disk loading bounds
  - Power feasibility
- Detailed score breakdown for debugging

#### 4. Physics Models
- Hover power via momentum theory
- Effective disk area calculation
- CFD results container for external solver integration

#### 5. Closeout & Gate Evaluation
- Comprehensive closeout report structure
- GO/NO-GO/NEEDS_DATA gate decisions
- Missing data tracking
- Mass delta breakdown
- JSON serialization

#### 6. Export & Reporting
- JSON export with deterministic formatting
- CSV batch export with 30+ metrics
- NaN-safe field handling

### Fragment 4: Integration & Tooling

#### 1. CLI Applications
- **lift_cli**: Design validation, hashing, disk area, hover power
- **closeout_demo**: Full closeout pipeline demonstration
- **optimizer_demo**: Batch candidate generation and evaluation

#### 2. Python CI Wrapper
- `run_closeout.py`: Python wrapper for CI/CD integration
- Exit code handling for pipeline integration
- Artifact management
- Summary generation

## Building

### Prerequisites
- CMake 3.20+
- C++20 compiler (GCC 13+, Clang 16+, or MSVC 2022+)
- Python 3.8+ (for CI wrapper)

### Build Steps

```bash
# Configure
mkdir build && cd build
cmake ..

# Build all targets
make -j4

# Or build specific targets
make lift_cli
make closeout_demo
make optimizer_demo
```

## Usage

### 1. Design Validation

```bash
./build/lift_cli validate
```

### 2. Design Hashing

```bash
./build/lift_cli hash
```

### 3. Disk Area Calculation

```bash
./build/lift_cli disk-area
```

### 4. Hover Power Computation

```bash
./build/lift_cli hover
```

### 5. Closeout Pipeline

```bash
# Direct invocation
./build/closeout_demo [output.json]

# Via Python wrapper (CI-friendly)
python scripts/run_closeout.py --verbose --output-dir ./artifacts
```

### 6. Batch Optimization

```bash
# Generate and evaluate 100 candidates
./build/optimizer_demo 100 candidates.csv

# Output: CSV file with scores and metrics
head candidates.csv
```

## Output Files

### JSON Closeout Report
```json
{
  "variant_concept": "Quad_With_SFCS",
  "variant_name": "D6_baseline_like_demo",
  "geom_hash": "demo_geom_hash_placeholder",
  "eval_hash": "demo_eval_hash_placeholder",
  "mass_delta": {
    "baseline_aircraft_mass_kg": 24.95,
    "delta_mass_total_kg": 0.53,
    "resulting_aircraft_mass_kg": 25.48,
    "resulting_payload_ratio": 4.112637,
    ...
  },
  "gate_result": {
    "decision": "Go",
    "failed_gates": [],
    "missing_data": []
  }
}
```

### CSV Candidate Export
Columns include:
- Identity: `variant_name`, `geom_hash`, `eval_hash`
- Mass: `delta_mass_total_kg`, `resulting_aircraft_mass_kg`, `resulting_payload_ratio`
- Disk/Power: `A_total_m2`, `disk_loading_N_per_m2`, `P_hover_total_W`
- Control: `yaw_margin_ratio`, `roll_margin_ratio`, `pitch_margin_ratio`
- Mission: `baseline_time_s`, `resulting_time_s`
- Gates: `gate_decision`, `failed_gates_count`, `missing_data_count`

## Architecture

### Design Space Parameters
- **Rotor count**: {4, 6, 8}
- **Rotor radius**: [0.15, 0.50] m
- **Rotor solidity**: [0.03, 0.15]
- **Rotor RPM**: [3000, 8000]
- **Mass components**: Continuous ranges
- **Architecture**: Enum (Multicopter, Coaxial, Tandem, etc.)

### Constraints
- Aircraft mass ≤ 30 kg
- Payload ratio ≥ 3.0
- Tip Mach ≤ 0.70
- Disk loading: [100, 600] N/m²
- Rotor power feasibility

### Scoring
```
score = base_fitness + penalties
base_fitness = payload_ratio (higher is better)
penalties = mass_penalty + disk_loading_penalty + 
            tip_speed_penalty + power_penalty
```

## Testing

All components have been validated:
- ✅ Build succeeds with C++20
- ✅ All CLI commands functional
- ✅ JSON generation validated
- ✅ CSV export validated
- ✅ Python wrapper tested
- ✅ Code review completed
- ✅ Security scan passed (0 vulnerabilities)

## Exit Codes

- **0**: Success (Go decision)
- **1**: Invalid arguments / script error
- **2**: NoGo decision
- **3**: NeedsData decision
- **10**: I/O error

## Development

### Code Style
- C++20 with explicit validation
- Hardening flags enabled
- No silent failures
- Explicit units in field names

### Adding New Components

1. **New Physics Model**: Add to `cpp/engine/physics/`
2. **New Optimizer**: Add to `cpp/engine/optimization/`
3. **New Export Format**: Add to `cpp/engine/exports/`
4. **Update CMakeLists.txt** to include new sources

### Future Enhancements

Remaining work as per Fragment 1-4 scope:
- [ ] Additional optimizer algorithms (GA, PSO, Bayesian Optimization)
- [ ] UI/UX components (Fragment 3)
- [ ] Machine learning integration (Fragment 2)
- [ ] Advanced CG/inertia calculations
- [ ] Rules PDF parser and clause mapping
- [ ] Multi-objective optimization
- [ ] Uncertainty quantification

## License

[License information to be added]

## Authors

DARPA LIFT Development Team

## References

- DARPA LIFT Program
- Momentum theory for rotorcraft
- Design optimization methods
