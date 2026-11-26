# radar-ekf — Continuation Notes

## Status: COMPLETE ✅

Library builds, tests pass, example runs.

## What's Done

- **Core EKF** (`include/radar_ekf/radar_ekf.hpp`): Header-only implementation with:
  - 3D constant velocity state `[x,y,z,vx,vy,vz]`
  - Polar measurement model `[azimuth, range, elevation]`
  - Analytic Jacobian `H` for EKF update
  - Continuous white-noise acceleration process noise `Q(dt)`
  - Angle normalization (azimuth wrap, elevation clamp)
  - Configurable noise parameters

- **Build System** (`CMakeLists.txt`): FetchContent fallback for Eigen3, builds static lib + example + tests

- **Tests** (`tests/test.cpp`): 9 passing unit tests covering roundtrip, Jacobian FD, EKF cycle

- **Example** (`examples/example.cpp`): Synthetic 3D trajectory demo showing convergence

## Usage

```bash
cd /opt/data/radar-ekf
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/radar_ekf_test    # Unit tests
./build/radar_ekf_example # Demo
```

## Integration

```cpp
#include <radar_ekf/radar_ekf.hpp>

radar_ekf::RadarEKF::Config cfg;
cfg.dt = 10.0;  // radar period
cfg.meas_noise.sigma_range = 50.0;
cfg.meas_noise.sigma_azimuth = 0.5 * M_PI / 180.0;

radar_ekf::RadarEKF ekf(cfg);
ekf.init(measurement, timestamp);
ekf.step(new_measurement, new_timestamp);
auto pos = ekf.position();
auto vel = ekf.velocity();
```

## TODO / Extensions (Priority Order)

1. **Multi-target wrapper**: Data association (GNN/JPDA), track lifecycle (init/confirm/delete), gating
2. **CV + CT models**: Coordinated turn model for maneuvering targets
3. **IMM filter**: Interacting Multiple Model for model switching
4. **Smoothing**: RTS backward pass for post-processing
5. **Serialization**: Save/load track state
6. **Python bindings**: pybind11 for rapid prototyping
7. **Real-time ingestion**: Async `step()` with per-detection timestamps (see `multi-target-tracking` skill patterns)

## Known Gotchas

- **Angle units**: All internal math uses radians. Input measurements must be radians.
- **Coordinate frame**: Assumes standard right-handed (ENU or NED). Elevation measured from XY plane.
- **Initialization**: First measurement sets position; velocity starts at zero with large covariance (1e4).
- **Process noise scaling**: `q` should scale with target maneuverability. Typical: 0.1–2.0 m²/s³.
- **Covariance growth**: Velocity uncertainty stays high without Doppler measurements — expected for bearings-only radar.

## Files to Reference

- `/opt/data/skills/multi-target-tracking/templates/radar_tracker.py` — Python Stone Soup reference for multi-target patterns
- `/opt/data/skills/radar-tracker-workstation` — C++ ImGui PPI display (separate project)