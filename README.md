# radar-ekf

Header-only C++20 Extended Kalman Filter for radar tracking with polar measurements (azimuth, range, elevation) using Eigen.

## Features

- **Polar → Cartesian EKF**: Native handling of radar measurements in spherical coordinates
- **3D Constant Velocity Model**: State vector `[x, y, z, vx, vy, vz]^T`
- **Analytic Jacobians**: Exact measurement Jacobian for azimuth/range/elevation
- **Process Noise**: Continuous white-noise acceleration model (discretized)
- **Angle Normalization**: Proper handling of azimuth/elevation wrap-around
- **Header-only core**: Single include `radar_ekf/radar_ekf.hpp`
- **Zero dependencies beyond Eigen3**

## Quick Start

```cpp
#include <radar_ekf/radar_ekf.hpp>

using namespace radar_ekf;

RadarEKF::Config cfg;
cfg.dt = 10.0;  // radar period (seconds)

RadarEKF ekf(cfg);

// First measurement initializes the filter
Measurement z;
z << azimuth_rad, range_m, elevation_rad;
ekf.init(z, timestamp);

// Subsequent frames: predict + update
ekf.step(z_new, timestamp_new);

// Access estimates
Eigen::Vector3d position = ekf.position();
Eigen::Vector3d velocity = ekf.velocity();
Measurement predicted_meas = ekf.predictedMeasurement();
```

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/radar_ekf_example
./build/radar_ekf_test
```

## Installation

```bash
cmake --install build --prefix /usr/local
# Then in your CMakeLists.txt:
find_package(radar_ekf CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE radar_ekf::radar_ekf)
```

## Coordinate Conventions

| Variable | Frame | Units |
|----------|-------|-------|
| State `x,y,z` | Cartesian (ENU or NED) | meters |
| State `vx,vy,vz` | Cartesian velocity | m/s |
| Measurement `azimuth` | Angle from +X toward +Y | radians `[-pi, pi]` |
| Measurement `range` | Radial distance | meters |
| Measurement `elevation` | Angle from XY plane toward +Z | radians `[-pi/2, pi/2]` |

## Configuration

```cpp
struct Config {
    ProcessNoise process_noise;   // q: accel variance (m²/s³)
    MeasurementNoise meas_noise;  // sigma_az, sigma_range, sigma_el
    double dt = 10.0;             // default prediction step
};
```

Typical radar values:
- `sigma_range`: 25–100 m
- `sigma_azimuth/elevation`: 0.1–1.0 degree (`0.0017–0.017 rad`)
- `process_noise.q`: 0.1–2.0 (higher for maneuvering targets)

## Architecture

```
include/radar_ekf/radar_ekf.hpp   # Public API (header-only implementation)
src/radar_ekf.cpp                 # Future out-of-line implementations
examples/example.cpp              # Synthetic trajectory demo
tests/test.cpp                    # Unit tests (Jacobian FD, EKF cycle, etc.)
```

## Extending

For multi-target tracking, wrap `RadarEKF` in a track manager that handles:
- Data association (GNN, JPDA, MHT)
- Track initiation/confirmation/deletion
- Gating (Mahalanobis distance on innovation)

See `multi-target-tracking` skill for Python reference implementation patterns.

## License

MIT