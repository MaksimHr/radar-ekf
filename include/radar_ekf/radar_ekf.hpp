#ifndef RADAR_EKF_RADAR_EKF_HPP
#define RADAR_EKF_RADAR_EKF_HPP

#include <Eigen/Dense>
#include <optional>
#include <cmath>

namespace radar_ekf {

// State: [x, y, z, vx, vy, vz]^T (Cartesian position + velocity)
using State = Eigen::Matrix<double, 6, 1>;
using Covariance = Eigen::Matrix<double, 6, 6>;

// Measurement: [azimuth, range, elevation]^T (polar coordinates, radians/meters)
using Measurement = Eigen::Matrix<double, 3, 1>;
using MeasCovariance = Eigen::Matrix<double, 3, 3>;

/// Continuous white-noise acceleration process noise model
struct ProcessNoise {
    double q = 0.5;  // acceleration variance (m^2/s^3) per axis

    /// Discrete-time process noise covariance Q for time step dt
    [[nodiscard]] Covariance Q(double dt) const noexcept {
        Covariance Q = Covariance::Zero();
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        for (int i = 0; i < 3; ++i) {
            Q(i, i)     = q * dt3 / 3.0;
            Q(i, i + 3) = q * dt2 / 2.0;
            Q(i + 3, i) = q * dt2 / 2.0;
            Q(i + 3, i + 3) = q * dt;
        }
        return Q;
    }
};

/// Radar measurement noise (independent per dimension)
struct MeasurementNoise {
    double sigma_azimuth = 0.5 * M_PI / 180.0;  // rad
    double sigma_range   = 50.0;                 // meters
    double sigma_elevation = 0.5 * M_PI / 180.0; // rad

    [[nodiscard]] MeasCovariance R() const noexcept {
        MeasCovariance R = MeasCovariance::Zero();
        R(0, 0) = sigma_azimuth * sigma_azimuth;
        R(1, 1) = sigma_range * sigma_range;
        R(2, 2) = sigma_elevation * sigma_elevation;
        return R;
    }
};

/// State transition matrix for constant velocity model
[[nodiscard]] inline Eigen::Matrix<double, 6, 6> transitionMatrix(double dt) noexcept {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = F(1, 4) = F(2, 5) = dt;
    return F;
}

/// Convert Cartesian state (x,y,z) to polar measurement (az, range, el)
[[nodiscard]] inline Measurement h(const State& x) noexcept {
    double px = x(0), py = x(1), pz = x(2);
    double range = std::sqrt(px*px + py*py + pz*pz);
    Measurement z;
    if (range > 1e-9) {
        z(0) = std::atan2(py, px);              // azimuth
        z(1) = range;                            // range
        z(2) = std::asin(pz / range);            // elevation
    } else {
        z.setZero();
    }
    return z;
}

/// Jacobian of h(x) w.r.t. state [x,y,z,vx,vy,vz]
[[nodiscard]] inline Eigen::Matrix<double, 3, 6> jacobianH(const State& x) noexcept {
    double px = x(0), py = x(1), pz = x(2);
    double range2 = px*px + py*py + pz*pz;
    double range = std::sqrt(range2);
    double range3 = range2 * range;
    double xy2 = px*px + py*py;
    double xy = std::sqrt(xy2);

    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();

    if (range > 1e-9 && xy > 1e-9) {
        // dh/dx, dh/dy, dh/dz for azimuth
        H(0, 0) = -py / xy2;
        H(0, 1) =  px / xy2;
        H(0, 2) =  0.0;

        // dh/dx, dh/dy, dh/dz for range
        H(1, 0) = px / range;
        H(1, 1) = py / range;
        H(1, 2) = pz / range;

        // dh/dx, dh/dy, dh/dz for elevation
        H(2, 0) = -px * pz / (range2 * xy);
        H(2, 1) = -py * pz / (range2 * xy);
        H(2, 2) =  xy / range2;
    }
    // Velocity derivatives are zero (measurement doesn't depend on velocity)
    return H;
}

/// Convert polar measurement to Cartesian position (for initialization)
[[nodiscard]] inline Eigen::Vector3d polarToCartesian(const Measurement& z) noexcept {
    double az = z(0), r = z(1), el = z(2);
    double ce = std::cos(el);
    return Eigen::Vector3d(r * ce * std::cos(az),
                           r * ce * std::sin(az),
                           r * std::sin(el));
}

/// Extended Kalman Filter for radar tracking (polar measurements -> Cartesian state)
class RadarEKF {
public:
    struct Config {
        ProcessNoise process_noise;
        MeasurementNoise meas_noise;
        double dt = 10.0;  // default radar period (seconds)

        static Config defaults() { return Config{}; }
    };

    explicit RadarEKF(Config cfg = Config::defaults()) : cfg_(cfg), initialized_(false) {}

    /// Initialize from first polar measurement (az, range, el)
    void init(const Measurement& z, double timestamp) {
        Eigen::Vector3d pos = polarToCartesian(z);
        x_ << pos(0), pos(1), pos(2), 0.0, 0.0, 0.0;
        // Generous initial covariance: position ~1km, velocity ~100m/s
        P_ = Covariance::Zero();
        P_.diagonal() << 1e6, 1e6, 1e6, 1e4, 1e4, 1e4;
        last_timestamp_ = timestamp;
        initialized_ = true;
    }

    /// Predict step to given timestamp
    void predict(double timestamp) {
        if (!initialized_) return;
        double dt = timestamp - last_timestamp_;
        if (dt <= 0) return;

        Eigen::Matrix<double, 6, 6> F = transitionMatrix(dt);
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + cfg_.process_noise.Q(dt);
        last_timestamp_ = timestamp;
    }

    /// Update step with polar measurement
    void update(const Measurement& z) {
        if (!initialized_) return;

        Eigen::Matrix<double, 3, 6> H = jacobianH(x_);
        Measurement z_pred = h(x_);
        MeasCovariance R = cfg_.meas_noise.R();

        // Innovation
        Measurement y = z - z_pred;
        // Normalize azimuth to [-pi, pi]
        y(0) = std::atan2(std::sin(y(0)), std::cos(y(0)));
        // Normalize elevation to [-pi/2, pi/2]
        y(2) = std::max(-M_PI/2, std::min(M_PI/2, y(2)));

        Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Covariance::Identity() - K * H) * P_;
    }

    /// Full predict+update cycle (convenience)
    void step(const Measurement& z, double timestamp) {
        if (!initialized_) {
            init(z, timestamp);
            return;
        }
        predict(timestamp);
        update(z);
    }

    [[nodiscard]] bool isInitialized() const noexcept { return initialized_; }
    [[nodiscard]] const State& state() const noexcept { return x_; }
    [[nodiscard]] const Covariance& covariance() const noexcept { return P_; }
    [[nodiscard]] double lastTimestamp() const noexcept { return last_timestamp_; }

    /// Get Cartesian position
    [[nodiscard]] Eigen::Vector3d position() const noexcept { return x_.head<3>(); }
    /// Get Cartesian velocity
    [[nodiscard]] Eigen::Vector3d velocity() const noexcept { return x_.tail<3>(); }
    /// Get predicted polar measurement
    [[nodiscard]] Measurement predictedMeasurement() const noexcept { return h(x_); }

private:
    Config cfg_;
    State x_;
    Covariance P_;
    double last_timestamp_ = 0.0;
    bool initialized_ = false;
};

}  // namespace radar_ekf

#endif  // RADAR_EKF_RADAR_EKF_HPP