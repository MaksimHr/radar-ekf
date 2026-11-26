#include <radar_ekf/radar_ekf.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace radar_ekf;

int main() {
    std::cout << "=== Radar EKF Demo: 3D Tracking from Polar Measurements ===" << std::endl << std::endl;

    // Configure tracker
    RadarEKF::Config cfg;
    cfg.dt = 10.0;  // 10-second radar period
    cfg.process_noise.q = 0.5;  // m^2/s^3
    cfg.meas_noise.sigma_azimuth = 0.5 * M_PI / 180.0;
    cfg.meas_noise.sigma_range = 50.0;
    cfg.meas_noise.sigma_elevation = 0.5 * M_PI / 180.0;

    RadarEKF ekf(cfg);

    // True trajectory: constant velocity target
    // Starts at (1000, 500, 1500) moving at (200, 0, 0) m/s
    Eigen::Vector3d true_pos(1000.0, 500.0, 1500.0);
    Eigen::Vector3d true_vel(200.0, 0.0, 0.0);

    double timestamp = 0.0;
    const int n_frames = 20;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Frame | True Pos (x,y,z)       | Est Pos (x,y,z)        | Error (m) | Est Vel (m/s)" << std::endl;
    std::cout << "------|------------------------|------------------------|-----------|--------------" << std::endl;

    for (int frame = 0; frame < n_frames; ++frame) {
        // True state at this timestamp
        true_pos += true_vel * cfg.dt;

        // Generate polar measurement with noise
        double range = true_pos.norm();
        double azimuth = std::atan2(true_pos.y(), true_pos.x());
        double elevation = std::asin(true_pos.z() / range);

        // Add measurement noise
        Measurement z;
        z(0) = azimuth + cfg.meas_noise.sigma_azimuth * (std::rand() / double(RAND_MAX) - 0.5) * 2.0;
        z(1) = range + cfg.meas_noise.sigma_range * (std::rand() / double(RAND_MAX) - 0.5) * 2.0;
        z(2) = elevation + cfg.meas_noise.sigma_elevation * (std::rand() / double(RAND_MAX) - 0.5) * 2.0;

        // EKF step
        ekf.step(z, timestamp);

        // Report
        Eigen::Vector3d est_pos = ekf.position();
        Eigen::Vector3d est_vel = ekf.velocity();
        double pos_error = (est_pos - true_pos).norm();

        std::cout << std::setw(5) << frame
                  << " | " << std::setw(7) << true_pos.x() << " " << std::setw(7) << true_pos.y() << " " << std::setw(7) << true_pos.z()
                  << " | " << std::setw(7) << est_pos.x() << " " << std::setw(7) << est_pos.y() << " " << std::setw(7) << est_pos.z()
                  << " | " << std::setw(9) << pos_error
                  << " | " << std::setw(7) << est_vel.x() << " " << std::setw(7) << est_vel.y() << " " << std::setw(7) << est_vel.z()
                  << std::endl;

        timestamp += cfg.dt;
    }

    std::cout << std::endl << "=== Final covariance (diagonal) ===" << std::endl;
    std::cout << "Pos var: " << ekf.covariance().diagonal().head<3>().transpose() << std::endl;
    std::cout << "Vel var: " << ekf.covariance().diagonal().tail<3>().transpose() << std::endl;

    return 0;
}