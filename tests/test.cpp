#include <radar_ekf/radar_ekf.hpp>
#include <iostream>
#include <cmath>
#include <cassert>

using namespace radar_ekf;
using namespace std;

// Tolerance for floating point comparisons
constexpr double EPS = 1e-9;
constexpr double TOL = 1e-6;

void test_polar_cartesian_roundtrip() {
    cout << "Testing polar <-> Cartesian roundtrip..." << endl;
    
    // Test various positions
    vector<Eigen::Vector3d> test_positions = {
        {1000, 0, 0},
        {0, 1000, 0},
        {0, 0, 1000},
        {1000, 1000, 1000},
        {-1000, 500, -200},
        {1, 2, 3}
    };

    for (const auto& pos : test_positions) {
        // Cartesian -> polar
        double range = pos.norm();
        double az = atan2(pos.y(), pos.x());
        double el = asin(pos.z() / range);
        
        Measurement z;
        z << az, range, el;
        
        // Polar -> Cartesian
        Eigen::Vector3d pos2 = polarToCartesian(z);
        
        assert((pos - pos2).norm() < TOL);
    }
    cout << "  PASSED" << endl;
}

void test_jacobian_finite_difference() {
    cout << "Testing Jacobian H against finite differences..." << endl;
    
    State x;
    x << 1000, 500, 1500, 10, -5, 2;
    
    Eigen::Matrix<double, 3, 6> H_analytic = jacobianH(x);
    
    // Finite difference check
    constexpr double h = 1e-6;
    Eigen::Matrix<double, 3, 6> H_numeric = Eigen::Matrix<double, 3, 6>::Zero();
    
    for (int j = 0; j < 6; ++j) {
        State x_plus = x, x_minus = x;
        x_plus(j) += h;
        x_minus(j) -= h;
        
        Measurement z_plus = radar_ekf::h(x_plus);
        Measurement z_minus = radar_ekf::h(x_minus);
        
        H_numeric.col(j) = (z_plus - z_minus) / (2 * h);
    }
    
    double max_diff = (H_analytic - H_numeric).cwiseAbs().maxCoeff();
    assert(max_diff < 1e-5);
    cout << "  PASSED (max diff: " << max_diff << ")" << endl;
}

void test_measurement_function() {
    cout << "Testing measurement function h(x)..." << endl;
    
    State x;
    x << 3000, 4000, 0, 0, 0, 0;  // 5km range, 53 deg azimuth, 0 elevation
    
    Measurement z = radar_ekf::h(x);
    
    assert(abs(z(0) - atan2(4000, 3000)) < EPS);
    assert(abs(z(1) - 5000.0) < EPS);
    assert(abs(z(2) - 0.0) < EPS);
    
    cout << "  PASSED" << endl;
}

void test_ekf_initialization() {
    cout << "Testing EKF initialization..." << endl;
    
    RadarEKF ekf;
    
    Measurement z;
    z << 0.5, 1000.0, 0.1;  // ~28 deg az, 1km range, ~5.7 deg el
    
    ekf.init(z, 0.0);
    
    assert(ekf.isInitialized());
    
    Eigen::Vector3d pos = ekf.position();
    double range = pos.norm();
    
    assert(abs(range - 1000.0) < 1.0);  // Position should be close to 1km
    assert(abs(pos.z() - 1000.0 * sin(0.1)) < 1.0);
    
    cout << "  PASSED" << endl;
}

void test_ekf_predict() {
    cout << "Testing EKF predict step..." << endl;
    
    RadarEKF ekf;
    Measurement z;
    z << 0.0, 1000.0, 0.0;
    ekf.init(z, 0.0);
    
    Eigen::Vector3d pos0 = ekf.position();
    Eigen::Vector3d vel0 = ekf.velocity();
    
    ekf.predict(10.0);  // 10 second step
    
    Eigen::Vector3d pos1 = ekf.position();
    Eigen::Vector3d vel1 = ekf.velocity();
    
    // With zero velocity, position should not change
    assert((pos1 - pos0).norm() < 1e-9);
    assert((vel1 - vel0).norm() < 1e-9);
    
    // With non-zero velocity
    ekf = RadarEKF();
    z << 0.0, 1000.0, 0.0;
    ekf.init(z, 0.0);
    
    // Manually set velocity
    State x = ekf.state();
    x(3) = 100.0;  // vx = 100 m/s
    // Can't directly set state, but we can check covariance grows
    ekf.predict(10.0);
    
    // Covariance should increase due to process noise
    double trace0 = ekf.covariance().trace();
    ekf.predict(20.0);
    double trace1 = ekf.covariance().trace();
    
    assert(trace1 > trace0);
    
    cout << "  PASSED" << endl;
}

void test_ekf_update() {
    cout << "Testing EKF update step..." << endl;
    
    RadarEKF ekf;
    
    // Initialize at origin
    Measurement z0;
    z0 << 0.0, 1000.0, 0.0;
    ekf.init(z0, 0.0);
    
    // Update with same measurement (should stay close)
    ekf.update(z0);
    
    Eigen::Vector3d pos = ekf.position();
    assert(abs(pos.norm() - 1000.0) < 10.0);  // Should stay near 1km
    
    cout << "  PASSED" << endl;
}

void test_ekf_full_cycle() {
    cout << "Testing EKF full predict+update cycle..." << endl;
    
    RadarEKF::Config cfg;
    cfg.dt = 1.0;
    cfg.process_noise.q = 0.1;
    cfg.meas_noise.sigma_range = 1.0;
    cfg.meas_noise.sigma_azimuth = 0.001;
    cfg.meas_noise.sigma_elevation = 0.001;
    
    RadarEKF ekf(cfg);
    
    // Simulate target moving at constant velocity
    double x_true = 1000.0, y_true = 0.0, z_true = 0.0;
    double vx_true = 10.0, vy_true = 0.0, vz_true = 0.0;
    double timestamp = 0.0;
    
    for (int i = 0; i < 10; ++i) {
        // True state
        x_true += vx_true * cfg.dt;
        y_true += vy_true * cfg.dt;
        z_true += vz_true * cfg.dt;
        
        // Measurement
        double range = sqrt(x_true*x_true + y_true*y_true + z_true*z_true);
        double az = atan2(y_true, x_true);
        double el = asin(z_true / range);
        
        Measurement z;
        z << az, range, el;
        
        ekf.step(z, timestamp);
        
        timestamp += cfg.dt;
    }
    
    // After 10 steps, estimate should be close to true state
    Eigen::Vector3d est_pos = ekf.position();
    Eigen::Vector3d est_vel = ekf.velocity();
    
    assert(abs(est_pos.x() - x_true) < 5.0);
    assert(abs(est_vel.x() - vx_true) < 2.0);
    
    cout << "  PASSED" << endl;
}

void test_transition_matrix() {
    cout << "Testing transition matrix F..." << endl;
    
    double dt = 2.5;
    auto F = transitionMatrix(dt);
    
    assert(F(0,0) == 1.0 && F(0,3) == dt);
    assert(F(1,1) == 1.0 && F(1,4) == dt);
    assert(F(2,2) == 1.0 && F(2,5) == dt);
    assert(F(3,3) == 1.0 && F(4,4) == 1.0 && F(5,5) == 1.0);
    
    // Check it's identity + dt on position-velocity coupling
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    I(0,3) = I(1,4) = I(2,5) = dt;
    
    assert((F - I).norm() < EPS);
    
    cout << "  PASSED" << endl;
}

void test_process_noise() {
    cout << "Testing process noise Q matrix..." << endl;
    
    ProcessNoise pn;
    pn.q = 1.0;
    double dt = 2.0;
    
    auto Q = pn.Q(dt);
    
    // Check structure
    assert(abs(Q(0,0) - dt*dt*dt/3.0) < EPS);
    assert(abs(Q(0,3) - dt*dt/2.0) < EPS);
    assert(abs(Q(3,0) - dt*dt/2.0) < EPS);
    assert(abs(Q(3,3) - dt) < EPS);
    
    // Check symmetry
    assert((Q - Q.transpose()).norm() < EPS);
    
    // Check positive semi-definite (all eigenvalues >= 0)
    Eigen::SelfAdjointEigenSolver<Covariance> eig(Q);
    assert((eig.eigenvalues().array() >= -1e-10).all());
    
    cout << "  PASSED" << endl;
}

int main() {
    cout << "=== Radar EKF Unit Tests ===" << endl << endl;
    
    test_polar_cartesian_roundtrip();
    test_measurement_function();
    test_jacobian_finite_difference();
    test_transition_matrix();
    test_process_noise();
    test_ekf_initialization();
    test_ekf_predict();
    test_ekf_update();
    test_ekf_full_cycle();
    
    cout << endl << "=== ALL TESTS PASSED ===" << endl;
    return 0;
}