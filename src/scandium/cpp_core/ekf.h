#pragma once
#include <vector>
#include <stdexcept>
#include <cmath>

namespace scandium {
namespace cpp_core {

// A minimal Matrix struct for our 6x6 and 6x3 EKF operations to keep zero dependencies
struct Matrix {
    int rows, cols;
    std::vector<double> data;

    Matrix(int r, int c, double val = 0.0) : rows(r), cols(c), data(r * c, val) {}
    
    double& operator()(int r, int c) { return data[r * cols + c]; }
    const double& operator()(int r, int c) const { return data[r * cols + c]; }

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Matrix transpose() const;
    Matrix inverse3x3() const; // For S inverse
};

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(double dt, double process_noise, double measurement_noise);
    
    void predict();
    void update(const std::vector<double>& measurement);
    
    std::vector<double> get_state() const;
    std::vector<double> get_position() const;
    std::vector<double> get_velocity() const;
    
    void reset(const std::vector<double>& initial_state);

private:
    double dt_;
    Matrix x_; // State vector: [x, y, z, vx, vy, vz]^T
    Matrix P_; // Covariance matrix 6x6
    Matrix F_; // State transition matrix 6x6
    Matrix Q_; // Process noise covariance 6x6
    Matrix H_; // Measurement matrix 3x6
    Matrix R_; // Measurement noise covariance 3x3
    
    void init_matrices(double process_noise, double measurement_noise);
};

} // namespace cpp_core
} // namespace scandium
