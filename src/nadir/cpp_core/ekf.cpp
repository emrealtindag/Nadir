#include "ekf.h"

namespace nadir {
namespace cpp_core {

Matrix Matrix::operator+(const Matrix& other) const {
    Matrix res(rows, cols);
    for (size_t i = 0; i < data.size(); ++i) {
        res.data[i] = data[i] + other.data[i];
    }
    return res;
}

Matrix Matrix::operator-(const Matrix& other) const {
    Matrix res(rows, cols);
    for (size_t i = 0; i < data.size(); ++i) {
        res.data[i] = data[i] - other.data[i];
    }
    return res;
}

Matrix Matrix::operator*(const Matrix& other) const {
    if (cols != other.rows) throw std::invalid_argument("Dim mismatch in mult");
    Matrix res(rows, other.cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < other.cols; ++j) {
            double sum = 0;
            for (int k = 0; k < cols; ++k) {
                sum += (*this)(i, k) * other(k, j);
            }
            res(i, j) = sum;
        }
    }
    return res;
}

Matrix Matrix::transpose() const {
    Matrix res(cols, rows);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            res(j, i) = (*this)(i, j);
        }
    }
    return res;
}

// Simple hardcoded 3x3 inverse for S matrix
Matrix Matrix::inverse3x3() const {
    if (rows != 3 || cols != 3) throw std::invalid_argument("Must be 3x3");
    Matrix res(3, 3);
    double det = 
        (*this)(0,0) * ((*this)(1,1) * (*this)(2,2) - (*this)(2,1) * (*this)(1,2)) -
        (*this)(0,1) * ((*this)(1,0) * (*this)(2,2) - (*this)(1,2) * (*this)(2,0)) +
        (*this)(0,2) * ((*this)(1,0) * (*this)(2,1) - (*this)(1,1) * (*this)(2,0));
        
    if (std::abs(det) < 1e-9) throw std::runtime_error("Singular matrix");
    
    double invdet = 1.0 / det;
    res(0,0) = ((*this)(1,1) * (*this)(2,2) - (*this)(2,1) * (*this)(1,2)) * invdet;
    res(0,1) = ((*this)(0,2) * (*this)(2,1) - (*this)(0,1) * (*this)(2,2)) * invdet;
    res(0,2) = ((*this)(0,1) * (*this)(1,2) - (*this)(0,2) * (*this)(1,1)) * invdet;
    res(1,0) = ((*this)(1,2) * (*this)(2,0) - (*this)(1,0) * (*this)(2,2)) * invdet;
    res(1,1) = ((*this)(0,0) * (*this)(2,2) - (*this)(0,2) * (*this)(2,0)) * invdet;
    res(1,2) = ((*this)(1,0) * (*this)(0,2) - (*this)(0,0) * (*this)(1,2)) * invdet;
    res(2,0) = ((*this)(1,0) * (*this)(2,1) - (*this)(2,0) * (*this)(1,1)) * invdet;
    res(2,1) = ((*this)(2,0) * (*this)(0,1) - (*this)(0,0) * (*this)(2,1)) * invdet;
    res(2,2) = ((*this)(0,0) * (*this)(1,1) - (*this)(1,0) * (*this)(0,1)) * invdet;
    
    return res;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(double dt, double process_noise, double measurement_noise)
    : dt_(dt), x_(6, 1), P_(6, 6), F_(6, 6), Q_(6, 6), H_(3, 6), R_(3, 3) 
{
    init_matrices(process_noise, measurement_noise);
}

void ExtendedKalmanFilter::init_matrices(double q_std, double r_std) {
    // Identity initialization for P
    for(int i=0; i<6; i++) P_(i,i) = 1.0;
    
    // State transition F
    for(int i=0; i<6; i++) F_(i,i) = 1.0;
    F_(0,3) = dt_;
    F_(1,4) = dt_;
    F_(2,5) = dt_;
    
    // Process noise Q
    double dt2 = dt_*dt_;
    double dt3 = dt2*dt_ / 2.0;
    double dt4 = dt2*dt2 / 4.0;
    
    // Q uses constant velocity kinematic model assumption
    for(int i=0; i<3; i++) {
        Q_(i, i) = dt4 * q_std;
        Q_(i+3, i+3) = dt2 * q_std;
        Q_(i, i+3) = dt3 * q_std;
        Q_(i+3, i) = dt3 * q_std;
    }
    
    // Measurement H (we measure x,y,z directly)
    H_(0,0) = 1.0; H_(1,1) = 1.0; H_(2,2) = 1.0;
    
    // Measurement noise R
    R_(0,0) = r_std; R_(1,1) = r_std; R_(2,2) = r_std;
}

void ExtendedKalmanFilter::predict() {
    x_ = F_ * x_;
    P_ = (F_ * P_) * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::update(const std::vector<double>& measurement) {
    if (measurement.size() != 3) return;
    Matrix z(3, 1);
    z(0,0) = measurement[0]; z(1,0) = measurement[1]; z(2,0) = measurement[2];
    
    Matrix y = z - (H_ * x_); // Innovation
    Matrix S = (H_ * P_) * H_.transpose() + R_; // Innovation covariance
    
    Matrix K = P_ * H_.transpose() * S.inverse3x3(); // Kalman gain
    
    x_ = x_ + (K * y); // State update
    
    // P update: P = (I - K*H)*P
    Matrix I(6, 6);
    for(int i=0; i<6; i++) I(i,i) = 1.0;
    P_ = (I - (K * H_)) * P_;
}

void ExtendedKalmanFilter::reset(const std::vector<double>& initial_state) {
    for(int i=0; i<6; i++) {
        if(i < (int)initial_state.size()) x_(i,0) = initial_state[i];
        else x_(i,0) = 0.0;
    }
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P_(i,j) = (i==j) ? 1.0 : 0.0;
        }
    }
}

std::vector<double> ExtendedKalmanFilter::get_state() const {
    return {x_(0,0), x_(1,0), x_(2,0), x_(3,0), x_(4,0), x_(5,0)};
}

std::vector<double> ExtendedKalmanFilter::get_position() const {
    return {x_(0,0), x_(1,0), x_(2,0)};
}

std::vector<double> ExtendedKalmanFilter::get_velocity() const {
    return {x_(3,0), x_(4,0), x_(5,0)};
}

} // namespace cpp_core
} // namespace nadir
