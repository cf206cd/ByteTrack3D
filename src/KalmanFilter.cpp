#include "KalmanFilter.h"
#include <stdexcept>

// KF_Base implementation
KF_Base::KF_Base(int n, int m, const Eigen::MatrixXd& P, 
                  const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                  const Eigen::VectorXd& init_x) 
    : n_(n), m_(m), Q_(Q), R_(R), I_(Eigen::MatrixXd::Identity(n, n)) {
    
    if (init_x.size() > 0) {
        if (init_x.size() != n) {
            throw std::runtime_error("Initial state dimension mismatch");
        }
        x_ = init_x;
    } else {
        x_ = Eigen::VectorXd::Zero(n);
    }
    
    if (P.size() > 0) {
        P_ = P;
    } else {
        P_ = Eigen::MatrixXd::Identity(n, n);
    }
}

Eigen::VectorXd KF_Base::update(const Eigen::VectorXd& z) {
    // Predict
    x_ = f(x_);
    Eigen::MatrixXd F = getF(x_);
    P_ = F * P_ * F.transpose() + Q_;
    
    // Update
    Eigen::MatrixXd H = getH(x_);
    Eigen::MatrixXd G = P_ * H.transpose() * (H * P_ * H.transpose() + R_).inverse();
    x_ += G * (z - h(x_));
    P_ = (I_ - G * H) * P_;
    
    return x_;
}

Eigen::VectorXd KF_Base::predict() {
    return f(x_);
}

// KF_YAW implementation
KF_YAW::KF_YAW(double dt, const Eigen::MatrixXd& P, 
                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                const Eigen::VectorXd& init_x) 
    : KF_Base(2, 2, P, Q, R, init_x), dt_(dt) {
    JH_ << 1.0, 0.0,
            0.0, 1.0;
}

Eigen::VectorXd KF_YAW::f(const Eigen::VectorXd& x) {
    Eigen::VectorXd x_fil = x;
    x_fil[0] = norm_radian(x[0] + dt_ * x[1]);
    return x_fil;
}

Eigen::MatrixXd KF_YAW::getF(const Eigen::VectorXd& x) {
    Eigen::Matrix2d F;
    F << 1.0, dt_,
         0.0, 1.0;
    return F;
}

Eigen::VectorXd KF_YAW::h(const Eigen::VectorXd& x) {
    return JH_ * x;
}

Eigen::MatrixXd KF_YAW::getH(const Eigen::VectorXd& x) {
    (void)x;
    return JH_;
}

// KF_SIZE implementation
KF_SIZE::KF_SIZE(double dt, const Eigen::MatrixXd& P, 
                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                const Eigen::VectorXd& init_x) 
    : KF_Base(4, 2, P, Q, R, init_x), dt_(dt) {
    JH_ << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0;
}

Eigen::VectorXd KF_SIZE::f(const Eigen::VectorXd& x) {
    Eigen::VectorXd x_fil = x;
    x_fil[0] = x[0] + dt_ * x[2];
    x_fil[1] = x[1] + dt_ * x[3];
    return x_fil;
}

Eigen::MatrixXd KF_SIZE::getF(const Eigen::VectorXd& x) {
    Eigen::Matrix4d F;
    F << 1.0, 0.0, dt_, 0.0,
         0.0, 1.0, 0.0, dt_,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    return F;
}

Eigen::VectorXd KF_SIZE::h(const Eigen::VectorXd& x) {
    return JH_ * x;
}

Eigen::MatrixXd KF_SIZE::getH(const Eigen::VectorXd& x) {
    (void)x;
    return JH_;
}

// EKF_CV implementation
EKF_CV::EKF_CV(double dt, const Eigen::MatrixXd& P, 
                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                const Eigen::VectorXd& init_x) 
    : KF_Base(4, 2, P, Q, R, init_x), dt_(dt) {
    JH_ = Eigen::MatrixXd::Identity(2, 4);
}

Eigen::VectorXd EKF_CV::f(const Eigen::VectorXd& x) {
    Eigen::VectorXd x_fil = x;
    x_fil[0] = x[0] + dt_ * x[2];
    x_fil[1] = x[1] + dt_ * x[3];
    return x_fil;
}

Eigen::MatrixXd EKF_CV::getF(const Eigen::VectorXd& x) {
    Eigen::Matrix4d F;
    F << 1.0, 0.0, dt_, 0.0,
         0.0, 1.0, 0.0, dt_,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    return F;
}

Eigen::VectorXd EKF_CV::h(const Eigen::VectorXd& x) {
    return JH_ * x;
}

Eigen::MatrixXd EKF_CV::getH(const Eigen::VectorXd& x) {
    (void)x;
    return JH_;
}

// EKF_CA implementation
EKF_CA::EKF_CA(double dt, const Eigen::MatrixXd& P, 
                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                const Eigen::VectorXd& init_x) 
    : KF_Base(6, 2, P, Q, R, init_x), dt_(dt) {
    JH_ = Eigen::MatrixXd::Identity(2, 6);
}

Eigen::VectorXd EKF_CA::f(const Eigen::VectorXd& x) {
    Eigen::VectorXd x_fil = x;
    x_fil[0] = x[0] + dt_ * x[2] + 0.5 * dt_ * dt_ * x[4];
    x_fil[1] = x[1] + dt_ * x[3] + 0.5 * dt_ * dt_ * x[5];
    x_fil[2] = x[2] + dt_ * x[4];
    x_fil[3] = x[3] + dt_ * x[5];
    return x_fil;
}

Eigen::MatrixXd EKF_CA::getF(const Eigen::VectorXd& x) {
    Eigen::Matrix<double, 6, 6> F;
    double dt = dt_;
    double at2 = 0.5 * dt * dt;
    F << 1.0, 0.0, dt, 0.0, at2, 0.0,
         0.0, 1.0, 0.0, dt, 0.0, at2,
         0.0, 0.0, 1.0, 0.0, dt, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, dt,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    return F;
}

Eigen::VectorXd EKF_CA::h(const Eigen::VectorXd& x) {
    return JH_ * x;
}

Eigen::MatrixXd EKF_CA::getH(const Eigen::VectorXd& x) {
    (void)x;
    return JH_;
}

// EKF_CTRA implementation
EKF_CTRA::EKF_CTRA(double dt, const Eigen::MatrixXd& P, 
                    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                    const Eigen::VectorXd& init_x) 
    : KF_Base(6, 2, P, Q, R, init_x), dt_(dt) {
    JH_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
}

Eigen::VectorXd EKF_CTRA::f(const Eigen::VectorXd& x) {
    Eigen::VectorXd x_fil = x;
    double v = x[3], w = x[4], a = x[5];
    double dt = dt_;
    
    if (std::abs(w) < 1e-6) {
        // Straight line motion
        x_fil[0] = x[0] + v * dt * std::cos(x[2]) + 0.5 * a * dt * dt * std::cos(x[2]);
        x_fil[1] = x[1] + v * dt * std::sin(x[2]) + 0.5 * a * dt * dt * std::sin(x[2]);
    } else {
        // Curved motion
        x_fil[0] = x[0] + (1.0 / (w * w)) * (
            (v * w + a * w * dt) * std::sin(x[2] + w * dt) + 
            a * std::cos(x[2] + w * dt) - v * w * std::sin(x[2]) - a * std::cos(x[2]));
        x_fil[1] = x[1] + (1.0 / (w * w)) * (
            (-v * w - a * w * dt) * std::cos(x[2] + w * dt) + 
            a * std::sin(x[2] + w * dt) + v * w * std::cos(x[2]) - a * std::sin(x[2]));
    }
    x_fil[2] = x[2] + w * dt;
    x_fil[3] = x[3] + a * dt;
    return x_fil;
}

Eigen::MatrixXd EKF_CTRA::getF(const Eigen::VectorXd& x) {
    Eigen::Matrix<double, 6, 6> F;
    double dt = dt_;
    double v = x[3], w = x[4], a = x[5];
    
    if (std::abs(w) < 1e-6) {
        // Straight line approximation
        F << 1.0, 0.0, -v * dt * std::sin(x[2]) - 0.5 * a * dt * dt * std::sin(x[2]), 
             dt * std::cos(x[2]), 0.0, 0.5 * dt * dt * std::cos(x[2]),
             0.0, 1.0, v * dt * std::cos(x[2]) + 0.5 * a * dt * dt * std::cos(x[2]), 
             dt * std::sin(x[2]), 0.0, 0.5 * dt * dt * std::sin(x[2]),
             0.0, 0.0, 1.0, 0.0, dt, 0.0,
             0.0, 0.0, 0.0, 1.0, 0.0, dt,
             0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    } else {
        // Curved motion Jacobian (simplified for stability)
        double sin_term = std::sin(x[2] + w * dt);
        double cos_term = std::cos(x[2] + w * dt);
        
        F << 1.0, 0.0, (v * std::cos(x[2] + w * dt) + a * dt * cos_term) / w, 
             (sin_term - std::sin(x[2])) / w, 0.0, 0.0,
             0.0, 1.0, (v * std::sin(x[2] + w * dt) + a * dt * sin_term) / w, 
             (std::cos(x[2]) - cos_term) / w, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, dt, 0.0,
             0.0, 0.0, 0.0, 1.0, 0.0, dt,
             0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }
    return F;
}

Eigen::VectorXd EKF_CTRA::h(const Eigen::VectorXd& x) {
    return JH_ * x;
}

Eigen::MatrixXd EKF_CTRA::getH(const Eigen::VectorXd& x) {
    (void)x;
    return JH_;
}