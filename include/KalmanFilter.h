#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>

class KF_Base {
public:
    virtual ~KF_Base() = default;
    
    KF_Base(int n, int m, const Eigen::MatrixXd& P = Eigen::MatrixXd(), 
            const Eigen::MatrixXd& Q = Eigen::MatrixXd(), 
            const Eigen::MatrixXd& R = Eigen::MatrixXd(), 
            const Eigen::VectorXd& init_x = Eigen::VectorXd());

    virtual Eigen::VectorXd update(const Eigen::VectorXd& z);
    virtual Eigen::VectorXd predict();
    
protected:
    virtual Eigen::VectorXd f(const Eigen::VectorXd& x) = 0;
    virtual Eigen::MatrixXd getF(const Eigen::VectorXd& x) = 0;
    virtual Eigen::VectorXd h(const Eigen::VectorXd& x) = 0;
    virtual Eigen::MatrixXd getH(const Eigen::VectorXd& x) = 0;

protected:
    int n_, m_;
    Eigen::MatrixXd Q_, R_, P_, I_;
    Eigen::VectorXd x_;
};

class KF_YAW : public KF_Base {
public:
    KF_YAW(double dt, const Eigen::MatrixXd& P = Eigen::MatrixXd(), 
           const Eigen::MatrixXd& Q = Eigen::MatrixXd(), 
           const Eigen::MatrixXd& R = Eigen::MatrixXd(), 
           const Eigen::VectorXd& init_x = Eigen::VectorXd());

protected:
    Eigen::VectorXd f(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getF(const Eigen::VectorXd& x) override;
    Eigen::VectorXd h(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getH(const Eigen::VectorXd& x) override;

private:
    double dt_;
    Eigen::Matrix2d JH_;
};

class KF_SIZE : public KF_Base {
public:
    KF_SIZE(double dt, const Eigen::MatrixXd& P = Eigen::MatrixXd(), 
            const Eigen::MatrixXd& Q = Eigen::MatrixXd(), 
            const Eigen::MatrixXd& R = Eigen::MatrixXd(), 
            const Eigen::VectorXd& init_x = Eigen::VectorXd());

protected:
    Eigen::VectorXd f(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getF(const Eigen::VectorXd& x) override;
    Eigen::VectorXd h(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getH(const Eigen::VectorXd& x) override;

private:
    double dt_;
    Eigen::Matrix<double, 2, 4> JH_;
};

class EKF_CV : public KF_Base {
public:
    EKF_CV(double dt, const Eigen::MatrixXd& P = Eigen::MatrixXd(), 
           const Eigen::MatrixXd& Q = Eigen::MatrixXd(), 
           const Eigen::MatrixXd& R = Eigen::MatrixXd(), 
           const Eigen::VectorXd& init_x = Eigen::VectorXd());

protected:
    Eigen::VectorXd f(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getF(const Eigen::VectorXd& x) override;
    Eigen::VectorXd h(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getH(const Eigen::VectorXd& x) override;

private:
    double dt_;
    Eigen::MatrixXd JH_;
};

class EKF_CA : public KF_Base {
public:
    EKF_CA()=default;
    EKF_CA(double dt, const Eigen::MatrixXd& P = Eigen::MatrixXd(), 
           const Eigen::MatrixXd& Q = Eigen::MatrixXd(), 
           const Eigen::MatrixXd& R = Eigen::MatrixXd(), 
           const Eigen::VectorXd& init_x = Eigen::VectorXd());

protected:
    Eigen::VectorXd f(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getF(const Eigen::VectorXd& x) override;
    Eigen::VectorXd h(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getH(const Eigen::VectorXd& x) override;

private:
    double dt_;
    Eigen::MatrixXd JH_;
};

class EKF_CTRA : public KF_Base {
public:
    EKF_CTRA()=default;
    EKF_CTRA(double dt, const Eigen::MatrixXd& P = Eigen::MatrixXd(), 
             const Eigen::MatrixXd& Q = Eigen::MatrixXd(), 
             const Eigen::MatrixXd& R = Eigen::MatrixXd(), 
             const Eigen::VectorXd& init_x = Eigen::VectorXd());

protected:
    Eigen::VectorXd f(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getF(const Eigen::VectorXd& x) override;
    Eigen::VectorXd h(const Eigen::VectorXd& x) override;
    Eigen::MatrixXd getH(const Eigen::VectorXd& x) override;

private:
    double dt_;
    Eigen::Matrix<double, 2, 6> JH_;
};

inline double norm_radian(double radians) {
    double n = std::floor(radians / (2.0 * M_PI));
    radians = radians - n * 2.0 * M_PI;
    if (radians > M_PI) {
        radians -= 2.0 * M_PI;
    }
    return radians;
}

inline double norm_relative_radian(double radians_diff) {
    if (radians_diff < -M_PI) {
        radians_diff += 2.0 * M_PI;
    } else if (radians_diff > M_PI) {
        radians_diff -= 2.0 * M_PI;
    }
    return radians_diff;
}

inline Eigen::VectorXd norm_radian(const Eigen::VectorXd& radians) {
    Eigen::VectorXd result(radians.size());
    for (int i = 0; i < radians.size(); ++i) {
        result[i] = norm_radian(radians[i]);
    }
    return result;
}

inline Eigen::VectorXd norm_relative_radian(const Eigen::VectorXd& radians_diff) {
    Eigen::VectorXd result(radians_diff.size());
    for (int i = 0; i < radians_diff.size(); ++i) {
        result[i] = norm_relative_radian(radians_diff[i]);
    }
    return result;
}