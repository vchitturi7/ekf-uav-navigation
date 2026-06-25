#pragma once
#include <Eigen/Dense>

using Vec5 = Eigen::Matrix<double, 5, 1>;
using Mat5 = Eigen::Matrix<double, 5, 5>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat25 = Eigen::Matrix<double, 2, 5>;

// Propagates state one timestep using drone dynamics
Vec5 state_transition(const Vec5& x, double T, double dt, double m, double g);

// Computes Jacobian of state transition (F matrix)
Mat5 compute_jacobian(const Vec5& x, double T, double dt, double m);

// Runs one full EKF predict + update cycle
void ekf_step(Vec5& x_est, Mat5& P,
              double T, const Vec2& z,
              double dt, double m, double g,
              const Mat5& Q, const Mat2& R, const Mat25& H);
