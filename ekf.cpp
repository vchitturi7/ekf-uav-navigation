#include "ekf.h"
#include <cmath>

Vec5 state_transition(const Vec5& x, double T, double dt, double m, double g)
{
    double px    = x(0);
    double pz    = x(1);
    double vx    = x(2);
    double vz    = x(3);
    double theta = x(4);

    double ax = (T / m) * std::sin(theta);
    double az = (T / m) * std::cos(theta) - g;

    Vec5 x_new;
    x_new << px + vx * dt,
             pz + vz * dt,
             vx + ax * dt,
             vz + az * dt,
             theta;
    return x_new;
}

Mat5 compute_jacobian(const Vec5& x, double T, double dt, double m)
{
    double theta   = x(4);
    double T_over_m = T / m;

    Mat5 F;
    F << 1, 0, dt, 0,  0,
         0, 1, 0,  dt, 0,
         0, 0, 1,  0,  T_over_m * std::cos(theta) * dt,
         0, 0, 0,  1, -T_over_m * std::sin(theta) * dt,
         0, 0, 0,  0,  1;
    return F;
}

void ekf_step(Vec5& x_est, Mat5& P,
              double T, const Vec2& z,
              double dt, double m, double g,
              const Mat5& Q, const Mat2& R, const Mat25& H)
{
    // PREDICT
    Vec5 x_pred = state_transition(x_est, T, dt, m, g);
    Mat5 F      = compute_jacobian(x_est, T, dt, m);
    Mat5 P_pred = F * P * F.transpose() + Q;

    // UPDATE
    Vec2 y = z - H * x_pred;                          // innovation
    Mat2 S = H * P_pred * H.transpose() + R;          // innovation covariance
    Eigen::Matrix<double, 5, 2> K = P_pred * H.transpose() * S.inverse(); // Kalman gain

    x_est = x_pred + K * y;
    P     = (Mat5::Identity() - K * H) * P_pred;
}
