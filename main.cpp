#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include "ekf.h"

int main()
{
    // ---- Parameters (mirrors params.m) ----
    const double m  = 1.0;
    const double g  = 9.81;
    const double dt = 0.01;
    const int    N  = 1000;
    const double T_hover = m * g;

    const int dropout_start = 300;
    const int dropout_end   = 500;

    // Initial state [px, pz, vx, vz, theta]
    Vec5 x0;
    x0 << 0, 10, 0, 0, 0;

    // Measurement matrix H (2x5) — GPS measures px and pz
    Mat25 H;
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;

    // Process noise Q
    Mat5 Q = Mat5::Zero();
    Q(0,0) = 0.001;
    Q(1,1) = 0.001;
    Q(2,2) = 0.01;
    Q(3,3) = 0.01;
    Q(4,4) = 0.0001;

    // Measurement noise R
    Mat2 R = Mat2::Zero();
    R(0,0) = 2.0;
    R(1,1) = 2.0;

    // Initial covariance
    Mat5 P = Mat5::Identity() * 0.01;

    // ---- Random number generators ----
    std::default_random_engine rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);

    // ---- Initialize states ----
    Vec5 x_true = x0;
    Vec5 x_est  = x0;
    for (int i = 0; i < 5; i++)
        x_est(i) += noise(rng) * 0.1;

    // ---- CSV output ----
    std::ofstream csv("results.csv");
    csv << "t,true_px,true_pz,true_vx,true_vz,true_theta,"
        << "est_px,est_pz,est_vx,est_vz,est_theta,"
        << "gps_px,gps_pz,gps_active\n";

    // ---- Simulation loop ----
    for (int k = 1; k <= N; k++)
    {
        // Propagate true state with process noise
        x_true = state_transition(x_true, T_hover, dt, m, g);
        x_true(0) += noise(rng) * 0.01;
        x_true(1) += noise(rng) * 0.01;
        x_true(2) += noise(rng) * 0.05;
        x_true(3) += noise(rng) * 0.05;
        x_true(4) += noise(rng) * 0.001;

        // Noisy GPS measurement
        Vec2 z;
        z(0) = x_true(0) + noise(rng) * 2.0;
        z(1) = x_true(1) + noise(rng) * 2.0;

        bool gps_on = !(k >= dropout_start && k <= dropout_end);

        if (!gps_on)
        {
            // GPS dropout — predict only, no update
            Mat5 F  = compute_jacobian(x_est, T_hover, dt, m);
            x_est   = state_transition(x_est, T_hover, dt, m, g);
            P       = F * P * F.transpose() + Q;
        }
        else
        {
            ekf_step(x_est, P, T_hover, z, dt, m, g, Q, R, H);
        }

        double t = k * dt;
        csv << t << ","
            << x_true(0) << "," << x_true(1) << "," << x_true(2) << ","
            << x_true(3) << "," << x_true(4) << ","
            << x_est(0)  << "," << x_est(1)  << "," << x_est(2)  << ","
            << x_est(3)  << "," << x_est(4)  << ","
            << z(0) << "," << z(1) << ","
            << (gps_on ? 1 : 0) << "\n";

        // Print dropout boundaries to console
        if (k == dropout_start)
            std::cout << "t=" << t << "s  GPS DROPOUT START\n";
        if (k == dropout_end)
            std::cout << "t=" << t << "s  GPS RESTORED\n";
    }

    csv.close();
    std::cout << "Simulation complete. Results saved to results.csv\n";
    return 0;
}
