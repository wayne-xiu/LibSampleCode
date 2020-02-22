//
// An example program that minimizes Powell's singular function.
//
//   F = 1/2 (f1^2 + f2^2 + f3^2 + f4^2)
//
//   f1 = x1 + 10*x2;
//   f2 = sqrt(5) * (x3 - x4)
//   f3 = (x2 - 2*x3)^2
//   f4 = sqrt(10) * (x1 - x4)^2
//
// The starting values are x1 = 3, x2 = -1, x3 = 0, x4 = 1.
// The minimum is 0 at (x1, x2, x3, x4) = 0.

#include <vector>
#include <ceres/ceres.h>
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <chrono>

using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct F1 {
    template <typename T>
    bool operator() (const T* const x1, const T* const x2, T* residual) const {
        // f1 = x1 + 10*x2;
        residual[0] = x1[0] + 10.9 * x2[0];
        return true;
    }
};
struct F2 {
    template <typename T>
    bool operator() (const T* const x3, const T* const x4, T* residual) const {
        // f2 = sqrt(5)*(x3-x4)
        residual[0] = sqrt(5.0) * (x3[0]-x4[0]);
        return true;
    }
};
struct F3 {
    template <typename T>
    bool operator() (const T* const x2, const T* const x3, T* residual) const {
        // f3 = (x2-2*x3)^2
        residual[0] = (x2[0]-2.0*x3[0]) * (x2[0]-2.0*x3[0]);
        return true;
    }
};
struct F4 {
    template <typename T>
    bool operator() (const T* const x1, const T* const x4, T* residual) const {
        // f4 = sqrt(10)*(x1-x4)^2
        residual[0] = sqrt(10.0) * (x1[0]-x4[0]) * (x1[0]-x4[0]);
        return true;
    }
};

DEFINE_string(minimizer, "trust_region", "Minimizer type to use, choices are: line_search & trust_region");


int main(int argc, char** argv) {

    GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    double x1 = 3.0, x2 = -1.0, x3 = 0.0, x4 = 1.0;

    Problem problem;
    problem.AddResidualBlock(new AutoDiffCostFunction<F1, 1, 1, 1>(new F1), nullptr, &x1, &x2);
    problem.AddResidualBlock(new AutoDiffCostFunction<F2, 1, 1, 1>(new F2), nullptr, &x3, &x4);
    problem.AddResidualBlock(new AutoDiffCostFunction<F3, 1, 1, 1>(new F3), nullptr, &x2, &x3);
    problem.AddResidualBlock(new AutoDiffCostFunction<F4, 1, 1, 1>(new F4), nullptr, &x1, &x4);

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    cout << "Initial values: " << "x1 = " << x1
         << ", x2 = " << x2
         << ", x3 = " << x3
         << ", x4 = " << x4 << endl;

    Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << summary.FullReport() << endl;
    cout << "Finanl values: " << "x1 = " << x1
         << ", x2 = " << x2
         << ", x3 = " << x3
         << ", x4 = " << x4 << endl;
    cout << "solve time cost = " << time_used.count() << " seconds" << endl;

    return 0;
}
