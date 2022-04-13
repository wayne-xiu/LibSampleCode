#include <Eigen/Core>
#include <ceres/ceres.h>

using namespace std;

struct CostFunctor {
//    F(x) = 1/2*(10-x)^2
    template <typename T>
    bool operator() (const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);

    double initial_x = 5.0;
    double x = initial_x;

    ceres::Problem problem;

    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;
    cout << "x: " << initial_x << " -> " << x << endl;


    return 0;
}
