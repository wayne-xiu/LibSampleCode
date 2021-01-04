// Bundle Adjustment
//Given a set of measured image feature locations and correspondences, the goal of bundle adjustment
//is to find 3D point positions and camera parameters that minimize the reprojection error. This optimization
//problem is usually formulated as a non-linear least squares problem, where the error is the squared L2
//norm of the difference between the observed feature location and the projection of the corresponding 3D
//point on the image plane of the camera. Ceres has extensive support for solving bundle adjustment problems.

//Let us solve a problem from the BAL dataset.
//http://grail.cs.washington.edu/projects/bal/

//Each residual in a BAL problem depends on a three dimensional point and a nine parameter camera.
//The nine parameters defining the camera are: three for rotation as a Rodriques’ axis-angle vector,
//three for translation, one for focal length and two for radial distortion.

// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <chrono>

using namespace std;

// Read a bundle Adjustment in the Large dataset
class BALProblem {
public:
    ~BALProblem() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    int num_observations() const {
        return num_observations_;
    }
    const double* observations() const {
        return observations_;
    }
    double* mutable_cameras() {
        return parameters_;
    }
    double* mutable_points() {
        return parameters_ + 9*num_cameras_;
    }
    double* mutable_camera_for_observation(int i) {
        return mutable_cameras() + camera_index_[i]*9;
    }
    double* mutable_point_for_observation(int i) {
        return mutable_points() + point_index_[i]*3;
    }

    bool LoadFile(const char* filename) {
        FILE* fptr = fopen(filename, "r");
        if (fptr == nullptr)
            return false;

        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);

        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2*num_observations_];

        num_parameters_ = 9*num_cameras_ + 3*num_points_;
        parameters_ = new double[num_parameters_];

        for (int i = 0; i < num_observations_; ++i) {
            FscanfOrDie(fptr, "%d", camera_index_+i);
            FscanfOrDie(fptr, "%d", point_index_+i);
            for (int j = 0; j < 2; ++j) {
                FscanfOrDie(fptr, "%lf", observations_+2*i+j);
            }
        }
        for (int i = 0; i < num_parameters_; ++i) {
            FscanfOrDie(fptr, "%lf", parameters_+i);
        }

        return true;
    }

private:
    template<typename T>
    void FscanfOrDie(FILE* fptr, const char* format, T* value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            LOG(FATAL) << "Invalid UW data file.";
        }
    }

    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
};

// Templated pinhole camera model for used with Ceres. The camera is parameterized
// using 9 parameters: 3 for rotation, 3 for translation, 1 for focal length and 2
// for radial distortion. The principal point is not modeled (i.e. it is assumed to
// be located at the image center)
struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}

    template<typename T>
    bool operator() (const T* const camera, const T* const point, T* residual) const {
        // camera[0, 1, 2] are the angle-axis rotation
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3, 4, 5] are the translation
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis
        T xp = -p[0]/p[2];
        T yp = -p[1]/p[2];

        // Apply second and fourth order radial distortion
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp*xp + yp*yp;
        T distortion = 1.0 + r2*(l1 + l2*r2);

        // Compute final projected point position
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicated and observed position
        residual[0] = predicted_x - observed_x;
        residual[1] = predicted_y - observed_y;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                    new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);

    if (argc != 2) {
        cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
        return 1;
    }

    BALProblem bal_problem;
    if (!bal_problem.LoadFile(argv[1])) {
        cerr << "ERROR: unable to open file " << argv[1] << "\n";
        return 1;
    }

    const double* observations = bal_problem.observations();

    // Create residuals for each observation in the bundle adjustment problem.
    // The parameters for cameras and points are added automatically
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        // Each residual block takes a point and a camera as input and outputs a 2
        // dimensional residual.Internally, the cost function stores the observed image
        // location and compares the reprojection against the observation
        ceres::CostFunction* cost_function =
                SnavelyReprojectionError::Create(observations[2*i+0], observations[2*i+1]);
        problem.AddResidualBlock(cost_function, nullptr,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }

    // Make Ceres automatically detect the bundle structure. Note that the standard sovler
    // SPARE_NORMAL_CHOLESKY, also works fine but it is slower for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "time cost: " << time_used.count() << endl;

    cout << summary.FullReport() << endl;

    return 0;
}
