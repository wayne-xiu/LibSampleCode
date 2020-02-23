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

// Read a bundle Adjustment in the Large dataset


int main(int argc, char** argv) {


    return 0;
}
