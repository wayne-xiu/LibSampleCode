#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

// quaternion demo using Eigen

// Spherical linear interpolation (Slerp) of two quaternions
Eigen::Quaterniond slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2,
                         double t) {
    double theta     = std::acos(q1.dot(q2));
    double sin_theta = std::sin(theta);
    double alpha     = std::sin((1.0 - t) * theta) / sin_theta;
    double beta      = std::sin(t * theta) / sin_theta;

    Eigen::Quaterniond qt =
        Eigen::Quaterniond(alpha * q1.coeffs() + beta * q2.coeffs());
    return qt;
    // return q1.slerp(t, q2);
}

void testQuaternion() {
    // Quaternion object reprsenting a rotation of 45 degrees around the z-axis
    Eigen::Quaterniond q1(
        Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d v1(1, 0, 0);

    Eigen::Vector3d rotated_v1 = q1 * v1;

    std::cout << "original vector: (" << v1.x() << ", " << v1.y() << ", "
              << v1.z() << ")" << std::endl;
    std::cout << "rotated vector: (" << rotated_v1.x() << ", " << rotated_v1.y()
              << ", " << rotated_v1.z() << ")" << std::endl;
}

void testQuaternionSlerp() {
    Eigen::Quaterniond q1(0.858921, 0.509339, 0.019188, 0.049596);
    Eigen::Quaterniond q2(0.858905, 0.509443, 0.018806, 0.048944);

    auto ts1 = 700901879318945, ts2 = 700901884127851;
    auto ts_image = 700901880170406;

    double t = (ts_image - ts1) / (ts2 - ts1);

    Eigen::Quaterniond qt = slerp(q1, q2, t);

    std::cout << "qt interpolation: \n" << qt.coeffs() << std::endl;
}

int main() {
    testQuaternion();
    testQuaternionSlerp();

    return EXIT_SUCCESS;
}