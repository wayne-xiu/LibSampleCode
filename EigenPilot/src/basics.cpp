#include <Eigen/Dense>
#include <iostream>

using namespace std;

const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

int rotation_x(double* tm, double angle) {
    tm[0] = 1.0;
    tm[1] = 0.;
    tm[2] = 0.;
    tm[3] = 0.;
    tm[4] = cos(angle);
    tm[5] = -sin(angle);
    tm[6] = 0.;
    tm[7] = sin(angle);
    tm[8] = cos(angle);
    return 0;
}

int rotation_y(double* tm, double angle) {
    tm[0] = cos(angle);
    tm[1] = 0.;
    tm[2] = sin(angle);
    tm[3] = 0.;
    tm[4] = 1.;
    tm[5] = 0.;
    tm[6] = -sin(angle);
    tm[7] = 0.;
    tm[8] = cos(angle);
    return 0;
}

int rotation_z(double* tm, double angle) {
    tm[0] = cos(angle);
    tm[1] = -sin(angle);
    tm[2] = 0.;
    tm[3] = sin(angle);
    tm[4] = cos(angle);
    tm[5] = 0.;
    tm[6] = 0.;
    tm[7] = 0.;
    tm[8] = 1.;
    return 0;
}

void MatrixMultiply(const double* m1, const double* m2, int m, int n, int k,
                    double* result) {
    int i, j, l, u;

    for (i = 0; i <= m - 1; ++i)
        for (j = 0; j <= k - 1; ++j) {
            u         = i * k + j;
            result[u] = 0.0;
            for (l = 0; l <= n - 1; ++l)
                result[u] = result[u] + m1[i * n + l] * m2[l * k + j];
        }
    return;
}

void EulerXYZ2R(double* RPY, double* R, bool useDeg = true) {
    double rotx[9], roty[9], rotz[9];
    double temp[9];
    double roll = RPY[0], pitch = RPY[1], yaw = RPY[2];
    if (useDeg) {
        roll *= DEG2RAD;
        pitch *= DEG2RAD;
        yaw *= DEG2RAD;
    }
    rotation_x(rotx, roll);
    rotation_y(roty, pitch);
    rotation_z(rotz, yaw);
    // R = Rx * Ry * Rz
    MatrixMultiply(rotx, roty, 3, 3, 3, temp);
    MatrixMultiply(temp, rotz, 3, 3, 3, R);
}

void EulerZYX2R(double* RPY, double* R, bool useDeg = true) {
    double rotx[9], roty[9], rotz[9];
    double temp[9];
    double roll = RPY[0], pitch = RPY[1], yaw = RPY[2];
    if (useDeg) {
        roll *= DEG2RAD;
        pitch *= DEG2RAD;
        yaw *= DEG2RAD;
    }
    rotation_z(rotz, roll);
    rotation_y(roty, pitch);
    rotation_x(rotx, yaw);
    // R = Rx * Ry * Rz
    MatrixMultiply(rotz, roty, 3, 3, 3, temp);
    MatrixMultiply(temp, rotx, 3, 3, 3, R);
}

void printArray(double* array, int size) {
    for (auto i = 0; i < size; ++i) {
        cout << array[i] << " ";
    }
    cout << endl;
}

int main() {
    using Real = double;
    // typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<Real, 3, 3> Matrix33;
    using Mat3 = Eigen::Matrix<double, 3, 3>;

    typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Matrix<Real, 3, 1>              Vector3;
    using Eigen::Matrix3f;
    using Eigen::Matrix4d;
    using Eigen::MatrixXd;

    typedef Eigen::AngleAxis<Real> AngleAxis;

    typedef Eigen::Transform<Real, 3, Eigen::Affine> Transform;
    // typedef Eigen::Translation<Real, 3> Translation;
    typedef Eigen::Quaternion<Real> Quaternion;

    Vector a(3); // dynamic size vector
    a << 1.0f, 2.0f, 3.0f;
    Real    b = 2.0f;
    Vector3 c = a * b;
    cout << c << endl << endl;

    Eigen::Matrix<float, 3, 3> matrixA;
    matrixA.setZero();
    cout << matrixA << endl;
    Matrix3f matrixA1;
    matrixA1.setZero();
    cout << "\n" << matrixA1 << endl;

    Mat3 matrixB;
    matrixB.setIdentity();
    cout << "\n" << matrixB << endl;

    // define a dynamic matrix
    MatrixXd matrixC1;

    MatrixXd matrixC3(2, 2);
    matrixC3(0, 0) = 1;
    matrixC3(0, 1) = 2;
    matrixC3(1, 0) = 3;
    matrixC3(1, 1) = 4;
    cout << "\n" << matrixC3 << endl;

    // constructor, allocate memory but do not initialize
    MatrixXd matrixC2(4, 4);
    matrixC2 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
    cout << "\n" << matrixC2 << endl;

    int rowNumber = 5;
    int colNumber = 5;
    matrixC1.setOnes(rowNumber, colNumber);
    cout << "\n" << matrixC1 << endl;

    MatrixXd matrixC4;
    matrixC4 = MatrixXd::Zero(rowNumber, colNumber);
    cout << "\n" << matrixC4 << endl;

    // block operation
    MatrixXd matrixpartial = matrixC2.block(0, 0, 2, 2);
    cout << "\n" << matrixpartial << endl;

    // accessing columns and rows of a matrix
    cout << "Row 1 of matrixC2: \n" << matrixC2.row(0) << endl;
    cout << "Column 2 of matrixC2: \n" << matrixC2.col(1) << endl;
    // diagonal matrix
    Vector3 vec1;
    vec1 << 1, 2, 3;
    MatrixXd diagMatrix = vec1.asDiagonal();
    cout << "\n" << diagMatrix << endl;
    cout << "matrix size: " << diagMatrix.size() << endl;
    cout << "matrix size: " << diagMatrix.rows() << "x" << diagMatrix.cols()
         << endl;

    // Matrix transpose
    MatrixXd matrixD1;
    matrixD1 = matrixC2.transpose();
    cout << "transpose: \n" << matrixC2 << "\n\n" << matrixD1 << endl;
    // matrixD1 = matrixD1.transposeInPlace(); won't work
    matrixD1.transposeInPlace();
    cout << "transpose in place: \n" << matrixD1 << endl;

    // Matrix inverse
    MatrixXd                   matrixD2;
    Eigen::FullPivLU<Matrix4d> lu(matrixD1);
    cout << "matrix rank:\n" << lu.rank() << endl;

    MatrixXd                   matrixD4 = MatrixXd::Random(4, 4);
    Eigen::FullPivLU<Matrix4d> lu2(matrixD4);
    cout << "matrix rank:" << lu2.rank();
    matrixD2 = matrixD4.inverse();
    cout << "\ninverse:\n " << matrixD2;
    cout << "\n getting I:\n " << matrixD4 * matrixD2 << endl;

    // 3D rotations, quaternions, transformations
    Matrix33 r(AngleAxis(45.0f * DEG2RAD, Vector3::UnitZ()) *
               AngleAxis(30.0f * DEG2RAD, Vector3::UnitY()) *
               AngleAxis(60.0f * DEG2RAD, Vector3::UnitX()));
    double   rpy[3] = {45, 30, 60};
    double   ROTATION[9];
    EulerZYX2R(rpy, ROTATION);
    cout << "manual wrote rotation matrix: " << endl;
    printArray(ROTATION, 9);
    cout << "original rotation matrix:\n" << r << endl;
    Transform t(r);
    // cout << "transformation matrix:\n" << t << endl;  // t couldn't be
    // printed directly
    Vector3 euler_xyz =
        t.rotation()
            .eulerAngles(2, 1, 0)
            .reverse(); // didn't know reverse() can be used this way
    cout << "Euler angles: xyz\n" << euler_xyz << endl;
    Vector3 txyz = t.translation();
    cout << "translation part:\n" << txyz << endl;
    Quaternion q(r);
    cout << "Quaternion: \n" << q << endl;
    r = q.toRotationMatrix();
    cout << "transformed rotation matrix:\n" << r << endl;

    return 0;
}
