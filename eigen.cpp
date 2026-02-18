#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Vector3d v1(1, 2, 3);
    Eigen::Vector3d v2(4, 5, 6);

    Eigen::Vector3d v_sum = v1 + v2;
    std::cout << "Sum: " << v_sum.transpose() << std::endl;

    Eigen::Matrix3d M;
    M << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

    Eigen::Vector3d result = M * v1;
    std::cout << "Matrix * vector: " << result.transpose() << std::endl;
}
