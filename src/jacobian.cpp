#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;

// Efficient Jacobian computation using precomputed trigonometric expressions
MatrixXd computeJacobian(const std::vector<double>& theta, const std::vector<double>& dh_l) {
    double l1 = dh_l[0], l2 = dh_l[1], l3 = dh_l[2], l4 = dh_l[3], l5 = dh_l[4];

    double t1 = theta[0] + M_PI / 2;
    double t2 = theta[1];
    double t3 = theta[2];
    double t4 = theta[3];
    double t5 = theta[4];
    double t6 = theta[5];

    // Trigonometric precomputation
    double s1 = std::sin(t1), c1 = std::cos(t1);
    double s2 = std::sin(t2), c2 = std::cos(t2);
    double s3 = std::sin(t3), c3 = std::cos(t3);
    double s4 = std::sin(t4), c4 = std::cos(t4);
    double s5 = std::sin(t5), c5 = std::cos(t5);
    double s6 = std::sin(t6), c6 = std::cos(t6);

    // Create Jacobian matrix (6x7 to hold all expressions from original code)
    MatrixXd J(6, 7);
    J.setZero();

    // Row 1 example (fill with expressions accordingly)
    J(0, 0) = s1;
    J(0, 1) = -c1 * s2;
    J(0, 2) = c3 * s1 + c2 * c1 * s3;
    J(0, 3) = s4 * (s3 * s1 - c2 * c3 * c1) - c4 * c1 * s2;
    J(0, 4) = c5 * (c3 * s1 + c2 * c1 * s3) - s5 * (c4 * (s3 * s1 - c2 * c3 * c1) + c1 * s2 * s4);
    J(0, 5) = c6 * (s4 * (s3 * s1 - c2 * c3 * c1) - c4 * c1 * s2)
              + s6 * (c5 * (c4 * (s3 * s1 - c2 * c3 * c1) + c1 * s2 * s4)
              + s5 * (c3 * s1 + c2 * c1 * s3));
    J(0, 6) = 0;

    // Row 2
    J(1, 0) = -1;
    J(1, 1) = 0;
    J(1, 2) = -c2;
    J(1, 3) = -s2 * s3;
    J(1, 4) = c3 * s2 * s4 - c2 * c4;
    J(1, 5) = -s5 * (c2 * s4 + c3 * c4 * s2) - c5 * s2 * s3;
    J(1, 6) = s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5)
              - c6 * (c2 * c4 - c3 * s2 * s4);

    // Row 3
    J(2, 0) = 0;
    J(2, 1) = -c1;
    J(2, 2) = -s2 * s1;
    J(2, 3) = c2 * s3 * s1 - c3 * c1;
    J(2, 4) = -s4 * (c1 * s3 + c2 * c3 * s1) - c4 * s2 * s1;
    J(2, 5) = s5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s2 * s4 * s1)
              - c5 * (c3 * c1 - c2 * s3 * s1);
    J(2, 6) = -s6 * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s2 * s4 * s1)
              + s5 * (c3 * c1 - c2 * s3 * s1))
              - c6 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s2 * s1);

    // Row 4 to 6 would be filled similarly using simplified expressions...
    double A = c1 * s3+c2*c3*s1;
    double B = c3*c1-c2*s3*s1;
    double C = s4*A+c4*s2*s1;
    double D = c4*A-s2*s4*s1;
    double E = c3*s1+c2*c1*s3;
    double F = s3*s1-c2*c3*c1;
    double G = s4*F-c4*c1*s2;
    double H = c4*F+c1*s2*s4;

    J(3,0) =l4 * C + l5 * (s6 * (c5 * D + s5 * B) + c6 * C) + l3 * s2 * s1;
    J(3,1) = l5 * (s6 * (c5 * (c2 * c1 * s4 + c3 * c4 * c1 * s2)
            - c1 * s2 * s3 * s5)- c6 * (c2 * c4 * c1 - c3 * c1 * s2 * s4)
            ) - l4 * (c2 * c4 * c1 - c3 * c1 * s2 * s4) - l3 * c2 * c1;
    J(3,2) = l4 * s4 * E - l5 * (s6 * (
            s5 * (s3 * s1 - c2 * c3 * c1)
            - c4 * c5 * E)
            - c6 * s4 * E);
    J(3,3) = l5 * (
        c6 * (c4 * F + c1 * s2 * s4)
        - c5 * s6 * (s4 * F - c4 * c1 * s2)) + l4 * (c4 * F + c1 * s2 * s4);

    J(3,4) = -l5 * s6 * (s5 * (c4 * F + c1 * s2 * s4)- c5 * E);
    J(3,5) = -l5 * (s6 * G- c6 * (c5 * H + s5 * E));
    J(3,6) = 0.0;

    double term3 = c2*s4 + c3*c4*s2;
    double term4 = c2*c4-c3*s2*s4;

    J(4,0) = 0.0;
    J(4,1) = -l5 * (s6 * (c3 * s2 * s5 + c4 * c5 * s2 * s3) + c6 * s2 * s3 * s4) - l4 * s2 * s3 * s4;
    J(4,2) = l4 * term3 + l5 * (c6 * term3 + c5 * s6 * term4);
    J(4,3) = -l5 * s6 * (s5 * term3 + c5 * s2 * s3);
    J(4,4) = l5 * (c6 * (c5 * term3 - s2 * s3 * s5) + s6 * term4);
    J(4,5) = 0.0;
    J(4,6) = 0.0;

    double A1 = s3 * s1 - c2 * c3 * c1;
    double B1 = c4 * c1 * s2;
    double C1 = s4 * A - B;
    double D1 = c4 * A + c1 * s2 * s4;
    double E1 = c3 * s1 + c2 * c1 * s3;

    double F1 = c2 * s4 * s1 + c3 * c4 * s2 * s1;
    double G1 = c2 * c4 * s1 - c3 * s2 * s4 * s1;

    double H1 = c1 * s3 + c2 * c3 * s1;
    double I1 = s2 * s4 * s1;

    J(5,0) = l4 * C1
            + l5 * (c6 * C1 + s6 * (c5 * D1 + s5 * E1))
            - l3 * c1 * s2;
    J(5,1) =l5* (s6* (c5 * F1- s2 * s3 * s5 * s1)- c6 * G1)- l4 * G- l3 * c2 * s1;
    J(5,2) = l5* (s6* (s5 * H1- c4 * c5 * (c3 * c1 - c2 * s3 * s1))- c6 * s4 * (c3 * c1 - c2 * s3 * s1))- l4 * s4 * (c3 * c1 - c2 * s3 * s1);
    J(5,3) = -(l4* (c4 * H1- s2 * s4 * s1)+ l5* (c6
            * (c4 * H1- s2 * s4 * s1)- c5 * s6* (s4 * H1+ c4 * s2 * s1)));
    J(5,4) = l5* s6* (s5* (c4 * H1- s2 * s4 * s1)- c5 * (c3 * c1 - c2 * s3 * s1));
    J(5,5) = l5*(s6* (s4 * H1+ c4 * s2 * s1)- c6* (c5* (c4 * H- s2 * s4 * s1)+ s5 * (c3 * c1 - c2 * s3 * s1)));
    J(5,6) = 0.0;
    return J;
}

#include <iostream>

int main() {
    std::vector<double> theta = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    std::vector<double> dh_l = {0.1, 0.2, 0.3, 0.4, 0.5};

    MatrixXd J = computeJacobian(theta, dh_l);
    std::cout << "Jacobian Matrix:\n" << J << std::endl;

    return 0;
}
