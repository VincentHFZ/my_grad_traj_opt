/**
 * @brief Quadratic Programming二次规划　用来　生成路径
*/

#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <vector>
#include <string>

class TrajectoryGenerator {
private:
    std::vector<double> qp_cost;
    Eigen::MatrixXd _A;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _C;
    Eigen::MatrixXd _L;

    Eigen::MatrixXd _R;
    Eigen::MatrixXd _Rff;
    Eigen::MatrixXd _Rpp;
    Eigen::MatrixXd _Rpf;
    Eigen::MatrixXd _Rfp;

    Eigen::VectorXd _Pxi;
    Eigen::VectorXd _Pyi;
    Eigen::VectorXd _Pzi;

    Eigen::VectorXd _Dx;
    Eigen::VectorXd _Dy;
    Eigen::VectorXd _Dz;

public:
    Eigen::MatrixXd _Path;
    Eigen::MatrixXd _Time;

    TrajectoryGenerator();
    ~TrajectoryGenerator();

    Eigen::MatrixXd PolyQPGeneration(const Eigen::MatrixXd &path,
                                     const Eigen::Vector3d &vel,
                                     const Eigen::Vector3d &acc,
                                     const Eigen::VectorXd &time,
                                     const int &type);

    Eigen::MatrixXd PloyCoeffGeneration(const Eigen::MatrixXd &pathCorridor,
                                        const Eigen::MatrixXd &pathConnect,
                                        const Eigen::VectorXd &radius,
                                        const Eigen::VectorXd &pathRadius,
                                        const Eigen::VectorXd &time,
                                        const Eigen::VectorXd &vel,
                                        const Eigen::MatrixXd &acc,
                                        const double maxVel,
                                        const double maxAcc );
    std::vector<double> getCost();

    void StackOptiDep();        // Stack the optimization's dependency, the intermediate matrix and initial derivatives

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getInitialD();  // Initial Derivatives variable for the following optimization

    Eigen::MatrixXd getA();
    Eigen::MatrixXd getQ();
    Eigen::MatrixXd getC();
    Eigen::MatrixXd getL();

    Eigen::MatrixXd getR();
    Eigen::MatrixXd getRpp();
    Eigen::MatrixXd getRff();
    Eigen::MatrixXd getRfp();
    Eigen::MatrixXd getRpf();
};

#endif // TRAJ_GENERATOR_H
