#include "traj_generator.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <string>

#include <stdio.h>

using namespace std;
using namespace Eigen;

Vector3d startVel;
Vector3d startAcc;

#define inf 1>>30       // 就是０　?

int m = 3;              // number of segmentsin the trajectory

TrajectoryGenerator::TrajectoryGenerator() {}

TrajectoryGenerator::~TrajectoryGenerator() {}











vector<double> TrajectoryGenerator::getCost() {
    return qp_cost;
}

Eigen::MatrixXd TrajectoryGenerator::getA() {
    return _A;
}

Eigen::MatrixXd TrajectoryGenerator::getC() {
    return _C;
}

Eigen::MatrixXd TrajectoryGenerator::getQ() {
    return _Q;
}

Eigen::MatrixXd TrajectoryGenerator::getL() {
    return _L;
}

Eigen::MatrixXd TrajectoryGenerator::getR() {
    return _R;
}

Eigen::MatrixXd TrajectoryGenerator::getRff() {

}
