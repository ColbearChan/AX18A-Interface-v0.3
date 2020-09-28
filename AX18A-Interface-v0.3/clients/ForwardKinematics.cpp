//
// Created by 16076710 on 25/04/18.
//

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/AX18ARemoteInterface.h>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include "tcp/AX-18A-Comm.h"

using namespace std;
using namespace Eigen;


//joint[alpha i, ai, theita i, di];
double joint1DH[4] = {3.14/2, 0.052, 0, 0.148};
double joint2DH[4] = {0, 0.173, 3.14, 0};
double joint3DH[4] = {-3.14/2, 0.025, -3.14/2, 0};
double joint4DH[4] = {0, 0, 0, 0.23};

//dh convention
Eigen::MatrixXd setDH(double dhinfo[], double angle){
    Eigen::MatrixXd dhFormat(4, 4);
    dhFormat(0,0) = cos(angle + dhinfo[2]);
    dhFormat(0,1) = -sin(angle + dhinfo[2])*cos(dhinfo[0]);
    dhFormat(0,2) = sin(angle + dhinfo[2])*sin(dhinfo[0]);
    dhFormat(0,3) = dhinfo[1]*cos(angle + dhinfo[2]);
    dhFormat(1,0) = sin(angle + dhinfo[2]);
    dhFormat(1,1) = cos(angle + dhinfo[2])*cos(dhinfo[0]);
    dhFormat(1,2) = -cos(angle + dhinfo[2])*sin(dhinfo[0]);
    dhFormat(1,3) = dhinfo[1]*sin(angle + dhinfo[2]);
    dhFormat(2,0) = 0;
    dhFormat(2,1) = sin(dhinfo[0]);
    dhFormat(2,2) = cos(dhinfo[0]);
    dhFormat(2,3) = dhinfo[3];
    dhFormat(3,0) = 0;
    dhFormat(3,1) = 0;
    dhFormat(3,2) = 0;
    dhFormat(3,3) = 1;
    return dhFormat;
};

Eigen::VectorXd getPositionFK(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd c, Eigen::MatrixXd d){
    Eigen::MatrixXd final = a*b*c*d;
    Eigen::VectorXd zeroZeroZeroOne(4);
    zeroZeroZeroOne[0] = 0;
    zeroZeroZeroOne[1] = 0;
    zeroZeroZeroOne[2] = 0;
    zeroZeroZeroOne[3] = 1;
    Eigen::VectorXd x = final * zeroZeroZeroOne;
    return x;
}


//rgb xyz
/*int main(int argc, char* argv[]) {

    //Forward kinematics
    Eigen::MatrixXd joint1 = setDH(joint1DH, 0);
    Eigen::MatrixXd joint2 = setDH(joint2DH, 0);
    Eigen::MatrixXd joint3 = setDH(joint3DH, 0);
    Eigen::MatrixXd joint4 = setDH(joint4DH, 0);
    std::cout << getPositionFK(joint1, joint2, joint3, joint4);

    string remoteHost = "localhost";
    if (argc == 2) {
        remoteHost = argv[1];
    }

    shared_ptr <AX18ARobotInterface> robot = AX18ARemoteInterface::create(remoteHost);

    while (true) {
        usleep(4000000);
        Eigen::VectorXd jointAngles = robot->getCurrentJointAngles();
        Eigen::MatrixXd joint1 = setDH(joint1DH, jointAngles[0]);
        Eigen::MatrixXd joint2 = setDH(joint2DH, jointAngles[1]);
        Eigen::MatrixXd joint3 = setDH(joint3DH, jointAngles[2]);
        Eigen::MatrixXd joint4 = setDH(joint4DH, jointAngles[3]);
        std::cout << "x = " << getPositionFK(joint1, joint2, joint3, joint4)[0] << endl;
        std::cout << "y = " << getPositionFK(joint1, joint2, joint3, joint4)[1] << endl;
        std::cout << "z = " << getPositionFK(joint1, joint2, joint3, joint4)[2] << endl;
        std::cout << getPositionFK(joint1, joint2, joint3, joint4)[3] << endl;
    }*
}*/

VectorXd getCurrentPositionFK(VectorXd jointAngles){
    Eigen::MatrixXd joint1 = setDH(joint1DH, jointAngles[0]);
    Eigen::MatrixXd joint2 = setDH(joint2DH, jointAngles[1]);
    Eigen::MatrixXd joint3 = setDH(joint3DH, jointAngles[2]);
    Eigen::MatrixXd joint4 = setDH(joint4DH, jointAngles[3]);
    std::cout << "x = " << getPositionFK(joint1, joint2, joint3, joint4)[0] << endl;
    std::cout << "y = " << getPositionFK(joint1, joint2, joint3, joint4)[1] << endl;
    std::cout << "z = " << getPositionFK(joint1, joint2, joint3, joint4)[2] << endl;
    VectorXd currentPos(3);
    currentPos[0] = getPositionFK(joint1, joint2, joint3, joint4)[0];
    currentPos[1] = getPositionFK(joint1, joint2, joint3, joint4)[1];
    currentPos[2] = getPositionFK(joint1, joint2, joint3, joint4)[2];
    return currentPos;
}