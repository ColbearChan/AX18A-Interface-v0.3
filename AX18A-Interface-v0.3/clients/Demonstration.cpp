//
// Created by 16076710 on 25/04/18.
//

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/AX18ARemoteInterface.h>

#include "tcp/AX-18A-Comm.h"
#include "InverseKinematics.cpp"
#include "ForwardKinematics.cpp"
#include <Eigen/Geometry>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
// Initial position in angles for the arm: 0, -2.0, 2.0, 0

bool difference(VectorXd vector, Vector3d final){
    bool reached = false;
    double endVal = 0.001;
    VectorXd diff = final-vector;
    double distance = sqrt(diff.transpose()*diff);
    if (distance < endVal) {
        reached = true;
    }
    return reached;
}

void reachingDemo(Vector3d finalTarget, shared_ptr<AX18ARobotInterface> robot){
    bool reached = false;
    double k = 0.01;

    while (reached == false) {
        reached = difference(getCurrentPositionFK(robot->getCurrentJointAngles()), finalTarget);
        VectorXd nextAngle(4);
        Vector4d currentAngle;
        currentAngle[0] = robot->getCurrentJointAngles()[0];
        currentAngle[1] = robot->getCurrentJointAngles()[1];
        currentAngle[2] = robot->getCurrentJointAngles()[2];
        currentAngle[3] = robot->getCurrentJointAngles()[3];

        nextAngle[0] = getNextAngleIK(currentAngle, finalTarget, k)[0];
        nextAngle[1] = getNextAngleIK(currentAngle, finalTarget, k)[1];
        nextAngle[2] = getNextAngleIK(currentAngle, finalTarget, k)[2];
        nextAngle[3] = getNextAngleIK(currentAngle, finalTarget, k)[3];


        robot->setTargetJointAngles(nextAngle);
        cout << "Final Target:  " << finalTarget << endl;
        cout << "Current: " << endl;
        getCurrentPositionFK(robot->getCurrentJointAngles());

        k = k + 0.0001;

        usleep(25000);
    }

    reached = false;
}

void pointingDemo(Vector3d finalTarget, shared_ptr<AX18ARobotInterface> robot){
    bool reached = false;

    while (reached == false) {
        reached = difference(getCurrentPositionFK(robot->getCurrentJointAngles()), finalTarget);
        VectorXd nextAngle(4);
        Vector4d currentAngle;
        currentAngle[0] = robot->getCurrentJointAngles()[0];
        currentAngle[1] = robot->getCurrentJointAngles()[1];
        currentAngle[2] = robot->getCurrentJointAngles()[2];
        currentAngle[3] = robot->getCurrentJointAngles()[3];

        nextAngle[0] = getNextAngleIKPointing(currentAngle, finalTarget)[0];
        nextAngle[1] = getNextAngleIKPointing(currentAngle, finalTarget)[1];
        nextAngle[2] = getNextAngleIKPointing(currentAngle, finalTarget)[2];
        nextAngle[3] = getNextAngleIKPointing(currentAngle, finalTarget)[3];


        robot->setTargetJointAngles(nextAngle);
        cout << "Final Target:  "  << endl;
        cout << finalTarget << endl;
        cout << "Current: " << endl;
        getCurrentPositionFK(robot->getCurrentJointAngles());

        usleep(25000);
    }
}



int main(int argc, char* argv[]) {
    bool reached = false;
    string remoteHost = "localhost";
    if (argc == 2) {
        remoteHost = argv[1];
    }

    shared_ptr<AX18ARobotInterface> robot = AX18ARemoteInterface::create(remoteHost);
    Vector3d finalTarget;

    //Initial angles
    VectorXd initialAngles(4);
    initialAngles[0] = 0;
    initialAngles[1] = -2;
    initialAngles[2] = 2;
    initialAngles[3] = 0;

    Vector3d initialPosition;
    initialPosition[0] = getCurrentPositionFK(initialAngles)[0];
    initialPosition[1] = getCurrentPositionFK(initialAngles)[1];
    initialPosition[2] = getCurrentPositionFK(initialAngles)[2];


    //reaching
    finalTarget[0] = 0.2;
    finalTarget[1] = 0.2;
    finalTarget[2] = 0.25;

    reachingDemo(initialPosition, robot);

    reachingDemo(finalTarget, robot);

    reachingDemo(initialPosition, robot);

    finalTarget[0] = -0.05;
    finalTarget[1] = -0.05;
    finalTarget[2] = 0.4;

    reachingDemo(finalTarget, robot);

    reachingDemo(initialPosition, robot);

    finalTarget[0] = 0.1;
    finalTarget[1] = -0.05;
    finalTarget[2] = 0.3;

    reachingDemo(finalTarget, robot);

    reachingDemo(initialPosition, robot);

    finalTarget[0] = -0.2;
    finalTarget[1] = 0.1;
    finalTarget[2] = 0.2;

    reachingDemo(finalTarget, robot);

    reachingDemo(initialPosition, robot);

    //pointing
    finalTarget[0] = -0.6;
    finalTarget[1] = -0.6;
    finalTarget[2] = -0.6;

    pointingDemo(finalTarget, robot);

    return 0;

}

