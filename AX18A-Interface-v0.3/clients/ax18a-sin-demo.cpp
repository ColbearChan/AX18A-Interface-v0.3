
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/AX18ARemoteInterface.h>

#include "tcp/AX-18A-Comm.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]){

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }

    shared_ptr<AX18ARobotInterface> robot = AX18ARemoteInterface::create(remoteHost);

    uint t = 0;
    while(true){
        VectorXd joints = VectorXd::Zero(4);
        joints << 0, -1, 0, 0;
        const int length = 200;
        joints[(t/length)%4] += 3.14/4.0 * sin((t/(double)length) * 2 * M_PI);
        t++;

        robot->setTargetJointAngles(joints);
        cout << "Target:  " << joints.transpose() << endl;
        cout << "Current: " << robot->getCurrentJointAngles().transpose() << endl;
        cout << endl;

        usleep(25*1000);
    }
//    while(true){
//        VectorXd joints = VectorXd::Zero(4);
//        joints[0] = 0.0;
//
//        robot->setTargetJointAngles(joints);
//        cout << joints.transpose() << endl;
//
//        usleep(25*1000);
//    }

    return 0;
}
