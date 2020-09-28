
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

    while(true){
        VectorXd joints = robot->getCurrentJointAngles();

        cout << "Joint angles received: " << joints.transpose() << endl;

        usleep(250*1000);

    }

    return 0;
}