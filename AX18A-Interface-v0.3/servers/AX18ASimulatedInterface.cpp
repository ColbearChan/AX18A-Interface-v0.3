//
// Created by matthias on 11/03/18.
// Modified by Hiu on 16/04/18.
//

#include "AX18ASimulatedInterface.h"

std::shared_ptr<AX18ARobotInterface> AX18ASimulatedInterface::create(){
    return std::shared_ptr<AX18ARobotInterface>(new AX18ASimulatedInterface());
}
//135/180/3.14
AX18ASimulatedInterface::AX18ASimulatedInterface() {
    sampleLocale = Eigen::VectorXd(4);
    sampleLocale << 0, -2.0, 2.0, 0;
    //sampleLocale << 0, 0, 0.5, 0;
}

Eigen::VectorXd AX18ASimulatedInterface::getCurrentJointAngles(){

    return sampleLocale;
}

void AX18ASimulatedInterface::setTargetJointAngles(Eigen::VectorXd setAngles){
    targetAngles = setAngles;
    sampleLocale = setAngles;
}

Eigen::VectorXd AX18ASimulatedInterface::getTargetJointAngles(){
    return targetAngles;
}

void AX18ASimulatedInterface::step(){
    // TODO update joint angles here
}