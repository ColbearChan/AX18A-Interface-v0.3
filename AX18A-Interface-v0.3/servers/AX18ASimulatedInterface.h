//
// Created by matthias on 11/03/18.
// Modified by Hiu on 16/04/18.
//

#ifndef AX18A_SERVERCLIENT_AX18ASIMULATEDINTERFACE_H
#define AX18A_SERVERCLIENT_AX18ASIMULATEDINTERFACE_H

#include "../ax18a/AX18ARobotInterface.h"

#include <memory>

class AX18ASimulatedInterface : public AX18ARobotInterface {
public:
    AX18ASimulatedInterface();
    static std::shared_ptr<AX18ARobotInterface> create();
    Eigen::VectorXd getCurrentJointAngles();

    void setTargetJointAngles(Eigen::VectorXd setAngles);

    Eigen::VectorXd getTargetJointAngles();

    void step();

private:
    Eigen::VectorXd sampleLocale;
    Eigen::VectorXd targetAngles;
};



#endif //AX18A_SERVERCLIENT_AX18ASIMULATEDINTERFACE_H

