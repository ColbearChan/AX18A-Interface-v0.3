//
// Created by matthias on 11/03/18.
//

#ifndef AX18A_SERVERCLIENT_ROBOTINTERFACE_H
#define AX18A_SERVERCLIENT_ROBOTINTERFACE_H

#include <Eigen/Eigen>

class AX18ARobotInterface {
public:
    virtual Eigen::VectorXd getCurrentJointAngles() = 0;

    virtual void setTargetJointAngles(Eigen::VectorXd) = 0;

    virtual Eigen::VectorXd getTargetJointAngles() = 0;

    unsigned int numberOfJoints(){
        return 4;
    }

    Eigen::VectorXd getJointLowerLimits(){
        Eigen::VectorXd lim(4);
        lim << -150.0/180.0*M_PI , -4.0 , -2.04 , -3.0/4.0*M_PI;
        return lim;
    }

    Eigen::VectorXd getJointUpperLimits(){
        Eigen::VectorXd lim(4);
        lim <<  150.0/180.0*M_PI , -0.298651 , 2.04 , 3.0/4.0*M_PI;
        return lim;
    }

};


#endif //AX18A_SERVERCLIENT_ROBOTINTERFACE_H
