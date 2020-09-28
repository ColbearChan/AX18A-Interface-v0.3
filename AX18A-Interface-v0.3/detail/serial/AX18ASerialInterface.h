//
// Created by matthias on 11/03/18.
//

#ifndef AX18A_SERVERCLIENT_AX18SERIALINTERFACE_H
#define AX18A_SERVERCLIENT_AX18SERIALINTERFACE_H

#include "../../ax18a/AX18ARobotInterface.h"
#include "../detail/dynamixel_sdk/dynamixel_sdk.h"  

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

class AX18ASerialInterface : public AX18ARobotInterface {
public:
    AX18ASerialInterface(int portNumber);
    ~AX18ASerialInterface();

    static std::shared_ptr<AX18ARobotInterface> create(int portNumber);

    Eigen::VectorXd getCurrentJointAngles();

    void setTargetJointAngles(Eigen::VectorXd);

    Eigen::VectorXd getTargetJointAngles();

private:
    Eigen::VectorXd getCurrentJointAnglesFromSerial();
    void writeTargetJointAnglesToSerial(Eigen::VectorXd);

    void runUpdates();

    std::mutex dataMutex;
    std::condition_variable initialDataReadNotifier;

    bool initialDataRead;
    Eigen::VectorXd currentJointAngles;
    Eigen::VectorXd targetJointAngles;
    Eigen::VectorXd targetJointAnglesLastSent;

    bool running;

    unsigned int updateMicros;
    std::thread updater;


    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    uint16_t motorPositions[7];
    bool torquesEnabled;


};


#endif //AX18A_SERVERCLIENT_AX18SERIALINTERFACE_H
