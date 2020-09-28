//
// Created by matthias on 11/03/18.
//

#ifndef AX18A_SERVERCLIENT_AX18REMOTEINTERFACE_H
#define AX18A_SERVERCLIENT_AX18REMOTEINTERFACE_H

#include "../../ax18a/AX18ARobotInterface.h"

#include <memory>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

class AX18ARemoteInterface : public AX18ARobotInterface {
public:
    static std::shared_ptr<AX18ARobotInterface> create(std::string host = "localhost");

    AX18ARemoteInterface(std::string host);
    ~AX18ARemoteInterface();

    Eigen::VectorXd getCurrentJointAngles();

    void setTargetJointAngles(Eigen::VectorXd);

    Eigen::VectorXd getTargetJointAngles();

private:
    Eigen::VectorXd getCurrentJointAnglesFromServer();
    void writeTargetJointAnglesToServer(Eigen::VectorXd);

    void runUpdates();

    boost::asio::ip::tcp::socket *socket;

    std::mutex dataMutex;
    std::condition_variable initialDataReadNotifier;
    bool initialDataRead;
    Eigen::VectorXd currentJointAngles;
    Eigen::VectorXd targetJointAngles;

    bool running;

    unsigned int updateMicros;
    std::thread updater;
};


#endif //AX18A_SERVERCLIENT_AX18REMOTEINTERFACE_H
