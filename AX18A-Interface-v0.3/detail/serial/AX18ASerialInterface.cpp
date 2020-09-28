//
// Created by matthias on 11/03/18.
//



#if defined(__linux__) || defined(__APPLE__)

#include <fcntl.h>
#include <termios.h>

#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <sstream>

#include "AX18ASerialInterface.h"
#include "AX18ASerialTable.h"

using namespace std;


AX18ASerialInterface::AX18ASerialInterface(int portNumber) {
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows

    stringstream portS;
    portS << DEVICENAME_BARE << portNumber;

    string setLatencyCommand = "setserial " + portS.str() + " low_latency";
    int r = system(setLatencyCommand.c_str());
    if(r){
        cerr << "Warning: could not set serial port to low latency" << endl;
        cerr << "\t" << setLatencyCommand << endl;
        cerr << "\treturn: " << r << endl;
    }

    portHandler = dynamixel::PortHandler::getPortHandler(portS.str().c_str());

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort()) {
        cout << "Succeeded to open the port " << portS.str() << endl;
    } else {
        cerr << "Failed to open the port " << portS.str() << endl;
        abort();
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        cout << "Succeeded to change the baudrate!" << endl;
    } else {
        cerr << "Failed to change the baudrate!" << endl;
        abort();
    }


    initialDataRead = false;

    running = true;
    updateMicros = 40*1000;
    //updateMicros = 1000*1000;
    //updateMicros = 10;
    this->updater = thread(&AX18ASerialInterface::runUpdates, this);
}

AX18ASerialInterface::~AX18ASerialInterface() {
    // Close port
    portHandler->closePort();

}

std::shared_ptr<AX18ARobotInterface> AX18ASerialInterface::create(int portNumber) {
    return std::shared_ptr<AX18ARobotInterface>(new AX18ASerialInterface(portNumber));
}

Eigen::VectorXd AX18ASerialInterface::getCurrentJointAngles() {

    Eigen::VectorXd joints;
    {
        unique_lock<mutex> lk(dataMutex);
        if(initialDataRead){
            joints = this->currentJointAngles;
        }
        else{
            cout << "Waiting for initial data." << endl;
            initialDataReadNotifier.wait(lk);
            cout << "Initial data received." << endl;
            joints = this->currentJointAngles;
            initialDataRead = true;
        }

    }
    //dataMutex.unlock();

    return joints;

//    // Initialize GroupSyncWrite instance
//    //dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
//
//    int index = 0;
//    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//    bool dxl_addparam_result = false;               // addParam result
//    //int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};  // Goal position
//
//    uint8_t dxl_error = 0;                          // Dynamixel error
//    uint8_t param_goal_position[2];
//    //uint16_t dxl1_present_position = 0, dxl2_present_position = 0;                        // Present position
//
//    // Read Dynamixel# present position
////    for (uint i = 0; i < 1; i++) { // FIXME
//    for (uint i = 0; i < DXL_NUM; i++) {
//        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION,
//                                                       &motorPositions[i], &dxl_error);
//        //portHandler->clearPort();
//        if (dxl_comm_result != COMM_SUCCESS) {
//            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        } else if (dxl_error != 0) {
//            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        //cout << "Motor Position" << i << ": " << motorPositions[i] << endl;
//    }
//
//    Eigen::VectorXd angles(4);
//    angles[0] = DXL_RAD_PER_POS * (motorPositions[0] - DXL_POS_CENTER);
//    angles[1] = DXL_RAD_PER_POS * (motorPositions[1] - DXL_POS_CENTER) - 3.0 / 4.0 * M_PI;
//    angles[2] = DXL_RAD_PER_POS * (motorPositions[3] - DXL_POS_CENTER);
//    angles[3] = DXL_RAD_PER_POS * (motorPositions[5] - DXL_POS_CENTER);
//    return angles;
}

//Eigen::VectorXd AX18ASerialInterface::getCurrentJointAngles() {
//    // Initialize GroupSyncWrite instance
//    //dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
//
//    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//
//    uint8_t dxl_error = 0;                          // Dynamixel error
//    uint16_t dxl_model_number;
//
//    dxl_comm_result = packetHandler->ping(portHandler, 1, &dxl_model_number, &dxl_error);
//    if (dxl_comm_result != COMM_SUCCESS)
//    {
//        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//    }
//    else if (dxl_error != 0)
//    {
//        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//    }
//    else
//    {
//        printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", DXL_ID, dxl_model_number);
//    }
//
//    Eigen::VectorXd angles(4);
//    angles[0] = DXL_RAD_PER_POS * (motorPositions[0] - DXL_POS_CENTER);
//    angles[1] = DXL_RAD_PER_POS * (motorPositions[1] - DXL_POS_CENTER) - 3.0 / 4.0 * M_PI;
//    angles[2] = DXL_RAD_PER_POS * (motorPositions[3] - DXL_POS_CENTER);
//    angles[3] = DXL_RAD_PER_POS * (motorPositions[5] - DXL_POS_CENTER);
//    return angles;
//}

//Eigen::VectorXd AX18ASerialInterface::getCurrentJointAngles() {
//    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
//
//    cout << "LEN_MX_PRESENT_POSITION " << LEN_MX_PRESENT_POSITION << endl;
//    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//    bool dxl_addparam_result = false;               // addParam result
//    // Read Dynamixel# present position
////    for (uint i = 0; i < DXL_NUM; i++) {
//    for (uint i = 0; i < 1; i++) {
//        dxl_addparam_result = groupBulkRead.addParam(DXL_ID[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//        if (dxl_addparam_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed", DXL_ID[i]);
//            abort();
//        }
//    }
//
//    dxl_comm_result = groupBulkRead.txRxPacket();
//    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//
//    usleep(100*1000);
//    for (uint i = 0; i < DXL_NUM; i++) {
//        bool dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//        if (dxl_getdata_result != true) {
//            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", DXL_ID[i]);
//            //abort();
//        }
//        motorPositions[i] = groupBulkRead.getData(DXL_ID[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//    }
//
//
//    Eigen::VectorXd angles(4);
//    angles[0] = DXL_RAD_PER_POS * (motorPositions[0] - DXL_POS_CENTER);
//    angles[1] = DXL_RAD_PER_POS * (motorPositions[1] - DXL_POS_CENTER) - 3.0 / 4.0 * M_PI;
//    angles[2] = DXL_RAD_PER_POS * (motorPositions[3] - DXL_POS_CENTER);
//    angles[3] = DXL_RAD_PER_POS * (motorPositions[5] - DXL_POS_CENTER);
//    return angles;
//}

//Eigen::VectorXd AX18ASerialInterface::getCurrentJointAngles() {
//    // seems only for protocol version 2.0
//    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//
//    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//    bool dxl_addparam_result = false;               // addParam result
//    // Read Dynamixel# present position
////    for (uint i = 0; i < DXL_NUM; i++) {
//    for (uint i = 0; i < 1; i++) {
//        dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i]);
//        if (dxl_addparam_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);
//            //abort();
//        }
//    }
//
//    dxl_comm_result = groupSyncRead.txRxPacket();
//    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//
//    usleep(100*1000);
//    for (uint i = 0; i < DXL_NUM; i++) {
//        bool dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//        if (dxl_getdata_result != true) {
//            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", DXL_ID[i]);
//            //abort();
//        }
//        motorPositions[i] = groupSyncRead.getData(DXL_ID[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//    }
//
//
//    Eigen::VectorXd angles(4);
//    angles[0] = DXL_RAD_PER_POS * (motorPositions[0] - DXL_POS_CENTER);
//    angles[1] = DXL_RAD_PER_POS * (motorPositions[1] - DXL_POS_CENTER) - 3.0 / 4.0 * M_PI;
//    angles[2] = DXL_RAD_PER_POS * (motorPositions[3] - DXL_POS_CENTER);
//    angles[3] = DXL_RAD_PER_POS * (motorPositions[5] - DXL_POS_CENTER);
//    return angles;
//}

void AX18ASerialInterface::setTargetJointAngles(Eigen::VectorXd targets) {
    if(targets.rows()!=numberOfJoints()){
        throw invalid_argument("AX18ARemoteInterface::setTargetJointAngles(): Wrong number of target joint angles");
    }

    unique_lock<mutex> lk(dataMutex);

    targets = targets.cwiseMax(getJointLowerLimits());
    targets = targets.cwiseMin(getJointUpperLimits());

    this->targetJointAngles = targets;
}

Eigen::VectorXd AX18ASerialInterface::getTargetJointAngles() {
    Eigen::VectorXd t;
    {
        unique_lock<mutex> lk(dataMutex);
        t = this->targetJointAngles;
    }
    return t;
}

Eigen::VectorXd AX18ASerialInterface::getCurrentJointAnglesFromSerial(){

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint8_t dxl_error = 0;                          // Dynamixel error

    // Read Dynamixel# present position
    for (uint i = 0; i < DXL_NUM; i++) {
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION,
                                                       &motorPositions[i], &dxl_error);
        //portHandler->clearPort();
        if (dxl_comm_result != COMM_SUCCESS) {
            cerr << "ID " << DXL_ID[i] << ": " << packetHandler->getTxRxResult(dxl_comm_result) << endl;
        } else if (dxl_error != 0) {
            cerr << "ID " << DXL_ID[i] << ": " << packetHandler->getRxPacketError(dxl_error) << endl;
        }else {
            //cerr << "ID " << DXL_ID[i] << ": correct" << endl;
        }
        //cout << "Motor Position" << i << ": " << motorPositions[i] << endl;
    }


    //cout << "pos[1] " << motorPositions[1] << endl;
    //cout << "pos[2] " << motorPositions[2] << endl;
    //cout << "pos[6] " << motorPositions[6] << endl;

    Eigen::VectorXd angles(4);
    angles[0] = DXL_RAD_PER_POS * (motorPositions[0] - DXL_POS_CENTER);
    angles[1] = DXL_RAD_PER_POS * (motorPositions[1] - DXL_POS_CENTER) - 3.0 / 4.0 * M_PI;
    angles[2] = DXL_RAD_PER_POS * (motorPositions[3] - DXL_POS_CENTER);
    angles[3] = DXL_RAD_PER_POS * (motorPositions[5] - DXL_POS_CENTER);
    return angles;
}

void AX18ASerialInterface::writeTargetJointAnglesToSerial(Eigen::VectorXd angles){
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    // Enable Dynamixel Torque
    // TODO only once
    for (uint i = 0; i < DXL_NUM-1; i++) {// FIXME gripper currently ignored
    //for (uint i = 0; i < 1; i++) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE,
                                                        &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            cerr << "ID " << DXL_ID[i] << ": " << packetHandler->getTxRxResult(dxl_comm_result) << endl;
        } else if (dxl_error != 0) {
            cerr << "ID " << DXL_ID[i] << ": " << packetHandler->getRxPacketError(dxl_error) << endl;
        } else {
            //cerr << "ID " << DXL_ID[i] << ": correct" << endl;
        }
    }

    uint16_t motorTargets[7];

    motorTargets[0] = angles[0] / DXL_RAD_PER_POS + DXL_POS_CENTER;
    motorTargets[1] = (angles[1]+3.0/4.0*M_PI) / DXL_RAD_PER_POS + DXL_POS_CENTER;
    motorTargets[2] = DXL_POS_MAX - motorTargets[1];
    motorTargets[3] = angles[2] / DXL_RAD_PER_POS + DXL_POS_CENTER;
    motorTargets[4] = DXL_POS_MAX - motorTargets[3];
    motorTargets[5] = angles[3] / DXL_RAD_PER_POS + DXL_POS_CENTER;
    //motorTargets[6] = DXL_POS_CENTER; // FIXME gripper currently ignored

    //cout << "goal[1] " << motorTargets[1] << endl;
    //cout << "goal[2] " << motorTargets[2] << endl;

    //for (uint i = 0; i < DXL_NUM; i++) {
    for (uint i = 0; i < DXL_NUM-1; i++) { // FIXME gripper currently ignored
    //for (uint i = 0; i < 1; i++) {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_GOAL_POSITION,
                                                        motorTargets[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            cerr << "ID " << DXL_ID[i] << ": " << packetHandler->getTxRxResult(dxl_comm_result) << endl;
        } else if (dxl_error != 0) {
            cerr << "ID " << DXL_ID[i] << ": " << packetHandler->getRxPacketError(dxl_error) << endl;
        }
    }
}

void AX18ASerialInterface::runUpdates(){
    while(running) {
        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::VectorXd serverCurrentAngles = getCurrentJointAnglesFromSerial();
        //Eigen::VectorXd serverCurrentAngles;


        Eigen::VectorXd velocityLimit(4);
        velocityLimit << 1, 1, 1, 1; // 1 rad / s
        Eigen::VectorXd updateLimit = velocityLimit * (updateMicros / 1000000.0);

        Eigen::VectorXd command;
        Eigen::VectorXd last;

        dataMutex.lock();
        this->currentJointAngles = serverCurrentAngles;
        initialDataReadNotifier.notify_all();
        command = this->targetJointAngles;
        last = this->targetJointAnglesLastSent;
        if (command.rows() > 0){
            if (last.rows() == 0) {
                last = currentJointAngles;
            }

            Eigen::VectorXd controlDiff = command - last;
            controlDiff = controlDiff.cwiseMax(-updateLimit);
            controlDiff = controlDiff.cwiseMin(updateLimit);
            command = last + controlDiff;
            this->targetJointAnglesLastSent = command;
        }
        dataMutex.unlock();

        if(command.rows() > 0){
            writeTargetJointAnglesToSerial(command);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();

        if(micros < updateMicros){
            usleep( updateMicros - micros  );
        }
        else{
            cerr << "Warning: Serial port update takes too long: " << micros << "microseconds." << endl;
        }
    }
}