//
// Created by matthias on 10/03/18.
//

#ifndef AX18A_SERVERCLIENT_AX_18A_COMM_H_H
#define AX18A_SERVERCLIENT_AX_18A_COMM_H_H


#include <boost/asio.hpp>

#include <Eigen/Eigen>

#define AX18A_PORT 10000
#define AX18A_PORT_STR "10000"

#define AX18A_ECHO_REQUEST ((uint32_t)(1<<31))

#define AX18A_GET_JOINT_ANGLES (uint32_t)1

#define AX18A_SET_JOINT_COMMAND ((uint32_t)(1<<16))



void writeCommandToSocket(boost::asio::ip::tcp::socket &socket, uint32_t command);

void checkCommandEchoFromSocket(boost::asio::ip::tcp::socket &socket, uint32_t command);

Eigen::VectorXd readVectorXdFromSocket(boost::asio::ip::tcp::socket &socket, size_t expectedSize);

void writeVectorXdToSocket(boost::asio::ip::tcp::socket &socket, Eigen::VectorXd vector);


#endif //AX18A_SERVERCLIENT_AX_18A_COMM_H_H
