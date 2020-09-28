
#include "AX-18A-Comm.h"

#include <boost/array.hpp>

#include <iostream>

using namespace std;

void writeCommandToSocket(boost::asio::ip::tcp::socket &socket, uint32_t command){
    std::vector<uint32_t> comm;
    comm.push_back(command);
    boost::asio::write(socket, boost::asio::buffer(comm));
}

void checkCommandEchoFromSocket(boost::asio::ip::tcp::socket &socket, uint32_t command){
    boost::system::error_code error;

    boost::array<uint32_t, 1> commandEcho;
    size_t bytesLength = socket.read_some(boost::asio::buffer(commandEcho), error);

    if (error == boost::asio::error::eof){
        cerr << "Connection closed cleanly by peer." << endl;
        exit(1);
    }
    else if (error)
        throw boost::system::system_error(error); // Some other error.

    if(bytesLength<1){
        cerr << "Could not retrieve command echo" << endl;
        abort();
    }
    if(commandEcho[0] != (AX18A_ECHO_REQUEST | command)){
        cerr << "Incorrect command echo" << endl;
        abort();
    }
}

Eigen::VectorXd readVectorXdFromSocket(boost::asio::ip::tcp::socket &socket, size_t expectedSize){
    boost::system::error_code error;
    size_t bytesLength;

    boost::array<uint32_t, 1> length;
    bytesLength = socket.read_some(boost::asio::buffer(length), error);

    if (error == boost::asio::error::eof){
        cerr << "Connection closed cleanly by peer." << endl;
        exit(1);
    }
    else if (error)
        throw boost::system::system_error(error); // Some other error.

    if(bytesLength<1){
        cerr << "Could not retrieve size of vector from server." << endl;
        abort();
    }
    if(length[0] != expectedSize){
        cerr << "Expected " << expectedSize << " elements in vector from server, but size says " << length[0] << endl;
        abort();
    }

    //boost::array<double, 4> angles;
    vector<double> angles(expectedSize);
    bytesLength = socket.read_some(boost::asio::buffer(angles), error);

    if (error == boost::asio::error::eof){
        cerr << "Connection closed cleanly by peer." << endl;
        exit(1);
    }
    else if (error)
        throw boost::system::system_error(error); // Some other error.

    if(bytesLength!=(expectedSize*sizeof(double))){
        cerr << "Incorrect data size from server: " << bytesLength << endl;
        abort();
    }
    Eigen::VectorXd anglesEigen(expectedSize);
    for(uint i=0;i<expectedSize;i++){
        anglesEigen[i] = angles[i];
    }

    return anglesEigen;
}

void writeVectorXdToSocket(boost::asio::ip::tcp::socket &socket, Eigen::VectorXd vector){
    std::vector<uint32_t> messageLength;
    messageLength.push_back(vector.rows());

    //std::vector<double> message = {1,2,3,4};
    //messageLength.push_back(4);
    std::vector<double> message;
    for(uint i=0; i<vector.rows(); i++){
        message.push_back(vector[i]);
    }

    // send
    boost::system::error_code ignored_error;
    //boost::asio::write(*socket, boost::asio::buffer(echoCammand), ignored_error);
    boost::asio::write(socket, boost::asio::buffer(messageLength), ignored_error);
    boost::asio::write(socket, boost::asio::buffer(message), ignored_error);
}

