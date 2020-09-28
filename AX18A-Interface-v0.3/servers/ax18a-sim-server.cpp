
#include <iostream>
#include <boost/asio.hpp>
#include <thread>

#include "tcp/AX-18A-Comm.h"

#include "tcp/tcp_server.h"

#include "AX18ASimulatedInterface.h"

using namespace std;

int main(int argc, char* argv[]){

    try
    {
        boost::asio::io_service io_service;
        auto robot = AX18ASimulatedInterface::create();
        tcp_server server(io_service, AX18A_PORT, robot);
        thread modelUpdater([robot](){
            while(true){
                ((AX18ASimulatedInterface*)robot.get())->step();
                usleep(10*1000);
            }
        });
        io_service.run();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
