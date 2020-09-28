//
// Created by matthias on 11/03/18.
//

#include "tcp_server.h"

#include <boost/bind.hpp>
#include <thread>

using boost::asio::ip::tcp;
using namespace std;

tcp_server::tcp_server(boost::asio::io_service& io_service, short port, std::shared_ptr<AX18ARobotInterface> robotP)
: acceptor_(io_service, tcp::endpoint(tcp::v4(), port)), robot(robotP)
{
start_accept();
}

void tcp_server::start_accept()
{
    tcp_connection::pointer new_connection =
            tcp_connection::create(acceptor_.get_io_service(), robot);

    acceptor_.async_accept(new_connection->socket(),
                           boost::bind(&tcp_server::handle_accept, this, new_connection,
                                       boost::asio::placeholders::error));
}

void tcp_server::handle_accept(tcp_connection::pointer new_connection,
                   const boost::system::error_code& error){
    if (!error)
    {
        thread newThread(
                [new_connection](){new_connection->start();}
        );
        newThread.detach();
        //new_connection->start();
    }

    start_accept();
}