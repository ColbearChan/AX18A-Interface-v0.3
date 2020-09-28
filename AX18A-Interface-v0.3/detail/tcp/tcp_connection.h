//
// Created by matthias on 11/03/18.
//

#ifndef AX18A_SERVERCLIENT_TCP_CONNECTION_H
#define AX18A_SERVERCLIENT_TCP_CONNECTION_H

#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include "../../ax18a/AX18ARobotInterface.h"

class tcp_connection : public boost::enable_shared_from_this<tcp_connection>{
public:
    typedef boost::shared_ptr<tcp_connection> pointer;

    static pointer create(boost::asio::io_service& io_service, std::shared_ptr<AX18ARobotInterface> robotP);

    boost::asio::ip::tcp::socket& socket();

    void start();

private:
    void handleClient();
    tcp_connection(boost::asio::io_service& io_service, std::shared_ptr<AX18ARobotInterface> robotP);
private:
    boost::asio::ip::tcp::socket socket_;
    std::shared_ptr<AX18ARobotInterface> robot;
};


#endif //AX18A_SERVERCLIENT_TCP_CONNECTION_H
