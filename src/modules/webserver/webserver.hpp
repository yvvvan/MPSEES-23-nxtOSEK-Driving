#ifndef GEORDI_WEBSERVER_HPP
#define GEORDI_WEBSERVER_HPP

#include "communication/internal/BlackBoard.hpp"
#include <opencv2/opencv.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/connection.hpp>
#include <base64/base64.h>
#include <iostream>
#include <chrono>
#include <thread>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class webserver {
    GEORDI_PUBLIC:
    webserver() = default;
    ~webserver();
     void run();
    GEORDI_PRIVATE:
     //blackboard
     BlackBoard &blackboard{BlackBoard::getInstance()};

     //websocket
     server webserver_socket;

     int current_exit = -1;
     int intersection_next = -1;

     void on_message(connection_hdl hdl, server::message_ptr msg);
};

#endif //GEORDI_WEBSERVER_HPP