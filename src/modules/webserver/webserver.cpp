#include "webserver.hpp"

#include "modules/movement/Drive.hpp"

void webserver::on_message(connection_hdl hdl, server::message_ptr msg) {
  std::string message = msg->get_payload();
  int instruction = stof(message);

  if (instruction == 1) {
    server::connection_ptr con = this->webserver_socket.get_con_from_hdl(hdl);

    cv::Mat frame = this->blackboard.frame.get();
    if (!frame.empty()) {
      std::vector<uchar> image_encoded;
      cv::imencode(".jpeg", frame, image_encoded);
      std::string image_encoded_string(image_encoded.begin(),
                                       image_encoded.end());
      std::string image_base64 = macaron::Base64::Encode(image_encoded_string);
      con->send(image_base64, websocketpp::frame::opcode::BINARY);
    }

    int current_exit = this->blackboard.current_exit.get();
    int next_intersection = this->blackboard.next_intersection.get();
    bool destination_reached = this->blackboard.destination_reached.get();

    std::string message_send = "{";
    message_send =
        message_send + "\"current_exit\":" + std::to_string(current_exit);
    message_send = message_send + ", \"next_intersection\":" +
                   std::to_string(next_intersection);
    message_send = message_send + ", \"destination_reached\":" +
                   std::to_string(destination_reached);
    message_send = message_send + "}";
    con->send(message_send);
  } else if (instruction == 2) {
    std::cout << "Program stopped" << std::endl;
    this->webserver_socket.stop();
    this->blackboard.running.set(false);
    IMovement &drive = Drive::getInstance();
    drive.coast();
  } else if (instruction >= 100 && instruction < 200) {
    std::cout << "Current exit chosen as " << instruction - 100 << std::endl;
    this->current_exit = instruction - 100;
  } else if (instruction >= 200 && instruction < 300) {
    std::cout << "Next intersection chosen as " << instruction - 200
              << std::endl;
    this->intersection_next = instruction - 200;

    this->blackboard.current_exit.set(this->current_exit);
    this->blackboard.next_intersection.set(this->intersection_next);
  } else if (instruction >= 300 && instruction < 400) {
    std::cout << "Destination " << instruction - 300 << " chosen" << std::endl;
    int destination = instruction - 300;
    this->blackboard.destination.set(destination);
    this->blackboard.destination_reached.set(false);
    this->blackboard.navigation_enabled.set(true);
    this->blackboard.intersection_handled.set(true);
    switch (destination) {
      case 8:
        this->blackboard.target_color_name.set("green");
        break;
      case 9:
        this->blackboard.target_color_name.set("orange");
        break;
      case 10:
        this->blackboard.target_color_name.set("red");
        break;
      case 11:
        this->blackboard.target_color_name.set("blue");
        break;
    }
  }
}

void webserver::run() {
  std::cout << "Webserver started" << std::endl;

  this->webserver_socket.set_access_channels(websocketpp::log::alevel::none);
  this->webserver_socket.set_error_channels(websocketpp::log::elevel::all);

  this->webserver_socket.init_asio();
  this->webserver_socket.set_message_handler(
      bind(&webserver::on_message, this, ::_1, ::_2));

  this->webserver_socket.listen(8765);
  this->webserver_socket.start_accept();

  this->webserver_socket.run();
}

webserver::~webserver() { this->webserver_socket.stop(); }
