#include "torosamy_automatic_aiming/ros/ShootPublisher.h"


ShootPublisher::ShootPublisher(const int& id) : 
    Node("shoot_publisher_"+std::to_string(id)) {

    const std::string topic = "send_packet_"+std::to_string(id);
    mPublisher = this->create_publisher<SendDataMsg>(topic.c_str(), 10);

}

void ShootPublisher::publish() {
    mPublisher->publish(this->mPacket);
}