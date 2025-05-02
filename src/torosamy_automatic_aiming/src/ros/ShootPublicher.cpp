#include "torosamy_automatic_aiming/ros/ShootPublisher.h"


ShootPublisher::ShootPublisher(const int& id) : 
    Node("shoot_publisher_"+std::to_string(id)) {

    const std::string topic = "send_packet_"+std::to_string(id);
    mPublisher = this->create_publisher<SendDataMsg>(topic.c_str(), 10);

}

void ShootPublisher::initData() {
    mPacket.yaw = 0.0f;
    mPacket.pitch = 0.0f;
    mPacket.distance = 0.0f;
    mPacket.is_find_target = false;
    mPacket.start_fire = false;
    mPacket.is_turn_right = false;
}

void ShootPublisher::publish() {
    mPublisher->publish(this->mPacket);
}