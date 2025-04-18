#include "torosamy_automatic_aiming/ros/ShootSubscriber.h"


ShootSubscriber::ShootSubscriber(const int& id) : 
    Node("shoot_subscriber_"+std::to_string(id)) {

    const std::string topic = "/receive_packet_"+std::to_string(id);
    
    mSubscriber = this->create_subscription<ReceiveDataMsg>(
        topic.c_str(), 10, std::bind(&ShootSubscriber::subscribe, this, std::placeholders::_1)
    );

}

void ShootSubscriber::subscribe(const ReceiveDataMsg& msg) {
    mPacket.pitch = msg.pitch;
    mPacket.yaw = msg.yaw;
}