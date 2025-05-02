#ifndef SHOOTPUBLISHER_H
#define SHOOTPUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "torosamy_ros_msgs/msg/send_data_msg.hpp"


using torosamy_ros_msgs::msg::SendDataMsg;

class ShootPublisher : public rclcpp::Node {
public:
    ShootPublisher(const int& id);
    void publish();
    void initData();
    SendDataMsg mPacket;
private:
    rclcpp::Publisher<SendDataMsg>::SharedPtr mPublisher;
    //rclcpp::TimerBase::SharedPtr mPublishTimer;
};

#endif 