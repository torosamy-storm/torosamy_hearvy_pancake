#ifndef SHOOTSUBSCRIBER_H
#define SHOOTSUBSCRIBER_H


#include "rclcpp/rclcpp.hpp"
#include "torosamy_ros_msgs/msg/receive_data_msg.hpp"


using torosamy_ros_msgs::msg::ReceiveDataMsg;

class ShootSubscriber : public rclcpp::Node {
public:
    ShootSubscriber(const int& id);
    void subscribe(const ReceiveDataMsg& msg);
    ReceiveDataMsg mPacket;
private:
    rclcpp::Subscription<ReceiveDataMsg>::SharedPtr mSubscriber;
};

#endif 