#ifndef SERIALPORTDECORATOR_H
#define SERIALPORTDECORATOR_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "torosamy_ros_msgs/msg/send_data_msg.hpp"
#include "torosamy_ros_msgs/msg/receive_data_msg.hpp"
#include "torosamy_ros_msgs/msg/referee_system_msg.hpp"
#include "torosamy_ros_msgs/msg/navigation_client.hpp"
using torosamy_ros_msgs::msg::SendDataMsg;
using torosamy_ros_msgs::msg::ReceiveDataMsg;
using torosamy_ros_msgs::msg::RefereeSystemMsg;
using torosamy_ros_msgs::msg::NavigationClient;
class SerialPortDecorator : public rclcpp::Node {
public:
    SerialPortDecorator();
    //void subscribeSendData(const geometry_msgs::msg::Twist::SharedPtr twist) const;

    void subscribeSendData0(const SendDataMsg& msg) const;
    void publishReceiveData0();

    void subscribeSendData1(const SendDataMsg& msg) const;
    void publishReceiveData1();


    void publishRefereeSystem();
    void subscribeCmdVel(const geometry_msgs::msg::Twist::SharedPtr twist) const;
    void subscribeStatus(const NavigationClient& msg) const;
private:
    rclcpp::Subscription<SendDataMsg>::SharedPtr mSendDataSubscriber0;
    rclcpp::Publisher<ReceiveDataMsg>::SharedPtr mReceiveDataPublisher0;
    rclcpp::TimerBase::SharedPtr mReceiveDataTimer0;
    
    rclcpp::Subscription<SendDataMsg>::SharedPtr mSendDataSubscriber1;
    rclcpp::Publisher<ReceiveDataMsg>::SharedPtr mReceiveDataPublisher1;
    rclcpp::TimerBase::SharedPtr mReceiveDataTimer1;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mCmdVelSubscriber;
    rclcpp::Subscription<NavigationClient>::SharedPtr mNavigationClientSubscriber;
    rclcpp::Publisher<RefereeSystemMsg>::SharedPtr mRefereeSystemPublisher;
    rclcpp::TimerBase::SharedPtr mRefereeSystemTimer;
};


#endif 