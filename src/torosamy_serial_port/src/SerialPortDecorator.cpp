#include "SerialPortDecorator.h"
#include "packet/ReceiveDataPacket.h"
#include "packet/SendDataPacket.h"
#include "packet/NavigationPacket.h"
#include "packet/RefereeSystemPacket.h"
#include "pod/manager/PacketManager.h"
using namespace Torosamy;
SerialPortDecorator::SerialPortDecorator() : Node("serial_port_node") {
    //0 shoot
    mSendDataSubscriber0 = this->create_subscription<SendDataMsg>(
        "/send_packet_0", 10, std::bind(&SerialPortDecorator::subscribeSendData0, this, std::placeholders::_1)
    );
    mReceiveDataPublisher0 = this->create_publisher<ReceiveDataMsg>("receive_packet_0", 10);
    mReceiveDataTimer0 = this->create_wall_timer(
        std::chrono::microseconds(1), std::bind(&SerialPortDecorator::publishReceiveData0, this)
    );
    


    //1 shoot
    mSendDataSubscriber1 = this->create_subscription<SendDataMsg>(
        "/send_packet_1", 10, std::bind(&SerialPortDecorator::subscribeSendData1, this, std::placeholders::_1)
    );
    mReceiveDataPublisher1 = this->create_publisher<ReceiveDataMsg>("receive_packet_1", 10);
    mReceiveDataTimer1 = this->create_wall_timer(
        std::chrono::microseconds(1), std::bind(&SerialPortDecorator::publishReceiveData1, this)
    );




    //navigation
    mRefereeSystemPublisher = this->create_publisher<RefereeSystemMsg>("referee_system", 10);
    mRefereeSystemTimer = this->create_wall_timer(
        std::chrono::microseconds(1), std::bind(&SerialPortDecorator::publishRefereeSystem, this)
    );

    mCmdVelSubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&SerialPortDecorator::subscribeCmdVel, this, std::placeholders::_1)
    );
    mNavigationClientSubscriber = this->create_subscription<NavigationClient>(
        "/navigation_status", 10, std::bind(&SerialPortDecorator::subscribeStatus, this, std::placeholders::_1)
    );
}



void SerialPortDecorator::subscribeSendData0(const SendDataMsg& msg) const {
    std::shared_ptr<SendDataPacket> sendDataPacket = PacketManager::getInstance()->getSendPacketById<SendDataPacket>(0);
    sendDataPacket->pitch.f = msg.pitch;
    sendDataPacket->yaw.f = msg.yaw;
    sendDataPacket->distance.f = msg.distance;

    sendDataPacket->isFindTarget.b = msg.is_find_target;
    sendDataPacket->startFire.b = msg.start_fire;
    sendDataPacket->isTurnRight.b = msg.is_turn_right;
}
void SerialPortDecorator::publishReceiveData0() {
    auto receiveDataMsg = ReceiveDataMsg();

    std::shared_ptr<ReceiveDataPacket> receiveDataPacket = PacketManager::getInstance()->getReceivePacketById<ReceiveDataPacket>(0);

    receiveDataMsg.pitch = receiveDataPacket->pitch.f;
    receiveDataMsg.yaw = receiveDataPacket->yaw.f;


    mReceiveDataPublisher0->publish(receiveDataMsg);
}







void SerialPortDecorator::subscribeSendData1(const SendDataMsg& msg) const {
    std::shared_ptr<SendDataPacket> sendDataPacket = PacketManager::getInstance()->getSendPacketById<SendDataPacket>(1);
    sendDataPacket->pitch.f = msg.pitch;
    sendDataPacket->yaw.f = msg.yaw;
    sendDataPacket->distance.f = msg.distance;
    sendDataPacket->isFindTarget.b = msg.is_find_target;
    sendDataPacket->startFire.b = msg.start_fire;
    sendDataPacket->isTurnRight.b = msg.is_turn_right;
}
void SerialPortDecorator::publishReceiveData1() {
    auto receiveDataMsg = ReceiveDataMsg();

    std::shared_ptr<ReceiveDataPacket> receiveDataPacket = PacketManager::getInstance()->getReceivePacketById<ReceiveDataPacket>(1);

    receiveDataMsg.pitch = receiveDataPacket->pitch.f;
    receiveDataMsg.yaw = receiveDataPacket->yaw.f;


    mReceiveDataPublisher1->publish(receiveDataMsg);
}






void SerialPortDecorator::publishRefereeSystem() {
    auto msg = RefereeSystemMsg();

    std::shared_ptr<RefereeSystemPacket> packet = PacketManager::getInstance()->getReceivePacketById<RefereeSystemPacket>(2);


    msg.hp = packet->hp.s;
    msg.mode = packet->mode.s;
    msg.remain_time = packet->remain_time.s;
    msg.remain_exchange_times = packet->remain_exchange_times.s;
    msg.remain_bullet = packet->remain_bullet.s;
    msg.damage_reduction = packet->damage_reduction.s;


    msg.is_our_outpost_break = packet->is_our_outpost_break.b;
    msg.is_enemy_outpost_break = packet->is_enemy_outpost_break.b;
    msg.game_start = packet->game_start.b;
    msg.is_fort_occupy = packet->is_fort_occupy.b;


    mRefereeSystemPublisher->publish(msg);
}

void SerialPortDecorator::subscribeStatus(const NavigationClient& msg) const {
    std::shared_ptr<NavigationPacket> packet = PacketManager::getInstance()->getSendPacketById<NavigationPacket>(2);

    packet->serverStatus.s = msg.server_status;
    packet->clientStatus.s = msg.client_status;
}

void SerialPortDecorator::subscribeCmdVel(const geometry_msgs::msg::Twist::SharedPtr twist) const {
    std::shared_ptr<NavigationPacket> packet = PacketManager::getInstance()->getSendPacketById<NavigationPacket>(2);

    packet->xSpeed.f = twist->linear.x;
    packet->ySpeed.f = twist->linear.y;
}