#include "SerialPortDecorator.h"
#include "packet/ReceiveDataPacket.h"
#include "packet/SendDataPacket.h"
#include "packet/NavigationPacket.h"
#include "packet/RefereeSystemPacket.h"
#include "pod/manager/PacketManager.h"
#include "pod/manager/ModuleManager.h"
#include "pod/manager/SerialPortManager.h"
#include "HeartJumpModule.h"
int main(int argc, char* argv[]) {
    using namespace Torosamy;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPortDecorator>();

    PacketManager::getInstance()
        ->addReceivePacket(std::make_shared<ReceiveDataPacket>(0))
        ->addReceivePacket(std::make_shared<ReceiveDataPacket>(1))
        ->addReceivePacket(std::make_shared<RefereeSystemPacket>(2))
        ->addSendPacket(std::make_shared<SendDataPacket>(0))
        ->addSendPacket(std::make_shared<SendDataPacket>(1))
        ->addSendPacket(std::make_shared<NavigationPacket>(2));

    SerialPortManager::getInstance()->detachTasks();

    ModuleManager::getInstance()
        ->addModule(HeartJumpModule::makeModule())
        ->detachTasks();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
