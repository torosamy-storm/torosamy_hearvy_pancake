//
// Created by torosamy on 25-4-3.
//
#include "HeartJumpModule.h"
#include "packet/RefereeSystemPacket.h"
#include "pod/manager/SerialPortManager.h"
#include "pod/manager/PacketManager.h"
#include "yaml-cpp/yaml.h"


HeartJumpModule::HeartJumpModule(const YAML::Node& fileReader) :
    Torosamy::TorosamyModule(fileReader["Id"].as<int>()),
    mLastHeartJumpNum(-1),
    mTimeOff(fileReader["TimeOff"].as<int>()),
    mConnectCounter(0){
    // const YAML::Node fileReader = YAML::LoadFile(getConfigLocation("heart_jump")+ "config.yml");
    // mReceivePacketId = fileReader["ReceivePacketId"].as<int>();
}

void HeartJumpModule::run() {
    const std::shared_ptr<RefereeSystemPacket>& packet = Torosamy::PacketManager::getInstance()->getReceivePacketById<RefereeSystemPacket>(2);
    while (true) {
        const short newNumber = packet->heart_jump_num.s;
        // std::cout << "last: "<<mLastHeartJumpNum<<", this: "<<newNumber<<std::endl;


        if(mLastHeartJumpNum == newNumber) mConnectCounter++;
        else {
            
            mConnectCounter = 0;
        }

        if(mConnectCounter > 4) {
            std::cout << "Error: fail to heart jump, try to reload serial port" << std::endl;
            Torosamy::SerialPortManager::getInstance()->reloadSerialPorts();
            Torosamy::SerialPortManager::getInstance()->detachTasks();
            std::cout << "reload serial port successfully"<< std::endl;
        }
        mLastHeartJumpNum = newNumber;
        std::this_thread::sleep_for(std::chrono::seconds(mTimeOff));
    }
}

std::shared_ptr<Torosamy::TorosamyModule> HeartJumpModule::makeModule() {
    return std::make_shared<HeartJumpModule>(YAML::LoadFile(getConfigLocation("uart")+ "config.yml")["HeartJump"]);
}
