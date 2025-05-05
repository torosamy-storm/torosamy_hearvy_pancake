//
// Created by torosamy on 25-4-3.
//

#ifndef HEARTJUMPMODULE_H
#define HEARTJUMPMODULE_H
#include "module/TorosamyModule.h"
#include <memory>
#include <vector>
#include <yaml-cpp/node/node.h>
class HeartJumpModule : public Torosamy::TorosamyModule {
public:
    HeartJumpModule(const YAML::Node& fileReader);
    void run() override;

    static std::shared_ptr<Torosamy::TorosamyModule> makeModule();
private:
    const int mTimeOff;
    int mConnectCounter;
    short mLastHeartJumpNum;
};

#endif //HEARTJUMPMODULE_H
