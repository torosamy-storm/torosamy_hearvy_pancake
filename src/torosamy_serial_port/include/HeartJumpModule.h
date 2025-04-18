//
// Created by torosamy on 25-4-3.
//

#ifndef HEARTJUMPMODULE_H
#define HEARTJUMPMODULE_H
#include "module/TorosamyModule.h"
#include <memory>
#include <vector>
class HeartJumpModule : public Torosamy::TorosamyModule {
public:
    HeartJumpModule(const int& id);
    void run() override;

    static std::shared_ptr<Torosamy::TorosamyModule> makeModule();
private:
    int mConnectCounter;
    short mLastHeartJumpNum;
};

#endif //HEARTJUMPMODULE_H
