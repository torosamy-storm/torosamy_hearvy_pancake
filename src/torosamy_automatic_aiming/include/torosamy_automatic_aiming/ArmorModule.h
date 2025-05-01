//
// Created by torosamy on 24-11-12.
//

#ifndef ARMORMODULE_H
#define ARMORMODULE_H

#include "module/TorosamyModule.h"
#include "pod/camera/VideoCamera.h"
#include "pod/camera/MindCamera/MindCamera.h"

#include "torosamy_automatic_aiming/light/LightManager.h"
#include "torosamy_automatic_aiming/armor/ArmorManager.h"

#include "torosamy_automatic_aiming/handler/LostFrameHandler.h"
#include "torosamy_automatic_aiming/handler/FireControlHandler.h"

// #include "torosamy_automatic_aiming/packet/ReceiveDataPacket.h"
// #include "torosamy_automatic_aiming/packet/SendDataPacket.h"
#include "torosamy_automatic_aiming/ros/ShootPublisher.h"
#include "torosamy_automatic_aiming/ros/ShootSubscriber.h"

#include "torosamy_automatic_aiming/handler/predict/PredictHandler.h"


class ArmorModule : public Torosamy::TorosamyModule{
public:
    ArmorModule(const int& id);
    void run() override;
    bool doOnce();
    void enableShow();
    void disableShow();
    const std::shared_ptr<ShootSubscriber>& getSubscriber() const;
    static YAML::Node getConfigNode(const int& id);
    static std::vector<std::shared_ptr<TorosamyModule>> makeModules();
    static std::vector<int> getIds();
private:
    bool findArmor();
    bool handlerLostTrack();
    
    ArmorPnpResult getPnpResult();
    bool shouldFire() const;
    // bool isTurnRight() const;
    void drawParams(const Torosamy::TIME& startTime);

    // void drawParams(const double& time) const;


    std::shared_ptr<Torosamy::MindCamera> mCamera;
    // std::shared_ptr<Torosamy::VideoCamera> mCamera;
    // std::shared_ptr<SendDataPacket> mSendDataPacket;
    // std::shared_ptr<ReceiveDataPacket> mReceiveDataPacket;


    std::shared_ptr<ShootPublisher> mShootPublisher;
    std::shared_ptr<ShootSubscriber> mShootSubscriber;
    
    LightManager mLightManager;
    ArmorManager mArmorManager;



    FireControlHandler mFireControlHandler;
    LostFrameHandler mLostFrameHandler;
    PredictHandler mPredictHandler;

    // double mLastWorldCenterX;
    bool mShowSrc;
    cv::Mat mSrc;
    static std::vector<int> mIds;
};

#endif //ARMORMODULE_H
