//
// Created by torosamy on 25-3-27.
//

#ifndef FIRECONTROLHANDLER_H
#define FIRECONTROLHANDLER_H

#include "torosamy_automatic_aiming/armor/ArmorPnpResult.h"
#include "yaml-cpp/yaml.h"

class FireControlHandler {
public:
    FireControlHandler(const YAML::Node& fileReader);
    float getPitch(const ArmorPnpResult& ArmorPnpResult);
    float getYaw(const ArmorPnpResult& ArmorPnpResult) const;

    void updateFilghtTime(const float& depth, const float& pitch);

    float getFilghtTime() const;
private:
    float bulletModel(const float& pitch) const;
    float computePitch(const float& depth, const float& height);
    std::vector<float> mYawOffestsArmor;
    std::vector<float> mYawOffestsOutpost;

    const float mBulletPitchArmor;
    std::vector<float> mPitchOffestsArmor;

    const float mBulletPitchOutpost;
    std::vector<float> mPitchOffestsOutpost;

    float mFilghtTime;

    static const float V;  //28 30 弹速
    //用0.02510132乘当前空气密度
    //23.7摄氏度下为1.192
    //15摄氏度下为1.225
    static const float K; //0.0300f,0269f,0.020
    static const float  GRAVITY;        //重力g
};


#endif //FIRECONTROLHANDLER_H
