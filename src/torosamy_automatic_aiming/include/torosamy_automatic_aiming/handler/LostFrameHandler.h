//
// Created by torosamy on 24-11-12.
//

#ifndef LOSTFRAMEHANDLER_H
#define LOSTFRAMEHANDLER_H
//#include "torosamy_automatic_aiming/armor/Armor.h"
class LostFrameHandler {
public:
    LostFrameHandler();
    void trackTarget();
    bool isLostFrame() const;
    void lostTrack();
    //void updateAttribute(const Armor& newArmor);
private:
//    Armor mTargetArmorLast;
    double mNowBetweenLastDistance;
    //是否为第一帧标志位
    bool mIsFirstFrame; //
    //是否跟踪标志位
    bool mIsTrack; //
    //装甲板掉帧模式帧数
    int mLostTrackCounter; //
    //跟踪计数
    int mTrackerCount; //
};


#endif //LOSTFRAMEHANDLER_H
