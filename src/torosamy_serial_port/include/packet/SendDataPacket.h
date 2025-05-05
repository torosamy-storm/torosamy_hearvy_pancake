#ifndef SENDDATAPACKET_H
#define SENDDATAPACKET_H


#include "packet/SendDataPacketInterface.h"
#include <memory>



class SendDataPacket : public Torosamy::SendDataPacketInterface{
    using short2uchar = Torosamy::short2uchar;
    using float2uchar = Torosamy::float2uchar;
    using bool2uchar = Torosamy::bool2uchar;
public:
    SendDataPacket(const int& id);
    int writeData(unsigned char* dataArr,const int& startIndex) override;
    void initData() override;
    float2uchar pitch;
    float2uchar yaw;
    float2uchar distance;

    bool2uchar isFindTarget;
    bool2uchar startFire;
    bool2uchar isTurnRight;

};


#endif //SENDDATAPACKET_H
