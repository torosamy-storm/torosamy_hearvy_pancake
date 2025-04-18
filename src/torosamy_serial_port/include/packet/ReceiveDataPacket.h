#ifndef RECEIVEDATAPACKET_H
#define RECEIVEDATAPACKET_H

#include "packet/ReceiveDataPacketInterface.h"
#include <memory>


class ReceiveDataPacket : public Torosamy::ReceiveDataPacketInterface {
    using short2uchar = Torosamy::short2uchar;
    using float2uchar = Torosamy::float2uchar;
public:
    ReceiveDataPacket(const int& id);
    int readData(const unsigned char* dataArr,const int& startIndex) override;
    void initData() override;
    float2uchar pitch;
    float2uchar yaw;
};



#endif
