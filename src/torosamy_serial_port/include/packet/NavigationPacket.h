#ifndef CMDVELPACKET_H
#define CMDVELPACKET_H




#include "packet/SendDataPacketInterface.h"
#include <memory>

class NavigationPacket : public Torosamy::SendDataPacketInterface {
    using float2uchar = Torosamy::float2uchar;
    using short2uchar = Torosamy::short2uchar;
public:
    NavigationPacket(const int& id);
    int writeData(unsigned char* dataArr,const int& startIndex) override;
    void initData() override;

    float2uchar xSpeed;
    float2uchar ySpeed;
    short2uchar heartJumpNum;
    short2uchar serverStatus;
    short2uchar clientStatus;

    static bool enablePrintSpeed;
};



#endif //CMDVELPACKET_H
