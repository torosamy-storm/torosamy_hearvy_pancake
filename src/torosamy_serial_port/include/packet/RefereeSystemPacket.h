#ifndef REFEREESYSTEM_H
#define REFEREESYSTEM_H

#include "packet/ReceiveDataPacketInterface.h"
#include <memory>


class RefereeSystemPacket : public Torosamy::ReceiveDataPacketInterface {
    using short2uchar = Torosamy::short2uchar;
    using float2uchar = Torosamy::float2uchar;
    using bool2uchar = Torosamy::bool2uchar;
public:
    RefereeSystemPacket(const int& id);
    int readData(const unsigned char* const dataArr,const int& startIndex) override;
    void initData() override;
    short2uchar hp;
    short2uchar mode;
    short2uchar remain_time;
    short2uchar remain_exchange_times;
    short2uchar remain_bullet;
    short2uchar damage_reduction;

    bool2uchar is_our_outpost_break;
    bool2uchar is_enemy_outpost_break;
    bool2uchar game_start;
    bool2uchar is_fort_occupy;
    bool2uchar is_blue_us;
    short2uchar heart_jump_num; 
    
};



#endif //REFEREESYSTEM_H
