#include "packet/RefereeSystemPacket.h"
#include <iostream>
RefereeSystemPacket::RefereeSystemPacket(const int& id):
    ReceiveDataPacketInterface(id){
    this->mSize = 18;
    initData();
}


void RefereeSystemPacket::initData() {
    this->hp.s = 0;
    this->mode.s = 0;
    this->remain_time.s = 0;
    this->remain_exchange_times.s = 0;
    this->remain_bullet.s = 0;
    this->damage_reduction.s = 0;
    this->is_our_outpost_break.b = false;
    this->is_enemy_outpost_break.b = false;
    this->game_start.b = false;
    this->is_fort_occupy.b = false;

    this->heart_jump_num.s = 0;
}


int RefereeSystemPacket::readData(const unsigned char* const dataArr,const int& startIndex) {
    int index = 0;
    this->hp.c[0] = dataArr[startIndex + index++];
    this->hp.c[1] = dataArr[startIndex + index++];
    this->mode.c[0] = dataArr[startIndex + index++];
    this->mode.c[1] = dataArr[startIndex + index++];
    


    this->remain_time.c[0] = dataArr[startIndex + index++];
    this->remain_time.c[1] = dataArr[startIndex + index++];
    this->remain_exchange_times.c[0] = dataArr[startIndex + index++];
    this->remain_exchange_times.c[1] = dataArr[startIndex + index++];



    this->remain_bullet.c[0] = dataArr[startIndex + index++];
    this->remain_bullet.c[1] = dataArr[startIndex + index++];
    this->damage_reduction.c[0] = dataArr[startIndex + index++];
    this->damage_reduction.c[1] = dataArr[startIndex + index++];


    
    this->is_our_outpost_break.c[0] = dataArr[startIndex + index++];
    this->is_enemy_outpost_break.c[0] = dataArr[startIndex + index++];
    this->game_start.c[0] = dataArr[startIndex + index++];
    this->is_fort_occupy.c[0] = dataArr[startIndex + index++];
 

    this->heart_jump_num.c[0] = dataArr[startIndex + index++];
    this->heart_jump_num.c[1] = dataArr[startIndex + index];


    // std::cout<<this->heart_jump_num.s<<std::endl;
    return mSize;
}