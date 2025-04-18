#include "packet/SendDataPacket.h"
SendDataPacket::SendDataPacket(const int& id):
    SendDataPacketInterface(id) {
    this->mSize = 15;
    initData();
}

void SendDataPacket::initData() {
    this->pitch.f = 0;
    this->yaw.f = 0;
    this->distance.f = 0;
    this->isFindTarget.b = false;
    this->startFire.b = false;
    this->isTurnRight.b = false;
}

int SendDataPacket::writeData(unsigned char* dataArr,const int& startIndex) {
    int index = 0;
    dataArr[startIndex + index++] = this->pitch.c[0];
    dataArr[startIndex + index++] = this->pitch.c[1];
    dataArr[startIndex + index++] = this->pitch.c[2];
    dataArr[startIndex + index++] = this->pitch.c[3];



    dataArr[startIndex + index++] = this->yaw.c[0];
    dataArr[startIndex + index++] = this->yaw.c[1];
    dataArr[startIndex + index++] = this->yaw.c[2];
    dataArr[startIndex + index++] = this->yaw.c[3];


    dataArr[startIndex + index++] = this->distance.c[0];
    dataArr[startIndex + index++] = this->distance.c[1];
    dataArr[startIndex + index++] = this->distance.c[2];
    dataArr[startIndex + index++] = this->distance.c[3];

    dataArr[startIndex + index++] = this->isFindTarget.c[0];
    dataArr[startIndex + index++] = this->startFire.c[0];
    dataArr[startIndex + index++] = this->isTurnRight.c[0];


    // std::cout<<this->distance.f<<std::endl;
    return mSize;
}


