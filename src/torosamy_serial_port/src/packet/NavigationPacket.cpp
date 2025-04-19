#include "packet/NavigationPacket.h"

#include <iostream>
NavigationPacket::NavigationPacket(const int& id):
    SendDataPacketInterface(id) {
    this->mSize = 14;
    initData();
}


void NavigationPacket::initData() {
    this->xSpeed.f = 0;
    this->ySpeed.f = 0;
    this->heartJumpNum.s = 0;
    this->serverStatus.s = 0;
    this->clientStatus.s = 0;
}

int NavigationPacket::writeData(unsigned char* dataArr,const int& startIndex) {
    int index = 0;

    this->xSpeed.f = 0;
    this->ySpeed.f = 0;

    dataArr[startIndex + index++] = this->xSpeed.c[0];
    dataArr[startIndex + index++] = this->xSpeed.c[1];
    dataArr[startIndex + index++] = this->xSpeed.c[2];
    dataArr[startIndex + index++] = this->xSpeed.c[3];


    dataArr[startIndex + index++] = this->ySpeed.c[0];
    dataArr[startIndex + index++] = this->ySpeed.c[1];
    dataArr[startIndex + index++] = this->ySpeed.c[2];
    dataArr[startIndex + index++] = this->ySpeed.c[3];

    this->heartJumpNum.s = (this->heartJumpNum.s + 1) % 10;
    dataArr[startIndex + index++] = this->heartJumpNum.c[0];
    dataArr[startIndex + index++] = this->heartJumpNum.c[1];

    dataArr[startIndex + index++] = this->serverStatus.c[0];
    dataArr[startIndex + index++] = this->serverStatus.c[1];

    dataArr[startIndex + index++] = this->clientStatus.c[0];
    dataArr[startIndex + index] = this->clientStatus.c[1];
    return mSize;
}

