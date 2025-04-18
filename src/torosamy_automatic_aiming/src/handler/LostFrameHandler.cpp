//
// Created by torosamy on 24-11-12.
//

#include "torosamy_automatic_aiming/handler/LostFrameHandler.h"

LostFrameHandler::LostFrameHandler() :
    mLostTrackCounter(5) {
}



void LostFrameHandler::trackTarget(){
    mLostTrackCounter = 0;
}

bool LostFrameHandler::isLostFrame() const{
    return mLostTrackCounter >= 5;
}


void LostFrameHandler::lostTrack() {
    mLostTrackCounter++;
}
