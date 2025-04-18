//
// Created by torosamy on 25-3-31.
//
#include "torosamy_automatic_aiming/handler/predict/PredictHandler.h"

PredictHandler::PredictHandler(const YAML::Node& fileReader) :
    mEnabled(fileReader["enable"].as<bool>()),
    mEnableUpdateResult(fileReader["updateResult"].as<bool>()),
    mMode(static_cast<PredictMode>(fileReader["mode"].as<int>())),
    mMotionPredicter(fileReader["motion"]),
    mPointPredicter(fileReader["point"]){
}




bool PredictHandler::enabled() const {
    return mEnabled;
}
void PredictHandler::disable() {
    mEnabled = false;
}
void PredictHandler::enable() {
    mEnabled = true;
}

bool PredictHandler::isUpdateResult() const {
    return mEnableUpdateResult;
}
void PredictHandler::setUpdateResult(const bool& updateResult) {
    this->mEnableUpdateResult = updateResult;
}

PredictMode PredictHandler::getMode() const {
    return mMode;
}
void PredictHandler::setMode(const PredictMode& mode) {
    this->mMode = mode;
}

// bool PredictHandler::isTurnRight() const {
//     // if (PredictMode::MOTION == mMode) return mMotionPredicter.isTurnRight();

//     // return mPointPredicter.isTurnRight();
// }



// TODO
// const MotionPredicter& PredictHandler::getMotionPredicter() const {
//     return mMotionPredicter;
// }
// const PointPredicter& PredictHandler::getPointPredicter() const {
//     return mPointPredicter;
// }