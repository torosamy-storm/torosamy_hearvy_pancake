//
// Created by torosamy on 25-3-31.
//

#ifndef PREDICTHANDLER_H
#define PREDICTHANDLER_H

#include "torosamy_automatic_aiming/handler/predict/MotionPredicter.h"
#include "torosamy_automatic_aiming/handler/predict/PointPredicter.h"

enum class PredictMode {
    POINT,
    MOTION
};


class PredictHandler {
public:
    PredictHandler(const YAML::Node& fileReader);

    bool enabled() const;
    void disable();
    void enable();
    bool isUpdateResult() const;
    void setUpdateResult(const bool& updateResult);
    PredictMode getMode() const;
    void setMode(const PredictMode& mode);
    // bool isTurnRight() const;
    // const MotionPredicter& getMotionPredicter() const;
    // const PointPredicter& getPointPredicter() const;

    MotionPredicter mMotionPredicter;
    PointPredicter mPointPredicter;
private:


    bool mEnabled;
    bool mEnableUpdateResult;
    PredictMode mMode;
};

#endif //PREDICTHANDLER_H
