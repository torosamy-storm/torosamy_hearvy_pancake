//
// Created by torosamy on 25-3-29.
//

#ifndef CLASSIFYHANDLER_H
#define CLASSIFYHANDLER_H
#include "yaml-cpp/yaml.h"
#include "torosamy_automatic_aiming/armor/Armor.h"
enum class ClassifyMode {
    ONNX,
    DIGIT
};

class ClassifyHandler {
public:
    ClassifyHandler(const YAML::Node& fileReader);
    std::pair<ArmorType, float> classify(const cv::Mat& src, const Armor& armor) const;
    bool enabled() const;
    float getMinConfidence() const;
private:
    const ClassifyMode mClassifyMode;
    const bool mEnable;
    const float mOnnxMinConfidence;
    std::vector<float> mDigitMinConfidences;
};

#endif //CLASSIFYHANDLER_H
