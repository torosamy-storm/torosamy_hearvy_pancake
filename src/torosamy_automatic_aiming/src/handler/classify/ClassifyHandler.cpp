//
// Created by torosamy on 25-3-29.
//
#include "torosamy_automatic_aiming/handler/classify/ClassifyHandler.h"
#include "torosamy_automatic_aiming/handler/classify/OnnxFilter.h"
#include "torosamy_automatic_aiming/handler/classify/DigitFilter.h"

ClassifyHandler::ClassifyHandler(const YAML::Node& fileReader) :
	mClassifyMode(static_cast<ClassifyMode>(fileReader["mode"].as<int>())),
    mEnable(fileReader["enable"].as<bool>()),
    mOnnxMinConfidence(fileReader["minConfidence"]["onxx"].as<float>()) {

    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["ONE"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["TWO"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["THREE"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["FOUR"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["FIVE"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["OUTPOST"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["GUARD"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["BASE"].as<float>());
    mDigitMinConfidences.push_back(fileReader["minConfidence"]["digit"]["NEGATIVE"].as<float>());

    // for(int i = 0 ; i<mDigitMinConfidences.size() ; i++) std::cout << mDigitMinConfidences.at(i) <<std::endl;
}




std::pair<ArmorType, float> ClassifyHandler::classify(const cv::Mat& src, const Armor& armor) const {
    switch (mClassifyMode) {
        case ClassifyMode::ONNX:
            return OnnxFilter(armor, src).classify(mOnnxMinConfidence);
        case ClassifyMode::DIGIT:
            return DigitFilter(armor, src).classify(mDigitMinConfidences);
        default:
            throw std::runtime_error("unknown classify mode");
    }
}


bool ClassifyHandler::enabled() const {
    return mEnable;
}

// float ClassifyHandler::getMinConfidence() const {
//     switch (mClassifyMode) {
//         case ClassifyMode::ONNX:
//             return mOnnxMinConfidence;
//         case ClassifyMode::DIGIT:
//             return mDigitMinConfidence;
//         default:
//             throw std::runtime_error("unknown classify mode");
//     }
// }