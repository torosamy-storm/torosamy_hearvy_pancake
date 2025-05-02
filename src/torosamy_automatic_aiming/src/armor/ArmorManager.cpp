#include "torosamy_automatic_aiming/armor/ArmorManager.h"


#include "torosamy_automatic_aiming/handler/classify/OnnxFilter.h"
#include "torosamy_automatic_aiming/handler/classify/DigitFilter.h"


ArmorManager::ArmorManager(const YAML::Node& fileReader) :
    mArmorFilter(fileReader),
    mEnableDraw(fileReader["DrawAll"].as<bool>()),
    mClassifyHandler(fileReader["Classify"]) {
    mDebugOptions.emplace_back(fileReader["Debug"]["angle"].as<bool>());
    mDebugOptions.emplace_back(fileReader["Debug"]["angleMate"].as<bool>());
    mDebugOptions.emplace_back(fileReader["Debug"]["rate"].as<bool>());
    mDebugOptions.emplace_back(fileReader["Debug"]["lengthRate"].as<bool>());
    mDebugOptions.emplace_back(fileReader["Debug"]["area"].as<bool>());
}
const std::vector<Armor>& ArmorManager::getArmors() const{
    return mArmors;
}

void ArmorManager::setArmors(const std::vector<Armor>& armors) {
    this->mArmors = armors;
}

bool ArmorManager::selectTarget() {
    if (mArmors.empty()) return false;

    if (mArmors.size() == 1) {
        this->mTargetArmor = mArmors.at(0);
        return true;
    }

    float area = -1;
    for (const Armor& armor : mArmors) {
        const float armorArea = armor.getArea();
        if(armorArea > area) {
            this->mTargetArmor = armor;
            area = armorArea;
        }
    }
    return true;
}


bool ArmorManager::findArmors(const std::vector<Light>& lights) {
    mArmors.clear();

    #pragma omp parallel for
    for (auto i = lights.begin(); i != lights.end(); i++) {
        for (auto j = i + 1; j != lights.end(); j++) {

            Armor armor = Armor(*i, *j);
            if (mArmorFilter.isArmor(armor)) {
                mArmors.emplace_back(armor);
            }
        }
    }
    if(!mArmors.empty()) return true;
    return false;
}

bool ArmorManager::findArmors(const cv::Mat& src) {
    if(!mClassifyHandler.enabled()) return true;

    std::vector<Armor> result;
    for (auto & armor : mArmors) {

        std::pair<ArmorType, float> classify = mClassifyHandler.classify(src, armor);
        const ArmorType type = classify.first;
        if (type == ArmorType::NEGATIVE) {
            continue;
        }
        
        armor.setArmorType(type);
        // if(type == ArmorType::OUTPOST) std::cout <<"yes"<<std::endl;
        // else std::cout<<"no"<<std::endl;
        // if (classify.second < mClassifyHandler.getMinConfidence()) continue;

        // std::cout <<static_cast<int>(type)<< "   "<<   classify.second <<std::endl;
        result.emplace_back(armor);
    }
    if (result.empty()) return false;
    mArmors = result;
    return true;
}


void ArmorManager::drawArmors(cv::Mat& outSrc) const{
    if (!mEnableDraw) return;
    for (const Armor& armor : mArmors) {
        armor.drawArmor(outSrc,mDebugOptions);
    }
}

void ArmorManager::drawTarget(cv::Mat& outSrc) const {
    mTargetArmor.drawArmor(outSrc, mDebugOptions);
}

const Armor& ArmorManager::getTargetArmor() const{
    return mTargetArmor;
}


void ArmorManager::resetTargetArmorType() {
    mTargetArmor.resetType();
}