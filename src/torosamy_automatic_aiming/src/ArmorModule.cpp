#include "torosamy_automatic_aiming/ArmorModule.h"

#include <pod/manager/ModuleManager.h>

#include "torosamy_automatic_aiming/armor/ArmorPnpResult.h"

#include "utils/MessageUtils.h"
#include "pod/manager/CameraManager.h"
#include "pod/manager/PacketManager.h"


std::vector<int> ArmorModule::mIds;

void ArmorModule::run() {
    using namespace Torosamy;
    const std::string id = std::to_string(mId);
    while (mRunning) {
        try{
            const TIME startTime = MessageUtils::getTimePoint();
            mCamera->updateSrc();
            mCamera->cloneSrc(mSrc);

            if(mSrc.empty()) continue;
            doOnce();
            mShootPublisher->publish();


    		if(mShowSrc) {
                drawParams(startTime);
                imshow(id, mSrc);
            }else {
                cv::Mat img(cv::Size(50, 20), CV_8UC3, cv::Scalar(0, 0, 0));
                Torosamy::MessageUtils::putText(img, std::to_string(Torosamy::MessageUtils::getFpsByTimePoint(startTime)), cv::Point2f(0, 20));
                cv::imshow("fps_"+id, img);
            }


    		cv::waitKey(mCamera->getTimeOff());

        }catch(const std::runtime_error& e) {
            std::cerr<<e.what()<<std::endl;
        }
    }
}

void ArmorModule::doOnce() {
    if(!findArmor()) return;
    if(!mArmorManager.selectTarget()) {
    	handlerLostTrack();
    	return;
	}

    const ArmorPnpResult& targetArmorPnpResult = getPnpResult();


	mLostFrameHandler.trackTarget();
    mArmorManager.drawTarget(mSrc);

    mShootPublisher->mPacket.yaw = mFireControlHandler.getYaw(targetArmorPnpResult);
    mShootPublisher->mPacket.pitch = mFireControlHandler.getPitch(targetArmorPnpResult);
    mShootPublisher->mPacket.is_find_target = true;
    mShootPublisher->mPacket.distance = targetArmorPnpResult.getDistance();
    mShootPublisher->mPacket.start_fire = shouldFire();
}


bool ArmorModule::findArmor() {

	if(!mLightManager.findLights(mSrc, Torosamy::targetColor)) {
    	handlerLostTrack();
    	return false;
	}
    
    if(!mArmorManager.findArmors(mLightManager.getLights())) {
        handlerLostTrack();
    	return false;
    }
    
    if (!mArmorManager.findArmors(mSrc)) {
        handlerLostTrack();
        return false;
    }

    return true;
}




ArmorPnpResult ArmorModule::getPnpResult() {
    const Armor& targetArmor = mArmorManager.getTargetArmor();
    const ArmorPnpResult& armor_pnp_result = ArmorPnpResult(
        mCamera->getCameraMatrix(),
        mCamera->getDistCoeffs(),
        mShootSubscriber->mPacket.pitch,
        mShootSubscriber->mPacket.yaw,
        targetArmor
    );

    if (!mPredictHandler.enabled()) return armor_pnp_result;

    if (mPredictHandler.getMode() == PredictMode::POINT) {
        const ArmorType& armorType = targetArmor.getArmorType();

        const cv::Point2f predictPoint = armorType == ArmorType::OUTPOST ?
        mPredictHandler.mPointPredicter.getPredictPoint(mFireControlHandler.getFilghtTime(), targetArmor,mShootSubscriber->mPacket.yaw,mShootSubscriber->mPacket.pitch) :
        mPredictHandler.mPointPredicter.getPredictPoint(mFireControlHandler.getFilghtTime(), targetArmor);

        //TODO
        mShootPublisher->mPacket.is_turn_right = predictPoint.x > targetArmor.getCenter().x;

        if (mPredictHandler.isUpdateResult()) {
            return ArmorPnpResult(
                mCamera->getCameraMatrix(),
                mCamera->getDistCoeffs(),
                mShootSubscriber->mPacket.pitch,
                mShootSubscriber->mPacket.yaw,
                targetArmor.generatePredictArmor(predictPoint)
            );
        }
    }





    if (mPredictHandler.getMode() == PredictMode::MOTION) {
        if (mLostFrameHandler.isLostFrame()) {
            mPredictHandler.mMotionPredicter.init(armor_pnp_result);
        }

        std::vector<ArmorPnpResult> armorPnpResults;
        for (const auto& armor : mArmorManager.getArmors()) {
            armorPnpResults.emplace_back(
                ArmorPnpResult(
                    mCamera->getCameraMatrix(),
                    mCamera->getDistCoeffs(),
                    mShootSubscriber->mPacket.pitch,
                    mShootSubscriber->mPacket.yaw,
                    armor
                )
            );
        }
        if (mPredictHandler.mMotionPredicter.update(armorPnpResults)) {
            
            //TODO
            mShootPublisher->mPacket.is_turn_right = mPredictHandler.mMotionPredicter.isTurnRight();

            if (mPredictHandler.isUpdateResult()) return armor_pnp_result;


            const Eigen::Vector3d& tvec = mPredictHandler.mMotionPredicter.generatePredictTvec(
                mFireControlHandler.getFilghtTime(),
                mShootSubscriber->mPacket.pitch,
                mShootSubscriber->mPacket.yaw
            );
            
            return ArmorPnpResult(tvec, targetArmor);
        }

        mShootPublisher->mPacket.is_turn_right = false;

    }

    return armor_pnp_result;
}


bool ArmorModule::handlerLostTrack() {
  	mLostFrameHandler.lostTrack();
    if (mLostFrameHandler.isLostFrame()) {
        mShootPublisher->mPacket.yaw = 0.0f;
        mShootPublisher->mPacket.pitch = 0.0f;
        mShootPublisher->mPacket.distance = 0.0f;
        mShootPublisher->mPacket.is_find_target = false;
        mShootPublisher->mPacket.start_fire = false;
        mShootPublisher->mPacket.is_turn_right = false;
        return true;
    }
    return false;
}




std::vector<std::shared_ptr<Torosamy::TorosamyModule>> ArmorModule::makeModules() {
    std::vector<std::shared_ptr<Torosamy::TorosamyModule>> result;

    const YAML::Node fileReader = YAML::LoadFile(Torosamy::TorosamyModule::getConfigLocation("armor")+"config.yml");

    std::vector<int> ids;
    for(const auto& idNode : fileReader["LoadIds"]) {
        ids.push_back(idNode.as<int>());
    }


    if(ids.empty()) {
        std::cout<<"no id for armor found"<<std::endl;
        return result;
    }

    for(const int& id : ids) {
        if (Torosamy::ModuleManager::getInstance()->isRepeat(id)) continue;
        mIds.emplace_back(id);
        result.emplace_back(std::make_shared<ArmorModule>(id));
    }

    return result;
}

ArmorModule::ArmorModule(const int& id) : 
    TorosamyModule(id),
    // mLastWorldCenterX(0),
    mShootPublisher(std::make_shared<ShootPublisher>(id)),
    mShootSubscriber(std::make_shared<ShootSubscriber>(id)), 
    mLightManager(getConfigNode(id)["Light"]),
    mArmorManager(getConfigNode(id)["Armor"]),
    mFireControlHandler(getConfigNode(id)["FireControl"]),
    mPredictHandler(getConfigNode(id)["Predict"]){
	std::string stringId = std::to_string(id);
    const YAML::Node fileReader = getConfigNode(id);

    mShowSrc = fileReader["ShowSrc"].as<bool>();
    mCamera = Torosamy::CameraManager::getInstance()->getCameraById<Torosamy::MindCamera>(fileReader["CameraID"].as<int>());
}
// bool ArmorModule::isTurnRight() const {
    // const PredictMode mode = mPredictHandler.getMode();

    // if (PredictMode::MOTION == mode) return mPredictHandler.mMotionPredicter.isTurnRight();


    // mPredictHandler.getPointPredicter().

    //     if(mLastWorldCenterX == 0) {return false;}
//     // std::cout<<"mlast:"<<mLastWorldCenterX<<std::endl;
//     // std::cout<<"nowX:"<<nowX<<std::endl;

//     // const double dx = fabs(nowX - mLastWorldCenterX);
//     const double dx = nowX - mLastWorldCenterX;

//     if (fabs(dx) > 1) return false;//ci ke wei huan ban
//     if(nowX > mLastWorldCenterX||0<dx<0.5) {return true;}

//     // if(dx < 0 && fabs(dx)<1) {return false;}
    

//     return false;
// }

bool ArmorModule::shouldFire() const{
    if (!mShootPublisher->mPacket.is_find_target) return false;

	const float yaw = fabs(mShootPublisher->mPacket.yaw);
    const float pitch = fabs(mShootPublisher->mPacket.pitch);
    const float distance = mShootPublisher->mPacket.distance;


    if (pitch >= 1.2) return false;
    if (yaw < 1.0 && pitch < 1.0) return true;
    return false;
}


void ArmorModule::drawParams(const Torosamy::TIME& startTime) {
    using namespace Torosamy;

    std::string params[9] = {
        "pitch:" + std::to_string(mShootPublisher->mPacket.pitch),
        "yaw:" + std::to_string(mShootPublisher->mPacket.yaw),
        "distance:" + std::to_string(mShootPublisher->mPacket.distance),
        "find_target:" + std::to_string(mShootPublisher->mPacket.is_find_target),
        "start_fire:" + std::to_string(mShootPublisher->mPacket.start_fire),
        "gain_yaw:" + std::to_string(mShootSubscriber->mPacket.yaw),
        "gain_pitch:" + std::to_string(mShootSubscriber->mPacket.pitch),
        "turn_right:" + std::to_string(mShootPublisher->mPacket.is_turn_right),
        "fps:" + std::to_string(MessageUtils::getFpsByTimePoint(startTime))
    };


    for(int i = 0;i<9;i++) {
        MessageUtils::putText(mSrc, params[i], cv::Point(0, (i + 1) * 20));
    }


    // 纵坐标
    cv::line(mSrc, cv::Point2f(mSrc.size().width / 2, 0), cv::Point2f(mSrc.size().width / 2, mSrc.size().height), cv::Scalar(208, 224, 64), 1, 8);
    // 横坐标
    cv::line(mSrc, cv::Point2f(0, mSrc.size().height / 2), cv::Point2f(mSrc.size().width, mSrc.size().height / 2), cv::Scalar(208, 224, 64), 1, 8);
    // 预估击打横坐标
    cv::line(mSrc, cv::Point2f(mSrc.size().width / 2 - mSrc.size().width / 6, mSrc.size().height / 2 + 40), cv::Point2f(mSrc.size().width / 2 + mSrc.size().width / 6, mSrc.size().height / 2 + 40), cv::Scalar(215, 235, 250), 1, 8);
    cv::line(mSrc, cv::Point2f(mSrc.size().width / 2 - mSrc.size().width / 8, mSrc.size().height / 2 + 80), cv::Point2f(mSrc.size().width / 2 + mSrc.size().width / 8, mSrc.size().height / 2 + 80), cv::Scalar(215, 235, 250), 1, 8);

}



std::vector<int> ArmorModule::getIds() {
    return mIds;
}

YAML::Node ArmorModule::getConfigNode(const int& id) {
    return YAML::LoadFile(getConfigLocation("armor")+ "config.yml")[std::to_string(id)];
}

void ArmorModule::enableShow() {
    mShowSrc = true;
}
void ArmorModule::disableShow() {
    mShowSrc = false;
}

const std::shared_ptr<ShootSubscriber>& ArmorModule::getSubscriber() const{
    return this->mShootSubscriber;
}