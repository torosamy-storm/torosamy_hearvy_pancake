//
// Created by torosamy on 25-3-31.
//
#include "torosamy_automatic_aiming/handler/predict/MotionPredicter.h"
#include "torosamy_automatic_aiming/handler/predict/utils/EigenUtil.h"
MotionPredicter::MotionPredicter(const YAML::Node& fileReader) :
 	//TODO outpost param
    mMaxTargetHeight(fileReader["armor"]["maxTargetHeight"].as<float>()),
    mMaxMatchDistance(fileReader["armor"]["maxMatchDistance"].as<float>()),
    mMaxMatchYawDiff(fileReader["armor"]["maxMatchYawDiff"].as<float>()),
    mTrackingThres(fileReader["armor"]["trackingThres"].as<float>()),
	mArmorKalmanFilter(fileReader["armor"]["kalman"]),
    mStartTime(Torosamy::MessageUtils::getTimePoint()),
    mDz(0),
    mDt(fileReader["armor"]["dt"].as<float>()),
    mAnotherR(0),
    mGapTime(0),
    mInited(false){
}

void MotionPredicter::updateGapTime() {
    Torosamy::TIME endTime = Torosamy::MessageUtils::getTimePoint();
    mGapTime = Torosamy::MessageUtils::getTimeByPoint(mStartTime, endTime);
    mStartTime = endTime;
}

void MotionPredicter::init(const ArmorPnpResult& armorPnpResult) {
    const double r = 0.26;

    double yaw = (90.0 - armorPnpResult.getArmorYaw()) * CV_PI / 180.0;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    mLastYaw = yaw;
    mAnotherR = r;
    mDz = 0;

    const auto& worldCenter = armorPnpResult.getWorldCenter();
    const double xc = worldCenter.x() + r * cos(yaw);
    const double yc = worldCenter.y() + r * sin(yaw);
    const double zc = worldCenter.z();

    mHandleResult = Eigen::VectorXd::Zero(9);
    mHandleResult << xc, 0, yc, 0, zc, 0, yaw, 0, r;
    mArmorKalmanFilter.setState(mHandleResult);
    mTrackedType = armorPnpResult.getArmorType();
    mInited = true;
}



bool MotionPredicter::update(const std::vector<ArmorPnpResult>& handlers) {
    if(!mInited) return false;
    // KF predict

    updateGapTime();

    // Use KF prediction as default target state if no matched armor is found
    mHandleResult = mArmorKalmanFilter.predict(mGapTime);

    if (handlers.empty()) {
        handlerUpdateResult();
        return false;
    }

 //***************************这个预测点得到的结果出错，直接导致了后续计算出错
    const Eigen::Vector3d predicted_position = getArmorPositionFromState(mHandleResult);

    double min_position_diff = DBL_MAX;

    int targetIndex = 0;
    int sameTypeWithTargetCount = 0;
    for(int i = 0; i < handlers.size(); ++i) {
        const auto& handler = handlers.at(i);
        if (handler.getArmorType() != mTrackedType) continue;


        if (handler.getWorldCenter().z() > mMaxTargetHeight) continue;

        if (Eigen::Vector2d(handler.getWorldCenter().x(), handler.getWorldCenter().y()).norm() > 10) continue;

        sameTypeWithTargetCount++;

        const double position_diff = (predicted_position - handler.getWorldCenter()).norm();

        if (position_diff < min_position_diff) {
            min_position_diff = position_diff;
            targetIndex = i;
        }
    }



    if(min_position_diff >= mMaxMatchDistance) {
        handlerUpdateResult();
        return false;
    }

    const auto& handler = handlers.at(targetIndex);

    if(abs(orientationToYaw(handler.getArmorYaw()) - mHandleResult(6)) >= mMaxMatchYawDiff) {
        if (sameTypeWithTargetCount == 1) handleArmorJump(handler.getWorldCenter(), handler.getArmorYaw());
        handlerUpdateResult();
        return false;
    }

    // Check if the distance and yaw difference of closest armor are within the threshold
    const Eigen::Vector3d& worldCenter = handler.getWorldCenter();

    mHandleResult = mArmorKalmanFilter.update(
        Eigen::Vector4d(
        	worldCenter.x(),
        	worldCenter.y(),
        	worldCenter.z(),
        	orientationToYaw(handler.getArmorYaw())
    	)
    );


    // Prevent radius from spreading
    handlerUpdateResult();
    return true;
}

void MotionPredicter::handlerUpdateResult() {
    if (mHandleResult(8) < 0.12) {
        mHandleResult(8) = 0.12;
        mArmorKalmanFilter.setState(mHandleResult);
    } else if (mHandleResult(8) > 0.4) {
        mHandleResult(8) = 0.4;
        mArmorKalmanFilter.setState(mHandleResult);
    }
}


void MotionPredicter::handleArmorJump(const Eigen::Vector3d& worldCenter, const float& armorYaw) {
    const double yaw = orientationToYaw(armorYaw);
    mHandleResult(6) = yaw;


    mDz = mHandleResult(4) - worldCenter.z();
    mHandleResult(4) = worldCenter.z();
    std::swap(mHandleResult(8), mAnotherR);

    const Eigen::Vector3d current(
        worldCenter.x(),
        worldCenter.y(),
        worldCenter.z()
    );

    const Eigen::Vector3d infer = getArmorPositionFromState(mHandleResult);

    if ((current - infer).norm() > mTrackingThres) {
        const double r = mHandleResult(8);
        mHandleResult(0) = worldCenter.x() + r * cos(yaw);  // xc
        mHandleResult(1) = 0;                   // vxc
        mHandleResult(2) = worldCenter.y() + r * sin(yaw);  // yc
        mHandleResult(3) = 0;                   // vyc
        mHandleResult(4) = worldCenter.z();                 // za
        mHandleResult(5) = 0;                   // vza
    }

    mArmorKalmanFilter.setState(mHandleResult);
}


double MotionPredicter::orientationToYaw(const float& armorYaw) {
    //从装甲板在左侧到在右侧原本是 pi ~ -pi -> 0 -> 3.14
    double yaw = (90 - armorYaw) * CV_PI / 180;

    const double normalize_angle = fmod(yaw - mLastYaw + M_PI, 2.0 * M_PI);
    yaw = mLastYaw + normalize_angle;
    if (normalize_angle <= 0.0) yaw += M_PI;
    else yaw -= M_PI;

    mLastYaw = yaw;
    return yaw;
}

Eigen::Vector3d MotionPredicter::getArmorPositionFromState(const Eigen::VectorXd & x) {
	const double yaw = x(6);
    const double r = x(8);

    // Calculate predicted position of the current armor
    return Eigen::Vector3d(
        x(0) - r * cos(yaw),
        x(2) - r * sin(yaw),
        x(4)
    );
}


const Eigen::VectorXd& MotionPredicter::getHandleResult() const {
	return mHandleResult;
}

Eigen::Vector3d MotionPredicter::generatePredictTvec(const double& filghtTime, const float& gainPitch, const float& gainYaw) const {
    const double time = mGapTime + mDt + filghtTime;
    const double radius = mHandleResult(8);
    const double yaw = mHandleResult(6) + mHandleResult(7) * time;

    const Eigen::Vector3d world_point(
        mHandleResult(0) + mHandleResult(1) * time - radius * cos(yaw),
        mHandleResult(2) + mHandleResult(3) * time - radius * sin(yaw),
        mHandleResult(4) + mDz / 2 + mHandleResult(5) * time
    );

    const Eigen::Vector3d imu_point(
        world_point.x(),
        -world_point.z(),
        world_point.y()
    );


    return EigenUtil::getQuaternion(gainPitch, gainYaw).conjugate() *
           (imu_point + Eigen::Vector3d(0, 0, 0));
}


bool MotionPredicter::isTurnRight() const {
    return mArmorKalmanFilter.isTurnRight();
}