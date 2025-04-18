//
// Created by torosamy on 25-3-30.
//
//
// Created by torosamy on 25-3-29.
//
#include "torosamy_automatic_aiming/armor/ArmorPnpResult.h"
#include "torosamy_automatic_aiming/handler/predict/utils/EigenUtil.h"

ArmorPnpResult::ArmorPnpResult(
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const float& pitch,
    const float& yaw,
    const Armor& armor) :
 	mArmorType(armor.getArmorType()),
    mRealPoints(generateRealPoints(armor)),
    mWorldPoints(generateWorldPoints(armor)){
    initPnpDistance(cameraMatrix,distCoeffs);
    initArmorYaw(cameraMatrix);
    initWorldCenter(pitch,yaw);
}

ArmorPnpResult::ArmorPnpResult(const Eigen::Vector3d& tvec, const Armor& armor) :
	mArmorType(armor.getArmorType()),
	mRealPoints(generateRealPoints(armor)),
	mWorldPoints(generateWorldPoints(armor)) {
	this->mTvec = tvec;
    const double tx = mTvec.x();
    const double ty = mTvec.y();
    const double tz = mTvec.z();
    mDistance = sqrt(tz * tz + tx * tx + ty * ty);
}


void ArmorPnpResult::initWorldCenter(const float& gainPitch, const float& gainYaw) {
    const Eigen::Vector3d& tvec = Eigen::Vector3d(-0.08, 0.05, 0);

    // 获取四元数
    const Eigen::Quaterniond q = EigenUtil::getQuaternion(gainPitch, gainYaw);

    // 应用旋转和平移（注意四元数旋转操作顺序）
    const Eigen::Vector3d imu_point = q * (mTvec - tvec);

    // 坐标系轴转换（根据实际硬件定义调整）
    mWorldCenter = Eigen::Vector3d(imu_point.x(),imu_point.z(),-imu_point.y());
}

void ArmorPnpResult::initPnpDistance(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    cv::Mat rvec, tvec;
    cv::solvePnP(mWorldPoints, mRealPoints, cameraMatrix, distCoeffs, rvec, tvec,false,cv::SOLVEPNP_IPPE);

    mTvec = Eigen::Vector3d(tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));

    const double tx = mTvec.x();
    const double ty = mTvec.y();
    const double tz = mTvec.z();
    mDistance = sqrt(tz * tz + tx * tx + ty * ty);
}

void ArmorPnpResult::initArmorYaw(const cv::Mat& cameraMatrix) {
    Eigen::Matrix3d cameraMatrixEigen = EigenUtil::getCameraMatrix(cameraMatrix);

    double left = -60.0;    // 修正区间方向
    double right = 60.0;
    double best_angle = 0;
    double min_cost = std::numeric_limits<double>::max();
    std::vector<cv::Point2f> best_projection;

    for (int i = 0; i < 10; ++i) {
        double mid1 = left + (right - left) / 3.0;
        double mid2 = right - (right - left) / 3.0;

        // 计算mid1
        auto points1 = generateTransPoint(EigenUtil::getRotationMatrix(mid1), cameraMatrixEigen);
        double cost1 = generateCost(points1);

        auto points2 = generateTransPoint(EigenUtil::getRotationMatrix(mid2), cameraMatrixEigen);
        double cost2 = generateCost(points2);

        // 记录全局最优
        if (cost1 < min_cost) {
            min_cost = cost1;
            best_angle = mid1;
            best_projection = points1;
        }
        if (cost2 < min_cost) {
            min_cost = cost2;
            best_angle = mid2;
            best_projection = points2;
        }

        // 更新搜索区间
        if (cost1 < cost2) {
            right = mid2;  // 保留左区间
        } else {
            left = mid1;   // 保留右区间
        }
    }

    best_angle = (left + right) / 2.0;
    auto final_points = generateTransPoint(EigenUtil::getRotationMatrix(best_angle), cameraMatrixEigen);
    double final_cost = generateCost(final_points);

    if (final_cost < min_cost)
    {
        best_projection = final_points;
        min_cost = final_cost;
    }
    mProjectionPoints = best_projection;
    mArmorYaw = best_angle;
}

std::vector<cv::Point2f> ArmorPnpResult::generateTransPoint(const Eigen::Matrix3d& rvec, const Eigen::Matrix3d& cameraMatrix) const{
    std::vector<cv::Point2f> points(4);

    const Eigen::Matrix4d pose = EigenUtil::getPose(rvec, mTvec);

    //坐标转换
    for(int i = 0; i < 4; ++i) {
        Eigen::Vector4d world_point(mWorldPoints.at(i).x,mWorldPoints.at(i).y,mWorldPoints.at(i).z,1);

        Eigen::Vector4d camera_rmx = pose * world_point;

        Eigen::Vector3d normal_point(
            camera_rmx.x()/camera_rmx.z(),
            camera_rmx.y()/camera_rmx.z(),
            1
        );
        Eigen::Vector2d image_point = cameraMatrix.block<2,3>(0,0) * normal_point.head(3);
        points[i] = cv::Point2f(image_point.x(), image_point.y());
    }
    return points;
}

double ArmorPnpResult::generateCost(const std::vector<cv::Point2f>& points) const{
    const double r_p_top = cv::norm(mRealPoints.at(0) - points.at(0));
    const double r_p_bom = cv::norm(mRealPoints.at(3) - points.at(3));

    //    面积法，实测不知道为啥不大行
    // double area1 = TransMath::calculateArea(rea_points);
    // double area2 = TransMath::calculateArea(Points);
    return r_p_top + r_p_bom;
}

std::vector<cv::Point2f> ArmorPnpResult::generateRealPoints(const Armor& armor) {
    std::vector<cv::Point2f> result(4);

    result[0] = armor.getLeftLight().getTopPoint();
    result[1] = armor.getRightLight().getTopPoint();
    result[2] = armor.getRightLight().getBottomPoint();
    result[3] = armor.getLeftLight().getBottomPoint();

    return result;
}
std::vector<cv::Point3f> ArmorPnpResult::generateWorldPoints(const Armor& armor) {
    std::vector<cv::Point3f> result(4);

    const double x = armor.isBigArmor() ? 0.117 : 0.068;
    const double y = 0.028;
    const double z = 0;

    result[0] = cv::Point3f(-x, -y, z);
    result[1] = cv::Point3f(x, -y, z);
    result[2] = cv::Point3f(x, y, z);
    result[3] = cv::Point3f(-x, y, z);

    return result;
}

ArmorType ArmorPnpResult::getArmorType() const {
    return mArmorType;
}

float ArmorPnpResult::getArmorYaw() const {
    //if(!mInited) throw std::runtime_error("ArmorPnpResult::getArmorYaw() called, is not inited");
    return mArmorYaw;
}
float ArmorPnpResult::getDistance() const {
    //if(!mInited) throw std::runtime_error("ArmorPnpResult::getDistance() called, is not inited");
    return mDistance;
}
const Eigen::Vector3d& ArmorPnpResult::getWorldCenter() const {
    //if(!mInited) throw std::runtime_error("ArmorPnpResult::getWorldCenter() called, is not inited");
    return mWorldCenter;
}

const Eigen::Vector3d& ArmorPnpResult::getPose() const {
    return mTvec;
}