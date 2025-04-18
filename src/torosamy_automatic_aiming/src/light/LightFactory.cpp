//
// Created by torosamy on 25-3-19.
//
#include "torosamy_automatic_aiming/light/LightFactory.h"


Light LightFactory::extendLight(const Light& oldLight) {
    Light newLight = oldLight;
    newLight.setLength(oldLight.getLength() * 2);
    return newLight;
}

Light LightFactory::makeLight(const std::vector<cv::Point2i>& contour, const LightFilter& filter) {
    cv::RotatedRect rect = minAreaRect(contour);

    float area = rect.size.area();
    if (!filter.checkArea(area)) throw std::runtime_error("light area condition");
    bool flag = rect.size.height > rect.size.width;

    float length = flag ? rect.size.height : rect.size.width;
    float width = flag ? rect.size.width : rect.size.height;
    float rate = width / length;
    if (!filter.checkRate(rate)) throw std::runtime_error("light rate condition");

    float angle = flag ? rect.angle : 90 - rect.angle;
    if (!filter.checkAngle(angle)) throw std::runtime_error("light angle condition");

    std::vector<cv::Point2f> tempPoints;
    rect.points(tempPoints);

    cv::Point2f topPoint = flag ? (tempPoints.at(1) + tempPoints.at(2)) / 2 : (tempPoints.at(0) + tempPoints.at(1)) / 2;
    cv::Point2f bottomPoint = flag ? (tempPoints.at(3) + tempPoints.at(0)) / 2 : (tempPoints.at(2) + tempPoints.at(3)) / 2;

    Light light;
    light.setLength(length)
         .setWidth(width)
         .setPoints(tempPoints)
         .setCenter(rect.center)
         .setBottomPoint(bottomPoint)
         .setTopPoint(topPoint);
    
    return light;
    
}

Light LightFactory::makeLight(const std::vector<cv::Point2i>& contour) {
    cv::Rect rect = boundingRect(contour);
    std::vector<cv::Point> points;
    cv::findNonZero(getMaskImg(getMaskContour(contour, rect), rect), points);

    cv::Vec4f filedParam;
    cv::fitLine(points, filedParam, cv::DIST_L2, 0, 0.01, 0.01);

    Light light;

    if (static_cast<int>(filedParam[0] * 100) == 100 || static_cast<int>(filedParam[1] * 100) == 0) {
        light.setTopPoint(cv::Point2f(rect.x + rect.width / 2, rect.y))
             .setBottomPoint(cv::Point2f(rect.x + rect.width / 2, rect.y + rect.height))
             .setCenter((light.getBottomPoint() + light.getTopPoint() ) / 2)
             .setLength(cv::norm(light.getTopPoint() - light.getBottomPoint()))
             .setArea(points.size())
             .setAngle(0)
             .setWidth(points.size() / light.getLength());
    }
    else {
        const float k = filedParam[1] / filedParam[0];
        const float b = (filedParam[3] + rect.y) - k * (filedParam[2] + rect.x);

        float angle = std::atan(k) / CV_PI * 180 - 90;
        light.setTopPoint(cv::Point2f((rect.y - b) / k, rect.y))
             .setBottomPoint(cv::Point2f((rect.y + rect.height - b) / k, rect.y + rect.height))
             .setAngle(angle > 90 ? 180 - angle : angle)
             .setLength(cv::norm(light.getTopPoint() - light.getBottomPoint()))
             .setCenter((light.getBottomPoint() + light.getTopPoint() ) / 2)
             .setArea(points.size())
             .setWidth(points.size() / light.getLength());
    }
    return light;
}

std::vector<cv::Point2i> LightFactory::getMaskContour(const std::vector<cv::Point2i>& contour, const cv::Rect& rect) {
    std::vector<cv::Point2i> maskContour;
    for (const auto& p : contour) {
        maskContour.emplace_back(p - cv::Point(rect.x, rect.y));
    }
    return maskContour;
}


cv::Mat LightFactory::getMaskImg(const std::vector<cv::Point2i>& maskContour, const cv::Rect& rect) {
    cv::Mat mask = cv::Mat::zeros(rect.size(), CV_8UC1);
    cv::fillPoly(mask, {maskContour}, 255);
    return mask;
}

