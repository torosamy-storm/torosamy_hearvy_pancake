#include "NavigationDecision.h"
#include <yaml-cpp/yaml.h>
using namespace std::chrono_literals;

NavigationDecision::NavigationDecision() : 
    Node("navigation_action_client"),
    mClientStatus(ClientStatus::CONNECTING),
    mServerStatus(ServerStatus::UNKNOWN){
    mActionClient = rclcpp_action::create_client<NavigationServer>(this, "navigate_to_pose");

    mSerialSubscriber = this->create_subscription<RefereeSystemMsg>(
        "/referee_system", 10, std::bind(&NavigationDecision::subscribeRefereeSystem, this, std::placeholders::_1)
    );

    mNavigationClientPublisher = this->create_publisher<NavigationClient>("navigation_status", 10);
    
    mNavigationClientTimer = this->create_wall_timer(
        std::chrono::microseconds(1000), std::bind(&NavigationDecision::publish, this)
    );

    initDecisionValue();
    initSendGoalOptions();
    initGoalMsg();
    initPoints();
}

void NavigationDecision::publish() {
    auto msg = NavigationClient();

    msg.client_status = static_cast<short>(mClientStatus);
    msg.server_status = static_cast<short>(mServerStatus);

    mNavigationClientPublisher->publish(msg);
}


void NavigationDecision::initDecisionValue() {
    YAML::Node fileReader = YAML::LoadFile("/opt/torosamy_robomaster_sdk/config/module/navigation/config.yml");
    mRemainBullet = fileReader["RemainBullet"].as<short>();
    mRemainTime = fileReader["RemainTime"].as<short>();
    mHp = fileReader["Hp"].as<short>();
    mHomeHp = fileReader["HomeHp"].as<short>();
    std::cout<<"bullet: "<<mRemainBullet<<" time: "<<mRemainTime<<" hp: "<<mHp<<" home_hp: "<<mHomeHp<<std::endl;
}

void NavigationDecision::initGoalMsg() {
    auto goal_msg = NavigationServer::Goal();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 0.0f;
    goal_msg.pose.pose.position.y = 0.0f;

    mGoalMsg = goal_msg;
}

void NavigationDecision::initSendGoalOptions() {
    auto send_goal_options = rclcpp_action::Client<NavigationServer>::SendGoalOptions();

    // 设置请求目标结果回调函数
    send_goal_options.goal_response_callback = [this](NavigationServerGoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) return;
        mServerStatus = ServerStatus::HANDLE_TASK;
    };

    // 设置移动过程反馈回调函数
    send_goal_options.feedback_callback =[this] (
        NavigationServerGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const NavigationServer::Feedback> feedback) {
            if (!goal_handle) {
                mServerStatus = ServerStatus::RUN_ERROR;
                return;
            }
        };

    // 设置执行结果回调函数
    send_goal_options.result_callback = [this](const NavigationServerGoalHandle::WrappedResult& result) {
          switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED : {
                    mServerStatus = ServerStatus::WAIT_TASK;
                    break;
              }

              case rclcpp_action::ResultCode::ABORTED : {
                    mServerStatus = ServerStatus::WAIT_TASK;
                    break;
              }

              case rclcpp_action::ResultCode::CANCELED : {
                    mServerStatus = ServerStatus::WAIT_TASK;
                    break;
              }

              default: {
                    mServerStatus = ServerStatus::WAIT_TASK;
                    break;
              }
          };
    };
    mSendGoalOptions = send_goal_options;
}

void NavigationDecision::initPoints() {
    YAML::Node fileReader = YAML::LoadFile("/opt/torosamy_robomaster_sdk/config/module/navigation/config.yml");

    mHomePoint.first = fileReader["HomePoint"]["x"].as<float>();
    mHomePoint.second = fileReader["HomePoint"]["y"].as<float>();

    mPatrolPointA.first = fileReader["PatrolPointA"]["x"].as<float>();
    mPatrolPointA.second = fileReader["PatrolPointA"]["y"].as<float>();


    mPatrolPointB.first = fileReader["PatrolPointB"]["x"].as<float>();
    mPatrolPointB.second = fileReader["PatrolPointB"]["y"].as<float>();

    mTargetDoorPoint.first = fileReader["TargetDoorPoint"]["x"].as<float>();
    mTargetDoorPoint.second = fileReader["TargetDoorPoint"]["y"].as<float>();

    mCenterPoint.first = fileReader["CenterPoint"]["x"].as<float>();
    mCenterPoint.second = fileReader["CenterPoint"]["y"].as<float>();
}



bool NavigationDecision::connectServer() {
    int times = 0;
    while (!mActionClient->wait_for_action_server(std::chrono::seconds(4))) {
        if (!rclcpp::ok()) {
            mServerStatus = ServerStatus::RUN_ERROR;
            mClientStatus = ClientStatus::FAILURE;
            return false;
        }

        std::cout<<"waiting action server..."<<std::endl;

        times++;
        
        if(times > 6) {
            mServerStatus = ServerStatus::RUN_ERROR;
            mClientStatus = ClientStatus::FAILURE;
            return false;
        }
    }
    mServerStatus = ServerStatus::WAIT_TASK;
    mClientStatus = ClientStatus::SUCCESS;
    std::cout<<"connect action server successfully"<<std::endl;
    return true;
}


bool tttt = false;
void NavigationDecision::subscribeRefereeSystem(const RefereeSystemMsg::SharedPtr msg) {
    if(!tttt) tttt = true;
    else return;
    
    mGoalMsg.pose.pose.position.x = 1.1405303478240967;
    mGoalMsg.pose.pose.position.y = -1.2488510608673096;


    mActionClient->async_send_goal(mGoalMsg, mSendGoalOptions);

    // std::cout<<msg->game_start<<" "<<msg->mode<<std::endl;
    // if(!msg->game_start) return;
    
    // if(msg->hp < mHomeHp) {
    //     goBase();
    //     return;
    // }
    
    // if(msg->mode == 3 && msg->hp > mHp && msg->remain_bullet > mRemainBullet && msg->remain_time > mRemainTime) {
    //     goTarget();
    //     return;
    // }

    // goCenter();
}



void NavigationDecision::goBase() {
    mGoalMsg.pose.pose.position.x = mHomePoint.first;
    mGoalMsg.pose.pose.position.y = mHomePoint.second;


    mActionClient->async_send_goal(mGoalMsg, mSendGoalOptions);
}
void NavigationDecision::goCenter() {
    mGoalMsg.pose.pose.position.x = mCenterPoint.first;
    mGoalMsg.pose.pose.position.y = mCenterPoint.second;


    mActionClient->async_send_goal(mGoalMsg, mSendGoalOptions);
}
void NavigationDecision::goTarget() {
    mGoalMsg.pose.pose.position.x = mTargetDoorPoint.first;
    mGoalMsg.pose.pose.position.y = mTargetDoorPoint.second;


    mActionClient->async_send_goal(mGoalMsg, mSendGoalOptions);
}

// void NavigationDecision::setServerStatus(const ServerStatus& status) {
//     this->mServerStatus = status;
// }
// void NavigationDecision::setClientStatus(const ClientStatus& status) {
//     this->mClientStatus = status;
// }