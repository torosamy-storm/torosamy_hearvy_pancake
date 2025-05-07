#include "NavigationDecision.h"
#include <yaml-cpp/yaml.h>
using namespace std::chrono_literals;

NavigationDecision::NavigationDecision() : 
    Node("navigation_action_client"),
    mIsWaiting(false),
    mClientStatus(ClientStatus::CONNECTING),
    mServerStatus(ServerStatus::UNKNOWN){
    mActionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    mSerialSubscriber = this->create_subscription<RefereeSystemMsg>(
        "/referee_system", 10, std::bind(&NavigationDecision::subscribeRefereeSystem, this, std::placeholders::_1)
    );

    mNavigationDecisionStatusPublisher = this->create_publisher<NavigationDecisionStatus>("navigation_status", 10);
    
    mNavigationDecisionStatusTimer = this->create_wall_timer(
        std::chrono::microseconds(1000), std::bind(&NavigationDecision::publishStatus, this)
    );

    mWaitPointTimer = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&NavigationDecision::waitPoint, this)
    );
    mWaitPointTimer->cancel();

    initConfig();
    initSendGoalOptions();
    initGoalMsg();
}


bool hasGoA = false;
void NavigationDecision::subscribeRefereeSystem(const RefereeSystemMsg::SharedPtr msg) {
    printMsg(msg);

    if(msg->hp < mConfig.mHp) {
        sendGoal(mConfig.mHomePoint);
        hasGoA = false; 
        return;
    }
    if(mIsWaiting) {
        return;
    }

    if (mServerStatus == ServerStatus::HANDLE_TASK) {
        return;
    }

    if (!hasGoA) {
        sendGoal(mConfig.mPatrolPointA);
        hasGoA = true;
    }else {
        sendGoal(mConfig.mPatrolPointB);
        hasGoA = false;
    }
    
}

void NavigationDecision::waitPoint() {
    mIsWaiting = false;
}


void NavigationDecision::initSendGoalOptions() {
    using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    // 向导航服务器发送目标点时的回调函数
    send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
            // mServerStatus = ServerStatus::RUN_ERROR;
            return;
        }

        // const std::string UUID = rclcpp_action::to_string(goal_handle->get_goal_id());
        // std::cout << UUID << std::endl;
        
        // const int8_t current_status = goal_handle->get_status();
    };

    // 设置移动过程反馈回调函数
    send_goal_options.feedback_callback =[this] (GoalHandle::SharedPtr goal_handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        if (!goal_handle) {
            // mServerStatus = ServerStatus::RUN_ERROR;
            return;
        }

        if (mServerStatus != ServerStatus::HANDLE_TASK) mServerStatus = ServerStatus::HANDLE_TASK;
    };

    // 目标点执行完毕时的回调函数
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& result) {
        if (rclcpp_action::ResultCode::SUCCEEDED == result.code) {
            mServerStatus = ServerStatus::WAIT_TASK;
            mIsWaiting = true;
            mWaitPointTimer->reset();
            return;
        }
        if (rclcpp_action::ResultCode::ABORTED == result.code) {
            mServerStatus = ServerStatus::WAIT_TASK;
            return;
        }
        if (rclcpp_action::ResultCode::CANCELED == result.code) {
            mServerStatus = ServerStatus::WAIT_TASK;
            return;
        }
        if (rclcpp_action::ResultCode::UNKNOWN == result.code) {
            mServerStatus = ServerStatus::WAIT_TASK;
            return;
        }
    };


    mSendGoalOptions = send_goal_options;
}

bool NavigationDecision::connectServer() {
    for (int i = 1; i <= mConfig.mConnectMaxTimes; i++) {
        if (mActionClient->wait_for_action_server(std::chrono::seconds(mConfig.mConnectWaitOnceSecond))) {
            mServerStatus = ServerStatus::WAIT_TASK;
            mClientStatus = ClientStatus::SUCCESS;
            std::cout<<"connect action server successfully"<<std::endl;
            return true;
        }

        if (!rclcpp::ok()) {
            mServerStatus = ServerStatus::RUN_ERROR;
            mClientStatus = ClientStatus::FAILURE;
            return false;
        }

        std::cout<<"this is " << i <<"times to try to connect server, has been useed: " << i * mConfig.mConnectWaitOnceSecond << std::endl;
    }

    mServerStatus = ServerStatus::RUN_ERROR;
    mClientStatus = ClientStatus::FAILURE;
    return false;
}


void NavigationDecision::initConfig() {
    YAML::Node fileReader = YAML::LoadFile("/opt/torosamy_robomaster_sdk/config/module/navigation/config.yml");

    mConfig.mHomeHp = fileReader["HomeHp"].as<short>();
    mConfig.mRemainBullet =  fileReader["RemainBullet"].as<short>();
    mConfig.mRemainTime = fileReader["RemainTime"].as<short>();
    mConfig.mHp = fileReader["Hp"].as<short>();
    mConfig.mTimeOff = fileReader["TimeOff"].as<short>();

    mConfig.mConnectMaxTimes = fileReader["ConnectMaxTimes"].as<short>();
    mConfig.mConnectWaitOnceSecond = fileReader["ConnectWaitOnceSecond"].as<short>();

    mConfig.mHomePoint = {
        fileReader["HomePoint"]["x"].as<float>(), 
        fileReader["HomePoint"]["y"].as<float>()
    };
    mConfig.mPatrolPointA = {
        fileReader["PatrolPointA"]["x"].as<float>(), 
        fileReader["PatrolPointA"]["y"].as<float>()
    };
    mConfig.mPatrolPointB = {
        fileReader["PatrolPointB"]["x"].as<float>(), 
        fileReader["PatrolPointB"]["y"].as<float>()
    };
    mConfig.mTargetDoorPoint = {
        fileReader["TargetDoorPoint"]["x"].as<float>(), 
        fileReader["TargetDoorPoint"]["y"].as<float>()
    };
    mConfig.mCenterPoint = {
        fileReader["CenterPoint"]["x"].as<float>(), 
        fileReader["CenterPoint"]["y"].as<float>()
    };
    mConfig.mEnableDebug = fileReader["Debug"]["enable"].as<bool>();
    mConfig.mDebugOption = {
        fileReader["Debug"]["hp"].as<bool>(),
        fileReader["Debug"]["mode"].as<bool>(),
        fileReader["Debug"]["remain_time"].as<bool>(),
        fileReader["Debug"]["remain_exchange_times"].as<bool>(),
        fileReader["Debug"]["remain_bullet"].as<bool>(),
        fileReader["Debug"]["damage_reduction"].as<bool>(),
        fileReader["Debug"]["is_our_outpost_break"].as<bool>(),
        fileReader["Debug"]["is_enemy_outpost_break"].as<bool>(),
        fileReader["Debug"]["game_start"].as<bool>(),
        fileReader["Debug"]["is_fort_occupy"].as<bool>()
    };
    std::cout<<"bullet: "<<mConfig.mRemainBullet<<" time: "<<mConfig.mRemainTime<<" hp: "<<mConfig.mHp<<" home_hp: "<<mConfig.mHomeHp<<std::endl;    
}

void NavigationDecision::printMsg(const RefereeSystemMsg::SharedPtr msg) const {
    if(!mConfig.mEnableDebug) return;

    std::cout << "--------------" << std::endl;
    if (mConfig.mDebugOption.at(0)) std::cout << "hp: "<< msg->hp << std::endl;
    if (mConfig.mDebugOption.at(1)) std::cout << "mode: "<< msg->mode << std::endl;
    if (mConfig.mDebugOption.at(2)) std::cout << "remain_time: "<< msg->remain_time << std::endl;
    if (mConfig.mDebugOption.at(3)) std::cout << "remain_exchange_times: "<< msg->remain_exchange_times << std::endl;
    if (mConfig.mDebugOption.at(4)) std::cout << "remain_bullet: "<< msg->remain_bullet << std::endl;
    if (mConfig.mDebugOption.at(5)) std::cout << "damage_reduction: "<< msg->damage_reduction << std::endl;
    if (mConfig.mDebugOption.at(6)) std::cout << "is_our_outpost_break: "<< msg->is_our_outpost_break << std::endl;
    if (mConfig.mDebugOption.at(7)) std::cout << "is_enemy_outpost_break: "<< msg->is_enemy_outpost_break << std::endl;
    if (mConfig.mDebugOption.at(8)) std::cout << "game_start: "<< msg->game_start << std::endl;
    if (mConfig.mDebugOption.at(9)) std::cout << "is_fort_occupy: "<< msg->is_fort_occupy << std::endl;
    std::cout << "--------------" << std::endl;
}


void NavigationDecision::publishStatus() {
    auto msg = NavigationDecisionStatus();

    msg.client_status = static_cast<short>(mClientStatus);
    msg.server_status = static_cast<short>(mServerStatus);

    mNavigationDecisionStatusPublisher->publish(msg);
}

void NavigationDecision::sendGoal(const std::pair<float, float>& point) {
    mGoalMsg.pose.pose.position.x = point.first;
    mGoalMsg.pose.pose.position.y = point.second;

    mActionClient->async_send_goal(mGoalMsg, mSendGoalOptions);
}

void NavigationDecision::initGoalMsg() {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 0.0f;
    goal_msg.pose.pose.position.y = 0.0f;

    mGoalMsg = goal_msg;
}
