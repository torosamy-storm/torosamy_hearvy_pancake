#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include "torosamy_ros_msgs/msg/navigation_client.hpp"
#include "torosamy_ros_msgs/msg/referee_system_msg.hpp"


enum class ClientStatus {
    CONNECTING,
    FAILURE,
    SUCCESS
};

enum class ServerStatus {
    UNKNOWN,
    WAIT_TASK,
    HANDLE_TASK,
    RUN_ERROR,
};



using torosamy_ros_msgs::msg::RefereeSystemMsg;
using NavigationDecisionStatus = torosamy_ros_msgs::msg::NavigationClient;
class NavigationDecision : public rclcpp::Node {
public:
  NavigationDecision();
  bool connectServer(); 
  void subscribeRefereeSystem(const RefereeSystemMsg::SharedPtr msg);
private:
  void initConfig();
  void initSendGoalOptions();
  void initGoalMsg();
  void sendGoal(const std::pair<float, float>& point);
  void printMsg(const RefereeSystemMsg::SharedPtr msg) const;
  void publishStatus();
  void waitPoint();

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr mActionClient;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions mSendGoalOptions;
  nav2_msgs::action::NavigateToPose::Goal mGoalMsg;


  rclcpp::Subscription<RefereeSystemMsg>::SharedPtr mSerialSubscriber;
  rclcpp::Publisher<NavigationDecisionStatus>::SharedPtr mNavigationDecisionStatusPublisher;
  rclcpp::TimerBase::SharedPtr mNavigationDecisionStatusTimer;

  rclcpp::TimerBase::SharedPtr mWaitPointTimer;
  bool mIsWaiting;

  struct {
    short mHomeHp;
    short mRemainBullet;
    short mRemainTime;
    short mHp;
    short mTimeOff;
    short mConnectMaxTimes;
    short mConnectWaitOnceSecond;
    bool mEnableDebug;
  
    std::pair<float,float> mHomePoint;
    std::pair<float,float> mPatrolPointA;
    std::pair<float,float> mPatrolPointB;
    std::pair<float,float> mTargetDoorPoint;
    std::pair<float,float> mCenterPoint;

    std::vector<bool> mDebugOption;
  }mConfig;

  ClientStatus mClientStatus;
  ServerStatus mServerStatus;
};



