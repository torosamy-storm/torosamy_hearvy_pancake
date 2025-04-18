#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include "torosamy_ros_msgs/msg/navigation_client.hpp"
#include "torosamy_ros_msgs/msg/referee_system_msg.hpp"
using torosamy_ros_msgs::msg::RefereeSystemMsg;
using torosamy_ros_msgs::msg::NavigationClient;
using NavigationServer = nav2_msgs::action::NavigateToPose;


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


class NavigationDecision : public rclcpp::Node {
public:
  using NavigationServerClient = rclcpp_action::Client<NavigationServer>;
  using NavigationServerGoalHandle = rclcpp_action::ClientGoalHandle<NavigationServer>;
  

  bool connectServer(); 
  void subscribeRefereeSystem(const RefereeSystemMsg::SharedPtr msg);


  NavigationDecision();
  // void setServerStatus(const ServerStatus& status);
  // void setClientStatus(const ClientStatus& status);
private:
  // static std::pair<float,float> transitionPoints = {-0.04243, -4.17162};
  // static bool isTransited;
  // void sendPoint(const float& x, const float& y);

  void goBase();
  void goCenter();
  void goTarget();

  void initPoints();
  void initSendGoalOptions();
  void initGoalMsg();
  void initDecisionValue();


  short mHomeHp;
  short mRemainBullet;
  short mRemainTime;
  short mHp;

  std::pair<float,float> mHomePoint;
  std::pair<float,float> mPatrolPointA;
  std::pair<float,float> mPatrolPointB;
  std::pair<float,float> mTargetDoorPoint;
  std::pair<float,float> mCenterPoint;
  
  NavigationServerClient::SharedPtr mActionClient;
  NavigationServer::Goal mGoalMsg;
  rclcpp_action::Client<NavigationServer>::SendGoalOptions mSendGoalOptions;
  
  ClientStatus mClientStatus;
  ServerStatus mServerStatus;

  rclcpp::Subscription<RefereeSystemMsg>::SharedPtr mSerialSubscriber;
  rclcpp::Publisher<NavigationClient>::SharedPtr mNavigationClientPublisher;
  rclcpp::TimerBase::SharedPtr mNavigationClientTimer;
  void publish();
};