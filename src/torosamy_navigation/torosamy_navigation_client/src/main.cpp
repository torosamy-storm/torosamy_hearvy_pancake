#include "NavigationDecision.h"
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavigationDecision>();
  
  while(!node->connectServer()) {
      std::cout<<"Error: fail to connect action server"<<std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
