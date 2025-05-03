
#include "pod/manager/PacketManager.h"
#include "pod/manager/ModuleManager.h"
#include "pod/manager/CameraManager.h"
#include "torosamy_automatic_aiming/ArmorModule.h"
#include "torosamy_automatic_aiming/FunctionController.h"
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    using namespace Torosamy;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    CameraManager::getInstance()->openCameras();
    CameraManager::getInstance()->startUpdateCameras();

    std::cout <<"allow to update camera src" <<std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    ModuleManager::getInstance()
        ->addModules(ArmorModule::makeModules())
        ->detachTasks();

    rclcpp::executors::MultiThreadedExecutor executor;

    for(const int& id : ArmorModule::getIds()) {
        std::shared_ptr<ArmorModule> armorModule = Torosamy::ModuleManager::getInstance()->getModuleById<ArmorModule>(id);
        executor.add_node(armorModule->getSubscriber());
    }

    executor.spin();
    ModuleManager::getInstance()->stopTasks();
    CameraManager::getInstance()->stopCameras();
    


    rclcpp::shutdown();

    return 0;
}
