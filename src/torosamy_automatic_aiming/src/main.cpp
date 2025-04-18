
#include "pod/manager/PacketManager.h"
#include "pod/manager/ModuleManager.h"
#include "pod/manager/CameraManager.h"
#include "torosamy_automatic_aiming/ArmorModule.h"
#include "torosamy_automatic_aiming/FunctionController.h"
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    using namespace Torosamy;
    CameraManager::getInstance()->openCameras();
    CameraManager::getInstance()->startUpdateCameras();


    ModuleManager::getInstance()
        ->addModules(ArmorModule::makeModules())
        ->detachTasks();

    rclcpp::executors::MultiThreadedExecutor executor;

    for(const int& id : ArmorModule::getIds()) {
        std::shared_ptr<ArmorModule> armorModule = Torosamy::ModuleManager::getInstance()->getModuleById<ArmorModule>(id);
        executor.add_node(armorModule->getSubscriber());
    }

    executor.spin();

    CameraManager::getInstance()->stopCameras();
    ModuleManager::getInstance()->stopTasks();


    rclcpp::shutdown();

    return 0;
}
