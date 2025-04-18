//
// Created by torosamy on 25-4-2.
//
#include "torosamy_automatic_aiming/FunctionController.h"
#include "torosamy_automatic_aiming/ArmorModule.h"
#include "pod/manager/ModuleManager.h"
void FunctionController::disableShowSrc() {
    int option = -1;
    const auto& ids = ArmorModule::getIds();
    std::cout<<"All Armor modules:";
    for (const auto& id : ids) {
        std::cout<<" "<<id;
    }
    std::cout<<std::endl;
    std::cout<<"please choose an Armor module to disable show src: ";
    std::cin>>option;
    Torosamy::ModuleManager::getInstance()->getModuleById<ArmorModule>(option)->disableShow();
    cv::destroyWindow(std::to_string(option));
}

void FunctionController::enableShowSrc() {
    int option = -1;
    const auto& ids = ArmorModule::getIds();
    std::cout<<"All Armor modules:";
    for (const auto& id : ids) {
        std::cout<<" "<<id;
    }
    std::cout<<std::endl;
    std::cout<<"please choose an Armor module to enable show src: ";
    std::cin>>option;
    Torosamy::ModuleManager::getInstance()->getModuleById<ArmorModule>(option)->enableShow();
}