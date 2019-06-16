cmake_minimum_required (VERSION 2.6)

project (pca9685)

add_library(pca9685 STATIC RCCar src/pca9685.cpp)
target_link_libraries(RCCar -lpigpio -pthread)