cmake_minimum_required (VERSION 2.6)

project (pca9685)

add_library(pca9685 STATIC src/pca9685.cpp)
target_link_libraries(pca9685 -lpigpio -pthread)
install(TARGETS pca9685 DESTINATION "/usr/local/lib")
install(FILES src/pca9685.h DESTINATION "/usr/local/include")