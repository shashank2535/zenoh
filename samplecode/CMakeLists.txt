cmake_minimum_required(VERSION 3.16)
project(zenohcxx_examples LANGUAGES C CXX)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake" ${CMAKE_MODULE_PATH})
#include(helpers)

if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_executable(subscriber sub.cpp)
target_link_libraries(subscriber PRIVATE zenohcxx::zenohc)
set_property(TARGET subscriber PROPERTY LANGUAGE CXX)
set_property(TARGET subscriber PROPERTY CXX_STANDARD 17)

add_executable(publisher pub.cpp)
target_link_libraries(publisher PRIVATE zenohcxx::zenohc)
set_property(TARGET publisher PROPERTY LANGUAGE CXX)
set_property(TARGET publisher PROPERTY CXX_STANDARD 17)
