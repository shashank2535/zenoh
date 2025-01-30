cmake_minimum_required(VERSION 3.16)
project(Zenoh_integrate)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake" ${CMAKE_MODULE_PATH})
# include(helpers)


if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif()


find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_executable(pub /home/shashank/zenoh/Zenoh_integrate/pub.cpp)
target_link_libraries(pub PRIVATE zenohcxx::zenohc)
set_property(TARGET pub PROPERTY LANGUAGE CXX)
set_property(TARGET pub PROPERTY CXX_STANDARD 17)

add_executable(sub /home/shashank/zenoh/Zenoh_integrate/sub.cpp)
target_link_libraries(sub PRIVATE zenohcxx::zenohc)
set_property(TARGET sub PROPERTY LANGUAGE CXX)
set_property(TARGET sub PROPERTY CXX_STANDARD 17)

add_executable(pub1 /home/shashank/zenoh/Zenoh_integrate/pub1.cpp)
target_link_libraries(pub1 PRIVATE zenohcxx::zenohc)
set_property(TARGET pub1 PROPERTY LANGUAGE CXX)
set_property(TARGET pub1 PROPERTY CXX_STANDARD 17)

add_executable(sub1 /home/shashank/zenoh/Zenoh_integrate/sub1.cpp)
target_link_libraries(sub1 PRIVATE zenohcxx::zenohc)
set_property(TARGET sub1 PROPERTY LANGUAGE CXX)
set_property(TARGET sub1 PROPERTY CXX_STANDARD 17)
