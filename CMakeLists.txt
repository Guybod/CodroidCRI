cmake_minimum_required(VERSION 3.16)
project(CodroidCRI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_definitions(ASIO_STANDALONE)

add_library(codroid_cri
    src/robot/DataPushConfigStore.cpp
    src/robot/RobotTcpClient.cpp
    src/robot/CriRealtimeParser.cpp
    src/robot/RobotUdpClient.cpp
    src/robot/RealtimePointSender.cpp
)

target_include_directories(codroid_cri
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/asio/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include
)

find_package(Threads REQUIRED)
target_link_libraries(codroid_cri PUBLIC Threads::Threads)

add_executable(criJointDemo examples/cri_joint_demo.cpp)
target_link_libraries(criJointDemo PRIVATE codroid_cri)

add_executable(criCartesianLiftDemo examples/cri_cartesian_lift_demo.cpp)
target_link_libraries(criCartesianLiftDemo PRIVATE codroid_cri)
