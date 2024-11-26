cmake_minimum_required(VERSION 3.8)
project(walker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(TARGET_NAME walker_node)

add_executable(${TARGET_NAME} src/walker_node.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        std_srvs
)

install(TARGETS ${TARGET_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL
)
install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL
)
install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL
)
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL
)

ament_package()
