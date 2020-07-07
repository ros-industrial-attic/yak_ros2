cmake_minimum_required(VERSION 3.5.0)
project(yak_ros2 VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(yak REQUIRED)

find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

# Build as an Ament package for ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME}_node
    src/yak_node.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${rclcpp_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_node
    ${rclcpp_LIBRARIES}
    yak::yak
    yak::yak_frontend
    yak::yak_marching_cubes
    ${sensor_msgs_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_eigen_LIBRARIES}
    ${cv_bridge_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_node
                          "rclcpp"
                          "yak"
                          "std_srvs"
                          "sensor_msgs"
                          "geometry_msgs"
                          "tf2"
                          "tf2_ros"
                          "tf2_eigen")

ament_export_include_directories(${rclcpp_INCLUDE_DIRS}
                                 ${yak_INCLUDE_DIRS})

ament_export_dependencies(rclcpp yak tf2_eigen tf2)

ament_package()

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch demo
        DESTINATION share/${PROJECT_NAME})

if (DEFINED BUILD_DEMO)
    install(TARGETS ${PROJECT_NAME}_image_simulator
            RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()


