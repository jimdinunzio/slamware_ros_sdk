cmake_minimum_required(VERSION 3.5)

# uncomment this to see full compile line and args for debugging build
#set(CMAKE_VERBOSE_MAKEFILE ON)

# The slamware_ros_sdk v2.8.3 with gcc7 support is the last one released
# gcc 7 without a lot of work. You can install gcc 7 along side of your newer compiler.
# specify the C compiler
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")

# specify the C++ compiler
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")

project(slamware_ros_sdk)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h $ENV{HOME}/thirdparty/slamware_ros_sdk_linux-x86_64-gcc7/src/slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a $ENV{HOME}/thirdparty/slamware_ros_sdk_linux-x86_64-gcc7/src/slamware_sdk/lib/linux-x86_64)
if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  set(SLTC_SDK_INC_DIR "${slamware_sdk_INCLUDE_DIR}")
  set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  if(NOT DEFINED SLTC_SDK_ROOT OR SLTC_SDK_ROOT STREQUAL "")
    set(SLTC_SDK_ROOT "$ENV{HOME}/robotics/slamtec/slamware_sdk")
  endif()
  set(SLTC_SDK_DIR "${SLTC_SDK_ROOT}/linux-x86_64-release")
  set(SLTC_SDK_INC_DIR "${SLTC_SDK_DIR}/include")
  set(SLTC_SDK_LIB_DIR "${SLTC_SDK_DIR}/lib")
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=gnu++11")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(visualization_msgs REQUIRED)

#find_package(ament_cmake REQUIRED COMPONENTS
#  nav_msgs
#  roscpp
#  rospy
#  sensor_msgs
#  std_msgs
#  tf
#  message_generation
#)

find_package(Threads)

find_package(slamware_ros_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(
        include
        ../../install/include
)

#Listing all the include directories -- fouad
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "include dir=='${dir}'")
endforeach()

#Listing all the link directories -- fouad
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "link dir='${dir}'")
endforeach()

#add_message_files(
#  FILES
#  OptionalBool.msg
#  OptionalInt8.msg
#  OptionalInt16.msg
#  OptionalInt32.msg
#  OptionalInt64.msg
#  OptionalUInt8.msg
#  OptionalUInt16.msg
#  OptionalUInt32.msg
#  OptionalUInt64.msg
#  OptionalFlt32.msg
#  OptionalFlt64.msg
#  Vec2DInt32.msg
#  Vec2DFlt32.msg
#  Line2DFlt32.msg
#  Line2DFlt32Array.msg
#  RectInt32.msg
#  RectFlt32.msg
#  RobotDeviceInfo.msg
#  MapKind.msg
#  ArtifactUsage.msg
#  SensorType.msg
#  ImpactType.msg
#  BasicSensorInfo.msg
#  BasicSensorInfoArray.msg
#  BasicSensorValue.msg
#  BasicSensorValueData.msg
#  BasicSensorValueDataArray.msg
#  ActionDirection.msg
#  RobotBasicState.msg
#  SyncMapRequest.msg
#  MoveOptionFlag.msg
#  MoveOptions.msg
#  MoveByDirectionRequest.msg
#  MoveByThetaRequest.msg
#  MoveToRequest.msg
#  MoveToLocationsRequest.msg
#  RotateToRequest.msg
#  RotateRequest.msg
#  LocalizationMovement.msg
#  OptionalLocalizationMovement.msg
#  LocalizationOptions.msg
#  RecoverLocalizationRequest.msg
#  ClearMapRequest.msg
#  SetMapUpdateRequest.msg
#  SetMapLocalizationRequest.msg
#  GoHomeRequest.msg
#  CancelActionRequest.msg
#  AddLineRequest.msg
#  AddLinesRequest.msg
#  RemoveLineRequest.msg
#  ClearLinesRequest.msg
#  MoveLineRequest.msg
#  MoveLinesRequest.msg
#)
#
#add_service_files(
#  FILES
#  SyncGetStcm.srv
#  SyncSetStcm.srv
#)

#generate_messages(
#  DEPENDENCIES
#  nav_msgs
#  sensor_msgs
#  std_msgs
#)

set(DEPENDENCIES
        "geometry_msgs"
        "nav_msgs"
        "rclcpp"
        "sensor_msgs"
        "std_srvs"
        "tf2_geometry_msgs"
        "tf2_msgs"
        "tf2"
        "std_msgs"
        "tf2_ros"
        "message_filters"
        slamware_ros_msgs
        example_interfaces
        )

add_library(${PROJECT_NAME} STATIC
        src/client/slamware_ros_sdk_client.cpp
        )
target_link_libraries(${PROJECT_NAME} ${ament_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES}    )

add_executable(slamware_ros_sdk_server_node
        src/server/msg_convert.cpp
        src/server/server_params.cpp
        src/server/server_map_holder.cpp
        src/server/server_work_data.cpp
        src/server/server_worker_base.cpp
        src/server/server_workers.cpp
        src/server/slamware_ros_sdk_server_node.cpp
        src/server/slamware_ros_sdk_server.cpp
        )
ament_target_dependencies(slamware_ros_sdk_server_node ${DEPENDENCIES}    )
target_include_directories(slamware_ros_sdk_server_node
        PRIVATE ${SLTC_SDK_INC_DIR}
        )
target_compile_options(slamware_ros_sdk_server_node
        PRIVATE -Wno-deprecated-declarations
        )
target_link_libraries(slamware_ros_sdk_server_node
        ${ament_LIBRARIES}
        ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
        ${SLTC_SDK_LIB_DIR}/librpos_framework.a
        ${SLTC_SDK_LIB_DIR}/libbase64.a
        ${SLTC_SDK_LIB_DIR}/librlelib.a
        ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
        ${SLTC_SDK_LIB_DIR}/libcurl.a
        ${SLTC_SDK_LIB_DIR}/libssl.a
        ${SLTC_SDK_LIB_DIR}/libcrypto.a
        ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
        ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
        ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
        ${SLTC_SDK_LIB_DIR}/libboost_regex.a
        ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
        ${SLTC_SDK_LIB_DIR}/libboost_system.a
        ${SLTC_SDK_LIB_DIR}/libboost_thread.a
        ${SLTC_SDK_LIB_DIR}/libboost_random.a
        ${SLTC_SDK_LIB_DIR}/libboost_zlib.a
        pthread
        dl
        rt
        )


add_executable(minimal_slamware_ros_sdk_server_node
        src/minimal/server/minimal_slamware_ros_sdk_server_node.cpp
        )
ament_target_dependencies(minimal_slamware_ros_sdk_server_node rclcpp  std_msgs example_interfaces )
#target_include_directories(slamware_ros_sdk_server_node
#        PRIVATE ${SLTC_SDK_INC_DIR}
#        )
target_compile_options(minimal_slamware_ros_sdk_server_node
        PRIVATE -Wno-deprecated-declarations
        )
target_link_libraries(minimal_slamware_ros_sdk_server_node
        ${ament_LIBRARIES}

        pthread

        )
#==================================
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slamware_ros_sdk
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs tf message_runtime
#)

#include_directories(
#  include
##  ${catkin_INCLUDE_DIRS}
#)
#message("${INCLUDE_DIRECTORIES}")

#add_library(${PROJECT_NAME} STATIC
#  src/client/slamware_ros_sdk_client.cpp
#)
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(slamware_ros_sdk_server_node
#  src/server/msg_convert.cpp
#  src/server/server_params.cpp
#  src/server/server_map_holder.cpp
#  src/server/server_work_data.cpp
#  src/server/server_worker_base.cpp
#  src/server/server_workers.cpp
#  src/server/slamware_ros_sdk_server_node.cpp
#  src/server/slamware_ros_sdk_server.cpp
#)
#add_dependencies(slamware_ros_sdk_server_node ${PROJECT_NAME}_generate_messages_cpp)
#target_include_directories(slamware_ros_sdk_server_node
#  PRIVATE ${SLTC_SDK_INC_DIR}
#)
#target_compile_options(slamware_ros_sdk_server_node
#  PRIVATE -Wno-deprecated-declarations
#)
#target_link_libraries(slamware_ros_sdk_server_node
#  ${catkin_LIBRARIES}
#  ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
#  ${SLTC_SDK_LIB_DIR}/librpos_framework.a
#  ${SLTC_SDK_LIB_DIR}/libbase64.a
#  ${SLTC_SDK_LIB_DIR}/librlelib.a
#  ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
#  ${SLTC_SDK_LIB_DIR}/libcurl.a
#  ${SLTC_SDK_LIB_DIR}/libssl.a
#  ${SLTC_SDK_LIB_DIR}/libcrypto.a
#  ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
#  ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
#  ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
#  ${SLTC_SDK_LIB_DIR}/libboost_regex.a
#  ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
#  ${SLTC_SDK_LIB_DIR}/libboost_system.a
#  ${SLTC_SDK_LIB_DIR}/libboost_thread.a
#  ${SLTC_SDK_LIB_DIR}/libboost_random.a
#  pthread
#  dl
#  rt
#)
#====================================================
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(message_filters)


set(TARGETS_TO_INSTALL
        slamware_ros_sdk_server_node
        minimal_slamware_ros_sdk_server_node
        ${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )

install(
        TARGETS ${TARGETS_TO_INSTALL}
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}
)

#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_server_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
#
#install(FILES
#  launch/slamware_ros_sdk_server_node.launch
#  launch/view_slamware_ros_sdk_server_node.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
#
#install(DIRECTORY rviz
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

ament_package()