cmake_minimum_required(VERSION 3.5)
project(slamware_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
        msg/OptionalBool.msg
        msg/OptionalInt8.msg
        msg/OptionalInt16.msg
        msg/OptionalInt32.msg
        msg/OptionalInt64.msg
        msg/OptionalUInt8.msg
        msg/OptionalUInt16.msg
        msg/OptionalUInt32.msg
        msg/OptionalUInt64.msg
        msg/OptionalFlt32.msg
        msg/OptionalFlt64.msg
        msg/Vec2DInt32.msg
        msg/Vec2DFlt32.msg
        msg/Line2DFlt32.msg
        msg/Line2DFlt32Array.msg
        msg/RectInt32.msg
        msg/RectFlt32.msg
        msg/RobotDeviceInfo.msg
        msg/MapKind.msg
        msg/ArtifactUsage.msg
        msg/SensorType.msg
        msg/ImpactType.msg
        msg/BasicSensorInfo.msg
        msg/BasicSensorInfoArray.msg
        msg/BasicSensorValue.msg
        msg/BasicSensorValueData.msg
        msg/BasicSensorValueDataArray.msg
        msg/ActionDirection.msg
        msg/RobotBasicState.msg
        msg/SyncMapRequest.msg
        msg/MoveOptionFlag.msg
        msg/MoveOptions.msg
        msg/MoveByDirectionRequest.msg
        msg/MoveByThetaRequest.msg
        msg/MoveToRequest.msg
        msg/MoveToLocationsRequest.msg
        msg/RotateToRequest.msg
        msg/RotateRequest.msg
        msg/LocalizationMovement.msg
        msg/OptionalLocalizationMovement.msg
        msg/LocalizationOptions.msg
        msg/RecoverLocalizationRequest.msg
        msg/ClearMapRequest.msg
        msg/SetMapUpdateRequest.msg
        msg/SetMapLocalizationRequest.msg
        msg/GoHomeRequest.msg
        msg/CancelActionRequest.msg
        msg/AddLineRequest.msg
        msg/AddLinesRequest.msg
        msg/RemoveLineRequest.msg
        msg/ClearLinesRequest.msg
        msg/MoveLineRequest.msg
        msg/MoveLinesRequest.msg
        msg/Bumper.msg
        )

set(srv_files
        srv/SyncGetStcm.srv
        srv/SyncSetStcm.srv
        )

#set(action_files
#  "action/Patrol.action"
#)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        #  ${action_files}
        DEPENDENCIES std_msgs sensor_msgs nav_msgs
        )


#target_include_directories(my_target
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)

#ament_export_targets(export_my_library HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime std_msgs sensor_msgs nav_msgs)



ament_package()


#install(
#        DIRECTORY include/
#        DESTINATION include
#)
