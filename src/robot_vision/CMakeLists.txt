cmake_minimum_required(VERSION 3.22)
project(robot_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    add_compile_options($<$<CONFIG:Release>:-O3>)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake         REQUIRED)
find_package(rclcpp              REQUIRED)
find_package(sensor_msgs         REQUIRED)
find_package(std_msgs            REQUIRED)
find_package(geometry_msgs       REQUIRED)
find_package(nav_msgs            REQUIRED)
find_package(vision_msgs         REQUIRED)
find_package(cv_bridge           REQUIRED)
find_package(tf2                 REQUIRED)
find_package(tf2_geometry_msgs   REQUIRED)
find_package(ament_index_cpp     REQUIRED)
find_package(irobot_create_msgs  REQUIRED)
find_package(behaviortree_cpp    REQUIRED)
find_package(PkgConfig           REQUIRED)
find_package(rclcpp_action       REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio aruco calib3d)

# ----- libevdev -----

pkg_check_modules(LIBEVDEV REQUIRED libevdev)
message(STATUS "libevdev include : ${LIBEVDEV_INCLUDE_DIRS}")
message(STATUS "libevdev lib     : ${LIBEVDEV_LIBRARIES}")

# ----- PortAudio -----

pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
message(STATUS "PortAudio include : ${PORTAUDIO_INCLUDE_DIRS}")
message(STATUS "PortAudio lib     : ${PORTAUDIO_LIBRARIES}")

# ----- yaml-cpp -----
find_package(yaml-cpp REQUIRED)
message(STATUS "yaml-cpp include : ${YAML_CPP_INCLUDE_DIR}")
message(STATUS "yaml-cpp lib     : ${YAML_CPP_LIBRARIES}")

# ----- ONNX Runtime -----
find_path(ONNXRUNTIME_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        PATHS
        /usr/local/include
        /usr/local/include/onnxruntime
        /usr/include
        /usr/include/onnxruntime
)

find_library(ONNXRUNTIME_LIBRARY
        NAMES onnxruntime
        PATHS
        /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        /usr/lib/x86_64-linux-gnu
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR "ONNX Runtime introuvable.")
endif()

add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION             "${ONNXRUNTIME_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

# ----- Sherpa-ONNX -----
find_path(SHERPA_ONNX_INCLUDE_DIR
        NAMES sherpa-onnx/c-api/c-api.h
        PATHS
        /usr/local/include
        /usr/include
)

find_library(SHERPA_ONNX_LIBRARY
        NAMES sherpa-onnx-c-api
        PATHS
        /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        /usr/lib/x86_64-linux-gnu
)

if(NOT SHERPA_ONNX_INCLUDE_DIR OR NOT SHERPA_ONNX_LIBRARY)
    message(FATAL_ERROR "Sherpa-ONNX introuvable.")
endif()

add_library(sherpa_onnx::c_api UNKNOWN IMPORTED)
set_target_properties(sherpa_onnx::c_api PROPERTIES
        IMPORTED_LOCATION             "${SHERPA_ONNX_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${SHERPA_ONNX_INCLUDE_DIR}"
)

# ----- Dépendances ROS2 communes -----
set(ROS_DEPS
        rclcpp
        rclcpp_action
        sensor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        vision_msgs
        cv_bridge
        tf2
        tf2_geometry_msgs
        ament_index_cpp
        irobot_create_msgs
        behaviortree_cpp
)

# ----- YOLO Detector Library -----
add_library(yolo_detector_lib src/librairies/yolo_detector.cpp)
target_include_directories(yolo_detector_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(yolo_detector_lib ${OpenCV_LIBS} onnxruntime::onnxruntime)
ament_target_dependencies(yolo_detector_lib ${ROS_DEPS})

# ----- LED Manager Library -----
add_library(led_manager_lib src/librairies/led_manager.cpp)
target_include_directories(led_manager_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(led_manager_lib ${ROS_DEPS})

# ----- Sound Manager Library -----
add_library(sound_manager_lib src/librairies/sound_manager.cpp)
target_include_directories(sound_manager_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(sound_manager_lib yaml-cpp)
ament_target_dependencies(sound_manager_lib ${ROS_DEPS})

# ----- Video Node -----
add_executable(video_node src/video_node.cpp)
target_include_directories(video_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(video_node ${OpenCV_LIBS})
ament_target_dependencies(video_node ${ROS_DEPS})

# ----- Detection Node -----
add_executable(detection_node src/detection_node.cpp)
target_include_directories(detection_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(detection_node ${OpenCV_LIBS} onnxruntime::onnxruntime yolo_detector_lib)
ament_target_dependencies(detection_node ${ROS_DEPS})

# ----- Follow Node -----
add_executable(follow_node src/follow_node.cpp)
target_include_directories(follow_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(follow_node sound_manager_lib)
ament_target_dependencies(follow_node ${ROS_DEPS})

# ---- Teleop Node -----
add_executable(teleop_node src/teleop_node.cpp)
target_include_directories(teleop_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${LIBEVDEV_INCLUDE_DIRS})
target_link_libraries(teleop_node ${LIBEVDEV_LIBRARIES})
ament_target_dependencies(teleop_node ${ROS_DEPS})

# ----- listen_node -----
add_executable(listen_node src/listen_node.cpp)
target_include_directories(listen_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${PORTAUDIO_INCLUDE_DIRS})
target_link_libraries(listen_node ${PORTAUDIO_LIBRARIES} sherpa_onnx::c_api)
ament_target_dependencies(listen_node ${ROS_DEPS})

# ----- BT Manager Node -----
add_executable(bt_manager_node src/bt_manager_node.cpp
                               src/bt_utils/nodes/is_mode.cpp
                               src/bt_utils/nodes/follow_action.cpp
                               src/bt_utils/nodes/listen_action.cpp
                               src/bt_utils/nodes/teleop_action.cpp
)
target_include_directories(bt_manager_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(bt_manager_node led_manager_lib sound_manager_lib yaml-cpp)
ament_target_dependencies(bt_manager_node ${ROS_DEPS})

# ----- Dock Detector Node -----
add_executable(dock_detector_node src/dock_detector_node.cpp)
target_include_directories(dock_detector_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(dock_detector_node ${OpenCV_LIBS} yaml-cpp )
ament_target_dependencies(dock_detector_node ${ROS_DEPS})

# ----- Dock Calibration Node -----
add_executable(dock_calibration_node src/dock_calibration_node.cpp)
target_include_directories(dock_calibration_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(dock_calibration_node ${ROS_DEPS})

# ----- Installation -----
install(TARGETS
        video_node
        detection_node
        follow_node
        teleop_node
        listen_node
        bt_manager_node
        dock_detector_node
        dock_calibration_node
        led_manager_lib
        sound_manager_lib
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/  DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config OPTIONAL)

install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

# ----- Tests -----
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)

    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_yolo_detector test/test_yolo_detector.cpp)
    target_link_libraries(test_yolo_detector
            ${OpenCV_LIBS}
            onnxruntime::onnxruntime
            yolo_detector_lib
    )

    ament_add_pytest_test(test_follow_node       test/python/test_follow_node.py       TIMEOUT 60)
#    ament_add_pytest_test(test_bt_manager_node   test/python/test_bt_manager_node.py   TIMEOUT 90)
endif()

ament_package()
