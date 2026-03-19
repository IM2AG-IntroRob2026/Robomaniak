cmake_minimum_required(VERSION 3.22)
project(robot_vision LANGUAGES CXX)

# ── Standard C++20 ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimisations release par défaut si aucun type de build n'est spécifié
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)


# ── Dépendances ROS2 ───────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(irobot_create_msgs REQUIRED)

# ── OpenCV ─────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

# ── ONNX Runtime ───────────────────────────────────────────────────────────────
# On contourne find_package() car le fichier cmake précompilé d'ONNX Runtime
# contient un chemin codé en dur (lib64/) qui ne correspond pas à l'endroit
# réel de la lib sur Ubuntu (lib/). On fait une recherche manuelle à la place.

find_path(ONNXRUNTIME_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        PATHS
        /usr/local/include/onnxruntime
        /usr/include/onnxruntime
        /usr/local/include
        /usr/include
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
    message(FATAL_ERROR
            "ONNX Runtime introuvable.\n"
            "Télécharger le package précompilé depuis :\n"
            "  https://github.com/microsoft/onnxruntime/releases\n"
            "Puis installer avec :\n"
            "  sudo cp -r onnxruntime-*/include/* /usr/local/include/\n"
            "  sudo cp -r onnxruntime-*/lib/*     /usr/local/lib/\n"
            "  sudo ldconfig\n"
    )
endif()

message(STATUS "ONNX Runtime headers : ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime lib     : ${ONNXRUNTIME_LIBRARY}")

# Création d'une target importée propre
add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION             "${ONNXRUNTIME_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)


# ── Helpers réutilisables ──────────────────────────────────────────────────────

# Applique les dépendances ROS2 communes aux deux nœuds
set(ROS_DEPS
        rclcpp
        sensor_msgs
        std_msgs
        cv_bridge
        irobot_create_msgs
)


# ── video_node ─────────────────────────────────────────────────────────────────
add_executable(video_node src/video_node.cpp)

target_include_directories(video_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(video_node
        ${OpenCV_LIBS}
)
ament_target_dependencies(video_node ${ROS_DEPS})


# ── detection_node ─────────────────────────────────────────────────────────────
add_executable(detection_node src/detection_node.cpp)

target_include_directories(detection_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(detection_node
        ${OpenCV_LIBS}
        onnxruntime::onnxruntime
)
ament_target_dependencies(detection_node ${ROS_DEPS})


# ── Installation ───────────────────────────────────────────────────────────────
install(TARGETS
        video_node
        detection_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

# Copie du dossier models/ dans le share/ du package pour y accéder via
# ament_index_python ou share_directory en runtime
install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/models
)


# ── Tests (optionnel) ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()