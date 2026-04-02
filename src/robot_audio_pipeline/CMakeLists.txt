cmake_minimum_required(VERSION 3.8)
project(robot_audio_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(BLAS REQUIRED)
find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)
  pkg_check_modules(PORTAUDIO QUIET portaudio-2.0)
endif()

if(NOT PORTAUDIO_FOUND)
  find_path(PORTAUDIO_INCLUDE_DIR
    NAMES portaudio.h
    PATHS
      /usr/local/include
      /usr/include
  )

  find_library(PORTAUDIO_LIBRARY
    NAMES portaudio
    PATHS
      /usr/local/lib
      /usr/lib
      /usr/lib/x86_64-linux-gnu
  )

  if(NOT PORTAUDIO_INCLUDE_DIR OR NOT PORTAUDIO_LIBRARY)
    message(FATAL_ERROR
      "PortAudio introuvable.\n"
      "Installer par exemple:\n"
      "  sudo apt install portaudio19-dev\n")
  endif()

  set(PORTAUDIO_INCLUDE_DIRS ${PORTAUDIO_INCLUDE_DIR})
  set(PORTAUDIO_LIBRARIES ${PORTAUDIO_LIBRARY})
  set(PORTAUDIO_LIBRARY_DIRS "")
  set(PORTAUDIO_CFLAGS_OTHER "")
endif()

set(SNOWBOY_SDK_DIR "$ENV{SNOWBOY_SDK_DIR}" CACHE PATH
  "Chemin du SDK Snowboy (doit contenir include/ et lib/)")

find_path(SNOWBOY_INCLUDE_DIR
  NAMES snowboy-detect.h
  HINTS
    ${SNOWBOY_SDK_DIR}/include
    ${SNOWBOY_SDK_DIR}
  PATHS
    /usr/local/include
    /usr/include
)

find_library(SNOWBOY_LIBRARY
  NAMES snowboy-detect
  HINTS
    ${SNOWBOY_SDK_DIR}/lib
    ${SNOWBOY_SDK_DIR}/lib/linux/x86_64
    ${SNOWBOY_SDK_DIR}/lib/linux/aarch64
    ${SNOWBOY_SDK_DIR}/lib/ubuntu64
    ${SNOWBOY_SDK_DIR}
  PATHS
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
)

if(NOT SNOWBOY_INCLUDE_DIR OR NOT SNOWBOY_LIBRARY)
  message(FATAL_ERROR
    "Snowboy introuvable.\n"
    "Attendu: header snowboy-detect.h + lib snowboy-detect.\n"
    "Option recommandee: definir SNOWBOY_SDK_DIR vers le SDK Snowboy.\n"
    "Exemple:\n"
    "  export SNOWBOY_SDK_DIR=$HOME/snowboy\n")
endif()

add_library(snowboy_bridge STATIC
  src/snowboy_c_bridge.cpp
)

target_include_directories(snowboy_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SNOWBOY_INCLUDE_DIR}
)

# Snowboy prebuilt binaries are built with the old libstdc++ ABI.
target_compile_definitions(snowboy_bridge PRIVATE _GLIBCXX_USE_CXX11_ABI=0)

target_link_libraries(snowboy_bridge
  ${SNOWBOY_LIBRARY}
  ${BLAS_LIBRARIES}
)

add_executable(audio_pipeline_node
  src/audio_pipeline_node.cpp
  src/portaudio_input.cpp
  src/snowboy_engine.cpp
)

add_executable(voice_command_node
  src/voice_command_node.cpp
)

target_include_directories(audio_pipeline_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PORTAUDIO_INCLUDE_DIRS}
  ${SNOWBOY_INCLUDE_DIR}
)

target_link_directories(audio_pipeline_node PRIVATE
  ${PORTAUDIO_LIBRARY_DIRS}
)

target_link_libraries(audio_pipeline_node
  ${PORTAUDIO_LIBRARIES}
  snowboy_bridge
)

if(PORTAUDIO_CFLAGS_OTHER)
  target_compile_options(audio_pipeline_node PRIVATE ${PORTAUDIO_CFLAGS_OTHER})
endif()

ament_target_dependencies(audio_pipeline_node
  rclcpp
  std_msgs
)

ament_target_dependencies(voice_command_node
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  audio_pipeline_node
  voice_command_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
