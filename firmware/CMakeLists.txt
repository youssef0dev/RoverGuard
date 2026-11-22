cmake_minimum_required(VERSION 3.16)
project(roverguard_firmware_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(roverguard_firmware_core
    src/pid_controller.cpp
    src/drive_controller.cpp
)
target_include_directories(roverguard_firmware_core PUBLIC include)

add_executable(roverguard_firmware_demo src/desktop_main.cpp)
target_link_libraries(roverguard_firmware_demo PRIVATE roverguard_firmware_core)

if(MSVC)
  target_compile_options(roverguard_firmware_core PRIVATE /W4)
else()
  target_compile_options(roverguard_firmware_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

