# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "1.2.0")
set(PROJECT_VERU ${PROJECT_VER})
set(PROJECT_PREF "ble_rgb_controller")

string (REPLACE "\." "_" PROJECT_VERU ${PROJECT_VERU})
set(PROJECT_BIN "${PROJECT_PREF}_${PROJECT_VERU}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_BIN})
