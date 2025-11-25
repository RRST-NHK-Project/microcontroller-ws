cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(zephyr_f4_wip)

# micro‑ROS モジュール読み込み
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/micro_ros_zephyr_module/modules
)
FILE(GLOB app_sources ../src/*.c*)
target_sources(app PRIVATE ${app_sources})
