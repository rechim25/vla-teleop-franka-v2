cmake_minimum_required(VERSION 3.16)
project(franka_xr_teleop_superbuild LANGUAGES CXX)
include(CTest)

add_subdirectory(cpp/teleop_bridge)
