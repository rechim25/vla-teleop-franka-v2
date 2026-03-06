cmake_minimum_required(VERSION 3.16)
project(franka_xr_teleop_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Franka CONFIG QUIET)
set(USE_PKGCONFIG_FRANKA OFF)
if(NOT Franka_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(FRANKA QUIET franka)
  if(NOT FRANKA_FOUND)
    pkg_check_modules(FRANKA QUIET libfranka)
  endif()
  if(NOT FRANKA_FOUND)
    message(FATAL_ERROR
            "libfranka not found. Install libfranka 0.9.2 (or >=0.9.1,<0.10.0) and set "
            "Franka_DIR/CMAKE_PREFIX_PATH or pkg-config paths.")
  endif()
  set(USE_PKGCONFIG_FRANKA ON)
endif()

add_executable(franka_xr_teleop_bridge
  main.cpp
  xr_receiver.cpp
  teleop_mapper.cpp
  teleop_state_machine.cpp
  franka_controller.cpp
  safety.cpp
  gripper.cpp
  observation_pub.cpp
)

target_link_libraries(franka_xr_teleop_bridge PRIVATE Threads::Threads)

if(NOT USE_PKGCONFIG_FRANKA)
  if(DEFINED Franka_VERSION)
    if(Franka_VERSION VERSION_LESS 0.9.1 OR Franka_VERSION VERSION_GREATER_EQUAL 0.10.0)
      message(FATAL_ERROR
              "Unsupported libfranka version ${Franka_VERSION}. Expected >=0.9.1 and <0.10.0.")
    endif()
  endif()
endif()

if(TARGET Franka::Franka)
  target_link_libraries(franka_xr_teleop_bridge PRIVATE Franka::Franka)
elseif(TARGET franka::franka)
  target_link_libraries(franka_xr_teleop_bridge PRIVATE franka::franka)
elseif(TARGET franka)
  target_link_libraries(franka_xr_teleop_bridge PRIVATE franka)
elseif(USE_PKGCONFIG_FRANKA)
  target_include_directories(franka_xr_teleop_bridge PRIVATE ${FRANKA_INCLUDE_DIRS})
  target_link_directories(franka_xr_teleop_bridge PRIVATE ${FRANKA_LIBRARY_DIRS})
  target_link_libraries(franka_xr_teleop_bridge PRIVATE ${FRANKA_LIBRARIES})
  target_compile_options(franka_xr_teleop_bridge PRIVATE ${FRANKA_CFLAGS_OTHER})
  if(DEFINED FRANKA_VERSION)
    if(FRANKA_VERSION VERSION_LESS 0.9.1 OR FRANKA_VERSION VERSION_GREATER_EQUAL 0.10.0)
      message(FATAL_ERROR
              "Unsupported libfranka version ${FRANKA_VERSION}. Expected >=0.9.1 and <0.10.0.")
    endif()
  endif()
else()
  message(FATAL_ERROR "Could not find a known libfranka CMake target.")
endif()
