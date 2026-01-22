cmake_minimum_required(VERSION 3.5) # cmake version for ros2 humble

project(test_interfaces)

if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set (CMAKE_CXX_FLAGS "--std=c++14 ${CMAKE_CXX_FLAGS}")
    endif ()
else()
    set (CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set ( CMAKE_VERBOSE_MAKEFILE ON )

# find dependencies
find_package(ament_cmake REQUIRED) # default package
find_package(rosidl_default_generators REQUIRED) # default package
find_package(std_msgs REQUIRED) # Basic varaible type in ros2.
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# other Packages 
# geometry_msgs ==> 좌표계.

set(msg_files
    "msg/HeadTouchGesture.msg"
)
set(srv_files
    "srv/SetFanPWM.srv"
)
set(action_files
    "action/Spin.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs builtin_interfaces action_msgs # the interface names defiend by find_package
)

ament_export_dependencies(rosidl_default_runtime) # required for colcon build
ament_package()
