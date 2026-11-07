cmake_minimum_required(VERSION 3.10)
project(qmini_auto_nav CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Jetson 上的 ZED SDK 3.5 不在默认的 cmake 搜索根里。
set(ZED_DIR "/usr/local/zed" CACHE PATH "ZED SDK 根目录")
option(LINK_SHARED_ZED "Link the shared ZED SDK" ON)

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV ${OpenCV_VERSION}")
message(STATUS "ZED include ${ZED_INCLUDE_DIRS}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${ZED_LIBRARY_DIR} ${CUDA_LIBRARY_DIRS})

add_executable(qmini_auto_nav
    src/main.cpp
    src/zed_session.cpp
    src/perception.cpp
    src/planner.cpp
    src/visualizer.cpp
)

target_compile_options(qmini_auto_nav PRIVATE -O2 -Wall -Wno-deprecated-declarations)

if(LINK_SHARED_ZED)
    set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
endif()

target_link_libraries(qmini_auto_nav ${ZED_LIBS} ${OpenCV_LIBS} pthread)

set_target_properties(qmini_auto_nav PROPERTIES
    BUILD_RPATH "/usr/local/zed/lib"
    INSTALL_RPATH "/usr/local/zed/lib"
)
