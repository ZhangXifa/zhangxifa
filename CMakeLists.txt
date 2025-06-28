cmake_minimum_required(VERSION 3.12)
project(pointServer)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)  # 设置构建类型
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")  # 全局编译选项

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
# 查找依赖
find_package(PCL 1.7 REQUIRED COMPONENTS 
  common kdtree search registration io features
)
find_package(draco REQUIRED)

# 全局包含路径（所有模块共享）
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${draco_INCLUDE_DIRS}
)

# 添加模块（按依赖顺序）
add_subdirectory(logic)  # 基础模块最先（含PCL/Draco）
add_subdirectory(misc)
add_subdirectory(net)
add_subdirectory(proc)
add_subdirectory(signal)
add_subdirectory(app)    # 主程序最后