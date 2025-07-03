cmake_minimum_required(VERSION 3.12)
project(pointServer)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型和编译选项
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)  # 默认Debug模式
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")

# 增强版MySQL查找
set(MYSQL_POSSIBLE_PATHS
  "/usr/local/mysql"
  "/usr"
  "/opt/homebrew"  # macOS
  "C:/Program Files/MySQL/MySQL Server 8.0"  # Windows
)

find_path(MYSQL_INCLUDE_DIR mysql.h
  PATHS ${MYSQL_POSSIBLE_PATHS}
  PATH_SUFFIXES include mysql include/mysql
  REQUIRED
)

find_library(MYSQL_LIBRARY NAMES mysqlclient libmysql
  PATHS ${MYSQL_POSSIBLE_PATHS}
  PATH_SUFFIXES lib lib64 lib/mysql
  REQUIRED
)

# 包含目录（保持您原有的include_directories）
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

# 查找依赖（保持您原有的find_package顺序）
find_package(PCL 1.7 REQUIRED COMPONENTS 
    common kdtree search registration io features
)
find_package(draco REQUIRED)

# 全局包含路径（保持您原有的结构，但添加REQUIRED检查）
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${draco_INCLUDE_DIRS}
    ${MYSQL_INCLUDE_DIR}
)

# 定义全局链接库变量（新增）
set(PROJECT_LIBS
    ${PCL_LIBRARIES}
    ${draco_LIBRARIES}
    ${MYSQL_LIBRARY}
    pthread
)

# 添加模块（完全保持您原有的子模块顺序）
add_subdirectory(logic)  # 基础模块最先
add_subdirectory(persist)
add_subdirectory(misc)
add_subdirectory(net)
add_subdirectory(proc)
add_subdirectory(signal)
add_subdirectory(app)    # 主程序最后