# 显式列出源文件（推荐）
set(LOGIC_SOURCES
    ngx_c_slogic.cxx
)

# 编译为静态库
add_library(logic STATIC ${LOGIC_SOURCES})

target_include_directories(logic PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${draco_INCLUDE_DIRS}
)

# 链接第三方库（PUBLIC传递依赖）
target_link_libraries(logic PUBLIC
    ${PCL_LIBRARIES}
    draco::draco
    rt
)

# 安装规则（可选）
install(TARGETS logic DESTINATION lib)