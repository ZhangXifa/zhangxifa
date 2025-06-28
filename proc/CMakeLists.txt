# 显式列出源文件
set(PROC_SOURCES
    ngx_daemon.cxx
    ngx_event.cxx
    ngx_process_cycle.cxx
)
# 编译为静态库
add_library(proc STATIC ${PROC_SOURCES})

target_include_directories(proc PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${draco_INCLUDE_DIRS}
)

# 链接第三方库（PUBLIC传递依赖）
target_link_libraries(proc PUBLIC
    ${PCL_LIBRARIES}
    draco::draco
    rt
)

# 安装规则（可选）
install(TARGETS proc DESTINATION lib)