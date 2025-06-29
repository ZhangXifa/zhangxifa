# 显式列出源文件
set(MISC_SOURCES
    ngx_c_crc32.cxx
    ngx_c_memory.cxx
    ngx_c_threadpool.cxx
    ngx_lockfree_mirrorICP_threadPool.cxx
    ngx_lockfree_asymCal_threadPool.cxx
    ngx_lockfree_persistPool.cxx
    ngx_lockfree_threadPool.cxx
)
# 编译为静态库
add_library(misc STATIC ${MISC_SOURCES})

target_include_directories(misc PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${draco_INCLUDE_DIRS}
)

# 链接第三方库（PUBLIC传递依赖）
target_link_libraries(misc PUBLIC
    ${PCL_LIBRARIES}
    draco::draco
    rt
)

# 安装规则（可选）
install(TARGETS misc DESTINATION lib)