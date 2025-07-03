# 显式列出源文件
set(PERSIST_SOURCES
    ngx_mysql_connection_pool.cxx
    ngx_mysql_connection.cxx
)
# 编译为静态库
add_library(persist STATIC ${PERSIST_SOURCES})

target_include_directories(persist PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# 链接第三方库（PUBLIC传递依赖）
target_link_libraries(persist PUBLIC
    ${MYSQL_LIBRARY}
    draco::draco
)

# 安装规则（可选）
install(TARGETS persist DESTINATION lib)