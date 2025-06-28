# 显式列出源文件（按逻辑顺序）
set(NGINX_SOURCES
    ngx_c_conf.cxx      # 配置系统最先
    ngx_string.cxx      # 基础工具
    ngx_printf.cxx      # 输出功能
    ngx_log.cxx         # 日志系统
    ngx_setproctitle.cxx # 进程管理
    nginx.cxx           # 主程序最后
)

# 生成可执行文件
add_executable(nginx ${NGINX_SOURCES})

# 链接所有依赖模块
target_link_libraries(nginx PRIVATE
    logic    # 包含PCL/Draco依赖
    net
    proc
    signal
    misc
)

# 设置输出目录
set_target_properties(nginx PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 安装规则
install(TARGETS nginx DESTINATION bin)