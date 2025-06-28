# 显式列出源文件
set(SIGNAL_SOURCES
    ngx_signal.cxx
)

add_library(signal STATIC ${SIGNAL_SOURCES})

# 安装规则（可选）
install(TARGETS signal DESTINATION lib)