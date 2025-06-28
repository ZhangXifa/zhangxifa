# 显式列出源文件
set(NET_SOURCES
    ngx_c_socket_accept.cxx
    ngx_c_socket_conn.cxx
    ngx_c_socket_inet.cxx
    ngx_c_socket_request.cxx
    ngx_c_socket_time.cxx
    ngx_c_socket.cxx
)

add_library(net STATIC ${NET_SOURCES})

# 安装规则（可选）
install(TARGETS net DESTINATION lib)