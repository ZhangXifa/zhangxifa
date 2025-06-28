#include <cstdint>
#include <cstring>  // for memcpy

// 主机序转网络序 (double) - 内联优化
inline double htond(double host_value) noexcept {
    static_assert(sizeof(double) == sizeof(uint64_t), "double must be 8 bytes");//静态断言
    
    uint64_t host_int;
    std::memcpy(&host_int, &host_value, sizeof(host_int));
    uint64_t net_int = __builtin_bswap64(host_int);  // GCC/Clang 内置指令
    std::memcpy(&host_value, &net_int, sizeof(host_value));
    
    return host_value;
}

// 网络序转主机序 (double) - 直接复用 htond（双向转换）
inline double ntohd(double net_value) noexcept {
    return htond(net_value);  // 逻辑相同
}