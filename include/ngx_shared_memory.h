#ifndef NGX_SHARED_MEMORY_H
#define NGX_SHARED_MEMORY_H

#include "ngx_lockFreeQueue.h"
#include "ngx_func.h"
#include <sys/mman.h>    // shm_open, mmap, PROT_READ, etc
#include <fcntl.h>       // O_CREAT, O_RDWR
#include <unistd.h>      // ftruncate, close
#include <new>           // placement new
#include <cstdlib>       // exit, EXIT_FAILUR

#define NETWORK_TO_MASTER_SHM "/network_master_shm"
#define MASTER_TO_MIRROR_PROCESS_SHM "/master_mirror_process_shm"
#define MIRROR_PROCESS_TO_MASTER_SHM "/mirror_process_master_shm"
#define MASTER_TO_RESULT_PROCESS_SHM "/master_result_process_shm"
#define RESULT_PROCESS_TO_MASTER_SHM "/result_process_master_shm"
#define MASTER_TO_PERSIST_PROCESS_SHM "/master_persist_process_shm"

#define RETURN_TO_MASTER_SHM "/retrun_to_master_shm"
#define RETURN_TO_NETWORK_SHM "/return_to_network_shm"

#define QUEUE_SIZE 32  // 队列大小，最好是2的幂次

#pragma pack(1)
struct PointCloud{
    char serializedData[1024*1024];
    size_t dataLen;
    char ID[7];
    int fd;
};
struct MirrorICPPointCloud{
    char serializedData[1024*1024];
    char MirrorICPSerlzdData[1024*1024];
    size_t dataLen;
    size_t MirrorICPLen;
    char ID[7];
    int fd;
};
struct ResPointCloud
{
    PointCloud serlizdPCdata;
    double asymmetry;
    char ID[7];
};
struct ResToNetwork
{
    double asymmetry;
    int fd;
};
#pragma pack()

using NetworkToMasterQueue = LockFreeQueue<PointCloud,QUEUE_SIZE>;
using MasterToMirorProcessQueue = LockFreeQueue<PointCloud,QUEUE_SIZE>;
using MirorProcessToMasterQueue = LockFreeQueue<MirrorICPPointCloud,QUEUE_SIZE>;
using MasterToResProcessQueue = LockFreeQueue<MirrorICPPointCloud,QUEUE_SIZE>;
using ResProcessToMasterQueue = LockFreeQueue<ResPointCloud,QUEUE_SIZE>;
using MasterToPersistProcessQueue = LockFreeQueue<ResPointCloud,QUEUE_SIZE>;

using AsymmProcessToMaterQueue = LockFreeQueue<ResToNetwork,QUEUE_SIZE>;
using MasterToNetworkQueue = LockFreeQueue<ResToNetwork,QUEUE_SIZE>;

// 全局队列指针声明
extern NetworkToMasterQueue* g_net_to_master_queue;
extern MasterToMirorProcessQueue* g_master_to_miror_process_queue;
extern MirorProcessToMasterQueue* g_miror_process_to_master_queue;
extern MasterToResProcessQueue* g_master_to_res_process_queue;
extern ResProcessToMasterQueue* g_res_process_to_master_queue;
extern MasterToPersistProcessQueue* g_master_to_per_process_queue;

extern AsymmProcessToMaterQueue* g_asymm_process_to_master_queue;
extern MasterToNetworkQueue* g_master_to_network_queue;

//共享内存初始化
template <typename T>
T* open_shm_queue(char* shm_name, size_t size = sizeof(T)) {
    // 1. 创建/打开共享内存文件描述符
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        ngx_log_stderr(0, "shm_open failed for %s", shm_name);
        return nullptr;
    }
    /*
        shm_open函数原型：
        int shm_open(const char *name, int oflag, mode_t mode);
        name：共享内存对象的名称，必须以斜杠开头，例如"/my_shm"。
        oflag：打开标志，常用的有O_CREAT（如果不存在则创建）、O_RDONLY（只读）、O_RDWR（读写）等。
        mode：权限模式，如0666表示所有用户都有读写权限。
        成功时返回文件描述符，失败时返回-1
        底层机制：
            内核操作：
                1. 在/dev/shm/目录下创建文件（Linux）
                2. 分配内核内存对象
                3. 返回文件描述符用于后续操作
    */

    // 2. 调整共享内存大小
    if (ftruncate(shm_fd, size) == -1) {
        ngx_log_stderr(0, "ftruncate failed for %s", shm_name);
        close(shm_fd);
        return nullptr;
    }
    /*
        ftruncate函数原型：
        int ftruncate(int fd, off_t length);
        fd：文件描述符，shm_open返回的文件描述符。
        length：新的文件长度，单位为字节。
        成功时返回0，失败时返回-1。
        底层机制：
            1. 检查文件描述符是否有效。
            2. 调整文件大小到指定长度。
            3. 如果文件当前大小大于新长度，截断文件内容。
            4. 如果文件当前大小小于新长度，扩展文件内容（填充为0）。
    */

    // 3. 内存映射
    T* queue = static_cast<T*>(
        mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    /*
        mmap函数原型：
        void* mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset);
        addr：映射地址，通常设为nullptr让系统自动选择地址。
        length：映射的内存长度，单位为字节。
        prot：内存保护标志，如PROT_READ（只读）、PROT_WRITE（可写）等。
        flags：映射标志，如MAP_SHARED（共享映射）、MAP_PRIVATE（私有映射）等。
        fd：文件描述符，shm_open返回的文件描述符。
        offset：文件偏移量，从文件的哪个位置开始映射。
        成功时返回映射的内存地址，失败时返回MAP_FAILED。
        底层机制：
            1. 检查文件描述符是否有效。
            2. 检查映射地址是否有效。
        
    */
    
    if (queue == MAP_FAILED) {
        ngx_log_stderr(0, "mmap failed for %s", shm_name);
        close(shm_fd);
        return nullptr;
    }

    // 4. 使用placement new初始化对象
    new (queue) T();

    // 5. 关闭文件描述符（不影响已映射的内存）
    close(shm_fd);

    return queue;
}

/**
 * @brief 销毁共享内存队列
 * @param queue 要销毁的队列指针
 * @param shm_name 共享内存名称
 */
template <typename T>
void destroy_shm_queue(T* queue, const char* shm_name) {
    if (queue) {
        // 显式调用析构函数
        queue->~T();
        
        // 解除内存映射
        munmap(queue, sizeof(T));
        
        // 删除共享内存对象
        shm_unlink(shm_name);
    }
}

#endif // NGX_SHARED_MEMORY_H
/*
创建表
CREATE TABLE user (
    id INT PRIMARY KEY AUTO_INCREMENT,  -- 可选的自增主键
    IDC CHAR(7) NOT NULL,                  -- 固定长度为7的字符ID，将此键值设置为唯一
    asymmetry DOUBLE NOT NULL,              -- 存储双精度浮点数
    pc_data_path TEXT NOT NULL,  
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,  -- 可选：创建时间
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP   -- 可选：更新时间
);
CREATE INDEX idx_respointcloud_id ON ResPointCloud (ID);
*/