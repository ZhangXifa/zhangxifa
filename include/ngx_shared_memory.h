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

    // 2. 调整共享内存大小
    if (ftruncate(shm_fd, size) == -1) {
        ngx_log_stderr(0, "ftruncate failed for %s", shm_name);
        close(shm_fd);
        return nullptr;
    }

    // 3. 内存映射
    T* queue = static_cast<T*>(
        mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    
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
    IDC CHAR(7) NOT NULL,                  -- 固定长度为7的字符ID
    asymmetry DOUBLE NOT NULL,              -- 存储双精度浮点数
    pc_data_path TEXT NOT NULL,  
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,  -- 可选：创建时间
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP   -- 可选：更新时间
);
CREATE INDEX idx_respointcloud_id ON ResPointCloud (ID);
*/