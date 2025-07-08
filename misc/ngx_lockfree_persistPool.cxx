#include "ngx_lockfree_threadPool.h"
#include "ngx_mysql_connection.h"
#include "ngx_mysql_connection_pool.h"
#include <fstream>
#include <sstream>
#include <ctime>

PersistProcessingPool::PersistProcessingPool(size_t thread_count, LockFreeQueue<ResPointCloud,QUEUE_SIZE>& input_queue)
                :ThreadPool(thread_count, "PersistProcessing"), input_queue_(input_queue) {
    start();
}

bool PersistProcessingPool::get_task(Task& task) {
    ResPointCloud raw_data;
    if (input_queue_.try_pop(raw_data)) {
        task = [this, raw_data] {
            if (process_data(raw_data)) {
                ngx_log_stderr(0, "保存成功！");
            } else {
                ngx_log_stderr(0, "保存失败！");
            }
            ngx_log_stderr(0, "线程id:%P", std::this_thread::get_id());
        };
        return true;
    }
    return false;
}

bool PersistProcessingPool::process_data(ResPointCloud data) {
    // 生成唯一的文件名
    std::string file_path = generate_unique_filename("./point_clouds/", data.ID);

    // 存储点云数据到文件系统
    std::ofstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        ngx_log_stderr(0, "无法打开文件: %s", file_path.c_str());
        return false;
    }
    file.write(data.serlizdPCdata.serializedData, data.serlizdPCdata.dataLen);
    file.close();

    // 数据库操作：存储文件路径
    char sql[1024] = { 0 };
    sprintf(sql, "INSERT INTO user(IDC, asymmetry, pc_data_path) VALUES('%s', %f, '%s')",
            data.ID, data.asymmetry, file_path.c_str());

    // 获取数据库连接
    MySQLConnectionPool *cp = MySQLConnectionPool::getConnectionPool();
    std::shared_ptr<Connection> conn = cp->getConnection();
    if (conn == nullptr || !conn->update(sql)) {
        ngx_log_stderr(0, "数据库操作失败: %s", sql);
        return false;
    }

    return true;
}

// 生成唯一的文件名，避免多个线程存储时发生冲突
std::string PersistProcessingPool::generate_unique_filename(const char* base_path, const std::string& id) {
    std::stringstream filename;
    filename << base_path << id << "_" << std::this_thread::get_id() << "_" << std::time(0) << ".drc";
    return filename.str();
}

 