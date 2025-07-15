#include "ngx_lockfree_threadPool.h"
#include "ngx_mysql_connection.h"
#include "ngx_mysql_connection_pool.h"
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <sys/stat.h> // 用于目录操作
#include <unistd.h>   // 用于文件操作
#include <random>     // 用于生成UUID

PersistProcessingPool::PersistProcessingPool(size_t thread_count, LockFreeQueue<ResPointCloud, QUEUE_SIZE>& input_queue)
    : ThreadPool(thread_count, "PersistProcessing"), input_queue_(input_queue) {
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

std::string PersistProcessingPool::generate_uuid() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    
    const char* hex_chars = "0123456789abcdef";
    std::string uuid;
    uuid.reserve(36);
    
    for (int i = 0; i < 32; ++i) {
        if (i == 8 || i == 12 || i == 16 || i == 20) {
            uuid += '-';
        }
        uuid += hex_chars[dis(gen)];
    }
    
    return uuid;
}

bool PersistProcessingPool::process_data(ResPointCloud data) {
    MySQLConnectionPool* cp = MySQLConnectionPool::getConnectionPool();
    std::shared_ptr<Connection> conn = cp->getConnection();
    if (conn == nullptr) {
        ngx_log_stderr(0, "获取数据库连接失败");
        return false;
    }

    // 1. 生成临时文件名和最终文件名
    std::string temp_filename = "./point_clouds/temp/" + generate_uuid() + ".tmp";
    std::string final_filename = "./point_clouds/" + generate_uuid() + ".drc";
    
    // 2. 将数据写入临时文件
    {
        std::ofstream file(temp_filename, std::ios::binary);
        if (!file.is_open()) {
            ngx_log_stderr(0, "无法创建临时文件: %s", temp_filename.c_str());
            return false;
        }
        file.write(data.serlizdPCdata.serializedData, data.serlizdPCdata.dataLen);
        file.close();
    }

    // 3. 开始数据库事务
    if (!conn->update("START TRANSACTION")) {
        ngx_log_stderr(0, "开始事务失败");
        unlink(temp_filename.c_str()); // 删除临时文件
        return false;
    }

    try {
        // 4. 查询旧文件路径
        char query_sql[256] = {0};
        sprintf(query_sql, "SELECT pc_data_path FROM user WHERE TRIM(IDC)='%s'", data.ID);
        
        MYSQL_RES* res = conn->query(query_sql);
        if (res == nullptr) {
            ngx_log_stderr(0, "查询失败: %s", query_sql);
            throw std::runtime_error("查询失败");
        }

        std::string old_file_path;
        MYSQL_ROW row;
        if ((row = mysql_fetch_row(res)) != nullptr && row[0] != nullptr) {
            old_file_path = row[0];
        }
        mysql_free_result(res);

        // 5. 重命名临时文件为最终文件
        if (rename(temp_filename.c_str(), final_filename.c_str()) != 0) {
            ngx_log_stderr(0, "重命名文件失败: %s -> %s (错误代码: %d)", 
                temp_filename.c_str(), final_filename.c_str(), errno);
            throw std::runtime_error("文件重命名失败");
        }

        // 6. 更新数据库记录
        char sql[1024] = {0};
        sprintf(sql, 
            "INSERT INTO user(IDC, asymmetry, pc_data_path) VALUES('%s', %f, '%s') "
            "ON DUPLICATE KEY UPDATE asymmetry=VALUES(asymmetry), pc_data_path=VALUES(pc_data_path), updated_at=CURRENT_TIMESTAMP",
            data.ID, data.asymmetry, final_filename.c_str()
        );

        if (!conn->update(sql)) {
            ngx_log_stderr(0, "数据库操作失败: %s", sql);
            throw std::runtime_error("数据库操作失败");
        }

        // 7. 提交事务
        if (!conn->update("COMMIT")) {
            ngx_log_stderr(0, "提交事务失败");
            throw std::runtime_error("提交事务失败");
        }

        // 8. 成功后才删除旧文件
        if (!old_file_path.empty() && old_file_path != final_filename) {
            if (unlink(old_file_path.c_str()) != 0 && errno != ENOENT) {
                ngx_log_stderr(0, "警告: 删除旧文件失败: %s (错误代码: %d)", 
                    old_file_path.c_str(), errno);
                // 不视为致命错误
            }
        }

        return true;
    } catch (const std::exception& e) {
        ngx_log_stderr(0, "处理数据异常: %s", e.what());
        conn->update("ROLLBACK");
        
        // 清理文件
        unlink(temp_filename.c_str());
        unlink(final_filename.c_str());
        
        return false;
    }
}