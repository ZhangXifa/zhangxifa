#include "ngx_lockfree_threadPool.h"
#include "ngx_mysql_connection.h"
#include "ngx_mysql_connection_pool.h"

PersistProcessingPool::PersistProcessingPool(size_t thread_count, LockFreeQueue<ResPointCloud,QUEUE_SIZE>& input_queue)
                :ThreadPool(thread_count,"PersistProcessing"), input_queue_(input_queue){
                    start();
                }
bool PersistProcessingPool::get_task(Task& task){
    ResPointCloud raw_data;
    if (input_queue_.try_pop(raw_data)){
        task = [this, raw_data]{
            if(process_data(raw_data)){
                ngx_log_stderr(0,"保存成功！");
            }else{
                ngx_log_stderr(0,"保存失败！");
            }
            ngx_log_stderr(0,"线程id:%P",std::this_thread::get_id());
        };
        return true;
    }
    return false;
 }
 bool PersistProcessingPool::process_data(ResPointCloud data){
    Connection conn;
    MySQLConnectionPool *cp = MySQLConnectionPool::getConnectionPool();
    char sql[1024] = { 0 };
    sprintf(sql, "insert into user(IDC,asymmetry,pc_data_path) values('%s',%f,'%s')",
			data.ID, data.asymmetry, "D:/");
		//获取连接
		std::shared_ptr<Connection> sp = cp->getConnection();
		sp->update(sql);
 }
 