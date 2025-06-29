#include "ngx_lockfree_threadPool.h"

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
    double y = data.asymmetry;
    ngx_log_stderr(0, "不对称度为：%f",y);
    return true;
 }