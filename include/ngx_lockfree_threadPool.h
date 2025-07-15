#ifndef NGX_LOCKFREE_MIRRORICP_THREADPOOL_H
#define NGX_LOCKFREE_MIRRORICP_THREADPOOL_H
#include <vector>
#include <thread>
#include <functional>
#include <memory>  // 必须包含此头文件才能用 make_unique
#include "ngx_shared_memory.h"
#include "ngx_func.h"
#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>

class ThreadPool {
public:
    using Task = std::function<void()>;
    
    ThreadPool(size_t thread_count, const std::string& name) : stop_(false), name_(name), thread_count_(thread_count) {}
    
    virtual ~ThreadPool() {
        stop();
    }
    
    void start() {
        for (size_t i = 0; i < thread_count_; ++i) {
            workers_.emplace_back([this] {
                while (true) {
                    Task task;
                    
                    if (stop_.load(std::memory_order_acquire)) break;
                    
                    if (get_task(task)) {
                        task();
                    } else {
                        std::this_thread::yield();
                    }
                }
                ngx_log_stderr(0,"worker thread exiting!");
            });
        }
    }
    
    void stop() {
        stop_.store(true, std::memory_order_release);
        for (auto& worker : workers_) {
            if (worker.joinable()) worker.join();
        }
    }
    
    virtual bool get_task(Task& task) = 0;
    
    size_t size() const { return workers_.size(); }
    const std::string& name() const { return name_; }

protected:
    //基类的方法，子类调用
    std::unique_ptr<draco::PointCloud> decompressPointCloud(const char *pPkgBody,uint32_t iBodyLength);
    pcl::PointCloud<pcl::PointXYZ> DRCToPCD(const draco::PointCloud& draco_cloud);
    std::unique_ptr<draco::PointCloud> PCDToDRC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);
    bool CompressPCDToDraco(int encoder_speed, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, draco::EncoderBuffer& buffer);
    

    std::vector<std::thread> workers_;
    std::atomic<bool> stop_;
    std::string name_;
     size_t thread_count_;
};

//镜像/ICP配准模块线程池
class MirrorICPProcessingPool : public ThreadPool {
public:
    // 构造函数：接收输入队列和输出队列
    MirrorICPProcessingPool(size_t thread_count,
                 LockFreeQueue<PointCloud, QUEUE_SIZE>& input_queue,
                 LockFreeQueue<MirrorICPPointCloud, QUEUE_SIZE>& output_queue);
    
    // 从输入队列获取数据 -> 处理 -> 放入输出队列
    bool get_task(Task& task) override;

private:
    // 数据处理函数
    MirrorICPPointCloud process_data(PointCloud data);
    pcl::PointCloud<pcl::PointXYZ> pointCloudMirror(const pcl::PointCloud<pcl::PointXYZ>& pointCloudin);
    pcl::PointCloud<pcl::PointXYZ> ICPTransform(const pcl::PointCloud<pcl::PointXYZ>& pointCloudtarget, const pcl::PointCloud<pcl::PointXYZ>& pointCloudsource);

    //输入输出队列
    LockFreeQueue<PointCloud, QUEUE_SIZE>& input_queue_;   // 原始数据输入
    LockFreeQueue<MirrorICPPointCloud, QUEUE_SIZE>& output_queue_;  // 处理结果输出
};

//不对称度计算模块线程池
class ResultProcessingPool : public ThreadPool{
    public:
        //构造
        ResultProcessingPool(size_t thread_count,
                   LockFreeQueue<MirrorICPPointCloud,QUEUE_SIZE>& input_queue,
                   LockFreeQueue<ResPointCloud,QUEUE_SIZE>& output_queue);
        bool get_task(Task& task) override;
    private:
        //数据处理函数
        ResPointCloud process_data(MirrorICPPointCloud data);
        pcl::PointCloud<pcl::Normal>::Ptr computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int k_neighbors = 10);
        double computeMinDistanceToPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,const pcl::PointCloud<pcl::Normal>::Ptr& normals,const pcl::PointXYZ& query_point);
        double Asym(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMirror);

        
        //输入输出队列
        LockFreeQueue<MirrorICPPointCloud,QUEUE_SIZE>& input_queue_;
        LockFreeQueue<ResPointCloud,QUEUE_SIZE>& output_queue_;
};

//持久化进程模块的线程池
class PersistProcessingPool : public ThreadPool {
public:
    PersistProcessingPool(size_t thread_count, LockFreeQueue<ResPointCloud,QUEUE_SIZE>& input_queue);
    bool get_task(Task& task) override;

private:
    bool process_data(ResPointCloud data);
    //std::string generate_unique_filename(const char* base_path, const std::string& id);
    std::string generate_uuid();
    
    
    // 输入队列
    LockFreeQueue<ResPointCloud, QUEUE_SIZE>& input_queue_;
};



#endif // NGX_LOCKFREE_MIRRORICP_THREADPOOL_H