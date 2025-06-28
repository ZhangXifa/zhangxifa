#include "ngx_lockfree_threadPool.h"
#include <pcl/registration/icp.h>          // 用于 IterativeClosestPoint
#include <pcl/common/transforms.h>        // 用于 transformPointCloud

MirrorICPProcessingPool::MirrorICPProcessingPool(size_t thread_count,
                 LockFreeQueue<PointCloud, QUEUE_SIZE>& input_queue,
                 LockFreeQueue<MirrorICPPointCloud, QUEUE_SIZE>& output_queue)
    : ThreadPool(thread_count, "MirrorICPProcessing"), 
      input_queue_(input_queue),
      output_queue_(output_queue) {
        start();
      }

bool MirrorICPProcessingPool::get_task(Task& task) {
    PointCloud raw_data;
    if (input_queue_.try_pop(raw_data)) {
        task = [this, raw_data] {
            if (stop_.load()) return;
            MirrorICPPointCloud processed_data = process_data(raw_data);
            int attempts = 0;
            while (!output_queue_.try_push(std::move(processed_data))) {
                if (++attempts > 100 || stop_.load()) {
                    ngx_log_stderr(0,"Failed to enqueue processed data after 100 attempts!");
                    return;
                }
                std::this_thread::yield();
            }   
            ngx_log_stderr(0,"线程id:%P",std::this_thread::get_id() );
        };
        return true;
    }
    return false;
}

MirrorICPPointCloud MirrorICPProcessingPool::process_data(PointCloud data) {
    const char* pPkgBody = data.serializedData;
    auto draco_cloud = ThreadPool::decompressPointCloud(pPkgBody,data.dataLen);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud = ThreadPool::DRCToPCD(*draco_cloud);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_mirror = MirrorICPProcessingPool::pointCloudMirror(pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_ICP = MirrorICPProcessingPool::ICPTransform(pcl_cloud_mirror, pcl_cloud);
    draco::EncoderBuffer bufferMirrorIcp;
    if(ThreadPool::CompressPCDToDraco(5, pcl_cloud_ICP, bufferMirrorIcp)){
        ngx_log_stderr(0,"镜像ICP点云压缩成功!");
    }else{
        ngx_log_stderr(0,"镜像ICP点云压缩失败!");
    }
    MirrorICPPointCloud serializedPointCloud;
    memcpy(serializedPointCloud.serializedData, pPkgBody, data.dataLen);
    serializedPointCloud.dataLen = data.dataLen;
    serializedPointCloud.fd = data.fd;
    memcpy(serializedPointCloud.MirrorICPSerlzdData, bufferMirrorIcp.data(), bufferMirrorIcp.size());
    serializedPointCloud.MirrorICPLen = bufferMirrorIcp.size();
    return serializedPointCloud;
}
pcl::PointCloud<pcl::PointXYZ> MirrorICPProcessingPool::pointCloudMirror(const pcl::PointCloud<pcl::PointXYZ>& pointCloudin){
    pcl::PointCloud<pcl::PointXYZ> pointCloudout = pointCloudin;
     for (auto& point : pointCloudout.points) {
		point.x = -point.x;  // 镜像变换：取反 X 坐标
	}
	return pointCloudout;
}
pcl::PointCloud<pcl::PointXYZ> MirrorICPProcessingPool::ICPTransform(
    const pcl::PointCloud<pcl::PointXYZ>& pointCloudsource,
    const pcl::PointCloud<pcl::PointXYZ>& pointCloudtarget) {
    
    // 创建ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    // 将输入点云转换为shared_ptr
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_ptr = pointCloudsource.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_ptr = pointCloudtarget.makeShared();
    
    // 设置输入点云
    icp.setInputSource(source_ptr);
    icp.setInputTarget(target_ptr);
    
    // 执行配准
    pcl::PointCloud<pcl::PointXYZ> transformedTarget;
    icp.align(transformedTarget);
    
    // 获取变换矩阵
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    
    // 应用变换
    pcl::PointCloud<pcl::PointXYZ> finalResult;
    pcl::transformPointCloud(pointCloudsource, finalResult, transformation);
    
    return finalResult;
}
