#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/eigen.h>
#include <boost/make_shared.hpp>

#include "ngx_lockfree_threadPool.h"

ResultProcessingPool::ResultProcessingPool(size_t thread_count,
                   LockFreeQueue<MirrorICPPointCloud,QUEUE_SIZE>& input_queue,
                   LockFreeQueue<ResPointCloud,QUEUE_SIZE>& output_queue)
            : ThreadPool(thread_count, "ResultProcessing"),
              input_queue_(input_queue),
              output_queue_(output_queue) {
                start();
              } 

bool ResultProcessingPool::get_task(Task& task) {
     MirrorICPPointCloud raw_data;
    if (input_queue_.try_pop(raw_data)) {
        task = [this, raw_data] {
            ResPointCloud processed_data = process_data(raw_data);
            int attempts = 0;
            while (!output_queue_.try_push(std::move(processed_data))) {
                if (++attempts > 100) {
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

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::ArrayXd;

ResPointCloud ResultProcessingPool::process_data(MirrorICPPointCloud data){
    const char* pPkgBodySource = data.serializedData;
    auto draco_cloud_source = ThreadPool::decompressPointCloud(pPkgBodySource,data.dataLen); 
    const char* pPkgBodyMirrorICP = data.MirrorICPSerlzdData;
    auto draco_cloud_mirrorICP = ThreadPool::decompressPointCloud(pPkgBodyMirrorICP,data.MirrorICPLen); 
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_source = ThreadPool::DRCToPCD(*draco_cloud_source);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_mirrorICP = ThreadPool::DRCToPCD(*draco_cloud_mirrorICP);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr = pcl_cloud_source.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr mirror_ptr = pcl_cloud_mirrorICP.makeShared();
    double asymmetry = ResultProcessingPool::Asym(source_ptr, mirror_ptr);

    ResPointCloud res;
    memcpy(res.serlizdPCdata.serializedData, pPkgBodySource, data.dataLen);
    res.serlizdPCdata.dataLen = data.dataLen;
    res.asymmetry = asymmetry;
    memcpy(res.ID, data.ID, 7);
    ResToNetwork ans;
    ans.asymmetry = asymmetry;
    ans.fd = data.fd;
    int attempts = 0;
     while (!g_asymm_process_to_master_queue->try_push(std::move(ans))) {
        if (++attempts > 100) {
            ngx_log_stderr(0,"Failed to enqueue processed data after 100 attempts!");
        }
        std::this_thread::yield();
    }
    ngx_log_stderr(0,"socket fd = %d", ans.fd);
    return res;
}

// 计算点云法向量
pcl::PointCloud<pcl::Normal>::Ptr ResultProcessingPool::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int k_neighbors)
{
    if(cloud_in->empty()) return {};

    auto cloud_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(4);
    
    auto tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setKSearch(k_neighbors);
    ne.compute(*cloud_normals);
    
    return cloud_normals;
}

// 计算点到切平面的最小距离（矩阵优化版本）
double ResultProcessingPool::computeMinDistanceToPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,const pcl::PointCloud<pcl::Normal>::Ptr& normals,const pcl::PointXYZ& query_point)
{
    if(points->size() != normals->size() || points->empty()) {
        return std::numeric_limits<double>::max();
    }
    
    // 转换为Eigen格式
    Vector3d q(query_point.x, query_point.y, query_point.z);
    
    const size_t n = points->size();
    MatrixXd P(n, 3);  // 点云坐标矩阵
    MatrixXd N(n, 3);  // 法向量矩阵
    VectorXd D(n);     // 平面常数项
    
    // 填充矩阵
    for(size_t i = 0; i < n; ++i) {
        P(i, 0) = points->points[i].x;
        P(i, 1) = points->points[i].y;
        P(i, 2) = points->points[i].z;
        
        N(i, 0) = normals->points[i].normal_x;
        N(i, 1) = normals->points[i].normal_y;
        N(i, 2) = normals->points[i].normal_z;
        
        // D = -n·p
        D(i) = -N.row(i).dot(P.row(i));
    }
    
    // 计算距离: |n·q + D| / ||n||
    VectorXd distances = (N * q + D).array().abs();
    
    // 归一化处理（因为法向量可能不是单位长度）
    VectorXd norm_lengths = N.rowwise().norm();
    distances = distances.cwiseQuotient(norm_lengths);
    
    return distances.minCoeff();
}

// 计算不对称性度量（矩阵优化版本）
double ResultProcessingPool::Asym(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource, pcl::PointCloud<pcl::PointXYZ>::Ptr transformedTarget)
{
    // 预计算全局法向量
    auto globalNormals = computeNormals(transformedTarget, 50);
    if(!globalNormals || globalNormals->size() != transformedTarget->size()) {
        ngx_log_stderr(0,"Normals calculation failed!");
        return std::numeric_limits<double>::max();
    }

    // 创建全局KD树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    kdtree->setInputCloud(transformedTarget);

    const size_t total_points = cloudSource->size();
    const size_t batch_size = 1000;  // 批处理大小
    const size_t num_batches = (total_points + batch_size - 1) / batch_size;
    
    double sum = 0.0;
    std::vector<int> point_indices(50); // 每个点的K近邻索引
    std::vector<float> point_distances(50); // 每个点的距离
    
    ngx_log_stderr(0,"Starting asymmetry calculation with%dpoints",total_points);

    // 批处理
    for (size_t batch = 0; batch < num_batches; ++batch) {
        const size_t start_idx = batch * batch_size;
        const size_t end_idx = std::min((batch + 1) * batch_size, total_points);
        const size_t current_batch_size = end_idx - start_idx;
        
        // 处理当前批次
        for (size_t i = start_idx; i < end_idx; ++i) {
            const auto& query_point = cloudSource->points[i];
            
            // 搜索K近邻
            kdtree->nearestKSearch(query_point, 50, point_indices, point_distances);
            
            // 创建局部点云和法向量
            auto localPoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            auto localNormals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
            
            localPoints->reserve(50);
            localNormals->reserve(50);
            
            for (int idx : point_indices) {
                localPoints->push_back(transformedTarget->points[idx]);
                localNormals->push_back(globalNormals->points[idx]);
            }
            
            // 计算最小距离
            double min_distance = computeMinDistanceToPlanes(localPoints, localNormals, query_point);
            sum += min_distance;
        }
    }
    
    double averageMinDistance = sum / total_points;
    ngx_log_stderr(0,"Asymmetry calculation complete. Average minimum distance:%f",averageMinDistance);
    return averageMinDistance;
}
