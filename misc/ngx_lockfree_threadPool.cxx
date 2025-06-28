#include "ngx_lockfree_threadPool.h"

std::unique_ptr<draco::PointCloud> ThreadPool::decompressPointCloud(const char *pPkgBody,uint32_t iBodyLength){
    draco::DecoderBuffer buffer;
    buffer.Init(pPkgBody,iBodyLength);
    draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
        if (!statusor.ok()) {
            ngx_log_stderr(0,"解压失败.");
            return nullptr;
        }

        ngx_log_stderr(0,"解压成功.");
        return std::move(statusor).value();
}
pcl::PointCloud<pcl::PointXYZ> ThreadPool::DRCToPCD(const draco::PointCloud& draco_cloud) {
    // 创建PCL点云对象
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.width = draco_cloud.num_points();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
    pcl_cloud.points.resize(draco_cloud.num_points());

    // 获取位置属性
    const draco::PointAttribute* pos_attr = draco_cloud.GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (!pos_attr) {
        ngx_log_stderr(0,"无法获取属性");
        return pcl_cloud;
    }

    // 转换数据到PCL格式
    for (uint32_t i = 0; i < draco_cloud.num_points(); ++i) {
        draco::PointIndex point_index(i);  // 创建PointIndex对象
        float pos[3];
        pos_attr->GetMappedValue(point_index, pos);
        pcl_cloud.points[i].x = pos[0];
        pcl_cloud.points[i].y = pos[1];
        pcl_cloud.points[i].z = pos[2];
    }
    return pcl_cloud;
}
std::unique_ptr<draco::PointCloud> ThreadPool::PCDToDRC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) {
   auto draco_cloud = std::unique_ptr<draco::PointCloud>(new draco::PointCloud());
    draco_cloud->set_num_points(static_cast<uint32_t>(pcl_cloud.size()));

    // 添加位置属性
    draco::GeometryAttribute pos_att;
    pos_att.Init(draco::GeometryAttribute::POSITION, nullptr, 3,
        draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
    int pos_att_id = draco_cloud->AddAttribute(pos_att, true, pcl_cloud.size());

    // 填充点数据
    for (size_t i = 0; i < pcl_cloud.size(); ++i) {
        const auto& pt = pcl_cloud.points[i];
        const float pos[3] = { pt.x, pt.y, pt.z };
        draco_cloud->attribute(pos_att_id)->SetAttributeValue(
            draco::AttributeValueIndex(static_cast<uint32_t>(i)), pos);
    }

    return draco_cloud;
}
bool ThreadPool::CompressPCDToDraco(int encoder_speed, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, draco::EncoderBuffer& buffer){
    //.pcd转换为.drc
    auto draco_cloud = PCDToDRC(pcl_cloud);
    if (!draco_cloud) {
        ngx_log_stderr(0,"PCD To DRC失败!");
        return false;
    }
    //压缩点云
    draco::Encoder encoder;
    encoder.SetSpeedOptions(encoder_speed, encoder_speed);
    auto status = encoder.EncodePointCloudToBuffer(*draco_cloud, &buffer);
    if (!status.ok()) {
        ngx_log_stderr(0,"压缩失败!");
        return false;
    }
    return true;
}