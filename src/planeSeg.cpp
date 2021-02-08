#include "planeSeg.h"
#include <pcl/filters/random_sample.h>

PlaneSegment::PlaneSegment(Segment segment, DownSample downsample)
{
    _cloud.reset(new pcl::PointCloud<PointType>());
    _cloud->clear();

    _cloudDownSample.reset(new pcl::PointCloud<PointType>());
    _cloudDownSample->clear();

    _colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    _colored_cloud->clear();

    _segment = segment;
    _downsample = downsample;
}


void PlaneSegment::ReadData(std::string &in_file) {

    if (pcl::io::loadPCDFile<PointType> (in_file, *_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

}


void PlaneSegment::VoxelDownSample(const PointCloudPtr& cloudIn, const PointCloudPtr& cloudOut, float voxel)
{
    if(_downsample.type==Voxel)
    {
        pcl::VoxelGrid<PointType> vg;
        cout << "before filtering, point cloud->size: " << _cloud->points.size () << endl;
        vg.setInputCloud (_cloud);
        vg.setLeafSize (_downsample.voxelSize, _downsample.voxelSize, _downsample.voxelSize);
        vg.filter (*_cloudDownSample);
        cout << "After filtering, point cloud->size: " << _cloudDownSample->points.size () << endl;
    }else if(_downsample.type==Random)
    {
        pcl::RandomSample<PointType> rs;
        cout << "before filtering, point cloud->size: " << _cloud->points.size () << endl;
        rs.setInputCloud (_cloud);
        rs.setSample(_downsample.randomNum);
        rs.filter (*_cloudDownSample);
        cout << "After filtering, point cloud->size: " << _cloudDownSample->points.size () << endl;
    }
}

void PlaneSegment::SegOnePlane(const PointCloudPtr& cloudIn, PointCloudPtr& cloudLeave, PointCloudPtr& cloudPlane)
{
    // Euclidean Segment
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (_segment.maxIters);
//    查询点到目标模型的距离阈值（如果大于此阈值，则查询点不在目标模型上，默认值为0）
    seg.setDistanceThreshold (_segment.threshold);        //阀值
    // 从剩余点云中再分割出当前最大平面分量
    seg.setInputCloud (cloudIn);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        cerr << "Could not estimate a planar model for the given dataset." << endl;
        return;
    }

    // 提取平面模型内点
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (cloudIn);
    extract.setIndices (inliers);
    extract.setNegative (false);
    cloudPlane->clear();
    extract.filter (*cloudPlane);

    // 过滤平面点，得到剩余点云
    extract.setNegative (true);
    cloudLeave->clear();
    extract.filter (*cloudLeave);

}

void PlaneSegment::EuclideanSegment()
{

}

void PlaneSegment::SegmentPlanes()
{
    if(_segment.type==Euclidean))
    {
        EuclideanSegment();
    }
}