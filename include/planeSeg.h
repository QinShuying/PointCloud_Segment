#ifndef POINTCLOUD_SEGMENT_PLANESEG_H
#define POINTCLOUD_SEGMENT_PLANESEG_H

#include "seg.h"


class PlaneSegment {

public:

    PlaneSegment(Segment segment, DownSample downsample, bool visualize = false);

    void ReadData(std::string &in_file);

    void VoxelDownSample();

    void SegOnePlane(const PointCloudPtr &cloudIn, PointCloudPtr &cloudLeave, PointCloudPtr &cloudPlane);

    void SegOnePlaneWithNormal(const PointCloudPtr &cloudIn, const pcl::PointCloud<pcl::Normal>::Ptr nomalIn,
                               PointCloudPtr &cloudLeave, pcl::PointCloud<pcl::Normal>::Ptr nomalLeave,
                               PointCloudPtr &cloudPlane, pcl::PointCloud<pcl::Normal>::Ptr nomalPlane);

    void SegmentPlanes();

    void EuclideanSegment();

    void Visualize();

    void RegionGrowSeg();

    void EuclideanNormalSeg();

private:
    PointCloudPtr _cloud;

    PointCloudPtr _cloudDownSample;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _colored_cloud;

    Segment _segment;

    DownSample _downsample;

    bool _bVisual;
};


#endif //POINTCLOUD_SEGMENT_PLANESEG_H
