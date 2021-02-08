#ifndef POINTCLOUD_SEGMENT_PLANESEG_H
#define POINTCLOUD_SEGMENT_PLANESEG_H

#include "seg.h"


class PlaneSegment {

public:

    PlaneSegment(Segment segment, DownSample downsample);

    void ReadData(std::string &in_file);

    void VoxelDownSample(const PointCloudPtr& cloudIn,const PointCloudPtr& cloudOut, float voxel=0.01);

    void SegOnePlane(const PointCloudPtr& cloudIn, PointCloudPtr& cloudLeave, PointCloudPtr& cloudPlane);

    void SegmentPlanes();

    void EuclideanSegment();
private:
    PointCloudPtr _cloud;

    PointCloudPtr _cloudDownSample;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _colored_cloud;

    Segment _segment;

    DownSample _downsample;
};


#endif //POINTCLOUD_SEGMENT_PLANESEG_H
