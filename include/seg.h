#ifndef POINTCLOUD_SEGMENT_SEG_H
#define POINTCLOUD_SEGMENT_SEG_H

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Euclidean
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

//Region
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


enum SegmentType {
    Euclidean,
    RegionGrow,
    Euclidean_Normal
};

enum DownSampleType {
    Random,
    Voxel,
    NoneDS
};

typedef struct {
    DownSampleType type;
    float voxelSize;
    float randomRate;
} DownSample;

typedef struct {
    SegmentType type;
    int maxIters;
    float disTh;
    float smoTh;
    float curTh;
    float normalK;
    float regionK;
    float normalWeight;
} Segment;

typedef pcl::PointXYZ PointType;

typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

using namespace std;


#endif //POINTCLOUD_SEGMENT_SEG_H
