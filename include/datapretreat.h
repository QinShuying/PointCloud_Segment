#ifndef POINTCLOUD_SEGMENT_DATAPRETREAT_H
#define POINTCLOUD_SEGMENT_DATAPRETREAT_H

#include "seg.h"


class datapretreat {

public:
    void ReadData(std::string &in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


};


#endif //POINTCLOUD_SEGMENT_DATAPRETREAT_H
