/*
 * 基于 区域生成 的分割法
 * 从一个点出发，最终占领整个被分割区域
 * 可由法线、曲率估计算法获得其法线和曲率值。通过法线和曲率来判断某点是否属于该类
 * https://www.cnblogs.com/li-yao7758258/p/6697034.html
 * */


#include "include/seg.h"
#include "include/planeSeg.h"
#include "global_defination.h"
#include "boost/thread.hpp"
#include "include/seg.h"
#include "include/planeSeg.h"
#include "global_defination.h"


int main(int argc, char **argv) {

//    string filename = WORK_SPACE_PATH + "/data/local_pcs_158.pcd";
    string filename = WORK_SPACE_PATH + "/data/local_pcs_44.pcd";

    DownSample downsample = {type:Random, voxelSize:0.01f, randomRate:0.1f};

    Segment segment = {type:RegionGrow, maxIters:100, disTh:0.03f, smoTh:3.f / 180.f *
                                                                         M_PI, curTh:1.f, normalK:20, regionK:30};

    // Load data points
    PlaneSegment plane_segment(segment, downsample, true);
    plane_segment.ReadData(filename);
    plane_segment.SegmentPlanes();


    return 0;
}

