#include "include/seg.h"
#include "include/planeSeg.h"
#include "global_defination.h"


int main(int argc, char **argv) {

    string filename = WORK_SPACE_PATH + "/data/local_pcs_158.pcd";
//    string filename = WORK_SPACE_PATH + "/data/local_pcs_44.pcd";

    DownSample downsample = {type:Random, voxelSize:0.01f, randomRate:0.1};

    Segment segment = {type:Euclidean, maxIters:100, disTh:0.03f};

    // Load data points
    PlaneSegment plane_segment(segment, downsample, true);
    plane_segment.ReadData(filename);
    plane_segment.SegmentPlanes();


    return 0;
}
