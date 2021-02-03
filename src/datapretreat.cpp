#include "/home/qsy-5208/Documents/PointCloud_Segment/Code/include/datapretreat.h"


void datapretreat::ReadData(std::string &in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (in_file, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from test_file.pcd with the following fields: "
         << endl;

    /*
    //for (size_t i = 0; i < cloud->points.size (); ++i) //显示所有的点
    for (size_t i = 0; i < 5; ++i)
        cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << endl;
    */
}
