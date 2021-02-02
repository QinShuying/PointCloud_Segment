/*
 * 基于 区域生成 的分割法
 * 从一个点出发，最终占领整个被分割区域
 * 可由法线、曲率估计算法获得其法线和曲率值。通过法线和曲率来判断某点是否属于该类
 * */


#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

using namespace std;


void ReadData(std::string &in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


int main (int argc, char** argv)
{
    clock_t startTime,endTime;
    startTime = clock();


    // Load data points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string seq = "01";
    string filename = "/home/qsy-5208/Documents/PointCloud_Segment/Data/global_pcs/secen_pcd"+seq+".pcd";
    ReadData(filename, cloud);


    // Region Segment
    //设置搜索结构
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    //法线
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    //直通滤波在Z轴的0到1米之间
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    //聚类对象<点，法线>
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);  //最小的聚类的点数
    reg.setMaxClusterSize (1000000);  //最大的
    reg.setSearchMethod (tree);    //搜索方式
    reg.setNumberOfNeighbours (30);    //设置搜索的邻域点的个数
    reg.setInputCloud (cloud);         //输入点
    reg.setInputNormals (normals);     //输入的法线
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);  //设置平滑度
    reg.setCurvatureThreshold (1.0);     //设置曲率的阀值

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

//    cerr << "Region_based_seg, Cluster Number： " << clusters.size () << endl;
//    cerr << "First cluster has " << clusters[0].indices.size () << " points." << endl;


    // Save pcd
    pcl::PCDWriter writer;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    writer.write ("/home/qsy-5208/Documents/PointCloud_Segment/Results/region"+seq+".pcd", *colored_cloud, false);


    endTime = clock();
    cout << "The run time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}


void ReadData(std::string &in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

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