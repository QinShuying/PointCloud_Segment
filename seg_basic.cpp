#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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


    // PointCloud Segment
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;    //分割对象

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //模型类型_平面
    seg.setMethodType (pcl::SAC_RANSAC);		   //设置方法【聚类或随机样本一致性】
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud->makeShared ());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    cerr << "Model coefficients: " << coefficients->values[0] << " "
              <<coefficients->values[1] << " "
              <<coefficients->values[2] << " "
              <<coefficients->values[3] <<std::endl;    //模型系数
    cerr << "Model inliers: " << inliers->indices.size () << endl;    //估计平面模型过程中使用的内点


    // Extract Indices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    cerr << "After extracting the planar inliers, point cloud->size: " << cloud_filtered->size() << endl;


    // Save pcd
    pcl::PCDWriter writer;
    writer.write ("/home/qsy-5208/Documents/PointCloud_Segment/Results/basic"+seq+".pcd", *cloud_filtered, false);


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