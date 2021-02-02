/*
 * 基于 欧式距离 的分割
 * 使用邻居之间距离作为判定标准
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

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;


void ReadData(std::string &in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


int main (int argc, char** argv)
{
    clock_t startTime,endTime;
    startTime = clock();


    // Load data points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string seq = "02";
    string filename = "./global_pcs/secen_pcd"+seq+".pcd";
    ReadData(filename, cloud);


    // 滤波重采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    cout << "After filtering, point cloud->size: " << cloud_filtered->points.size () << endl;

    // Euclidean Segment
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();//剩余点云的数量
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // 从剩余点云中再分割出最大的平面分量 （因为我们要处理的点云的数据是两个平面的存在的）
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) //如果内点的数量已经等于0，就说明没有
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }

        // Extract Indices
        // 从输入的点云中提取平面模型的内点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
//        cerr << "After extracting the planar inliers, point cloud->size: " << cloud_plane->size() << endl;

        // 移去平面局内点，提取剩余点云
        extract.setNegative (true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // 创建用于提取搜索方法的kdtree树对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
    ec.setClusterTolerance (0.02);                     // 设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);                    //设置点云的搜索机制
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
    //迭代访问点云索引cluster_indices,直到分割处所有聚类
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    { //迭代容器中的点云的索引，并且分开保存索引的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            //设置保存点云的属性问题
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        pcl::PCDWriter writer;
        stringstream ss;
        ss << "./result/Euclidean/Euclidean_cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
        j++;
    }


    // Save pcd
    pcl::PCDWriter writer;
    writer.write ("./result/Euclidean"+seq+".pcd", *cloud_filtered, false);


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