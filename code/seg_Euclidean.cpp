/*
 * 基于 欧式距离 的分割
 * 使用邻居之间距离作为判定标准
 * https://www.cnblogs.com/li-yao7758258/p/6694873.html
 * */


#include "include/seg.h"
#include "include/datapretreat.h"
#include "boost/thread.hpp"
#include "global_defination.h"


int main (int argc, char** argv)
{
    clock_t startTime,endTime;
    startTime = clock();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);     //最终结果


    // Load data points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string seq = "02";
//    string filename = "/home/qsy-5208/Documents/PointCloud_Segment/global_pcs/secen_pcd"+seq+".pcd";
    string filename = WORK_SPACE_PATH+"/global_pcs/secen_pcd"+seq+".pcd";
    datapretreat d;
    d.ReadData(filename, cloud);


    // 滤波重采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    cout << "After filtering, point cloud->size: " << cloud_filtered->points.size () << endl;

    // Euclidean Segment
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (50);
//    查询点到目标模型的距离阈值（如果大于此阈值，则查询点不在目标模型上，默认值为0）
    seg.setDistanceThreshold (0.03);        //阀值

    // 每一次循环提取一个平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_plane = *cloud_filtered;
    int nr_points = (int) cloud_filtered->points.size ();   //降采样后剩余点云数量
    int r, g, b;
//    while (cloud_filtered->points.size () > 0.3 * nr_points)    //原：每次去掉一个平面后的剩余点数>阈值，问题：可能存在很多杂点却依然在循环
    while (cloud_plane->size() > 0.03 * nr_points)  //现：提取的当前平面点数>阈值
    {
        // 从剩余点云中再分割出当前最大平面分量
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }

        // 提取平面模型内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
//        cout << "cloud_plane->size: " << cloud_plane->size() << endl;

        //复制当前平面至最终结果
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
        for (int k=0; k<cloud_plane->size(); k++) {
            pcl::PointXYZRGB point;
            point.x = cloud_plane->points[k].x;
            point.y = cloud_plane->points[k].y;
            point.z = cloud_plane->points[k].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colored_cloud->points.push_back(point);
        }

        // 过滤平面点，得到剩余点云
        extract.setNegative (true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }


    // 创建用于提取搜索方法的kdtree树对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (1);            //近邻搜索半径
    ec.setMinClusterSize (100);         //最小聚类点数
    ec.setMaxClusterSize (25000);       //最大
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //迭代访问点云索引cluster_indices,直到分割处所有聚类
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    { //迭代容器中的点云的索引，并且分开保存索引的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cerr << "cloud_cluster->points.size:" << cloud_cluster->points.size() << endl;

        //复制当前聚类至最终结果
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
        for (int k=0; k<cloud_cluster->size(); k++) {
            pcl::PointXYZRGB point;
            point.x = cloud_cluster->points[k].x;
            point.y = cloud_cluster->points[k].y;
            point.z = cloud_cluster->points[k].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colored_cloud->points.push_back(point);
        }

        j++;
    }

    endTime = clock();
    cout << "The run time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    // Save pcd
    pcl::PCDWriter writer;
    colored_cloud->height = 1;
    colored_cloud->width = colored_cloud->size();
    char buffer[20];
    gcvt(DistanceThreshold, 3, buffer);
    writer.write ("/home/qsy-5208/Documents/PointCloud_Segment/result/Euclidean"+seq+"_Dis"+buffer+".pcd", *colored_cloud, false);

    // Visualize
    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(colored_cloud, "sample cloud");
    viewer->addCoordinateSystem(0.2);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }


    return 0;
}
