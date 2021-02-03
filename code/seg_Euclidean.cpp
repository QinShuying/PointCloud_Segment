/*
 * 基于 欧式距离 的分割
 * 使用邻居之间距离作为判定标准
 * */


#include "include/seg.h"
#include "include/datapretreat.h"



int main (int argc, char** argv)
{
    clock_t startTime,endTime;
    startTime = clock();


    // Load data points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string seq = "01";
    string filename = "/home/qsy-5208/Documents/PointCloud_Segment/global_pcs/secen_pcd"+seq+".pcd";
    datapretreat d;
    d.ReadData(filename, cloud);


    // 滤波重采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    cerr << "After filtering, point cloud->size: " << cloud_filtered->points.size () << endl;

    // Euclidean Segment
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);        //设置阀值

    pcl::PointCloud<pcl::PointXYZ>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZ>);     //最终结果
    int i=0;
    int nr_points = (int) cloud_filtered->points.size ();   //剩余点云数量
    // 每一次循环得到一个平面
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

        // 提取平面模型内点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
//        cerr << "After extracting the planar inliers, cloud_plane->size: " << cloud_plane->size() << endl;
        *colored_cloud += *cloud_plane;

        // 可视化每一个平面
//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green
//        viewer->addPointCloud<pcl::PointXYZ>(colored_cloud, single_color, "sample cloud");
//        while (!viewer->wasStopped())
//        {
//            viewer->spinOnce(100);
//            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//        }

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
    ec.setClusterTolerance (0.02);      //近邻搜索半径
    ec.setMinClusterSize (100);         //最小聚类点数
    ec.setMaxClusterSize (25000);       //最大
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

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

        pcl::PCDWriter writer;
        stringstream ss;
        ss << "/home/qsy-5208/Documents/PointCloud_Segment/result/Euclidean/"+seq+"/Euclidean_cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

        j++;
        *colored_cloud += *cloud_cluster;
    }


    // Save pcd
    pcl::PCDWriter writer;
    writer.write ("/home/qsy-5208/Documents/PointCloud_Segment/result/Euclidean"+seq+".pcd", *colored_cloud, false);

    // Visualize
    pcl::visualization::CloudViewer viewer("seg_Euclidean");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped()) {}


    endTime = clock();
    cout << "The run time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}
