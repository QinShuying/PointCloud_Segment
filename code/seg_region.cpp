/*
 * 基于 区域生成 的分割法
 * 从一个点出发，最终占领整个被分割区域
 * 可由法线、曲率估计算法获得其法线和曲率值。通过法线和曲率来判断某点是否属于该类
 * https://www.cnblogs.com/li-yao7758258/p/6697034.html
 * */


#include "include/seg.h"
#include "include/datapretreat.h"
#include "global_defination.h"
#include "boost/thread.hpp"


int main (int argc, char** argv)
{


    // Load data points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string seq = "02";
    string filename = WORK_SPACE_PATH+"/global_pcs/secen_pcd"+seq+".pcd";
    datapretreat d;
    d.ReadData(filename, cloud);


    clock_t startTime,endTime;
    startTime = clock();
    //设置搜索结构
    pcl::search::Search<pcl::PointXYZ>::Ptr tree =  shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    //计算点法线
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (20);       //法向量估计的k近邻数目
    normal_estimator.compute (*normals);

    //聚类对象<点，法线>
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);         //最小聚类点数
    reg.setMaxClusterSize (1000000);    //最大
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);     //邻域点个数，决定这是一个平面(决定容错率，设置大时有倾斜也可接受，设置小时检测到的平面会很小）
    reg.setInputCloud (cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);    //平滑阈值_两法线偏差允许范围，若小于该阈值则认为两点属于同一聚类
    reg.setCurvatureThreshold (1.0);                    //曲率阀值_两点曲率偏差允许范围，若满足前一条件并小于该阈值则采用新增加点迭代进行区域生长

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    endTime = clock();
    cout << "The run time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    // Save pcd
    pcl::PCDWriter writer;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    writer.write (WORK_SPACE_PATH+"/result/region"+seq+".pcd", *colored_cloud, false);

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

