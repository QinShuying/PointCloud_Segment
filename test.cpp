#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

int main()
{
        // creating a point cloud with random points just as an example.
        // Doesn't work for device captured point clouds too.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->height = 10;
	cloud->width = 20;
	cloud->points.resize(200);

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;

	normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
	normal_estimation.setDepthDependentSmoothing(true);
	normal_estimation.setMaxDepthChangeFactor(0.02);
	normal_estimation.setNormalSmoothingSize(10);

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setInputCloud(cloud);
	normal_estimation.compute(*normal_cloud);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> multi_plane_segmentation;
	multi_plane_segmentation.setMinInliers(0.001);
	multi_plane_segmentation.setAngularThreshold(4.00);
	multi_plane_segmentation.setDistanceThreshold(0.05);
	multi_plane_segmentation.setInputNormals(normal_cloud);
	multi_plane_segmentation.setInputCloud(cloud);

        /* crash causing line*/
	std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ>>> regions;

	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;

	multi_plane_segmentation.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

	return 0;
}