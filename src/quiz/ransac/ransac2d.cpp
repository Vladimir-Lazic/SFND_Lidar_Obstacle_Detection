/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() < 2)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		float x1, y1, x2, y2;
		auto iter = inliers.begin();

		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;

		iter++;

		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;

		float a, b, c;

		a = y1 - y2;
		b = x2 - x1;
		c = x1 * y2 - x2 * y1;

		for (int i; i < cloud->points.size(); i++)
		{
			if (inliers.count(i) > 0)
			{
				continue;
			}

			float point_x = cloud->points[i].x;
			float point_y = cloud->points[i].y;

			float distance = fabs(a * point_x + b * point_y + c) / sqrt(pow(a, 2) + pow(b, 2));

			if (distance <= distanceTol)
			{
				inliers.insert(i);
			}
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	return inliersResult;
}

static pcl::PointXYZ cross_product(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
	pcl::PointXYZ result;

	float a1, b1, c1;
	float a2, b2, c2;

	a1 = v1.x;
	b1 = v1.y;
	c1 = v1.z;

	a2 = v2.x;
	b2 = v2.y;
	c2 = v2.z;

	float i = b1 * c2 - c1 * b2;
	float j = c1 * a2 - a1 * c2;
	float k = a1 * b2 - b1 * a2;

	result.x = i;
	result.y = j;
	result.z = k;

	return result;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto iter = inliers.begin();

		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;

		iter++;

		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;

		iter++;

		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		float a, b, c, d;

		a = i;
		b = j;
		c = k;

		d = -1 * (i * x1 + j * y1 + k * z1);

		for (int i; i < cloud->points.size(); i++)
		{

			if (inliers.count(i) > 0)
			{
				continue;
			}

			float point_x = cloud->points[i].x;
			float point_y = cloud->points[i].y;
			float point_z = cloud->points[i].z;

			float dist = fabs(a * point_x + b * point_y + c * point_z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

			if (dist <= distanceTol)
			{
				inliers.insert(i);
			}
		}

		if (inliers.size() > inliersResult.size())
		{
			// std::cout << "Found better plane" << std::endl;
			inliersResult = inliers;
		}
	}

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 150, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::cout << "cloudInliers: " << cloudInliers->size() << std::endl;
	std::cout << "cloudOutliers: " << cloudOutliers->size() << std::endl;

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
