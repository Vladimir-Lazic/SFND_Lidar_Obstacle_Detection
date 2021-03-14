/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->scan();
    // renderRays(viewer, lidar->position, input_cloud);
    renderPointCloud(viewer, input_cloud, "input_cloud");

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlaneUsingPCL(input_cloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->ClusteringUsingPCL(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId]);
        ++clusterId;
    }
}

void process_pcd_data_using_pcl(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, std::string path_to_pcd_file)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(path_to_pcd_file);
    // renderPointCloud(viewer, inputCloud, "inputCloud");
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.5f, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(20, 6, 3, 1));
    renderPointCloud(viewer, filterCloud, "filterCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneUsingPCL(filterCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringUsingPCL(segmentCloud.first, 0.7, 10, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 0, 1), Color(0, 1, 1)};

    for (const auto &cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1, 0, 0));
        ++clusterId;
    }
}

void process_pcd_data(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, std::string path_to_pcd_file)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(path_to_pcd_file);
    // renderPointCloud(viewer, inputCloud, "inputCloud");
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.5f, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(20, 6, 3, 1));
    renderPointCloud(viewer, filterCloud, "filterCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<std::vector<float>> points;
    for (pcl::PointXYZI point : segmentCloud.first->points)
    {
        points.push_back({point.x, point.y, point.z});
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclidianClustering(segmentCloud.first, points, 0.9, 100, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 0, 1), Color(0, 1, 1)};

    for (const auto &cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1, 0, 0));
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    std::string path_to_pcd_dir = "../src/sensors/data/pcd/data_1";
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(path_to_pcd_dir);
    auto stream_iterator = stream.begin();

    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        std::string path_to_file = stream_iterator->string();
        process_pcd_data(viewer, pointProcessorI, path_to_file);
        stream_iterator++;
        if (stream_iterator == stream.end())
        {
            stream_iterator = stream.begin();
        }
        // Sleep for 10ms for debugging purposes
        // std::this_thread::sleep_for(10ms);
        viewer->spinOnce();
    }
}