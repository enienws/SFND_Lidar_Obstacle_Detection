/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
// {

//     Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
//     Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
//     Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
//     Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
//     std::vector<Car> cars;
//     cars.push_back(egoCar);
//     cars.push_back(car1);
//     cars.push_back(car2);
//     cars.push_back(car3);

//     if(renderScene)
//     {
//         renderHighway(viewer);
//         egoCar.render(viewer);
//         car1.render(viewer);
//         car2.render(viewer);
//         car3.render(viewer);
//     }

//     return cars;
// }


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    
//     // RENDER OPTIONS
//     bool renderScene = true;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
    
//     // TODO:: Create lidar sensor 

//     // TODO:: Create point processor
  
// }

/*****************************************************************/
/************ use cityBlock to test on real Lidar data *******/
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // hyperparameters
    // filter params
    float filterRes = 0.3;
    Eigen::Vector4f minPoint(-10, -6.5, -2, 1);
    Eigen::Vector4f maxPoint(30, 6.5, 1, 1);
    // segment params
    int maxIter = 100;
    float distanceThreshold = 0.2;
    // cluster params
    float clusterTolerance = 0.35;
    int minClusterSize = 10;
    int maxClusterSize = 300;

    // Filter cloud, to reduce omputational cost
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    //renderPointCloud(viewer, filteredCloud, "filtered cloud", Color(0,1,0));

    // Step 1. Segment the filtered cloud into two parts, road and obstacles.
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIter, distanceThreshold);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, maxIter, distanceThreshold);

    renderPointCloud(viewer,segmentCloud.first,"planeCloud",Color(1,1,0));
    renderPointCloud(viewer,segmentCloud.second,"obstacleCloud",Color(1,0,0));

    // Step 2. Cluster the obstacle cloud.
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.second, clusterTolerance, minClusterSize, maxClusterSize);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

      for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
      {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

            // Step 3. Find bounding boxes for the clusters
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);

            ++clusterId;
      }

}
/*****************************************************************/

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment..." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("./src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        // stream
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    }
}