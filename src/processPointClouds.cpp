// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "RANSAC.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and roi based filtering


    //Use pcl::VoxelGrid for downsampling and filtering the input point cloud
    pcl::VoxelGrid<PointT> voxelgrid;
    typename pcl::PointCloud<PointT>::Ptr filtered_pcd (new pcl::PointCloud<PointT>);

    voxelgrid.setInputCloud(cloud);
    voxelgrid.setLeafSize(filterRes, filterRes, filterRes);
    //Set the output pcl
    voxelgrid.filter(*filtered_pcd);
    

    typename pcl::PointCloud<PointT>::Ptr roi_point_cloud (new pcl::PointCloud<PointT>);

    //PCL CropBox is a filter that allows the user to filter all the data inside of a given box.
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(filtered_pcd);
    roi.filter(*roi_point_cloud);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(roi_point_cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point: indices) {
      inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(roi_point_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*roi_point_cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roi_point_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr outlier_cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr inlier_cloud (new pcl::PointCloud<PointT> ());

    //Create a separate point cloud from inliers
    for (int index : inliers->indices) 
    {
        inlier_cloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*outlier_cloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inlier_cloud, outlier_cloud);
    return segResult;
}


//Comment out for PCL based segmentation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    
    // TODO:: Fill in this function to find inliers for the cloud.


    int num_points = cloud->points.size();
    auto all_points = cloud->points;

    Ransac<PointT> segRansac(maxIterations, distanceThreshold, num_points);

    // get inliers from local-RANSAC implementation rather than PCL implementation
    std::unordered_set<int> inliersResult = segRansac.DoPlaneFit(cloud);
    std::cout << "Inliers: " << inliersResult.size() << std::endl;
    std::cout << "Total: " << cloud->size() << std::endl;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for(int ind: inliersResult) 
    {
        inliers->indices.push_back(ind);  
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "local-RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;

}


//disable comments for PCL based segmentation
// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = std::chrono::steady_clock::now();
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//     // function to find inliers for the cloud.
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

//     pcl::SACSegmentation<PointT> seg;
//     //
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setMaxIterations (maxIterations);
//     seg.setDistanceThreshold (distanceThreshold);

//     // Segment the largest planar component from the remaining cloud
//     seg.setInputCloud (cloud);
//     seg.segment (*inliers, *coefficients);

//     if (inliers->indices.size () == 0)
//     {
//       std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//     }
//     std::cout << "Inliers: " << inliers->indices.size() << std::endl;
//     std::cout << "Total: " << cloud->size() << std::endl;

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "PCL-RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
//     return segResult;
// }




// template<typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {

//     // Time clustering process
//     auto startTime = std::chrono::steady_clock::now();

//     std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//     // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

//     typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//     tree->setInputCloud(cloud);

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<PointT> ec;
//     ec.setClusterTolerance(clusterTolerance);
//     ec.setMinClusterSize(minSize);
//     ec.setMaxClusterSize(maxSize);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud);
//     ec.extract(cluster_indices);

//     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
//       typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
//       for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
//         cloud_cluster->points.push_back(cloud->points[*pit]);
//       }
//       cloud_cluster->width = cloud_cluster->points.size();
//       cloud_cluster->height = 1;
//       cloud_cluster->is_dense = true;
//       clusters.push_back(cloud_cluster);
//     }

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

//     return clusters;
// }

template<typename PointT>           
void ProcessPointClouds<PointT>::clusterHelper(int i, typename pcl::PointCloud<PointT>::Ptr cloud, KDTree* tree, float distanceTol, std::vector<bool> &processed, std::vector<int> &cluster)
{
    // from quiz
    processed[i] = true;
    cluster.push_back(i);
    std::vector<int> nearest = tree->search(cloud->points[i], distanceTol);
    for(int k : nearest)
    {
        if(!processed[k]) {
            clusterHelper(k, cloud, tree, distanceTol, processed, cluster);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    std::vector<std::vector<int>> clusters_idx;
    std::vector<bool> processed (cloud->points.size(), false);

    // from quiz
    KDTree *tree = new KDTree();
    for (int i=0; i<cloud->points.size(); ++i) {
        tree->insert(cloud->points[i], i);
    }

    for (int i=0; i < cloud->points.size(); ++i) {
        if (processed[i]) {
            continue;
        }
        std::vector<int> cluster_idx;
        clusterHelper(i, cloud, tree, clusterTolerance, processed, cluster_idx);
        clusters_idx.push_back(cluster_idx);
    }

    for (std::vector<int> idx : clusters_idx) {
        if(idx.size() < minSize || idx.size() > maxSize) {
            continue;
        }
        
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>());
        for (int ind : idx) {
            cloud_cluster->points.push_back(cloud->points[ind]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}