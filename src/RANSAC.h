
#include <unordered_set>

#include <pcl/common/common.h>

template<typename PointT>
class Ransac {
private:
  int maxIterations;
  float distanceTol;
  int num_points;

public:
  Ransac(int maxIter, float distTol, int nPts);
  ~Ransac();
  std::unordered_set<int> DoPlaneFit(typename pcl::PointCloud<PointT>::Ptr cloud);
};

template<typename PointT>
Ransac<PointT>::Ransac(int maxIter, float distTol, int nPts)
{
    //Constructor
    this->maxIterations = maxIter;
    this->distanceTol = distTol;
    this->num_points = nPts;
}

template<typename PointT>
Ransac<PointT>::~Ransac() 
{
    //Destructor
}


template<typename PointT>
std::unordered_set<int> Ransac<PointT>::DoPlaneFit(typename pcl::PointCloud<PointT>::Ptr cloud)
{
      std::unordered_set<int> iinliers;
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    int numcloud = cloud->points.size();
    for(int i=0; i<maxIterations; i++){
        iinliers.clear();
        for (int j=0; j < 3; j++) {
            iinliers.insert(rand() % (numcloud));
        }

        // sample 3 points
        PointT p1=cloud->points[(rand() % numcloud)];
        PointT p2=cloud->points[(rand() % numcloud)];
        PointT p3=cloud->points[(rand() % numcloud)];

        // fit plane
        float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
        float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
        float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
        float D = -(A*p1.x + B*p1.y + C*p1.z);
        // iterate through all points in the pointcloud
        for (int index=0; index < numcloud; index++) {
            if (iinliers.count(index)>0) {
                //skip if in iinliers already
                continue;
            }

            // for each point, do distance test
            PointT point = cloud->points[index];
            float ds = fabs(A*point.x + B*point.y + C*point.z + D) / sqrt(A*A + B*B + C*C);
            if (ds <= distanceTol) {
                iinliers.insert(index);
            }
        }

        if (iinliers.size() > inliersResult.size()) {
            inliersResult = iinliers;
        }
    }
    return inliersResult;
}