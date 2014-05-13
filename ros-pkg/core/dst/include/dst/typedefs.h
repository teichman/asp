#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <maxflow/graph.h>
#include <dst/helper_functions.h>

namespace dst
{
  
  typedef pcl::PointCloud<pcl::PointXYZRGB> KinectCloud;
  typedef pcl::PointXYZRGB Point;
  typedef Graph<double, double, double> Graph3d;
  typedef boost::shared_ptr<Graph3d> Graph3dPtr;
  //typedef pcl::KdTreeFLANN<pcl::PointXYZRGB> KdTree;
  typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;
  
}
