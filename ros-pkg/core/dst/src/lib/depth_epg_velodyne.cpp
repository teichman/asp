#include <dst/depth_epg_velodyne.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  DepthEPGVelodyne::DepthEPGVelodyne(pipeline2::Outlet<DepthProjector::Output>* index_otl,
                     pipeline2::Outlet<Normals::Ptr>* normals_otl,
                     pipeline2::Outlet<cv::Mat1b>* mask_otl,
                     int radius2d,
                     double sigma_norm,
                     double sigma_euc) :
    EdgePotentialGenerator(),
    weights_otl_(this),
    index_otl_(index_otl),
    normals_otl_(normals_otl),
    mask_otl_(mask_otl),
    radius2d_(radius2d),
    sigma_norm_(sigma_norm),
    sigma_euc_(sigma_euc)
  {
    registerInput(index_otl_->getNode());
    registerInput(normals_otl_->getNode());
    registerInput(mask_otl_->getNode());

    // TODO: radius should be 10?
//    ROS_ASSERT(radius2d_ == 1); // Otherwise edge_potential_generator drawLine fails.
  }

  DepthEPGVelodyne::~DepthEPGVelodyne()
  {

  }

  double DepthEPGVelodyne::getPotential(int y0, int x0, int y, int x) const
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    if(index(y0, x0) == -1 || index(y, x) == -1)
      return 0;
    
    const KinectCloud& pcd = *index_otl_->pull().current_pcd_;
    const Normals& normals = *normals_otl_->pull();
    
    const pcl::PointXYZRGB& pt0 = pcd[index(y0, x0)];
    const pcl::PointXYZRGB& pt1 = pcd[index(y, x)];
    const pcl::Normal& n = normals[index(y0, x0)];

    Eigen::Vector3f v0 = pt0.getVector3fMap();
    Eigen::Vector3f v1 = pt1.getVector3fMap();
    Eigen::Vector3f n0 = n.getNormalVector3fMap();
    Eigen::Vector3f d = v1 - v0;
    d[0] *= sqrt(27);
    
    double de = d.norm();
    double dn = fabs( d.dot(n0) );
    if(isnan(dn)) dn = 0;  // Not all surface normals get computed.

    return exp(-dn / sigma_norm_ - de / sigma_euc_);
  }

  double DepthEPGVelodyne::getWeight(int y0, int x0, int y, int x) const
  {
    ROS_ASSERT(false);
    cv::Mat1i index = index_otl_->pull().current_index_;
    if(index(y0, x0) == -1 || index(y, x) == -1)
      return 1.0;

    KinectCloud::ConstPtr pcd = index_otl_->pull().current_pcd_;
    const pcl::PointXYZRGB& pt0 = pcd->at(index(y0, x0));
    const pcl::PointXYZRGB& pt1 = pcd->at(index(y, x));

    return exp(-fabs(pt0.z - pt1.z) / sigma_norm_);
  }
  
  void DepthEPGVelodyne::processDepthPoint(int y0, int x0)
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    int idx = getIdx(y0, x0, index.cols);
    int ymin = fmax(0, y0 - radius2d_);
    int ymax = fmin(index.rows - 1, y0 + radius2d_);
    int xmin = fmax(0, x0 - radius2d_);
    int xmax = fmin(index.cols - 1, x0 + radius2d_);
    
    for(int y = ymin; y <= ymax; ++y) {
      for(int x = xmin; x <= xmax; ++x) {
        if(y == y0 && x == x0)
          continue;


        // Don't use diagonal edges.  TODO: Remove this.
        if(x != x0 && y != y0) 
          continue;
        
        int idx2 = getIdx(y, x, index.cols);
        weights_.insertBack(idx, idx2) = 0;//getWeight(y0, x0, y, x);
        potentials_.insertBack(idx, idx2) = getPotential(y0, x0, y, x);
      }
    }
  }

  void DepthEPGVelodyne::_compute()
  {
    // -- Initialize the potentials and weights storage.
    cv::Mat1i index = index_otl_->pull().current_index_;
    int num_nodes = index.rows * index.cols;
    initializeStorage(num_nodes);
    
    if(weights_.rows() != num_nodes || weights_.cols() != num_nodes) { 
      weights_ = SparseMatrix<double, RowMajor>(num_nodes, num_nodes);
      weights_.reserve(2 * num_nodes);
    }
    weights_.setZero();
    
    // -- For each pixel with a depth point, find all neighbors with a depth point.
    //    Add edges for those that are close.
    cv::Mat1b mask = mask_otl_->pull();
    for(int y = 0; y < index.rows; ++y) {
      for(int x = 0; x < index.cols; ++x) {
        int idx = getIdx(y, x, index.cols);
        potentials_.startVec(idx);
        weights_.startVec(idx);

        if(mask(y, x) == 255)
          processDepthPoint(y, x);
      }
    }
    potentials_.finalize();
    weights_.finalize();
    
    // -- Load the outlet.
    edge_otl_.push(&potentials_);
    weights_otl_.push(&weights_);
  }

  void DepthEPGVelodyne::_display() const
  {
    displayEdges(index_otl_->pull().current_img_);
  }

  void DepthEPGVelodyne::_flush()
  {
    potentials_.setZero();
    weights_.setZero();
    edge_otl_.push(NULL);
    weights_otl_.push(NULL);
  }

  std::string DepthEPGVelodyne::_getName() const
  {
    std::ostringstream oss;
    oss << "DepthEPGVelodyne_radius2d:" << radius2d_ << "_sigma_norm:" << sigma_norm_; // TODO: Set precision
    return oss.str();
  }

  
  
} // namespace dst
