#include <dst/distance_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  DistanceNPG::DistanceNPG(pipeline2::Outlet<KinectCloud::ConstPtr>* transformed_otl,
                           pipeline2::Outlet<KdTree::Ptr>* fg_kdtree_otl,
                           pipeline2::Outlet<cv::Mat3b>* img_otl,
                           pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl,
                           float sigma) :
    NodePotentialGenerator(),
    transformed_otl_(transformed_otl),
    fg_kdtree_otl_(fg_kdtree_otl),
    img_otl_(img_otl),
    pcd_indices_otl_(pcd_indices_otl),
    sigma_(sigma)
  {
    registerInput(transformed_otl_->getNode());
    registerInput(fg_kdtree_otl_->getNode());
    registerInput(img_otl_->getNode());
    registerInput(pcd_indices_otl_->getNode());
  }

  void DistanceNPG::_compute()
  {
    if(!fg_kdtree_otl_->pull())
      return;
    
    const KinectCloud& cloud = *transformed_otl_->pull();
    KdTree& fg_kdtree = *fg_kdtree_otl_->pull();
    cv::Mat3b img = img_otl_->pull();
    
    // -- Reallocate potentials if necessary.
    ROS_ASSERT(img.rows > 0);
    if(source_potentials_.rows() != img.rows ||
       source_potentials_.cols() != img.cols) {
      source_potentials_ = MatrixXd::Zero(img.rows, img.cols);
      sink_potentials_ = MatrixXd::Zero(img.rows, img.cols);
    }

    vector<int> indices(1);
    vector<float> distances(1);
    const vector<int>& pcd_indices = *pcd_indices_otl_->pull();
    for(size_t i = 0; i < pcd_indices.size(); ++i) {
      int idx = pcd_indices[i];
      if(isnan(cloud[idx].z))
        continue;
      fg_kdtree.nearestKSearch(cloud[idx], 1, indices, distances);
      float dist = distances[0];
      int y = (int)idx / cloud.width;
      int x = (int)idx - y * cloud.width;
      sink_potentials_(y, x) = 1.0 - exp(-dist / sigma_);
    }

    // -- Fill the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void DistanceNPG::_display() const
  {
    if(!fg_kdtree_otl_->pull())
      return;
    
    displayNodePotentials(img_otl_->pull());
  }

  void DistanceNPG::_flush()
  {
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
    source_potentials_.setZero();
    sink_potentials_.setZero();
  }

  std::string DistanceNPG::_getName() const
  {
    std::ostringstream oss;
    oss << "DistanceNPG_sigma:" << sigma_;
    return oss.str();
  }

} // namespace dst
