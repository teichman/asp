#include <dst/template_matcher_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  TemplateMatcherNPG::TemplateMatcherNPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
                                         pipeline2::Outlet<cv::Mat3b>* img_otl,
                                         pipeline2::Outlet<cv::Mat1b>* mask_otl) :
    NodePotentialGenerator(),
    index_otl_(index_otl),
    img_otl_(img_otl),
    mask_otl_(mask_otl)
  {
    registerInput(index_otl_->getNode());
    registerInput(img_otl_->getNode());
    registerInput(mask_otl_->getNode());
  }

  void TemplateMatcherNPG::_compute()
  {
    // -- Pull inputs from connected ComputeNodes.
    cv::Mat3b img = img_otl_->pull();
    cv::Mat1b mask = mask_otl_->pull();
    KinectCloud::ConstPtr pcd = index_otl_->pull().current_pcd_;

    // -- Reallocate potentials if necessary.
    if(source_potentials_.rows() != img.rows || source_potentials_.cols() != img.cols) {
      source_potentials_ = MatrixXd::Zero(img.rows, img.cols);
      sink_potentials_ = MatrixXd::Zero(img.rows, img.cols);
    }
    ROS_ASSERT(source_potentials_.rows() > 0 && source_potentials_.cols() > 0);
    ROS_ASSERT(sink_potentials_.rows() > 0 && sink_potentials_.cols() > 0);
    
    // -- Compute node potentials.
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x) {
        // Ignore points not on the boundary.
        if(mask(y, x) != 255)
          continue;

        // Random placeholder.
        int idx = y * img.cols + x;
        if(idx % 2 == 0)
          source_potentials_(y, x) = 1;
        else
          sink_potentials_(y, x) = 1;
      }
    }

    // -- Push node potential outputs.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void TemplateMatcherNPG::_display() const
  {
    displayNodePotentials(img_otl_->pull());
  }

  void TemplateMatcherNPG::_flush()
  {

  }

  void TemplateMatcherNPG::_reset()
  {
    
  }

  std::string TemplateMatcherNPG::_getName() const
  {
    ostringstream oss;
    oss << "TemplateMatcherNPG";  // Append params...
    return oss.str();
  }

  
}  // namespace
