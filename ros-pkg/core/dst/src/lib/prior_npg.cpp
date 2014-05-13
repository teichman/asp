#include <dst/prior_npg.h>

using namespace Eigen;

namespace dst
{

  PriorNPG::PriorNPG(pipeline2::Outlet<cv::Mat3b>* image_otl) :
    NodePotentialGenerator(),
    image_otl_(image_otl)
  {
    registerInput(image_otl_->getNode());
  }

  void PriorNPG::_compute()
  {
    cv::Mat3b img = image_otl_->pull();

    // -- Reallocate potentials if necessary.
    //    Set sink to all 1s, source to all zeros.
    ROS_ASSERT(img.rows > 0);
    if(source_potentials_.rows() != img.rows ||
       source_potentials_.cols() != img.cols) {
      source_potentials_ = MatrixXd::Zero(img.rows, img.cols);
      sink_potentials_ = MatrixXd::Ones(img.rows, img.cols);
    }
    ROS_ASSERT(source_potentials_.rows() > 0 && source_potentials_.cols() > 0);
    ROS_ASSERT(sink_potentials_.rows() > 0 && sink_potentials_.cols() > 0);

    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void PriorNPG::_display() const
  {
    displayNodePotentials(image_otl_->pull());
  }

  void PriorNPG::_flush()
  {
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }

  std::string PriorNPG::_getName() const
  {
    return "PriorNPG";
  }

} // namespace dst
