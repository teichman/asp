#include <dst/edge_potential_product_epg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  EdgePotentialProduct::EdgePotentialProduct(pipeline2::Outlet<cv::Mat3b>* img_otl,
                                             const std::vector<EdgePotentialGenerator*>& generators) :
    EdgePotentialGenerator(),
    img_otl_(img_otl),
    generators_(generators)
  {
    ROS_ASSERT(!generators_.empty());
    for(size_t i = 0; i < generators_.size(); ++i)
      registerInput(generators_[i]);
  }

  void EdgePotentialProduct::_compute()
  {
    SparseMatrix<double, RowMajor>* zeroth = generators_[0]->edge_otl_.pull();
    ROS_ASSERT(zeroth);
    potentials_ = *zeroth;
    for(size_t i = 1; i < generators_.size(); ++i)
      potentials_ = potentials_.cwiseProduct(*generators_[i]->edge_otl_.pull());

    edge_otl_.push(&potentials_);
  }

  void EdgePotentialProduct::_display() const
  {
    displayEdges(img_otl_->pull());
  }

  void EdgePotentialProduct::_flush()
  {
    potentials_.setZero();
    edge_otl_.push(NULL);
  }

  std::string EdgePotentialProduct::_getName() const
  {
    ostringstream oss;
    oss << "EPP";
    for(size_t i = 0; i < generators_.size(); ++i)
      oss << "_" << generators_[i]->getShortName();
    return oss.str();
  }

}
