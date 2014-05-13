#include <dst/surface_normal_epg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  SurfaceNormalEPG::SurfaceNormalEPG(pipeline2::Outlet<Normals::Ptr>* normals_otl,
                                     pipeline2::Outlet<DepthProjector::Output>* index_otl,
                                     pipeline2::Outlet<cv::Mat1b>* mask_otl,
                                     pipeline2::Outlet<cv::Mat3b>* image_otl,
                                     double sigma) :
    EdgePotentialGenerator(),
    normals_otl_(normals_otl),
    index_otl_(index_otl),
    mask_otl_(mask_otl),
    image_otl_(image_otl),
    sigma_(sigma)
  {
    registerInput(normals_otl_->getNode());
    registerInput(index_otl_->getNode());
    registerInput(mask_otl_->getNode());
    registerInput(image_otl_->getNode());
  }

  inline
  double SurfaceNormalEPG::computePotential(int idx0, int idx1, Normals::Ptr normals) const
  {
    ROS_ASSERT(idx0 > 0 && idx0 < (int)normals->size());
    ROS_ASSERT(idx1 > 0 && idx1 < (int)normals->size());
    size_t sz_start = normals->size();

    if(isnan(normals->at(idx0).normal[0]) || isnan(normals->at(idx1).normal[0]) ||
       isnan(normals->at(idx0).normal[1]) || isnan(normals->at(idx1).normal[1]) ||
       isnan(normals->at(idx0).normal[2]) || isnan(normals->at(idx1).normal[2]) ||
       isinf(normals->at(idx0).normal[0]) || isinf(normals->at(idx1).normal[0]) ||
       isinf(normals->at(idx0).normal[1]) || isinf(normals->at(idx1).normal[1]) ||
       isinf(normals->at(idx0).normal[2]) || isinf(normals->at(idx1).normal[2]))
    {
    // if(isnan(normals->at(idx0).normal[0]) || isnan(normals->at(idx1).normal[0]) ||
    //    isnan(normals->at(idx0).normal[1]) || isnan(normals->at(idx1).normal[1]) ||
    //    isnan(normals->at(idx0).normal[2]) || isnan(normals->at(idx1).normal[2]))
    // {
      return 0;
    }
    
    double dot = 0.0;
    for(int i = 0; i < 3; ++i) {
      ROS_ASSERT(!isnan(normals->at(idx0).normal[i]));
      ROS_ASSERT(!isnan(normals->at(idx1).normal[i]));
      dot += normals->at(idx0).normal[i] * normals->at(idx1).normal[i];
      if(isnan(dot)) {
        ROS_FATAL_STREAM("normals->at(idx0): " << normals->at(idx0));
        ROS_FATAL_STREAM("normals->at(idx1): " << normals->at(idx1));
        ROS_FATAL_STREAM("normals->at(idx0).normal[i]: " << normals->at(idx0).normal[i]);
        ROS_FATAL_STREAM("normals->at(idx1).normal[i]: " << normals->at(idx1).normal[i]);
        ROS_FATAL_STREAM("normals->at(idx0).normal[i] * normals->at(idx1).normal[i]: " << normals->at(idx0).normal[i] * normals->at(idx1).normal[i]);
        ROS_FATAL_STREAM("dot: " << dot);
        ROS_ASSERT(idx0 > 0 && idx0 < (int)normals->size());
        ROS_ASSERT(idx1 > 0 && idx1 < (int)normals->size());
        ROS_ASSERT(normals->size() == sz_start);

        ROS_ASSERT(!isnan(dot));
      }
    }
    
    double angle = acos(dot);
    if(fabs(dot - 1e-6) > 1 || fabs(dot + 1e-6) > 1)
      angle = 0; // acos(1) returns NaN.
    
    if(isnan(angle)) {
      ROS_FATAL_STREAM("NaN in SurfaceNormalEPG: " << dot << " " << acos(dot) << std::flush);
      abort();
    }

    ROS_ASSERT(normals->size() == sz_start);
    return exp(-angle / sigma_);
  }
  
  void SurfaceNormalEPG::fillPotentials(int y, int x, int idx)
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    Normals::Ptr normals = normals_otl_->pull();

    // TODO: make an abstraction for filling neighbors with bounds checking.
    if(y > 0 && index(y-1, x) != -1) { 
      int idx2 = getIdx(y-1, x, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y-1, x), normals);
    }
    if(x > 0 && index(y, x-1) != -1) { 
      int idx2 = getIdx(y, x-1, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y, x-1), normals);
    }
    if(x < index.cols - 1 && index(y, x+1) != -1) { 
      int idx2 = getIdx(y, x+1, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y, x+1), normals);
    }
    if(y < index.rows - 1 && index(y+1, x) != -1) { 
      int idx2 = getIdx(y+1, x, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y+1, x), normals);
    }
  }
  
  void SurfaceNormalEPG::_compute()
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    initializeStorage(index.rows * index.cols);

    cv::Mat1b mask = mask_otl_->pull();
    for(int y = 0; y < index.rows; ++y) {
      for(int x = 0; x < index.cols; ++x) {
        int idx = getIdx(y, x, index.cols);
        potentials_.startVec(idx);
        if(mask(y, x) != 255 || index(y, x) == -1)
          continue;
        fillPotentials(y, x, idx);
      }
    }
    potentials_.finalize();
    edge_otl_.push(&potentials_);
  }

  void SurfaceNormalEPG::_display() const
  {
    displayEdges(image_otl_->pull());
  }

  void SurfaceNormalEPG::_flush()
  {
    
  }

  std::string SurfaceNormalEPG::_getName() const
  {
    std::ostringstream oss;
    oss << "SurfaceNormalEPG";
    return oss.str();
  }

} // namespace dst
