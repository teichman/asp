#include <dst/boundary_mask_node.h>
#include <pcl/common/distances.h>

#define MASK (getenv("MASK") ? atoi(getenv("MASK")) : 0)

using namespace std;
using namespace Eigen;

namespace dst
{

  BoundaryMaskNode::BoundaryMaskNode(pipeline2::Outlet<cv::Mat1b>* seg_otl,
                                     pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl,
                                     pipeline2::Outlet<KinectCloud::ConstPtr>* prev_pcd_otl,
                                     pipeline2::Outlet<cv::Mat3b>* img_otl,
                                     pipeline2::Outlet<cv::Mat1b>* seed_otl,
                                     int radius_2d, double radius_3d) :
    NodePotentialGenerator(),
    mask_otl_(this),
    pcd_indices_otl_(this),
    seg_otl_(seg_otl),
    pcd_otl_(pcd_otl),
    prev_pcd_otl_(prev_pcd_otl),
    img_otl_(img_otl),
    seed_otl_(seed_otl),
    radius_2d_(radius_2d),
    radius_3d_(radius_3d),
    rough_pcd_indices_(new vector<int>()),
    refined_pcd_indices_(new vector<int>())
  {
    registerInput(seg_otl_->getNode());
    registerInput(pcd_otl_->getNode());
    registerInput(prev_pcd_otl_->getNode());
    registerInput(seed_otl_->getNode());
    registerInput(img_otl_->getNode());
  }

  void BoundaryMaskNode::_compute()
  {
    cv::Mat1b seg = seg_otl_->pull();
    if(seg.rows == 0) { 
      seg = seed_otl_->pull();
    }
    
    if(bg_mask_.rows == 0) {
      bg_mask_ = cv::Mat1b(seg.size(), 0);
      fg_mask_ = cv::Mat1b(seg.size(), 0);
      rough_mask_ = cv::Mat1b(seg.size(), 0);
      refined_mask_ = cv::Mat1b(seg.size(), 0);
      rough_pcd_indices_->reserve(seg.rows * seg.cols);
      refined_pcd_indices_->reserve(seg.rows * seg.cols);
    }

    // -- Reallocate potentials if necessary.
    if(source_potentials_.rows() != seg.rows ||
       source_potentials_.cols() != seg.cols) {
      source_potentials_ = MatrixXd::Zero(seg.rows, seg.cols);
      sink_potentials_ = MatrixXd::Zero(seg.rows, seg.cols);
    }
    
    // -- Get foreground and background masks.
    for(int y = 0; y < seg.rows; ++y) {
      for(int x = 0; x < seg.cols; ++x) {
        if(seg(y, x) == 0) {
          bg_mask_(y, x) = 255;
          fg_mask_(y, x) = 0;
        }
        else if(seg(y, x) == 255) {
          bg_mask_(y, x) = 0;
          fg_mask_(y, x) = 255;
        }
      }
    }

    // -- Make a first, rough boundary mask.
    int iters = 10;
    cv::dilate(bg_mask_, bg_mask_, cv::Mat(), cv::Point(-1, -1), iters);
    cv::dilate(fg_mask_, fg_mask_, cv::Mat(), cv::Point(-1, -1), iters);
    for(int y = 0; y < seg.rows; ++y) { 
      for(int x = 0; x < seg.cols; ++x) { 
        if(fg_mask_(y, x) == 255 && bg_mask_(y, x) == 255)  { 
          rough_mask_(y, x) = 255;
          rough_pcd_indices_->push_back(y * seg.cols + x);
        }
      }
    }
    
    // -- Refine by using 3D information.
    if(getenv("REFINE") && prev_pcd_otl_->pull()) { 
      const KinectCloud& prev_pcd = *prev_pcd_otl_->pull();
      for(int y = 0; y < seg.rows; ++y) { 
        for(int x = 0; x < seg.cols; ++x) { 
          if(rough_mask_(y, x) != 255 || seg(y, x) == 127)
            continue;
          
          const pcl::PointXYZRGB& center_xyz = prev_pcd[y * seg.cols + x];
          if(isnan(center_xyz.z))
            continue;
          
          // -- Determine if this point has any neighbors in 3d worth considering.
          int center_label = seg(y, x);
          ImageRegionIterator it(rough_mask_.size(), radius_2d_);
          for(it.setCenter(cv::Point2i(x, y)); !it.done(); ++it) {
            if(seg(*it) != center_label && seg(*it) != 127) { 
              const pcl::PointXYZRGB& pt = prev_pcd[it.index()];
              if(isnan(pt.z))
                continue;
              
              double dist = pcl::euclideanDistance(center_xyz, pt);
              if(dist < radius_3d_) { 
                refined_mask_(y, x) = 255;
                refined_pcd_indices_->push_back(y * seg.cols + x);


                // cout << "Center: " << center_xyz << endl;
                // cout << "Neighbor: " << pt << endl;
                // cout << "Center label: " << center_label << endl;
                // cout << "Neighbor label: " << (int)seg(*it) << endl;
                // cout << "Distance: " << dist << endl;
                // cv::Mat3b vis = img_otl_->pull().clone();
                // cv::circle(vis, cv::Point2i(x, y), 2, cv::Scalar(0, 0, 255));
                // cv::circle(vis, *it, 2, cv::Scalar(255, 0, 0));
                // cv::imshow("debug", vis);

                // cv::waitKey();
                
                break;
              }
            }
          }
        }
      }
    }
    else {
      rough_mask_.copyTo(refined_mask_);
      *refined_pcd_indices_ = *rough_pcd_indices_;
    }

    if(!MASK) {
      refined_mask_ = 255;
      refined_pcd_indices_->clear();
      const KinectCloud& pcd = *pcd_otl_->pull();
      for(size_t i = 0; i < pcd.size(); ++i)
        refined_pcd_indices_->push_back(i);
    }
    
    // -- For pixels that have nothing interesting going on,
    //    bump towards the appropriate label.
    for(int y = 0; y < seg.rows; ++y) { 
      for(int x = 0; x < seg.cols; ++x) { 
        if(refined_mask_(y, x) != 255) { 
          if(seg(y, x) == 255)
            source_potentials_(y, x) = 1;
          //else if(seg(y, x) == 0)
          else 
            sink_potentials_(y, x) = 1;
        }
      }
    }

    // -- Push outputs.
    mask_otl_.push(refined_mask_);
    pcd_indices_otl_.push(refined_pcd_indices_);
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void BoundaryMaskNode::_display() const
  {
    displayNodePotentials();
    cv::imwrite("debug/" + getRunName() + "-rough_mask.png", rough_mask_);
    cv::imwrite("debug/" + getRunName() + "-refined_mask.png", refined_mask_);
  }

  void BoundaryMaskNode::_flush()
  {
    rough_pcd_indices_->clear();
    refined_pcd_indices_->clear();
    mask_otl_.push(cv::Mat1b());
    pcd_indices_otl_.push(IndicesConstPtr());
    rough_mask_ = 0;
    refined_mask_ = 0;
    bg_mask_ = 0;
    fg_mask_ = 0;

    source_otl_.push(NULL);
    sink_otl_.push(NULL);
    source_potentials_.setZero();
    sink_potentials_.setZero();
  }

  std::string BoundaryMaskNode::_getName() const
  {
    ostringstream oss;
    oss << "BoundaryMaskNode";
    return oss.str();
  }

  
} // namespace dst
