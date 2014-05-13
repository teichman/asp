#include <dst/depth_projector.h>

using namespace std;

namespace dst
{

  DepthProjector::DepthProjector(pipeline2::Outlet<cv::Mat3b>* img_otl,
                                 pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl) :
                                 
    ComputeNode(),
    index_otl_(this),
    img_otl_(img_otl),
    pcd_otl_(pcd_otl),
    current_rindex_(NULL),
    previous_rindex_(NULL)
  {
    registerInput(img_otl_->getNode());
    registerInput(pcd_otl_->getNode());
  }

  cv::Point2i DepthProjector::findNearest(cv::Mat1i index, int radius, const cv::Point2i& pt0)
  {
    // center
    if(index(pt0.y, pt0.x) != -1)
      return pt0;

    // concentric rectangles
    for(int r = 1; r <= radius; ++r) {
      int min_x = max(0, pt0.x - r);
      int max_x = min(index.cols - 1, pt0.x + r);
      int min_y = max(0, pt0.y - r);
      int max_y = min(index.rows - 1, pt0.y + r);

      // top & bottom
      for(int x = min_x; x <= max_x; ++x) { 
        if(index(min_y, x) != -1) return cv::Point2i(x, min_y);
        if(index(max_y, x) != -1) return cv::Point2i(x, max_y);
      }
      
      // left & right
      for(int y = min_y + 1; y < max_y; ++y) {
        if(index(y, min_x) != -1) return cv::Point2i(min_x, y);
        if(index(y, max_x) != -1) return cv::Point2i(max_x, y);
      }
    }

    return cv::Point2i(-1, -1);
  }
  
  void DepthProjector::projectCloud(const KinectCloud& cloud,
                                    int spread,
                                    cv::Mat1i index) const
  {
    ROS_ASSERT((size_t)index.rows == cloud.height);
    ROS_ASSERT((size_t)index.cols == cloud.width);
    ROS_ASSERT(spread == 0);
    index = -1;

    int idx = 0;
    for(int y = 0; y < index.rows; ++y) {
      for(int x = 0; x < index.cols; ++x, ++idx) {
        if(isnan(cloud[idx].y) || isnan(cloud[idx].x) || isnan(cloud[idx].z))
          continue;
        index(y, x) = idx;
      }
    }
  }

  cv::Vec3b colorize(double depth, double min_range, double max_range)
  {
    double increment = (max_range - min_range) / 3;
    double thresh0 = min_range;
    double thresh1 = thresh0 + increment;
    double thresh2 = thresh1 + increment;
    double thresh3 = thresh2 + increment;
    
    if(depth < thresh0) {
      return cv::Vec3b(0, 0, 255);
    }
    if(depth >= thresh0 && depth < thresh1) {
      int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
      return cv::Vec3b(val, val, 255 - val);
    }
    else if(depth >= thresh1 && depth < thresh2) {
      int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
      return cv::Vec3b(255, 255 - val, 0);
    }
    else if(depth >= thresh2 && depth < thresh3) {
      int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
      return cv::Vec3b(255 - val, val, 0);
    }
    
    return cv::Vec3b(0, 255, 0);
  }
  
  cv::Mat3b DepthProjector::getZBuffer(const KinectCloud& cloud,
                                       int spread,
                                       float min_range,
                                       float max_range) const
  {
    cv::Mat1i index(cv::Size(cloud.width, cloud.height), -1);
    projectCloud(cloud, spread, index);
    cv::Mat3b zbuf(index.size(), cv::Vec3b(50, 50, 50));
    for(int y = 0; y < index.rows; ++y) {
      for(int x = 0; x < index.cols; ++x) {
        if(index(y, x) == -1)
          continue;
        zbuf(y, x) = colorize(cloud[index(y, x)].z, min_range, max_range);
      }
    }
    return zbuf;
  }
    
  void DepthProjector::swap()
  {
    cv::Mat1i tmp = previous_index_;
    previous_index_ = current_index_;
    current_index_ = tmp;
    if(current_index_.rows == 0)
      current_index_ = cv::Mat1i::ones(img_otl_->pull().size()) * -1;
    else
      current_index_ = -1;

    previous_img_ = current_img_;
    current_img_ = img_otl_->pull();
    
    previous_pcd_ = current_pcd_;
    current_pcd_ = pcd_otl_->pull();

    std::vector<cv::Point2i>* tmp2 = previous_rindex_;
    previous_rindex_ = current_rindex_;
    current_rindex_ = tmp2;
    if(!current_rindex_)
      current_rindex_ = new std::vector<cv::Point2i>();
    else
      current_rindex_->clear();
  }
  
  void DepthProjector::_compute()
  {
    swap();

    const KinectCloud& cloud = *pcd_otl_->pull();
    projectCloud(cloud, 0, current_index_);

    ROS_ASSERT(current_rindex_->empty());
    current_rindex_->resize(current_pcd_->size(), cv::Point2i(-1, -1));
    for(int y = 0; y < current_index_.rows; ++y) {
      for(int x = 0; x < current_index_.cols; ++x) {
        int idx = current_index_(y, x);
        if(idx == -1)
          continue;
        ROS_ASSERT(idx >= 0 && (size_t)idx < current_pcd_->size());
        current_rindex_->at(idx) = cv::Point2i(x, y);
      }
    }
    
    Output output;
    output.current_index_ = current_index_;
    output.previous_index_ = previous_index_;
    output.current_pcd_ = current_pcd_;
    output.previous_pcd_ = previous_pcd_;
    output.current_img_ = current_img_;
    output.previous_img_ = previous_img_;
    output.current_rindex_ = current_rindex_;
    output.previous_rindex_ = previous_rindex_;
    index_otl_.push(output);
  }

  cv::Mat1b DepthProjector::visualizeRangeIndex(cv::Mat1i index) const
  { 
    cv::Mat1b indvis(index.size(), 0);
    for(int x = 0; x < indvis.cols; ++x) {
      for(int y = 0; y < indvis.rows; ++y) {
        if(index(y, x) != -1)
          indvis(y, x) = 255;
      }
    }
    return indvis;
  }
  
  void DepthProjector::_display() const
  {
    cv::Mat1b indvis = visualizeRangeIndex(current_index_);
    cv::imwrite("debug/" + getRunName() + "-depth_index.png", indvis);
  }

  void DepthProjector::_flush()
  {
    index_otl_.push(Output());
  }

  void DepthProjector::_reset()
  {
    current_index_ = cv::Mat1i();
    previous_index_ = cv::Mat1i();
    current_img_ = cv::Mat3b();
    previous_img_ = cv::Mat3b();
    current_pcd_.reset();
    previous_pcd_.reset();
    if(current_rindex_) { 
      delete current_rindex_;
      current_rindex_ = NULL;
    }
    if(previous_rindex_) {
      delete previous_rindex_;
      previous_rindex_ = NULL;
    }
  }

  std::string DepthProjector::_getName() const
  {
    ostringstream oss;
    oss << "DepthProjector";
    return oss.str();
  }

}
