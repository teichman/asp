#ifndef DEPTH_PROJECTOR_H
#define DEPTH_PROJECTOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>

namespace dst
{

  class DepthProjector : public pipeline2::ComputeNode
  {
  public:

    class Output
    {
    public:
      //! Each pixel indexes into the pointcloud.
      //! -1 means no corresponding depth point.
      cv::Mat1i current_index_;
      cv::Mat1i previous_index_;
      //! current_index_[i] is the point in current_img_ that corresponds to
      //! current_pcd->at(i).  If there is no corresponding point, then current_index_[i]
      //! is (-1, -1).
      std::vector<cv::Point2i>* current_rindex_;
      std::vector<cv::Point2i>* previous_rindex_;
      KinectCloud::ConstPtr current_pcd_;
      KinectCloud::ConstPtr previous_pcd_;
      cv::Mat3b current_img_;
      cv::Mat3b previous_img_;
    };

    pipeline2::Outlet<Output> index_otl_;
    
    DepthProjector(pipeline2::Outlet<cv::Mat3b>* img_otl,
                   pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl);

    //! Finds the closest pixel to pt that has a corresponding depth point.
    //! If none was found, returns (-1, -1).
    static cv::Point2i findNearest(cv::Mat1i index,
                                   int radius,
                                   const cv::Point2i& pt);

    void projectCloud(const KinectCloud& cloud,
                      int spread,
                      cv::Mat1i index) const;

    cv::Mat3b getZBuffer(const KinectCloud& cloud,
                         int spread,
                         float min_range,
                         float max_range) const;
    cv::Mat1b visualizeRangeIndex(cv::Mat1i index) const;
        
  protected:
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl_;
    cv::Mat1i current_index_;
    cv::Mat1i previous_index_;
    cv::Mat3b current_img_;
    cv::Mat3b previous_img_;
    KinectCloud::ConstPtr current_pcd_;
    KinectCloud::ConstPtr previous_pcd_;
    std::vector<cv::Point2i>* current_rindex_;
    std::vector<cv::Point2i>* previous_rindex_;


    void swap();
    void _compute();
    void _display() const;
    void _flush();
    void _reset();
    std::string _getName() const;
  };
  
}

#endif // DEPTH_PROJECTOR_H
