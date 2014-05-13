#ifndef BOUNDARY_MASK_NODE_H
#define BOUNDARY_MASK_NODE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>
#include <dst/node_potential_generator.h>
#include <bag_of_tricks/image_region_iterator.h>

namespace dst
{

  class BoundaryMaskNode : public NodePotentialGenerator
  {
  public:
    typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;
    typedef boost::shared_ptr< std::vector<int> > IndicesPtr;
    
    pipeline2::Outlet<cv::Mat1b> mask_otl_;
    pipeline2::Outlet<IndicesConstPtr> pcd_indices_otl_;

    //! @param radius_2d Width and height of image square to consider when refining the
    //!   boundary mask using depth.
    //! @param radius_3d Points within this radius count as neighbors.
    BoundaryMaskNode(pipeline2::Outlet<cv::Mat1b>* seg_otl,
                     pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl,
                     pipeline2::Outlet<KinectCloud::ConstPtr>* prev_pcd_otl,
                     pipeline2::Outlet<cv::Mat3b>* img_otl,
                     pipeline2::Outlet<cv::Mat1b>* seed_otl,
                     int radius_2d, double radius_3d);

  protected:
    pipeline2::Outlet<cv::Mat1b>* seg_otl_;
    pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl_;
    pipeline2::Outlet<KinectCloud::ConstPtr>* prev_pcd_otl_;
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    pipeline2::Outlet<cv::Mat1b>* seed_otl_;
    int radius_2d_;
    double radius_3d_;
    cv::Mat1b rough_mask_;
    cv::Mat1b refined_mask_;
    cv::Mat1b bg_mask_;
    cv::Mat1b fg_mask_;
    IndicesPtr rough_pcd_indices_;
    IndicesPtr refined_pcd_indices_;
  
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // BOUNDARY_MASK_NODE_H
