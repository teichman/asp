#ifndef FOREGROUND_KDTREE_NODE_H
#define FOREGROUND_KDTREE_NODE_H

#include <pcl/io/pcd_io.h>
#include <pipeline2/pipeline2.h>
#include <dst/depth_projector.h>

namespace dst
{

  class ForegroundKdTreeNode : public pipeline2::ComputeNode
  {
  public:
    pipeline2::Outlet<KdTree::Ptr> kdtree_otl_;
    pipeline2::Outlet<KinectCloud::ConstPtr> pcd_otl_;
    
    ForegroundKdTreeNode(pipeline2::Outlet<cv::Mat1b>* seg_otl,
                         pipeline2::Outlet<DepthProjector::Output>* index_otl);
    
    
  protected:
    pipeline2::Outlet<cv::Mat1b>* seg_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    KinectCloud::Ptr fg_pcd_;
    KdTree::Ptr fg_kdtree_;

    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // FOREGROUND_KDTREE_NODE_H
