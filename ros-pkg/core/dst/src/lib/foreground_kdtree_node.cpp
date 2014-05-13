#include <dst/foreground_kdtree_node.h>

using namespace std;

namespace dst
{

  ForegroundKdTreeNode::ForegroundKdTreeNode(pipeline2::Outlet<cv::Mat1b>* seg_otl,
                                             pipeline2::Outlet<DepthProjector::Output>* index_otl) :
    kdtree_otl_(this),
    pcd_otl_(this),
    seg_otl_(seg_otl),
    index_otl_(index_otl),
    fg_pcd_(new KinectCloud())
  {
    registerInput(seg_otl_->getNode());
    registerInput(index_otl_->getNode());
  }
  
  void ForegroundKdTreeNode::_compute()
  {
    if(seg_otl_->pull().rows == 0)
      return;

    cv::Mat1b seg = seg_otl_->pull();
    const KinectCloud& pcd = *index_otl_->pull().previous_pcd_;
    cv::Mat1i index = index_otl_->pull().previous_index_;
    
    // -- Get the foreground object in its own cloud.
    fg_pcd_->reserve(pcd.size());
    for(int y = 0; y < seg.rows; ++y) {
      for(int x = 0; x < seg.cols; ++x) {
        if(seg(y, x) != 255)
          continue;
        if(index(y, x) == -1)
          continue;

        //if(rand() % 3 == 0)
        fg_pcd_->push_back(pcd[index(y, x)]);
      }
    }

    // -- Build a kdtree for it.
    if(fg_pcd_->empty())
      fg_kdtree_ = KdTree::Ptr();
    else { 
      fg_kdtree_ = KdTree::Ptr(new KdTree());
      fg_kdtree_->setInputCloud(fg_pcd_);
    }
    
    // -- Fill outlets.
    pcd_otl_.push(fg_pcd_);
    kdtree_otl_.push(fg_kdtree_);
  }

  void ForegroundKdTreeNode::_display() const
  {
    if(seg_otl_->pull().rows == 0)
      return;

    cout << "Previous foreground PCD has " << fg_pcd_->size() << " points." << endl;
    if(!fg_pcd_->empty())
      pcl::io::savePCDFileBinary("debug/" + getRunName() + ".pcd", *fg_pcd_);
    else { 
      int retval = system(("touch debug/" + getRunName() + ".pcd").c_str());
      --retval;
    }

  }

  void ForegroundKdTreeNode::_flush()
  {
    fg_pcd_->clear();
    fg_kdtree_.reset(); // TODO: Don't reallocate.
    pcd_otl_.push(KinectCloud::Ptr());
    kdtree_otl_.push(KdTree::Ptr());
    ROS_ASSERT(!pcd_otl_.pull());
    ROS_ASSERT(!kdtree_otl_.pull());
  }

  std::string ForegroundKdTreeNode::_getName() const
  {
    return "ForegroundKdTreeNode";
  }

} // namespace dst
