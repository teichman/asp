#ifndef ICP_NPG_H
#define ICP_NPG_H

#include <pcl/common/centroid.h>
#include <dst/depth_projector.h>
#include <dst/kdtree_node.h>
#include <dst/scene_alignment_node.h>
#include <dst/node_potential_generator.h>

namespace dst
{

  class IcpNPG : public NodePotentialGenerator
  {
  public:
    typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;
    
    //! Transform that takes the previous foreground object to
    //! its best location in the new scene after ICP.
    //pipeline2::Outlet<const Eigen::Affine3f*> prev_to_curr_;

    //! Pointcloud of the previous foreground object, aligned to fit in the current frame
    //! as best as possible.  If score_thresh was not met, this will be NULL.
    pipeline2::Outlet<KinectCloud::ConstPtr> aligned_fg_otl_;
    pipeline2::Outlet<KdTree::Ptr> aligned_fg_kdtree_otl_;
    
    IcpNPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
           pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
           pipeline2::Outlet<const Eigen::Affine3f*>* prev_to_curr_otl,
           pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
           pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl,
           float score_thresh,
           float distance_thresh,
           float fringe_radius,
           float sigma_dist,
           float sigma_color,
           float delta_transform_thresh,
           bool use_prev_bg,
           int skip,
           const std::vector<int>& lookback = std::vector<int>(1, 0));
      

    //! Iteratively transforms curr_fg to fit into curr_kdtree.
    //! Returns num_inliers / curr_fg->size as score.
    double runICP(KdTree& curr_kdtree,
                  const KinectCloud& curr_cloud,
                  KinectCloud::Ptr curr_fg,
                  Eigen::Affine3f* final_transform) const;
    
  protected:
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl_;
    pipeline2::Outlet<const Eigen::Affine3f*>* prev_to_curr_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;
    pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl_;
    //! Percentage of inliers required to use the ICP fit as a node potential.
    //! TODO: This could take into account occlusion.
    float score_thresh_;
    //! ICP correspondence threshold.
    float distance_thresh_;
    //! Bring along prev bg points that are within this radius of any prev fg point.
    float fringe_radius_;
    float sigma_dist_;
    float sigma_color_;
    //! ICP (transform - Identity) must have at least Frobenius norm
    //! of this much to continue searching.
    float delta_transform_thresh_;
    bool use_prev_bg_;
    int skip_;
    //! Do ICP using FG from these previous frames.  The first must be 0.
    //! lookback_[i] = k means that fg_buffer_[k] will be used.
    std::vector<int> lookback_;
    int max_buffer_size_;
    //! FG from previous frames.
    //! fg_buffer_.front() is the most recent.  fg_buffer_.back() is the oldest.
    std::deque<KinectCloud::Ptr> fg_buffer_;
    //! Fringe background from previous frames.
    std::deque<KinectCloud::Ptr> bg_buffer_;
    KinectCloud::Ptr aligned_fg_;
    KdTree::Ptr aligned_fg_kdtree_;
    
    void updateBuffers();
    void computeForLookback(int lb);
    void addPotentials2(KinectCloud::ConstPtr object,
                        Eigen::MatrixXd* potentials) const;
        
    void addPotentials(const KinectCloud& object,
                       const KinectCloud& curr_cloud,
                       KdTree& curr_kdtree,
                       Eigen::MatrixXd* potentials) const;

    void _compute();
    void _reset();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // ICP_NPG_H
