#ifndef SCENE_ALIGNMENT_NODE_H
#define SCENE_ALIGNMENT_NODE_H

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Eigen>
#include <Eigen/LU>
#include <dst/optical_flow_node.h>

namespace dst
{

  class SceneAlignmentNode : public pipeline2::ComputeNode
  {
  public:
    //pipeline2::Outlet<const Eigen::Affine3f*> transform_otl_;
    //! The current cloud, transformed to overlap with the previous cloud.
    pipeline2::Outlet<KinectCloud::ConstPtr> transformed_otl_;
    //!pipeline2::Outlet<const Eigen::Affine3f*> curr_to_prev_otl_;
    pipeline2::Outlet<const Eigen::Affine3f*> prev_to_curr_otl_;
    
    SceneAlignmentNode(pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
                       pipeline2::Outlet<DepthProjector::Output>* index_otl,
                       double distance_threshold = 0.02,
                       double edge_threshold = 0.1,
                       int num_samples = 200);
                       

    //! Finds a transform T * model = world.
    static Eigen::Affine3f computeTransform(const std::vector<pcl::PointXYZRGB>& world_points,
                                            const std::vector<pcl::PointXYZRGB>& model_points);
    
  protected:
    pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    double distance_threshold_;
    double edge_threshold_;
    int num_samples_;
    //! Only consider optical flow points that look less like an edge than this.  0 to 1.
    std::vector<bool> valid_;
    std::vector<pcl::PointXYZRGB> previous_points_;
    std::vector<pcl::PointXYZRGB> current_points_;
    std::vector<pcl::PointXYZRGB> prev_inliers_;
    std::vector<pcl::PointXYZRGB> curr_inliers_;
    //! curr_to_prev
    Eigen::Affine3f transform_;
    //! prev_to_curr
    Eigen::Affine3f inv_transform_;
    int best_score_;

    void get3DPoints(const std::vector<cv::Point2i>& img_pts,
                     const std::vector<bool>& valid,
                     cv::Mat1i index,
                     KinectCloud::ConstPtr pcd,
                     std::vector<pcl::PointXYZRGB>* pts) const;
    double scoreTransform(const Eigen::Affine3f& trans,
                          std::vector<pcl::PointXYZRGB>* curr_inliers,
                          std::vector<pcl::PointXYZRGB>* prev_inliers) const;
    void sampleCorrespondences(std::vector<pcl::PointXYZRGB>* prev,
                               std::vector<pcl::PointXYZRGB>* curr) const;
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // SCENE_ALIGNMENT_NODE_H
