#include <dst/scene_alignment_node.h>
#include <pcl/common/transformation_from_correspondences.h> // Can't put this in scene_alignment_node.h

using namespace std;

namespace dst
{

  SceneAlignmentNode::SceneAlignmentNode(pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
                                         pipeline2::Outlet<DepthProjector::Output>* index_otl,
                                         double distance_threshold,
                                         double edge_threshold,
                                         int num_samples) :
    ComputeNode(),
    transformed_otl_(this),
    prev_to_curr_otl_(this),
    optflow_otl_(optflow_otl),
    index_otl_(index_otl),
    distance_threshold_(distance_threshold),
    edge_threshold_(edge_threshold),
    num_samples_(num_samples),
    valid_(std::vector<bool>())
  {
    registerInput(optflow_otl_->getNode());
    registerInput(index_otl_->getNode());
  }

  void SceneAlignmentNode::get3DPoints(const std::vector<cv::Point2i>& img_pts,
                                       const std::vector<bool>& valid,
                                       cv::Mat1i index,
                                       KinectCloud::ConstPtr pcd,
                                       std::vector<pcl::PointXYZRGB>* pts) const
  {
    ROS_ASSERT(pts->empty());
    pts->reserve(img_pts.size());
    for(size_t i = 0; i < valid_.size(); ++i) {
      if(!valid_[i])
        continue;

      cv::Point2i impt;
      OpticalFlowNode::safePointRound(index.size(), img_pts[i], &impt);
      int idx = index(impt);
      ROS_FATAL_STREAM_COND(idx < 0 || (size_t)idx >= pcd->points.size(),
                            "idx is " << idx << ", but pcd->points is only "
                            << pcd->points.size() << " long." << endl
                            << "Point: " << impt);
      pts->push_back(pcd->points[idx]);
    }
  }

  double SceneAlignmentNode::scoreTransform(const Eigen::Affine3f& trans,
                                            std::vector<pcl::PointXYZRGB>* curr_inliers,
                                            std::vector<pcl::PointXYZRGB>* prev_inliers) const
  {
    curr_inliers->clear();
    prev_inliers->clear();
    
    int num_inliers = 0;
    for(size_t i = 0; i < current_points_.size(); ++i) {
      double d = (trans * current_points_[i].getVector3fMap() - previous_points_[i].getVector3fMap()).norm();
      if(d < distance_threshold_) { 
        ++num_inliers;
        prev_inliers->push_back(previous_points_[i]);
        curr_inliers->push_back(current_points_[i]);
      }
    }

    return num_inliers;
  }

  void SceneAlignmentNode::sampleCorrespondences(std::vector<pcl::PointXYZRGB>* prev,
                                                 std::vector<pcl::PointXYZRGB>* curr) const
  {
    set<size_t> sample;
    while(sample.size() < 3) {
      size_t s = rand() % current_points_.size();
      sample.insert(s);
    }
    set<size_t>::iterator it;
    int j = 0;
    for(it = sample.begin(); it != sample.end(); ++it, ++j) {
      prev->at(j) = previous_points_[*it];
      curr->at(j) = current_points_[*it];
    }
  }
  
  void SceneAlignmentNode::_compute()
  {
    OpticalFlowNode::Output optflow = optflow_otl_->pull();

    // -- If optical flow didn't compute (i.e. because this is the first frame)
    //    then don't do anything.
    if(!optflow.edge_scores_ || !index_otl_->pull().previous_pcd_) {
      transformed_otl_.push(KinectCloud::Ptr());
      prev_to_curr_otl_.push(NULL);
      return;
    }
    
    // -- Determine which of the correspondences we should bother with.
    cv::Mat1i current_index = index_otl_->pull().current_index_;
    cv::Mat1i previous_index = index_otl_->pull().previous_index_;
    const std::vector<double>& edge_scores = *optflow.edge_scores_;
    ROS_ASSERT(valid_.empty());
    valid_.resize(edge_scores.size(), false);
    int num_valid = 0;
    for(size_t i = 0; i < edge_scores.size(); ++i) {
      if(optflow.status_->at(i) == 0)
        continue;
      if(edge_scores[i] < edge_threshold_)
        continue;
      if(current_index(optflow.points_->at(i)) == -1)
        continue;
      if(previous_index(optflow.prev_points_->at(i)) == -1)
        continue;
      
      valid_[i] = true;
      ++num_valid;
    }

    if(debug_)
      cout << "Found " << num_valid << " valid correspondences." << endl;

    if(num_valid <= 5) {
      ROS_WARN_STREAM("Not enough valid correspondences found when aligning scenes.  Num found: " << num_valid);
      ROS_FATAL_STREAM("TODO: Ignore scene alignment node if not enough correspondences found.");
      abort();
    }

    // -- Get list of corresponding 3D points.
    KinectCloud::ConstPtr current_pcd = index_otl_->pull().current_pcd_;
    KinectCloud::ConstPtr previous_pcd = index_otl_->pull().previous_pcd_;
    get3DPoints(*optflow.points_, valid_, current_index, current_pcd, &current_points_);
    get3DPoints(*optflow.prev_points_, valid_, previous_index, previous_pcd, &previous_points_);
    ROS_ASSERT(current_points_.size() == previous_points_.size());
    
    // -- Sample correspondences, compute transform, see how many fit.
    prev_inliers_.reserve(current_points_.size());
    curr_inliers_.reserve(current_points_.size());
    vector<pcl::PointXYZRGB> best_prev_inliers;
    vector<pcl::PointXYZRGB> best_curr_inliers;
    vector<pcl::PointXYZRGB> prev(3);
    vector<pcl::PointXYZRGB> curr(3);
    for(int i = 0; i < num_samples_; ++i) {
      sampleCorrespondences(&prev, &curr);
      Eigen::Affine3f trans = computeTransform(prev, curr);
      double score = scoreTransform(trans, &curr_inliers_, &prev_inliers_);
      
      if(score > best_score_) {
        best_score_ = score;
        best_prev_inliers = prev_inliers_;
        best_curr_inliers = curr_inliers_;
        transform_ = computeTransform(best_prev_inliers, best_curr_inliers);
      }
    }
    
    // -- Get the current cloud aligned to the previous frame.
    KinectCloud::Ptr transformed(new KinectCloud());
    pcl::transformPointCloud(*current_pcd, *transformed, transform_);
    transformed_otl_.push(transformed);

    // -- Also provide the inverse transform.
    //    TODO: If we couldn't find a transform, provide the identity matrix.
    inv_transform_ = transform_.inverse(Eigen::Affine);
    prev_to_curr_otl_.push(&inv_transform_);
  }

  Eigen::Affine3f
  SceneAlignmentNode::computeTransform(const std::vector<pcl::PointXYZRGB>& world_points,
                                       const std::vector<pcl::PointXYZRGB>& model_points)
  {
    pcl::TransformationFromCorrespondences tfc;

    ROS_ASSERT(world_points.size() == model_points.size());
    Eigen::Matrix<float, 3, Eigen::Dynamic> w(3, world_points.size());
    Eigen::Matrix<float, 3, Eigen::Dynamic> m(3, world_points.size());

    for(size_t i = 0; i < world_points.size(); ++i) {
      w(0, i) = world_points[i].x;
      w(1, i) = world_points[i].y;
      w(2, i) = world_points[i].z;
      m(0, i) = model_points[i].x;
      m(1, i) = model_points[i].y;
      m(2, i) = model_points[i].z;

      tfc.add(m.col(i), w.col(i)); // tfc will return a matrix T such that T * model = world.
    }

    return tfc.getTransformation();
  }
  
  void SceneAlignmentNode::_display() const
  {
    if(!optflow_otl_->pull().edge_scores_ || !index_otl_->pull().previous_pcd_) {
      return;
    }

    cout << "Best transform has score " << best_score_ << endl;
    cout << transform_.matrix() << endl;

    KinectCloud::Ptr transformed(new KinectCloud());
    pcl::transformPointCloud(*index_otl_->pull().current_pcd_, *transformed, transform_);
    *transformed += *index_otl_->pull().previous_pcd_;

    if(!transformed->empty())
      pcl::io::savePCDFileBinary("debug/" + getRunName() + ".pcd", *transformed);
    else { 
      int retval = system(("touch debug/" + getRunName() + ".pcd").c_str());
      --retval;
    }
  }

  void SceneAlignmentNode::_flush()
  {
    valid_.clear();
    previous_points_.clear();
    current_points_.clear();
    transform_ = Eigen::Affine3f();
    best_score_ = 0;
    
    transformed_otl_.push(KinectCloud::Ptr());
    ROS_ASSERT(!transformed_otl_.pull());
  }

  std::string SceneAlignmentNode::_getName() const
  {
    std::ostringstream oss;
    oss << "SceneAlignmentNode_edge_threshold:" << edge_threshold_; // TODO: Precision
    return oss.str();
  }
  
} // namespace dst
