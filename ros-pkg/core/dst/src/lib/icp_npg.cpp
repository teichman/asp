#include <dst/icp_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  IcpNPG::IcpNPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
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
                 const std::vector<int>& lookback) :
    NodePotentialGenerator(),
    aligned_fg_otl_(this),
    aligned_fg_kdtree_otl_(this),
    index_otl_(index_otl),
    kdtree_otl_(kdtree_otl),
    prev_to_curr_otl_(prev_to_curr_otl),
    prev_seg_otl_(prev_seg_otl),
    pcd_indices_otl_(pcd_indices_otl),
    score_thresh_(score_thresh),
    distance_thresh_(distance_thresh),
    fringe_radius_(fringe_radius),
    sigma_dist_(sigma_dist),
    sigma_color_(sigma_color),
    delta_transform_thresh_(delta_transform_thresh),
    use_prev_bg_(use_prev_bg),
    skip_(skip),
    lookback_(lookback),
    aligned_fg_(new KinectCloud()),
    aligned_fg_kdtree_(new KdTree())
  {
    registerInput(index_otl_->getNode());
    registerInput(kdtree_otl_->getNode());
    registerInput(prev_to_curr_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());
    registerInput(pcd_indices_otl_->getNode());

    ROS_ASSERT(!lookback_.empty());
    max_buffer_size_ = 0;
    for(size_t i = 0; i < lookback_.size(); ++i)
      if(max_buffer_size_ < lookback_[i] + 1)
        max_buffer_size_ = lookback_[i] + 1;
  }

  double IcpNPG::runICP(KdTree& curr_kdtree,
                        const KinectCloud& curr_cloud,
                        KinectCloud::Ptr curr_fg,
                        Affine3f* final_transform) const
  {
    vector<Point> world_points;
    world_points.reserve(curr_fg->size());
    vector<Point> model_points;
    model_points.reserve(curr_fg->size());
    vector<int> indices(1);
    vector<float> distances(1);
    Eigen::Affine3f transform;
    double score;
    int iter = 0;
    KinectCloud original = *curr_fg;
    *final_transform = Affine3f::Identity();
    
    while(true) {
      score = 0;
      model_points.clear();
      world_points.clear();
      for(size_t i = 0; i < curr_fg->size(); ++i) {
        // indices.clear();
        // distances.clear();
        curr_kdtree.nearestKSearch(curr_fg->at(i), 1, indices, distances);
        if(distances[0] > distance_thresh_)
          continue;

        ++score;
        //int idx = curr_kdtree.getIndices()->at(indices[0]);
        int idx = indices[0];
        world_points.push_back(curr_cloud[idx]);
        model_points.push_back(curr_fg->at(i));
      }
      score /= (double)model_points.size();

      transform = SceneAlignmentNode::computeTransform(world_points, model_points);
      pcl::transformPointCloud(*curr_fg, *curr_fg, transform);
      *final_transform = transform * (*final_transform);
      
      double delta_transform = (transform.matrix() - Matrix4f::Identity()).norm();
      // cout << "--------------------" << endl;
      // cout << transform.matrix() << endl;
      // cout << "delta_transform: " << delta_transform << endl;
      // cout << "score: " << score << endl;
      if(delta_transform < delta_transform_thresh_)
        break;
      if(iter > 100) // TODO: Parameterize.
        break;
      ++iter;
    }
    
    return score;
  }

  void IcpNPG::addPotentials(const KinectCloud& object,
                             const KinectCloud& curr_cloud,
                             KdTree& curr_kdtree,
                             Eigen::MatrixXd* potentials) const
  {
    vector<int> indices(1);
    vector<float> distances(1);
    for(size_t i = 0; i < object.size(); ++i) {
      if(rand() % skip_ != 0)
        continue;
      
      curr_kdtree.nearestKSearch(object[i], 1, indices, distances);
      //int idx = curr_kdtree.getIndices()->at(indices[0]);
      int idx = indices[0];
      int y = idx / curr_cloud.width;
      int x = idx - y * curr_cloud.width;

      const Point& curr_pt = curr_cloud[idx];
      const Point& prev_pt = object[i];
      double dr = curr_pt.r - prev_pt.r;
      double dg = curr_pt.g - prev_pt.g;
      double db = curr_pt.b - prev_pt.b;
      double color_term = sqrt(dr*dr + dg*dg + db*db) / sigma_color_;
      double distance_term = distances[0] / sigma_dist_;
      potentials->coeffRef(y, x) = exp(-color_term - distance_term);
    }
  }

  void IcpNPG::computeForLookback(int lb)
  {
    //ScopedTimer st("IcpNPG::computeForLookback");
    
    // -- Transform older objects into the same pose as the foreground
    //    from the previous frame, then to the current frame.
    KinectCloud::Ptr prev_fg = fg_buffer_[lb];
    aligned_fg_->reserve(prev_fg->size());
    const Eigen::Affine3f& prev_to_curr = *prev_to_curr_otl_->pull();
    Affine3f old_to_prev_transform;
    if(lb == 1) { 
      *aligned_fg_ = *prev_fg;
    }
    else {
      Vector4f old_centroid;
      pcl::compute3DCentroid(*prev_fg, old_centroid);
      Vector4f prev_centroid;
      pcl::compute3DCentroid(*fg_buffer_[0], prev_centroid);
      Vector4f delta = prev_centroid - old_centroid;
      old_to_prev_transform = Translation3f(delta.head(3));
      pcl::transformPointCloud(*prev_fg, *aligned_fg_, old_to_prev_transform);
    }

    // If there are no points, then forget about it.
    if(aligned_fg_->empty())
      return;
    
    // Use scene alignment for the final step.
    pcl::transformPointCloud(*aligned_fg_, *aligned_fg_, prev_to_curr); 
    
    // -- Run ICP to find best location of the object.
    KdTree& curr_kdtree = *kdtree_otl_->pull().current_kdtree_;
    const KinectCloud& curr_cloud = *index_otl_->pull().current_pcd_;
    Affine3f final_transform;
    double score = runICP(curr_kdtree, curr_cloud, aligned_fg_, &final_transform);

    // If it's a bad match, don't return anything.
    if(score < score_thresh_)
      return;

    // -- Make a kdtree and push the aligned foreground obj + kdtree out.
    aligned_fg_otl_.push(aligned_fg_);
    // cout << "aligned_fg_ has " << aligned_fg_->size() << " points." << endl;
    // for(size_t i = 0; i < aligned_fg_->size(); ++i)
    //   cout << (*aligned_fg_)[i] << endl;
    aligned_fg_kdtree_->setInputCloud(aligned_fg_);
    aligned_fg_kdtree_otl_.push(aligned_fg_kdtree_);
    
    // Add source potentials.
    addPotentials(*aligned_fg_, curr_cloud, curr_kdtree, &source_potentials_);

    // -- Make the older background fringe follow the foreground.
    if(use_prev_bg_) {
      KinectCloud::Ptr prev_bg = bg_buffer_[lb];
      KinectCloud::Ptr curr_bg(new KinectCloud());
      curr_bg->reserve(prev_bg->size());
      if(lb == 1) {
        *curr_bg = *prev_bg;
      }
      else { 
        pcl::transformPointCloud(*prev_bg, *curr_bg, old_to_prev_transform);
      }
      // Use ICP transform from foreground object to make the final placement.
      pcl::transformPointCloud(*curr_bg, *curr_bg, final_transform);
      
      addPotentials(*curr_bg, curr_cloud, curr_kdtree, &sink_potentials_);
    }
  }
  
  void IcpNPG::_compute()
  {
    // -- Don't compute if no kdtree for this or the last frame.
    //    This happens on the first frame and when the mask is empty.
    if(!kdtree_otl_->pull().previous_kdtree_ ||
       !kdtree_otl_->pull().current_kdtree_)
      return;
        
    // -- Reallocate potentials if necessary.
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    ROS_ASSERT(prev_seg.rows > 0);
    if(source_potentials_.rows() != prev_seg.rows ||
       source_potentials_.cols() != prev_seg.cols) {
      source_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
      sink_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
    }
    ROS_ASSERT(source_potentials_.rows() > 0 && source_potentials_.cols() > 0);
    ROS_ASSERT(sink_potentials_.rows() > 0 && sink_potentials_.cols() > 0);

    // -- Get foreground and fringe, add to buffers.
    updateBuffers();
        
    // -- Add potentials for specified previous objects.
    for(size_t i = 0; i < lookback_.size(); ++i)
      if((int)fg_buffer_.size() > lookback_[i])
        computeForLookback(lookback_[i]);
    
    // -- Fill the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void IcpNPG::updateBuffers()
  {
    //ScopedTimer st("IcpNPG::updateBuffers");
    
    // -- Get the previous foreground segment.
    const KinectCloud& prev_cloud = *index_otl_->pull().previous_pcd_;
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    cv::Mat1i prev_index = index_otl_->pull().previous_index_;

    KinectCloud::Ptr prev_fg(new KinectCloud());
    prev_fg->reserve(prev_cloud.size());
    for(int y = 0; y < prev_seg.rows; ++y) {
      for(int x = 0; x < prev_seg.cols; ++x) {
        if(prev_seg(y, x) != 255)
          continue;

        int idx = prev_index(y, x);
        if(idx == -1)
          continue;
        
        prev_fg->push_back(prev_cloud[idx]);
      }
    }

    // Add to buffer.
    fg_buffer_.push_front(prev_fg);
    if(fg_buffer_.size() == (size_t)(max_buffer_size_ + 1))
      fg_buffer_.pop_back();

    // -- Find BG points from previous frame that were near the object.
    if(use_prev_bg_) {
      //ScopedTimer st("IcpNPG::updateBuffers, using prev bg");
      ROS_ASSERT(kdtree_otl_->pull().previous_kdtree_);
      KdTree& prev_kdtree = *kdtree_otl_->pull().previous_kdtree_;
      const vector<cv::Point2i>& prev_rindex = *index_otl_->pull().previous_rindex_;
      vector<int> indices;
      vector<float> distances;
      vector<bool> marked(prev_cloud.size(), false);
      int num_bg = 0;
      for(size_t i = 0; i < prev_fg->size(); ++i) {
        indices.clear();
        distances.clear();
        prev_kdtree.radiusSearch(prev_fg->at(i), fringe_radius_, indices, distances);

        for(size_t j = 0; j < indices.size(); ++j) {
          // if(indices[j] >= (int)prev_kdtree.getIndices()->size()) { 
          //   cout << "----" << endl;
          //   cout << indices.size() << endl;
          //   cout << prev_kdtree.getIndices()->size() << endl;
          //   cout << j << endl;
          //   cout << indices[j] << endl;
          //   cout << prev_rindex.size() << endl;
          // }  

          ///int idx = prev_kdtree.getIndices()->at(indices[j]);
          int idx = indices[j];
          if(marked[idx] || prev_seg(prev_rindex[idx]) != 0)
            continue;

          marked[idx] = true;
          ++num_bg;
        }
      }
      
      KinectCloud::Ptr bg(new KinectCloud());
      bg->reserve(num_bg);
      for(size_t i = 0; i < marked.size(); ++i) {
        if(!marked[i])
          continue;
        bg->push_back(prev_cloud[i]);
      }

      // Add to buffer.
      bg_buffer_.push_front(bg);
      if((int)bg_buffer_.size() == max_buffer_size_ + 1)
        bg_buffer_.pop_back();
    }
  }
  
  void IcpNPG::_display() const
  {
    if(!kdtree_otl_->pull().previous_kdtree_)
      return;
    displayNodePotentials(index_otl_->pull().current_img_);
  }

  void IcpNPG::_reset()
  {
    fg_buffer_.clear();
    bg_buffer_.clear();
  }
  
  void IcpNPG::_flush()
  {
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
    source_potentials_.setZero();
    sink_potentials_.setZero();

    aligned_fg_->clear();
    aligned_fg_otl_.push(KinectCloud::Ptr());
    aligned_fg_kdtree_otl_.push(KdTree::Ptr());
  }

  std::string IcpNPG::_getName() const
  {
  ostringstream oss;
  oss << "IcpNPG_score_thresh:" << score_thresh_ << "_distance_thresh:"
      << distance_thresh_ << "_fringe_radius:" << fringe_radius_ << "_sigma_dist:"
      << sigma_dist_ << "_sigma_color:" << sigma_color_ << "_dtrans_thresh:"
      << delta_transform_thresh_ << "_use_prev_bg:" << use_prev_bg_
      << "_lookback:";

  oss << lookback_[0];
  for(size_t i = 1; i < lookback_.size(); ++i)
    oss << "," << lookback_[i];
  
  return oss.str();
  }

} // namespace dst
