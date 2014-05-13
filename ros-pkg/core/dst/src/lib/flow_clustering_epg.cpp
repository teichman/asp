#include <dst/flow_clustering_epg.h>


using namespace std;
using namespace Eigen;

namespace dst
{

  FlowClusteringEPG::FlowClusteringEPG(pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
                                       pipeline2::Outlet<DepthProjector::Output>* index_otl,
                                       float inlier_thresh) :
    EdgePotentialGenerator(),
    optflow_otl_(optflow_otl),
    index_otl_(index_otl),
    inlier_thresh_(inlier_thresh),
    max_cluster_size_(0),
    max_cluster_(0)
  {
    registerInput(optflow_otl_->getNode());
    registerInput(index_otl_->getNode());
  }

  void FlowClusteringEPG::sampleFlows(const std::vector<bool>& active,
                                      const std::vector<int>& prev_pcd_indices,
                                      const std::vector<int>& pcd_indices,
                                      std::vector<int>* prev_sample_indices,
                                      std::vector<int>* sample_indices)
  {
    ROS_ASSERT(prev_pcd_indices.size() == pcd_indices.size());
    ROS_ASSERT(sample_indices->size() == 3);
    ROS_ASSERT(prev_sample_indices->size() == 3);
    
    set<int> sample;
    while(sample.size() < 3) {
      int s = rand() % active.size();
      if(active[s])
        sample.insert(s);
    }

    set<int>::iterator it;
    int i = 0;
    for(it = sample.begin(); it != sample.end(); ++it, ++i) {
      prev_sample_indices->at(i) = prev_pcd_indices[*it];
      sample_indices->at(i) = pcd_indices[*it];
    }
  }
  
  void FlowClusteringEPG::_compute()
  {
    if(!optflow_otl_->pull().prev_pcd_indices_) {
      edge_otl_.push(NULL);
      return;
    }

    cv::Mat3b img = optflow_otl_->pull().img_;
    initializeStorage(img.rows * img.cols, 0.10);

    if(cluster_index_.rows == 0)
      cluster_index_ = cv::Mat1i(img.size(), -1);
    
    const vector<int>& prev_pcd_indices = *optflow_otl_->pull().prev_pcd_indices_;
    const vector<int>& pcd_indices = *optflow_otl_->pull().pcd_indices_;
    const KinectCloud& prev_cloud = *index_otl_->pull().previous_pcd_;
    const KinectCloud& cloud = *index_otl_->pull().current_pcd_;
    ROS_ASSERT(prev_pcd_indices.size() == pcd_indices.size());

    int iter = 0;
    int max_iter = 500; // TODO: Parameterize.
    vector<int> prev_sample(3);
    vector<int> sample(3);
    vector<bool> active(pcd_indices.size(), true);
    while(iter < max_iter) {
      ++iter;
      
      sampleFlows(active, prev_pcd_indices, pcd_indices, &prev_sample, &sample);
      Eigen::Matrix4f transform;
      pcl::registration::TransformationEstimationSVD<Point, Point> te;
      te.estimateRigidTransformation(prev_cloud, prev_sample, cloud, sample, transform);

      // -- See if this transform made any sense at all.
      size_t num_inliers = 0;
      for(size_t i = 0; i < sample.size(); ++i) {
        Vector3f projected = (transform * prev_cloud[prev_sample[i]].getVector4fMap()).head(3);
        float dist = (projected - cloud[sample[i]].getVector3fMap()).norm();
        if(dist < inlier_thresh_)
          ++num_inliers;
      }
      if(num_inliers != 3)
        continue;

      // -- Refine the estimate.
      
      // -- Get inlier flows.
      vector<bool> inliers(pcd_indices.size(), false);
      num_inliers = 0;
      for(size_t i = 0; i < pcd_indices.size(); ++i) {
        if(!active[i])
          continue;
        Vector3f projected = (transform * prev_cloud[prev_pcd_indices[i]].getVector4fMap()).head(3);
        float dist = (projected - cloud[pcd_indices[i]].getVector3fMap()).norm();
        if(dist < inlier_thresh_) {
          inliers[i] = true;
          active[i] = false;
          ++num_inliers;
        }
      }
      //cout << "FC: Found cluster with " << num_inliers << " inliers out of " << pcd_indices.size() << endl;
      
      // -- Link inliers with edge potentials.      
      clusters_.push_back(vector<int>());
      vector<int>& cluster = clusters_.back();
      cluster.reserve(num_inliers);
      size_t id = clusters_.size() - 1;
      for(size_t i = 0; i < inliers.size(); ++i) {
        if(!inliers[i])
          continue;
        cluster.push_back(pcd_indices[i]); // Assumes ordered ptcld.

        // Assumes ordered point cloud.
        ROS_ASSERT((size_t)img.cols == prev_cloud.width);
        int y = prev_pcd_indices[i] / img.cols;
        int x = prev_pcd_indices[i] - y * img.cols;
        cluster_index_(y, x) = id;
      }
      sort(cluster.begin(), cluster.end());
      
      // Make a note if this is the one to ignore.
      if(num_inliers > max_cluster_size_) { 
        max_cluster_size_ = num_inliers;
        max_cluster_ = id;
      }
      
      // -- Break if few active flows remaining.
      int num_active = 0;
      for(size_t i = 0; i < active.size(); ++i)
        if(active[i])
          ++num_active;
      if(num_active < 3)
        break;
    }

    //cout << "FC: total clusters = " << clusters_.size() << endl;

    // -- potentials_ is a sparse matrix, so add links in order.
    // cluster_index_(y, x) is the id of the cluster this pixel is in.
    // clusters_[i] is a sorted (ascending) vector of indices into the image.
    for(int y = 0; y < cluster_index_.rows; ++y) {
      for(int x = 0; x < cluster_index_.cols; ++x) {
        int idx0 = y * cluster_index_.cols + x;
        potentials_.startVec(idx0);
        if(cluster_index_(y, x) == -1)
          continue;
        // Ignore the biggest cluster.
        if(cluster_index_(y, x) == (int)max_cluster_)
          continue;

        // TODO: Gross.
        const vector<int>& cluster = clusters_[cluster_index_(y, x)];
        if(cluster.size() < 5) {
          for(size_t i = 0; i < cluster.size(); ++i) { 
            int idx1 = cluster[i];
            if(i > 0) { 
              ROS_ASSERT(cluster[i] > cluster[i-1]);
            //cout << "FC: Adding edge between " << idx0 << " and " << idx1 << endl;
            }
            if(idx0 != idx1)
              potentials_.insertBack(idx0, idx1) = 1.0;
          }
        }
        else { 
          for(int i = 0; i < 1; ++i) { 
            int r = rand() % cluster.size();
            int idx1 = cluster[r];
            //cout << "FC: Adding edge between " << idx0 << " and " << idx1 << endl;
            if(idx0 != idx1)
              potentials_.insertBack(idx0, idx1) = 1.0;
          }
        }
      }
    }

    potentials_.finalize();
    edge_otl_.push(&potentials_);
  }

  void FlowClusteringEPG::_display() const
  {
    if(!optflow_otl_->pull().prev_pcd_indices_)
      return;

    displayEdges(optflow_otl_->pull().img_);
  }

  void FlowClusteringEPG::_flush()
  {
    max_cluster_ = 0;
    max_cluster_size_ = 0;
    potentials_.setZero();
    clusters_.clear();
    if(cluster_index_.rows > 0)
      cluster_index_ = -1;
    edge_otl_.push(NULL);
  }

  std::string FlowClusteringEPG::_getName() const
  {
    ostringstream oss;
    oss << "FlowClusteringEPG_inlier_thresh:" << inlier_thresh_;
    return oss.str();
  }
  
} // namespace dst
