#ifndef FLOW_CLUSTERING_EPG_H
#define FLOW_CLUSTERING_EPG_H

#include <pcl/registration/transformation_estimation_svd.h>
#include <dst/optical_flow_node.h>
#include <dst/edge_potential_generator.h>

namespace dst
{

  class FlowClusteringEPG : public EdgePotentialGenerator
  {
  public:
    FlowClusteringEPG(pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
                      pipeline2::Outlet<DepthProjector::Output>* index_otl,
                      float inlier_thresh);
    
  protected:
    pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    float inlier_thresh_;
    cv::Mat1i cluster_index_;
    std::vector< std::vector<int> > clusters_;
    size_t max_cluster_size_;
    size_t max_cluster_;

    void sampleFlows(const std::vector<bool>& active,
                     const std::vector<int>& prev_pcd_indices,
                     const std::vector<int>& pcd_indices,
                     std::vector<int>* prev_sample_indices,
                     std::vector<int>* sample_indices);

    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // FLOW_CLUSTERING_EPG_H
