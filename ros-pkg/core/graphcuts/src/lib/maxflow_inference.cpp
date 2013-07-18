#include <graphcuts/maxflow_inference.h>

using namespace Eigen;
using namespace std;

namespace gc
{

  MaxflowInference::MaxflowInference(const Model& model) :
    model_(model)
  {
    // for(int i = 0; i < model_.weights_.rows(); ++i)
    //   ROS_ASSERT(model_.weights_(i) > 0);
  }
  
  void MaxflowInference::segment(PotentialsCache::ConstPtr pc,
                                 Eigen::VectorXi* seg) const
  {
    // -- Check for monkey business and allocate.
    ROS_ASSERT(pc->numEdgePotentials() == model_.eweights_.rows());
    ROS_ASSERT(pc->numNodePotentials() == model_.nweights_.rows());

    if(seg->rows() != pc->numNodes())
      *seg = VecXi::Zero(pc->numNodes());

    for(size_t i = 0; i < pc->node_.size(); ++i) 
      ROS_ASSERT(seg->rows() == pc->node_[i].rows());

    int num_edges = 0;
    for(size_t i = 0; i < pc->edge_.size(); ++i) { 
      ROS_ASSERT(seg->rows() == pc->edge_[i].rows());
      ROS_ASSERT(seg->rows() == pc->edge_[i].cols());
      num_edges += pc->edge_[i].nonZeros();
    }
    
    Graph3d graph(seg->rows(), num_edges);
    graph.add_node(seg->rows());
                        

    Eigen::VectorXd weighted_node(seg->rows());
    SparseMat weighted_edge(seg->rows(), seg->rows());
    pc->applyWeights(model_, &weighted_edge, &weighted_node);
    
    // -- Fill the graph with weighted node potentials.
    for(int i = 0; i < weighted_node.rows(); ++i)
     graph.add_tweights(i, weighted_node(i), 0);
    
    // -- Fill the graph with edge potentials.  Assumes symmetry & upper triangular.
    for(int i = 0; i < weighted_edge.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(weighted_edge, i); it; ++it) {
        if(it.col() <= it.row())
          continue;

        ROS_WARN_STREAM_COND(it.value() < 0, "Edgepot weighted sum is negative: " << it.value());
        ROS_FATAL_STREAM_COND(isnan(it.value()), "NaN in edgepot.");
        graph.add_edge(it.row(), it.col(), it.value(), it.value());
      }
    }

    // -- Run graph cuts.
    HighResTimer hrt("maxflow");
    hrt.start();
    graph.maxflow();
    hrt.stop();
    //cout << hrt.reportMilliseconds() << endl;

    // -- Fill the segmentation.
    for(int i = 0; i < seg->rows(); ++i) {
      if(graph.what_segment(i, Graph3d::SINK) == Graph3d::SINK)
        seg->coeffRef(i) = -1;
      else
        seg->coeffRef(i) = 1;
    }
  }
  
}
