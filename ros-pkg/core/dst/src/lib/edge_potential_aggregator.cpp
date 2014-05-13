#include <dst/edge_potentials.h>

using namespace std;
using namespace Eigen;

namespace dst
{
  
  EdgePotentialAggregator::EdgePotentialAggregator(pipeline2::Outlet<Graph3dPtr>* graph_otl,
                                                   pipeline2::Outlet<cv::Mat3b>* img_otl,
                                                   pipeline2::Outlet<DepthProjector::Output>* index_otl,
                                                   const std::vector<EdgePotentialGenerator*>& generators,
                                                   const Eigen::VectorXd& weights,
                                                   bool ignore_depthless,
                                                   pipeline2::Outlet< Eigen::SparseMatrix<double, Eigen::RowMajor>* >* weights_otl) :
    graph_otl_(graph_otl),
    img_otl_(img_otl),
    index_otl_(index_otl),
    generators_(generators),
    weights_(weights),
    ignore_depthless_(ignore_depthless),
    weights_otl_(weights_otl)
  {
    registerInput(graph_otl_->getNode());
    registerInput(img_otl_->getNode());
    registerInput(index_otl_->getNode());
    if(weights_otl_)
      registerInput(weights_otl_->getNode()); // TODO: What happens when you register an input twice?
    
    for(size_t i = 0; i < generators_.size(); ++i) { 
      registerInput(generators_[i]);
    }
  }

  NameMapping EdgePotentialAggregator::getNameMapping() const
  {
    NameMapping epot_names;
    for(size_t i = 0; i < generators_.size(); ++i)
      epot_names.addName(generators_[i]->getShortName());

    return epot_names;
  }
  
  std::string EdgePotentialAggregator::weightsStatus() const
  {
    ostringstream oss;
    for(size_t i = 0; i < generators_.size(); ++i) { 
      oss << setiosflags(ios::left) << setw(10) << weights_[i] << "\t"
          << setiosflags(ios::left) << setw(100) << generators_[i]->getShortName() << endl;
    }
    return oss.str();
  }
  
  void EdgePotentialAggregator::setWeights(const Eigen::VectorXd& weights, bool verbose)
                                           
  {
    ROS_ASSERT((size_t)weights.rows() == generators_.size());
    weights_ = weights;

    if(verbose) {
      cout << weightsStatus() << endl;
    }
  }

  void EdgePotentialAggregator::ignoreDepthless(const SparseMatrix<double, RowMajor>& pot,
                                                SparseMatrix<double, RowMajor>* sanitized,
                                                const std::string& name) const
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    
    *sanitized = SparseMatrix<double, RowMajor>(pot.rows(), pot.cols());
    for(int i = 0; i < pot.outerSize(); ++i) { 
      SparseMatrix<double, RowMajor>::InnerIterator it(pot, i);
      sanitized->startVec(i);
      for(; it; ++it) {
        ROS_ASSERT(it.row() == i);
        
        int idx1 = it.row();
        int y1 = idx1 / index.cols;
        int x1 = idx1 - y1 * index.cols;
        int idx2 = it.col();
        int y2 = idx2 / index.cols;
        int x2 = idx2 - y2 * index.cols;
        if(index(y1, x1) == -1 || index(y2, x2) == -1)
          continue;
        sanitized->insertBack(it.row(), it.col()) = it.value();
        ROS_WARN_STREAM_COND(it.value() < 0, "Negative edge weight of " << it.value() << " in " << name);
        ROS_WARN_STREAM_COND(isnan(it.value()), "NaN in " << name);
        ROS_WARN_STREAM_COND(isinf(it.value()), "inf in " << name);
      }
    }
  }
    
  void EdgePotentialAggregator::cacheUnweightedPotentials(FramePotentialsCache::Ptr framecache) const
  {
    ROS_ASSERT(!weights_otl_);
    ROS_ASSERT(framecache->edge_potentials_.empty());
    ROS_ASSERT(ignore_depthless_);
    
    for(size_t i = 0; i < generators_.size(); ++i) {
      ROS_ASSERT(generators_[i]);
      if(!generators_[i]->edge_otl_.pull())
        continue;

      SparseMatrix<double, RowMajor> gen;
      ignoreDepthless(*generators_[i]->edge_otl_.pull(), &gen, generators_[i]->getFullName());
      framecache->edge_potentials_.push_back(gen);
    }
  }
  
  void EdgePotentialAggregator::_compute()
  {
    ROS_ASSERT(!weights_otl_);
    
    // -- Allocate more space if necessary.
    int num_nodes = graph_otl_->pull()->get_node_num();
    potentials_ = SparseMatrix<double, RowMajor>(num_nodes, num_nodes);
    potentials_.reserve(10 * num_nodes);

    // -- Weighted sum.
    if(weights_otl_) {
      SparseMatrix<double, RowMajor>& w = *weights_otl_->pull();
      for(size_t i = 0; i < generators_.size(); ++i) {
        if(!generators_[i]->edge_otl_.pull())
          continue;

        SparseMatrix<double, RowMajor>& inc = *generators_[i]->edge_otl_.pull();
        if(inc.nonZeros() == 0)
          continue;

        SparseMatrix<double, RowMajor> gen;
        ignoreDepthless(inc, &gen, generators_[i]->getFullName());
        if(generators_[i] == weights_otl_->getNode())
          potentials_ += weights_[i] * gen;
        else
          potentials_ += weights_[i] * gen.cwiseProduct(w);
      }
    }
    else { 
      for(size_t i = 0; i < generators_.size(); ++i) {
        if(!generators_[i]->edge_otl_.pull())
          continue;
        
        SparseMatrix<double, RowMajor> gen;
        ignoreDepthless(*generators_[i]->edge_otl_.pull(), &gen, generators_[i]->getFullName());
        potentials_ += weights_[i] * gen;
      }
    }
    
    edge_otl_.push(&potentials_);
  }

  void EdgePotentialAggregator::_display() const
  {
    displayEdges(img_otl_->pull());
  }

  void EdgePotentialAggregator::_flush()
  {
    potentials_.setZero();
    edge_otl_.push(NULL);
  }

  std::string EdgePotentialAggregator::_getName() const
  {
    return "EdgePotentialAggregator";
  }
  
} // namespace dst
