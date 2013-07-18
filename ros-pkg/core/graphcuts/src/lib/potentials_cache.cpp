#include <graphcuts/potentials_cache.h>

using namespace Eigen;

namespace gc
{

  long int PotentialsCache::bytes() const
  {
    long int bytes = 0;
    for(size_t i = 0; i < edge_.size(); ++i)
      bytes += sizeof(double) * edge_[i].nonZeros();
    for(size_t i = 0; i < node_.size(); ++i) { 
      bytes += sizeof(double) * node_[i].rows() * node_[i].cols();
    }
    
    return bytes;
  }

  int PotentialsCache::numNodes() const
  {
    ROS_ASSERT(!node_.empty());
    return node_[0].rows();
  }
  
  int PotentialsCache::numPotentials() const
  {
    return numNodePotentials() + numEdgePotentials();
  }

  int PotentialsCache::numNodePotentials() const
  {
    return node_.size();
  }

  void PotentialsCache::symmetrizeEdges()
  {
    // See http://eigen.tuxfamily.org/api/TutorialSparse.html
    for(size_t i = 0; i < edge_.size(); ++i)
      edge_[i] = 0.5 * (SparseMat(edge_[i].transpose()) + edge_[i]);
  }
  
  void PotentialsCache::applyWeights(const Model& model,
                                     SparseMat* edge,
                                     Eigen::VectorXd* node) const
  {
    ROS_ASSERT(node->rows() == edge->rows());
    ROS_ASSERT(node->rows() == edge->cols());
    ROS_ASSERT(model.nweights_.rows() == numNodePotentials());
    ROS_ASSERT(model.eweights_.rows() == numEdgePotentials());
    ROS_ASSERT(!node_.empty());
    ROS_ASSERT(!edge_.empty());

    node->setZero();
    edge->setZero();
    for(size_t i = 0; i < node_.size(); ++i)
      *node += model.nweights_(i) * node_[i];
    for(size_t i = 0; i < edge_.size(); ++i)
      *edge += model.eweights_(i) * edge_[i];
  }
  
  Eigen::VectorXd PotentialsCache::psi(const Eigen::VectorXi& seg) const
  {
    VectorXd psi = VectorXd::Zero(numPotentials()); // edge, then node.
    
    // -- Add up scores for edge potentials.
    for(size_t i = 0; i < edge_.size(); ++i) {
      const SparseMat& ep = edge_[i];
      for(int j = 0; j < ep.outerSize(); ++j) {
            for(SparseMat::InnerIterator it(ep, j); it; ++it) {
              if(it.col() <= it.row())
                continue;
          if(seg(it.row()) == seg(it.col()))
            psi(i) += it.value();
        }
      }
    }

    // -- Add up scores for node potentials.
    for(size_t i = 0; i < node_.size(); ++i) {
      int idx = i + edge_.size();
      ROS_ASSERT(node_[i].size() == seg.rows());
      for(int j = 0; j < seg.rows(); ++j)
        if(seg(j) == 1)
          psi(idx) += node_[i][j];
    }
     
    return psi;
  }

  void PotentialsCache::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
  {
    ROS_ASSERT(id == "emap" || id == "nmap");
    if(id == "emap") {
      translator.translate(&edge_, SparseMat());
    }
    else if(id == "nmap") {
      translator.translate(&node_, VectorXd());
    }
  }
}
