#ifndef GRAPHCUTS_POTENTIALS_CACHE_H
#define GRAPHCUTS_POTENTIALS_CACHE_H

#include <ros/assert.h>
#include <ros/console.h>
#include <graphcuts/typedefs.h>
#include <graphcuts/model.h>

namespace gc
{

  //! Represents a function that maps segmentations to score vectors
  //! i.e. \Psi_x : seg -> R^n.
  //! w^T \Psi_x(y) is the score for segmentation y on frame with features x.
  class PotentialsCache : public NameMappable
  {
  public:
    typedef boost::shared_ptr<PotentialsCache> Ptr;
    typedef boost::shared_ptr<const PotentialsCache> ConstPtr;

    
    //! In order of weights.
    std::vector<SparseMat> edge_;
    //! In order of weights.
    std::vector<Eigen::VectorXd> node_;
    
    long int bytes() const;
    int numNodes() const;
    int numPotentials() const;
    int numNodePotentials() const;
    int numEdgePotentials() const { return edge_.size(); }


    //! weights.dot(psi()) is the score for a given segmentation.
    //! Concatenates edge, then node potentials.
    //! seg must be in {-1, +1}^n.  Labels of "unknown", i.e. 0, are not allowed.
    Eigen::VectorXd psi(const Eigen::VectorXi& seg) const;

    void symmetrizeEdges();
    void applyWeights(const Model& model,
                      SparseMat* edge,
                      Eigen::VectorXd* node) const;
  protected:
    //! "nmap" or "emap"
    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
  };

}

#endif // GRAPHCUTS_POTENTIALS_CACHE_H
