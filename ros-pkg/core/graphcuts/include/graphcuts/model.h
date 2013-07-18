#ifndef GRAPHCUTS_MODEL_H
#define GRAPHCUTS_MODEL_H

#include <serializable/serializable.h>
#include <name_mapping/name_mapping.h>
#include <eigen_extensions/eigen_extensions.h>

namespace gc
{

  class Model : public Serializable, public NameMappable
  {
  public:
    Eigen::VectorXd nweights_;
    Eigen::VectorXd eweights_;
    
    Model();
    Model(const Eigen::VectorXd& nweights,
          const Eigen::VectorXd& eweights,
          const NameMapping& nmap,
          const NameMapping& emap);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    //! Edge, then node potentials.
    double score(const Eigen::VectorXd& psi) const;
    //double score(const VectorXd& edge_psi, const Eigen::VectorXd& node_psi) const;
    int size() const { return eweights_.rows() + nweights_.rows(); }
    Eigen::VectorXd concatenate() const;

  protected:
    //! "nmap" or "emap"
    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
  };
  
}

#endif // GRAPHCUTS_MODEL_H
