#ifndef STRUCTURAL_SVM_H
#define STRUCTURAL_SVM_H

#include <boost/filesystem.hpp>
#include <optimization/nips.h>
#include <optimization/common_functions.h>
#include <eigen_extensions/eigen_extensions.h>
#include <bag_of_tricks/agent.h>
#include <graphcuts/maxflow_inference.h>

namespace gc
{
  class Constraint;
  class ConstraintGenerator;
  
  class StructuralSVM
  {
  public:
    double c_;
    double precision_;
    int num_threads_;
    int debug_level_;
    
    StructuralSVM(double c, double precision, int num_threads, int debug_level);
    //! labels must all be in {-1, +1}^n.
    Model train(const std::vector<PotentialsCache::Ptr>& caches,
                const std::vector<VecXiPtr>& labels) const;

  protected:
    double updateModel(const std::vector<Constraint>& constraints,
                       Model* model, double* slacks) const;
    
    friend class ConstraintGenerator;
  };
  
  class Constraint
  {
  public:
    Constraint() :
      loss_(-1)
      {
      }
    
    Eigen::VectorXd dpsi_;
    double loss_;
  };

  class ConstraintGenerator : public Agent
  {
  public:
    Constraint con_;
    double hamming_loss_;
    
    ConstraintGenerator(const Model& model,
                        PotentialsCache::ConstPtr cache,
                        VecXiConstPtr labels);
    void _run();

  protected:
    Model model_;
    PotentialsCache::ConstPtr cache_;
    VecXiConstPtr labels_;

    ConstraintGenerator& operator=(const ConstraintGenerator& other);
    ConstraintGenerator(const ConstraintGenerator& other);
  };

  double hammingLoss(const Eigen::VectorXi& label,
                     const Eigen::VectorXi& pred);
  double zeroOneLoss(const Eigen::VectorXi& label,
                     const Eigen::VectorXi& pred);
  
}

#endif // STRUCTURAL_SVM_H
