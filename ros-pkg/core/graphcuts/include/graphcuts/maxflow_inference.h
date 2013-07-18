#ifndef GRAPHCUTS_MAXFLOW_INFERENCE_H
#define GRAPHCUTS_MAXFLOW_INFERENCE_H

#include <timer/timer.h>
#include <graphcuts/potentials_cache.h>
#include <graphcuts/model.h>

namespace gc
{
  
  class MaxflowInference
  {
  public:
    Model model_;
    
    MaxflowInference(const Model& model);
    //! seg is resized if necessary.
    //! Output is in {-1, 0, +1}^n.
    void segment(PotentialsCache::ConstPtr potentials, Eigen::VectorXi* seg) const;
  };
  
}

#endif // GRAPHCUTS_MAXFLOW_INFERENCE_H
