#ifndef PRIOR_NPG_H
#define PRIOR_NPG_H

#include <dst/node_potential_generator.h>

namespace dst
{
  
  class PriorNPG : public NodePotentialGenerator
  {
  public:
    PriorNPG(pipeline2::Outlet<cv::Mat3b>* image_otl);
    
  protected:
    pipeline2::Outlet<cv::Mat3b>* image_otl_;

    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // PRIOR_NPG_H
