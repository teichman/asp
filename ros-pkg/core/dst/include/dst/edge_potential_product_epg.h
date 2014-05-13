#ifndef EDGE_POTENTIAL_PRODUCT_EPG_H
#define EDGE_POTENTIAL_PRODUCT_EPG_H

#include <dst/edge_potential_generator.h>
#include <dst/helper_functions.h>

namespace dst
{

  class EdgePotentialProduct : public EdgePotentialGenerator
  {
  public:
    EdgePotentialProduct(pipeline2::Outlet<cv::Mat3b>* img_otl,
                         const std::vector<EdgePotentialGenerator*>& generators);

  protected:
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    std::vector<EdgePotentialGenerator*> generators_;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // EDGE_POTENTIAL_PRODUCT_EPG_H
