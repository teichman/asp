#ifndef DISTANCE_NPG_H
#define DISTANCE_NPG_H

#include <dst/typedefs.h>
#include <dst/node_potential_generator.h>

namespace dst
{

  class DistanceNPG : public NodePotentialGenerator
  {
  public:
    typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;
    
    DistanceNPG(pipeline2::Outlet<KinectCloud::ConstPtr>* transformed_otl,
                pipeline2::Outlet<KdTree::Ptr>* fg_kdtree_otl,
                pipeline2::Outlet<cv::Mat3b>* img_otl,
                pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl,
                float sigma);
    
  protected:
    pipeline2::Outlet<KinectCloud::ConstPtr>* transformed_otl_;
    pipeline2::Outlet<KdTree::Ptr>* fg_kdtree_otl_;
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl_;
    float sigma_;

    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // DISTANCE_NPG_H
