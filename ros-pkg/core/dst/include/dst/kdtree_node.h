#ifndef KDTREE_NODE_H
#define KDTREE_NODE_H

#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>

namespace dst
{

  class KdTreeNode : public pipeline2::ComputeNode
  {
  public:
    typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;
    typedef struct
    {
      KdTree::Ptr current_kdtree_;
      KdTree::Ptr previous_kdtree_;
      KinectCloud::ConstPtr current_pcd_;
      KinectCloud::ConstPtr previous_pcd_;
    } Output;

    pipeline2::Outlet<Output> kdtree_otl_;

    KdTreeNode(pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl,
               pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl,
               double radius);
    ~KdTreeNode();

  protected:
    pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl_;
    pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl_;
    double radius_;
    KdTree::Ptr current_kdtree_;
    KdTree::Ptr previous_kdtree_;
    KinectCloud::ConstPtr current_pcd_;
    KinectCloud::ConstPtr previous_pcd_;

    void swap();
    void _compute();
    void _display() const;
    void _flush();
    //! Clears the cached pointclouds and trees.
    void _reset();
    std::string _getName() const;
  };
  
}

#endif // KDTREE_NODE_H
