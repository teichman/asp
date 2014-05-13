#include <dst/kdtree_node.h>

using namespace std;

namespace dst
{

  KdTreeNode::KdTreeNode(pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl,
                         pipeline2::Outlet<IndicesConstPtr>* pcd_indices_otl,
                         double radius) :
    ComputeNode(),
    kdtree_otl_(this),
    pcd_otl_(pcd_otl),
    pcd_indices_otl_(pcd_indices_otl),
    radius_(radius),
    current_kdtree_(KdTree::Ptr((KdTree*)NULL)),
    previous_kdtree_(KdTree::Ptr((KdTree*)NULL)),
    current_pcd_(KinectCloud::Ptr((KinectCloud*)NULL)),
    previous_pcd_(KinectCloud::Ptr((KinectCloud*)NULL))
  {
    registerInput(pcd_otl_->getNode());
    registerInput(pcd_indices_otl_->getNode());
  }

  KdTreeNode::~KdTreeNode()
  {
  }

  void KdTreeNode::swap()
  {
    //KdTree::Ptr tmp = previous_kdtree_;
    previous_kdtree_ = current_kdtree_;
    //current_kdtree_ = tmp;
    current_kdtree_ = KdTree::Ptr(new KdTree(false));

    previous_pcd_ = current_pcd_;
    current_pcd_ = KinectCloud::Ptr((KinectCloud*)NULL);
  }

  void KdTreeNode::_compute()
  {
    swap();
    
    current_pcd_ = pcd_otl_->pull();
    if(!current_kdtree_)
      current_kdtree_ = KdTree::Ptr(new KdTree(false));

    if(pcd_indices_otl_->pull()->empty())
      current_kdtree_ = KdTree::Ptr((KdTree*)NULL);
    else {
      //cout << "KdTreeNode: Building a scene kdtree with " << pcd_indices_otl_->pull()->size() << " points." << endl;
      IndicesConstPtr copy(new vector<int>(*pcd_indices_otl_->pull()));
      current_kdtree_->setInputCloud(current_pcd_, copy);
    }

    // -- nearestKSearch returns indices that index into pcd_indices, which then index into the pcd.  ... Why?

    // const vector<int>& pcd_indices = *pcd_indices_otl_->pull();
    // vector<int> indices(1);
    // vector<float> distances(1);
    // for(size_t i = 0; i < pcd_indices.size(); ++i) {
    //   current_kdtree_->nearestKSearch(current_pcd_->at(pcd_indices[i]), 1, indices, distances);
    //   cout << "nearestKSearch: " << distances[0] << " " << i << " " << indices[0] << " " << pcd_indices[i] << endl;
    //   ROS_ASSERT(!indices.empty());
    //   ROS_ASSERT(distances[0] < 1e-6);
    //   //ROS_ASSERT(indices[0] == i); // Sometimes points lie on top of each other?  Approx search?
    // }


    // if(current_kdtree_ && current_kdtree_->getIndices()) 
    //   cout << "KdTreeNode: current kdtree has " << current_kdtree_->getIndices()->size() << " points." << endl;
    // if(previous_kdtree_ && previous_kdtree_->getIndices())
    //   cout << "KdTreeNode: previous kdtree has " << previous_kdtree_->getIndices()->size() << " points." << endl;
        
    Output out;
    out.current_kdtree_ = current_kdtree_;
    out.previous_kdtree_ = previous_kdtree_;
    out.current_pcd_ = current_pcd_;
    out.previous_pcd_ = previous_pcd_;
    kdtree_otl_.push(out);
  }

  void KdTreeNode::_display() const
  {
  }

  void KdTreeNode::_flush()
  {
    Output out;
    out.current_kdtree_ = KdTree::Ptr((KdTree*)NULL);
    out.previous_kdtree_ = KdTree::Ptr((KdTree*)NULL);
    out.current_pcd_ = KinectCloud::Ptr((KinectCloud*)NULL);
    out.previous_pcd_ = KinectCloud::Ptr((KinectCloud*)NULL);
    kdtree_otl_.push(out);
  }

  void KdTreeNode::_reset()
  {
    current_kdtree_.reset();
    previous_kdtree_.reset();
    current_pcd_.reset();
    previous_pcd_.reset();
  }
    
  std::string KdTreeNode::_getName() const
  {
    std::ostringstream oss;
    oss << "KdTreeNode_radius:" << radius_;
    return oss.str();
  }

} // namespace dst
