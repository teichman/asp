#include <dst/organized_surface_normal_node.h>
#include <timer/timer.h>
#include <pcl/common/distances.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  OrganizedSurfaceNormalNode::OrganizedSurfaceNormalNode(pipeline2::Outlet<KinectCloud::ConstPtr>* pcd_otl,
                                                         pipeline2::Outlet<cv::Mat1b>* mask_otl,
                                                         int radius) :
    ComputeNode(),
    normals_otl_(this),
    pcd_otl_(pcd_otl),
    mask_otl_(mask_otl),
    radius_(radius),
    normals_(new Normals())
  {
    // Gross hack.
    // TODO: There should be a good way to call ComputeNode functionality without having an entire pipeline.
    if(mask_otl_->getNode())
      registerInput(mask_otl_->getNode());
    if(pcd_otl_->getNode())
      registerInput(pcd_otl_->getNode());
  }

  void OrganizedSurfaceNormalNode::computeNormal(const KinectCloud& pcd,
                                                 const pcl::PointXYZRGB& center,
                                                   const std::vector<int>& indices,
                                                   pcl::Normal* normal)
  {
    // TODO:  This method could probably be made substantially better by using RANSAC.  This would help prevent the
    // "blurry" surface normals at edges, which is really what we're after.  Sounds expensive, but there are
    // probably some tricks one could do to bring the cost down.  See 2013-11-20 notes.
    
    weights_.clear();
    weights_.resize(indices.size(), 0);
    double total_weight = 0;
    valid_.clear();
    valid_.resize(indices.size(), true);

    // HighResTimer hrt("OrganizedSurfaceNormalNode: weighting");
    // hrt.start();
    
    Vector3f mean = Vector3f::Zero();
    float num_valid = 0;
    for(size_t i = 0; i < indices.size(); ++i) {
      const Vector3f& pt = pcd[indices[i]].getVector3fMap();
      if(isnan(pt(0)) || isnan(pt(1)) || isnan(pt(2))) {
        valid_[i] = false;
          continue;
      }

      double dist = pcl::euclideanDistance(pcd[indices[i]], center);
      //double dist = fabs(pcd[indices[i]].z - center.z); // Rough approximation.
      double sigma = 0.1; // TODO: Parameterize.
      weights_[i] = exp(-dist / sigma); 
      //weights_[i] = 1.0;
      total_weight += weights_[i];
      mean += pt;
      ++num_valid;
    }
    mean /= num_valid;
    
    // -- Normalize the weights.  - This is only necessary if there are numerical issues
    // or if we want valid curvature estimates.
    for(size_t i = 0; i < weights_.size(); ++i)
      weights_[i] /= total_weight;

    // hrt.stop();
    // cout << hrt.report() << endl;

    // hrt.reset("OrganizedSurfaceNormalNode: solve");
    // hrt.start();
    Matrix3f X = Matrix3f::Zero();
    for(size_t i = 0; i < indices.size(); ++i) {
      if(!valid_[i])
        continue;
      Vector3f pt = weights_[i] * (pcd[indices[i]].getVector3fMap() - center.getVector3fMap());
      //Vector3f pt = weights_[i] * (pcd[indices[i]].getVector3fMap() - mean);
      X += pt * pt.transpose();
    }

    pcl::solvePlaneParameters(X, 
                              normal->normal[0],
                              normal->normal[1],
                              normal->normal[2],
                              normal->curvature);
    
    pcl::flipNormalTowardsViewpoint(center, 0, 0, 0,
                                    normal->normal[0],
                                    normal->normal[1],
                                    normal->normal[2]);

    // hrt.stop();
    // cout << hrt.report() << endl;
  }
  
  void OrganizedSurfaceNormalNode::computeNormal(const KinectCloud& pcd,
                                                 const pcl::PointXYZRGB& pt,
                                                 const cv::Point2i& img_pt,
                                                 pcl::Normal* normal)
  {
    indices_.clear();
    inliers_.clear();
    
    ImageRegionIterator it(cv::Size(pcd.width, pcd.height), radius_);
    for(it.setCenter(img_pt); !it.done(); ++it) {
      int idx = it.index();
      // if(idx % 2 != 0)
      //         continue;
      
      if(isnan(pcd[idx].z))
        continue;

      indices_.push_back(it.index());
    }

    computeNormal(pcd, pt, indices_, normal);
  }
  
  void OrganizedSurfaceNormalNode::_compute()
  {
    KinectCloud::ConstPtr pcd = pcd_otl_->pull();
    ROS_ASSERT(normals_->empty());
    normals_->resize(pcd->size());

    cv::Mat1b mask = mask_otl_->pull();
    for(size_t y = 0; y < pcd->height; ++y) {
      for(size_t x = 0; x < pcd->width; ++x) {
        if(mask(y, x) != 255)
          continue;
        
        int idx = y * pcd->width + x;
        cv::Point2i img_pt(x, y);
        computeNormal(*pcd, pcd->at(idx), img_pt, &normals_->at(idx));
      }
    }
      
    normals_otl_.push(normals_);
  }

  void OrganizedSurfaceNormalNode::normalToColor(const pcl::Normal& p,
                                                 cv::Vec3b* color) const
  {
    cv::Vec3b& c = *color;
    if(!isfinite(p.normal[0]) || !isfinite(p.normal[1]) || !isfinite(p.normal[2])) { 
      c = cv::Vec3b(0, 0, 0);
      return;
    }
    
    ROS_ASSERT(p.normal[0] <= 1.0 && p.normal[0] >= -1.0);
    c[0] = fabs(p.normal[0]) * 255;
    c[1] = fabs(p.normal[1]) * 255;
    c[2] = fabs(p.normal[2]) * 255;
  }

  cv::Mat3b OrganizedSurfaceNormalNode::getSurfNorm(const KinectCloud& pcd)
  {
    cv::Mat3b vis(pcd.height, pcd.width, cv::Vec3b(0, 0, 0));
    normals_->resize(pcd.size());
    for(size_t y = 0; y < pcd.height; ++y) {
      for(size_t x = 0; x < pcd.width; ++x) {
        int idx = y * pcd.width + x;
        cv::Point2i img_pt(x, y);
        if(pcd.at(idx).x != pcd.at(idx).x) { // this will only occur if it's 'nan'
          continue;
        }
        computeNormal(pcd, pcd.at(idx), img_pt, &normals_->at(idx));
        normalToColor(normals_->at(idx), &vis(y, x));
      }
    }
    normals_otl_.push(normals_);
    return vis;
  }
  
  void OrganizedSurfaceNormalNode::_display() const
  {
    const KinectCloud& pcd = *pcd_otl_->pull();
    cv::Mat3b vis(cv::Size(pcd.width, pcd.height), cv::Vec3b(0, 0, 0));

    for(int y = 0; y < vis.rows; ++y) {
      for(int x = 0; x < vis.cols; ++x) {
        int idx = y * pcd.width + x;
        if(isnan(pcd[idx].x))
          continue;

        normalToColor(normals_->at(idx), &vis(y, x));
      }
    }

    cv::imwrite("debug/" + getRunName() + ".png", vis);

    pcl::PointCloud<pcl::PointXYZRGBNormal> cn;
    pcl::concatenateFields(pcd, *normals_, cn);
    if(!cn.empty())
      pcl::io::savePCDFileBinary("debug/" + getRunName() + ".pcd", cn);
    else { 
      int retval = system(("touch debug/" + getRunName() + ".pcd").c_str());
      --retval;
    }
  }

  void OrganizedSurfaceNormalNode::_flush()
  {
    indices_.clear();
    inliers_.clear();
    normals_->clear();
    normals_otl_.push(Normals::Ptr());
  }

  std::string OrganizedSurfaceNormalNode::_getName() const
  {
    std::ostringstream oss;
    oss << "OrganizedSurfaceNormalNode";
    return oss.str();
  }
    
} // namespace dst
