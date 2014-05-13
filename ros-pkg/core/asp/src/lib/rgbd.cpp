#include <asp/rgbd.h>
#include <pcl/filters/filter.h>
#include <pcl/common/distances.h>

namespace asp
{
  using namespace std;
  using namespace pl;
  using namespace Eigen;

  void registerPodTypesRGBD()
  {
    REGISTER_POD(DepthMatProjector);
    REGISTER_POD(OrganizedSurfaceNormalPod);
    //REGISTER_POD(SurfaceNormalEPG);
  }
  
  void ProjectivePoint::serialize(std::ostream& out) const
  {
    out << setprecision(16) << u_ << " " << v_ << " " << z_
        << " " << (int)r_ << " " << (int)g_ << " " << (int)b_ << endl;
  }

  void ProjectivePoint::deserialize(std::istream& in)
  {
    in >> u_;
    in >> v_;
    in >> z_;
    in >> r_;
    in >> g_;
    in >> b_;

    string buf;
    getline(in, buf);
  }

  void DepthMatProjector::project(const ProjectivePoint& ppt, Point* pt) const
  {
    ROS_ASSERT(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_);

    pt->r = ppt.r_;
    pt->g = ppt.g_;
    pt->b = ppt.b_;

    if(ppt.z_ == 0) {
      pt->x = std::numeric_limits<float>::quiet_NaN();
      pt->y = std::numeric_limits<float>::quiet_NaN();
      pt->z = std::numeric_limits<float>::quiet_NaN();
    }
    else {
      pt->z = ppt.z_ * 0.001;
      pt->x = pt->z * (ppt.u_ - cx_) / fx_;
      pt->y = pt->z * (ppt.v_ - cy_) / fy_;
    }
  }
  
  void DepthMatProjector::project(cv::Mat3b color, const DepthMat& depth,
                                  Cloud* pcd, double max_range) const
  {    
    ROS_ASSERT(depth.rows() == color.rows);
    ROS_ASSERT(depth.cols() == color.cols);
    ROS_ASSERT(color.rows == height_);
    ROS_ASSERT(color.cols == width_);
    ROS_ASSERT(fx_ > 0 && fy_ > 0 && cx_ > 0 && cy_ > 0);
    
    pcd->clear();
    pcd->height = depth.rows();
    pcd->width = depth.cols();
    pcd->is_dense = false;
    pcd->resize(depth.rows() * depth.cols());
    //pcd->header.stamp = (frame.timestamp_) * 1e9;

    int idx = 0;
    ProjectivePoint ppt;
    for(ppt.v_ = 0; ppt.v_ < depth.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < depth.cols(); ++ppt.u_, ++idx) {
        ppt.z_ = depth(ppt.v_, ppt.u_);
        if(ppt.z_ > max_range * 1000)
          ppt.z_ = 0;  // bad point.

        ppt.r_ = color(ppt.v_, ppt.u_)[2];
        ppt.g_ = color(ppt.v_, ppt.u_)[1];
        ppt.b_ = color(ppt.v_, ppt.u_)[0];
        project(ppt, &pcd->at(idx));
      }
    }
  }
  
  void DepthMatProjector::compute()
  {
    const DepthMat& depth = *pull<DepthMatConstPtr>("DepthMat");
    cv::Mat3b color = pull<cv::Mat3b>("Image");

    width_ = color.cols;
    height_ = color.rows;
    cx_ = width_ / 2;
    cy_ = height_ / 2;
    fx_ = 525 * (width_ / 640.);
    fy_ = 525 * (height_ / 480.);

    project(color, depth, cloud_.get(), 10);
    push<Cloud::ConstPtr>("Cloud", cloud_);
  }

  void DepthMatProjector::debug() const
  {
    cv::Mat3b vis(cloud_->height, cloud_->width, cv::Vec3b(0, 0, 0));
    for(size_t y = 0; y < cloud_->height; ++y) {
      for(size_t x = 0; x < cloud_->width; ++x) {
        vis(y, x) = colorize(cloud_->at(x, y).z, 0, 10);
      }
    }
    cv::imwrite(debugBasePath() + ".png", vis);
    pcl::io::savePCDFileBinary(debugBasePath() + ".pcd", *cloud_);
  }

  
  /************************************************************
   * OrganizedSurfaceNormalPod
   ************************************************************/
  
  void OrganizedSurfaceNormalPod::computeNormal(const Cloud& pcd,
                                                const Point& center,
                                                const std::vector<int>& indices,
                                                pcl::Normal* normal,
                                                std::vector<bool>* valid,
                                                std::vector<float>* weights)
  {
    
    // TODO:  This method could probably be made substantially better by using RANSAC.  This would help prevent the
    // "blurry" surface normals at edges, which is really what we're after.  Sounds expensive, but there are
    // probably some tricks one could do to bring the cost down.  See 2013-11-20 notes.
    
    weights->clear();
    weights->resize(indices.size(), 0);
    double total_weight = 0;
    valid->clear();
    valid->resize(indices.size(), true);

    // HighResTimer hrt("OrganizedSurfaceNormalPod: weighting");
    // hrt.start();
    
    Vector3f mean = Vector3f::Zero();
    float num_valid = 0;
    for(size_t i = 0; i < indices.size(); ++i) {
      const Vector3f& pt = pcd[indices[i]].getVector3fMap();
      if(isnan(pt(0)) || isnan(pt(1)) || isnan(pt(2))) {
        (*valid)[i] = false;
          continue;
      }


      //double dist = fabs(pcd[indices[i]].z - center.z); // Rough approximation.
      
      double dist = pcl::euclideanDistance(pcd[indices[i]], center);
      double sigma = 0.1; // TODO: Parameterize.
      (*weights)[i] = exp(-dist / sigma); 

      total_weight += (*weights)[i];
      mean += pt;
      ++num_valid;
    }
    mean /= num_valid;
    
    // -- Normalize the weights.  - This is only necessary if there are numerical issues
    // or if we want valid curvature estimates.
    for(size_t i = 0; i < weights->size(); ++i)
      (*weights)[i] /= total_weight;

    // hrt.stop();
    // cout << hrt.report() << endl;

    // hrt.reset("OrganizedSurfaceNormalPod: solve");
    // hrt.start();
    Matrix3f X = Matrix3f::Zero();
    for(size_t i = 0; i < indices.size(); ++i) {
      if(!(*valid)[i])
        continue;
      Vector3f pt = (*weights)[i] * (pcd[indices[i]].getVector3fMap() - center.getVector3fMap());
      //Vector3f pt = (*weights)[i] * (pcd[indices[i]].getVector3fMap() - mean);
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

  void OrganizedSurfaceNormalPod::computeNormal(const Cloud& pcd,
                                                const Point& pt,
                                                const cv::Point2i& img_pt,
                                                pcl::Normal* normal,
                                                std::vector<int>* indices,
                                                std::vector<bool>* valid,
                                                std::vector<float>* weights)
  {
    indices->clear();
    valid->clear();
    weights->clear();
    
    ImageRegionIterator it(cv::Size(pcd.width, pcd.height), radius_);
    for(it.setCenter(img_pt); !it.done(); ++it) {
      int idx = it.index();
      // if(idx % 2 != 0)
      //         continue;
      
      if(isnan(pcd[idx].z))
        continue;

      indices->push_back(it.index());
    }

    computeNormal(pcd, pt, *indices, normal, valid, weights);
  }
  
  void OrganizedSurfaceNormalPod::compute()
  {
    cv::Mat1b mask = pull<cv::Mat1b>("Mask");
    Cloud::ConstPtr pcd = pull<Cloud::ConstPtr>("Cloud");
    radius_ = param<double>("Radius");

    normals_->clear();
    normals_->height = pcd->height;
    normals_->width = pcd->width;
    normals_->is_dense = false;
    normals_->resize(pcd->size());

    #pragma omp parallel for
    for(size_t y = 0; y < pcd->height; ++y) {
      vector<int> indices;
      vector<bool> valid;
      vector<float> weights;
      indices.reserve(1e3);
      valid.reserve(1e3);
      weights.reserve(1e3);
      for(size_t x = 0; x < pcd->width; ++x) {
        if(mask(y, x) != 255)
          continue;
        int idx = y * pcd->width + x;
        cv::Point2i img_pt(x, y);
        computeNormal(*pcd, pcd->at(idx), img_pt, &normals_->at(idx), &indices, &valid, &weights);
      }
    }

    push<Normals::ConstPtr>("Normals", normals_);
  }

  void OrganizedSurfaceNormalPod::normalToColor(const pcl::Normal& p,
                                                 cv::Vec3b* color) const
  {
    cv::Vec3b& c = *color;
    if(!isfinite(p.normal[0]) || !isfinite(p.normal[1]) || !isfinite(p.normal[2])) { 
      c = cv::Vec3b(0, 0, 0);
      return;
    }
    
    ROS_ASSERT(p.normal[0] <= 1.0 && p.normal[0] >= -1.0);
    c[0] = max<int>(0, (p.normal[0] + 1.0) / 2.0 * 255);
    c[1] = max<int>(0, (p.normal[1] + 1.0) / 2.0 * 255);
    c[2] = max<int>(0, (p.normal[2] + 1.0) / 2.0 * 255);
  }

  cv::Mat3b OrganizedSurfaceNormalPod::visualize(const Normals& normals) const
  {
    cv::Mat3b vis(normals.height, normals.width, cv::Vec3b(0, 0, 0));
    for(size_t y = 0; y < normals.height; ++y) {
      for(size_t x = 0; x < normals.width; ++x) {
        int idx = y * normals.width + x;
        normalToColor(normals[idx], &vis(y, x));
      }
    }
    return vis;
  }
  
  void OrganizedSurfaceNormalPod::debug() const
  {
    cv::Mat3b vis = visualize(*normals_);
    cv::imwrite(debugBasePath() + ".png", vis);

    Cloud::ConstPtr pcd = pull<Cloud::ConstPtr>("Cloud");
    pcl::PointCloud<pcl::PointXYZRGBNormal> cn;
    pcl::concatenateFields(*pcd, *normals_, cn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cn, cn, indices);
    pcl::removeNaNNormalsFromPointCloud(cn, cn, indices);
    
    if(!cn.empty())
      pcl::io::savePCDFileBinary(debugBasePath() + ".pcd", cn);
  }
  
}  // namespace asp
