#ifndef RGBD_H
#define RGBD_H

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <bag_of_tricks/image_region_iterator.h>
#include <asp/asp.h>
#include <openni2_interface/openni_helpers.h>

namespace asp
{
  using namespace pl;

  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
  typedef pcl::PointCloud<pcl::Normal> Normals;

  void registerPodTypesRGBD();
  
  //! "Projective" point comes from the OpenNI terminology, and refers to (u, v, z), i.e.
  //! pixel id and depth value.  Here I've added color, too, so that this represents everything
  //! that is known about a pixel in an RBGD camera.
  class ProjectivePoint : public Serializable
  {
  public:
    int u_;
    int v_;
    //! In millimeters, same as the raw depth image from the primesense device.
    unsigned short z_;
    unsigned char r_;
    unsigned char g_;
    unsigned char b_;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };
  
  class DepthMatProjector : public Pod
  {
  public:
    DECLARE_POD(DepthMatProjector);
    DepthMatProjector(const std::string& name) :
      Pod(name),
      cloud_(new Cloud)
    {
      declareInput<cv::Mat3b>("Image");
      declareInput<DepthMatConstPtr>("DepthMat");
      declareOutput<Cloud::ConstPtr>("Cloud");
    }

    void compute();
    void debug() const;

  protected:
    Cloud::Ptr cloud_;
    int width_;
    int height_;
    double cx_;
    double cy_;
    double fx_;
    double fy_;

    void project(cv::Mat3b color, const DepthMat& depth,
                 Cloud* pcd, double max_range) const;
    void project(const ProjectivePoint& ppt, Point* pt) const;
  };
  
  class OrganizedSurfaceNormalPod : public Pod
  {
  public:
    DECLARE_POD(OrganizedSurfaceNormalPod);
    OrganizedSurfaceNormalPod(const std::string& name) :
      Pod(name),
      normals_(new Normals())
    {
      declareInput<cv::Mat1b>("Mask");
      declareInput<Cloud::ConstPtr>("Cloud");  // Must be organized.
      declareParam<double>("Radius");  // In pixels.
      declareOutput<Normals::ConstPtr>("Normals");
    }

    cv::Mat3b visualize(const Normals& normals) const;  // Must be organized
    void compute();
    void debug() const;
    
  protected:
    void computeNormal(const Cloud& depth,
                       const Point& center,
                       const std::vector<int>& indices,
                       pcl::Normal* normal,
                       std::vector<bool>* valid,
                       std::vector<float>* weights);
    void computeNormal(const Cloud& depth,
                       const Point& pt,
                       const cv::Point2i& img_pt,
                       pcl::Normal* normal,
                       std::vector<int>* indices,
                       std::vector<bool>* valid,
                       std::vector<float>* weights);
    void normalToColor(const pcl::Normal& normal,
                       cv::Vec3b* color) const;

    Normals::Ptr normals_;
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimator_;
    int radius_;
  };

  // class SurfaceNormalEPG : public EdgePotentialGenerator
  // {
  // public:
  //   DECLARE_POD(SurfaceNormalEPG);
  //   SurfaceNormalEPG(const std::string& name) :
  //     EdgePotentialGenerator(name)
  //   {
  //     declareInput<Normals::ConstPtr>("Normals");
  //     declareInput<cv::Mat1b>("Mask");
  //     declareParam<double>("Sigma");
  //   }

  //   ~SurfaceNormalEPG() {}
  //   void compute();
  //   void display() const;
  //   void debug() const {}


    
  // protected:
  //   double sigma_;

  //   double computePotential(int idx0, int idx1, Normals::Ptr normals) const;
  //   void fillPotentials(int y, int x, int idx);
  // };

  
}  // namespace asp

#endif // RGBD_H
