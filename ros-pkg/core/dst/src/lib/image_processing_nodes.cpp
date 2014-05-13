#include <dst/image_processing_nodes.h>

using namespace std;

namespace dst
{

  IntensityImageNode::IntensityImageNode(pipeline2::Outlet<cv::Mat3b>* color_otl) :
    ComputeNode(),
    outlet_(this),
    color_otl_(color_otl)
  {
    registerInput(color_otl_->getNode());
  }

  void IntensityImageNode::_compute()
  {
    cv::Mat3b color = color_otl_->pull();
    cv::cvtColor(color, intensity_, CV_BGR2GRAY);
    ROS_ASSERT(intensity_.rows == color.rows);
    ROS_ASSERT(intensity_.cols == color.cols);
    outlet_.push(intensity_);
  }

  void IntensityImageNode::_display() const
  {
  }

  void IntensityImageNode::_flush()
  {
    outlet_.push(cv::Mat1b());
  }

  std::string IntensityImageNode::_getName() const
  {
    ostringstream oss;
    oss << "IntensityImageNode";
    return oss.str();
  }

  HSVImageNode::HSVImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl) :
    ComputeNode(),
    outlet_(this),
    rgb_otl_(rgb_otl)
  {
    registerInput(rgb_otl_->getNode());
  }
  
  void HSVImageNode::_compute()
  {
    cv::Mat3b rgb = rgb_otl_->pull();
    ROS_ASSERT(rgb.cols > 0 && rgb.rows > 0);
    if(hsv_.rows != rgb.rows ||
       hsv_.cols != rgb.cols) { 
      hsv_ = cv::Mat3f(rgb.size(), 0);
      rgb3f_ = cv::Mat3f(rgb.size(), 0);
    }

    // There must be a one-liner for this.
    for(int y = 0; y < rgb.rows; ++y) {
      for(int x = 0; x < rgb.cols; ++x) { 
            rgb3f_(y, x)[0] = (float)rgb(y, x)[0] / 255.0;
            rgb3f_(y, x)[1] = (float)rgb(y, x)[1] / 255.0;
            rgb3f_(y, x)[2] = (float)rgb(y, x)[2] / 255.0;
      }
    }
    // hsv_ will be ([0,360], [0,1], [0,1]).
    cv::cvtColor(rgb3f_, hsv_, CV_BGR2HSV_FULL);

    
    // Unfortunately, this produces garbage.
    // cv::cvtColor(rgb, hsv_, CV_BGR2HSV_FULL);
    // for( int y = 0; y < hsv_.rows; ++y ){
    //   for( int x = 0; x < hsv_.cols; ++x ){
    //         cv::Vec3b vrgb = rgb(y, x);
    //         cv::Vec3f vhsv = hsv_(y, x);
    //         cv::Vec3f vrgb3f = rgb3f_(y, x);
    //         cerr << (int)vrgb[0] << " " << (int)vrgb[1] << " " << (int)vrgb[2] << ",\t";
    //         cerr << vrgb3f[0] << " " << vrgb3f[1] << " " << vrgb3f[2] << ",\t";
    //         cerr << vhsv[0] << " " << vhsv[1] << " " << vhsv[2] << endl;
    //   }
    // }
    
    outlet_.push(hsv_);
  }

  void HSVImageNode::_display() const
  {
  }

  void HSVImageNode::_flush()
  {
    outlet_.push(cv::Mat3f());
  }

  std::string HSVImageNode::_getName() const
  {
    ostringstream oss;
    oss << "HSVImageNode";
    return oss.str();
  }

  IntegralImageNode::IntegralImageNode(pipeline2::Outlet<cv::Mat1b>* intensity_otl) :
    ComputeNode(),
    outlet_(this),
    intensity_otl_(intensity_otl)
  {
    registerInput(intensity_otl_->getNode());
  }

  void IntegralImageNode::_compute()
  {
    cv::Mat1b intensity = intensity_otl_->pull();
    cv::integral(intensity, integral_);
    ROS_ASSERT(integral_.rows == intensity.rows + 1);
    ROS_ASSERT(integral_.cols == intensity.cols + 1);
    outlet_.push(integral_);
  }

  void IntegralImageNode::_display() const
  {

  }

  void IntegralImageNode::_flush()
  {
    outlet_.push(cv::Mat1i());
  }

  std::string IntegralImageNode::_getName() const
  {
    ostringstream oss;
    oss << "IntegralImageNode";
    return oss.str();
  }

  
} // namespace dst
