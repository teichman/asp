#include <dst/cie-xyz-image-node.h>

using namespace std;

CieXyzImageNode::CieXyzImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl) :
  ComputeNode(),
  outlet_(this),
  rgb_otl_(rgb_otl)
{
  registerInput(rgb_otl_->getNode());
}

void CieXyzImageNode::_compute()
{
  cv::Mat3b rgb = rgb_otl_->pull();
  ROS_ASSERT(rgb.cols > 0 && rgb.rows > 0);
  if(xyz_.rows != rgb.rows ||
     xyz_.cols != rgb.cols) { 
    xyz_ = cv::Mat3b(rgb.size(), 0);
  }

  cv::cvtColor( rgb, xyz_, CV_BGR2XYZ);

  std::vector< cv::Mat > xyz_vec;
  cv::split( xyz_, xyz_vec);

  xyz_vec[1].setTo( cv::Scalar( 128));
  cv::merge( xyz_vec, xyz_);

  outlet_.push( xyz_);
}

void CieXyzImageNode::_display() const
{
}

void CieXyzImageNode::_flush()
{
  outlet_.push(cv::Mat3f());
}

std::string CieXyzImageNode::_getName() const
{
  ostringstream oss;
  oss << "CieXyzImageNode";
  return oss.str();
}


