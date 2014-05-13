#include <dst/sobel-image-node.h>

using namespace std;

SobelImageNode::SobelImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl) :
  ComputeNode(),
  outlet_(this),
  rgb_otl_(rgb_otl)
{
  registerInput(rgb_otl_->getNode());
}

void SobelImageNode::_compute()
{
  cv::Mat3b rgb = rgb_otl_->pull();
  ROS_ASSERT(rgb.total() > 0);
  cv::Sobel( rgb, sobel_, CV_8U, 1, 1);
  outlet_.push( sobel_);
}

void SobelImageNode::_display() const
{
}

void SobelImageNode::_flush()
{
  outlet_.push(cv::Mat3b());
}

std::string SobelImageNode::_getName() const
{
  ostringstream oss;
  oss << "SobelImageNode";
  return oss.str();
}


