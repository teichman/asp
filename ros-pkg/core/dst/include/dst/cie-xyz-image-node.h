#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pipeline2/pipeline2.h>

class CieXyzImageNode : public pipeline2::ComputeNode
{
public:
  pipeline2::Outlet<cv::Mat3b> outlet_;
  CieXyzImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl);

protected:
  pipeline2::Outlet<cv::Mat3b>* rgb_otl_;
  cv::Mat3b xyz_;

  void _compute();
  void _display() const;
  void _flush();
  std::string _getName() const;
};
  
