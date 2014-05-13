#pragma once

#include <opencv2/opencv.hpp>
#include <pipeline2/pipeline2.h>

class SobelImageNode : public pipeline2::ComputeNode
{
public:
  pipeline2::Outlet<cv::Mat3b> outlet_;
  SobelImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl);

protected:
  pipeline2::Outlet<cv::Mat3b>* rgb_otl_;
  cv::Mat3b sobel_;

  void _compute();
  void _display() const;
  void _flush();
  std::string _getName() const;
};
  
