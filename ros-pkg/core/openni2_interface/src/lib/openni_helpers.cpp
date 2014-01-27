#include <openni2_interface/openni_helpers.h>
#include <ros/assert.h>
#include <timer/timer.h>

using namespace std;

void oniToCV(const openni::VideoFrameRef& oni, cv::Mat3b img)
{
  #if JARVIS_DEBUG
  ScopedTimer st("oniToCV");
  ROS_ASSERT(oni.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_RGB888);
  ROS_ASSERT(oni.getWidth() == img.cols && oni.getHeight() == img.rows);
  #endif
  
  uchar* data = (uchar*)oni.getData();
  int i = 0;
  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x, i+=3) {
      img(y, x)[0] = data[i+2];
      img(y, x)[1] = data[i+1];
      img(y, x)[2] = data[i];
    }
  }
}

cv::Mat3b oniToCV(const openni::VideoFrameRef& oni)
{
  cv::Mat3b cv(cv::Size(oni.getWidth(), oni.getHeight()));
  oniToCV(oni, cv);
  return cv;
}

DepthMatPtr oniDepthToEigenPtr(const openni::VideoFrameRef& oni)
{
  ROS_ASSERT(oni.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);

  DepthMatPtr depth(new DepthMat(oni.getHeight(), oni.getWidth()));
  ushort* data = (ushort*)oni.getData();
  int idx = 0;
  for(int y = 0; y < depth->rows(); ++y)
    for(int x = 0; x < depth->cols(); ++x, ++idx)
      depth->coeffRef(y,x) = data[idx];

  return depth;
}
  
DepthMat oniDepthToEigen(const openni::VideoFrameRef& oni)
{
  return *oniDepthToEigenPtr(oni);
}

cv::Vec3b colorize(double depth, double min_range, double max_range)
{
  if(depth == 0)
    return cv::Vec3b(0, 0, 0);
    
  double increment = (max_range - min_range) / 3;
  double thresh0 = min_range;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;
    
  if(depth < thresh0) {
    return cv::Vec3b(0, 0, 255);
  }
  if(depth >= thresh0 && depth < thresh1) {
    int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
    return cv::Vec3b(val, val, 255 - val);
  }
  else if(depth >= thresh1 && depth < thresh2) {
    int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
    return cv::Vec3b(255, 255 - val, 0);
  }
  else if(depth >= thresh2 && depth < thresh3) {
    int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
    return cv::Vec3b(255 - val, val, 0);
  }
    
  return cv::Vec3b(0, 255, 0);
}

cv::Mat3b colorize(DepthMat depth, double min_range, double max_range)
{
  cv::Mat3b img(cv::Size(depth.cols(), depth.rows()));
  for(int y = 0; y < img.rows; ++y)
    for(int x = 0; x < img.cols; ++x)
      img(y, x) = colorize(depth(y, x) * 0.001, min_range, max_range);
  return img;
}

cv::Mat3b visualize(const openni::VideoFrameRef& color,
                    const openni::VideoFrameRef& depth,
                    double intensity_threshold,
                    double min_range, double max_range)
{
  ROS_ASSERT(color.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_RGB888);
  ROS_ASSERT(depth.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);
  
  uchar* data = (uchar*)color.getData();
  double total = 0;
  for(int i = 0; i < 3 * color.getWidth() * color.getHeight(); i+=3)
    total += (double)(data[i] + data[i+1] + data[i+2]);
  double mean = total / (3 * color.getWidth() * color.getHeight());
  //cout << "Mean: " << mean << endl;
  cv::Mat3b img;
  if(mean > intensity_threshold) {
    img = oniToCV(color);
  }
  else {
    ushort* data = (ushort*)depth.getData();
    img = cv::Mat3b(cv::Size(depth.getWidth(), depth.getHeight()));
    int idx = 0;
    for(int y = 0; y < depth.getHeight(); ++y)
      for(int x = 0; x < depth.getWidth(); ++x, ++idx)
        img(y, x) = colorize(data[idx] * 0.001, min_range, max_range);
  }
  return img;
}
