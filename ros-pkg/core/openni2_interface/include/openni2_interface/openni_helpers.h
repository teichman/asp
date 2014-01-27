#ifndef OPENNI_HELPERS_H
#define OPENNI_HELPERS_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <OpenNI.h>

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DepthMat;  
typedef boost::shared_ptr<DepthMat> DepthMatPtr;
typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;

void oniToCV(const openni::VideoFrameRef& color, cv::Mat3b img);
cv::Mat3b oniToCV(const openni::VideoFrameRef& oni);
DepthMat oniDepthToEigen(const openni::VideoFrameRef& depth);
DepthMatPtr oniDepthToEigenPtr(const openni::VideoFrameRef& depth);
cv::Vec3b colorize(double depth, double min_range, double max_range);
cv::Mat3b colorize(DepthMat depth, double min_range, double max_range);

//! Returns the color image if bright enough.
//! Otherwise produces a false-color version of the depth image.
//! intensity_threshold is [0,255].
//! min_range and max_range are [0,10]m.
cv::Mat3b visualize(const openni::VideoFrameRef& color,
                    const openni::VideoFrameRef& depth,
                    double intensity_threshold = 30,
                    double min_range = 0.5, double max_range = 7);


#endif // OPENNI_HELPERS_H
