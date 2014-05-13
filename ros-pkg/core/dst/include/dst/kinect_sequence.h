#ifndef KINECT_SEQUENCE_H
#define KINECT_SEQUENCE_H

#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <serializable/serializable.h>
#include <dst/typedefs.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

namespace dst
{
  
  class KinectSequence : public Serializable
  {
  public:
    typedef boost::shared_ptr<KinectSequence> Ptr;
    typedef boost::shared_ptr<const KinectSequence> ConstPtr;
  
    std::vector<cv::Mat3b> images_;
    //! 127 == no data, 0 == ground, 255 == figure.
    std::vector<cv::Mat1b> seed_images_;
    //! 127 == no data, 0 == ground, 255 == figure.
    std::vector<cv::Mat1b> segmentations_;
    std::vector<KinectCloud::Ptr> pointclouds_;

    KinectSequence();
    //! Deep-copies.
    KinectSequence(const KinectSequence& seq);
    //! Deep-copies.
    KinectSequence& operator=(const KinectSequence& seq);
    void save(const std::string& filename) const;
    void load(const std::string& filename);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    size_t totalPixels() const;
    size_t pixelsPerFrame() const;
    void saveVisualization(const std::string& path) const;
  };

  //! path can point to a directory of sequences or a single sequence.
  //! sequences does not have to be empty.
  void loadSequences(const std::string& path,
                     std::vector<KinectSequence::Ptr>* sequences);
  void visualizeSeedLabels(cv::Mat1b seed, cv::Mat3b img, cv::Mat3b vis);
  void visualizeSegmentation(cv::Mat1b seg, cv::Mat3b img, cv::Mat3b vis);
}


#endif // KINECT_SEQUENCE_H
