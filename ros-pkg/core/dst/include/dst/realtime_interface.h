#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_labeler/opencv_view.h>
#include <eigen_extensions/eigen_extensions.h> 
#include <agent/lockable.h>
#include <dst/segmentation_pipeline.h>

namespace dst
{

  class RealTimeInterface : public OpenCVViewDelegate, public Lockable
  {
  public:
    SegmentationPipeline sp_;
    OpenCVView img_view_;
    int seed_radius_;
    
    RealTimeInterface(const std::string& device_id = "",
                      pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz,
                      double scale = 3);
    void cloudCallback(const KinectCloud::ConstPtr& cloud);
    void imageCallback(const boost::shared_ptr<openni_wrapper::Image>& image);
    void irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img);
    void depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void run();
    void mouseEvent(int event, int x, int y, int flags, void* param);
    
  protected:
    std::string device_id_;
    pcl::OpenNIGrabber::Mode mode_;
    cv::Size size_;
    pcl::OpenNIGrabber grabber_;
    pcl::visualization::CloudViewer cloud_viewer_;
    std::vector<KinectCloud::ConstPtr> pcds_;
    std::vector<cv::Mat3b> imgs_;
    std::vector<cv::Mat1b> seed_imgs_;
    bool segmenting_;
    cv::Mat1b seed_;
    cv::Mat3b vis_;
    cv::Mat3b current_img_;
    std::vector<cv::Mat1b> segmentations_;
    std::vector<KinectCloud::Ptr> pcd_results_;
    bool needs_redraw_;
    KinectCloud::ConstPtr pcd_;
    std::deque<cv::Mat3b> img_queue_;
    std::deque<double> img_stamp_queue_;
    std::deque<KinectCloud::ConstPtr> pcd_queue_;
    double thresh_;
    HighResTimer hrt_;
    int num_segmented_;
    
    cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const;
    cv::Mat1b irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const;
    void initializeGrabber();
    void segmentLatest();
    void processQueues();
    void drawVis();
    void saveSequence() const;
  };

}
