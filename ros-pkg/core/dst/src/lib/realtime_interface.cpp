#include <dst/realtime_interface.h>
#include <timer/timer.h>
#include <std_msgs/Header.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

using namespace std;
using namespace Eigen;

namespace dst
{
  
  RealTimeInterface::RealTimeInterface(const std::string& device_id,
                                       pcl::OpenNIGrabber::Mode mode,
                                       double scale) :
    sp_(NUM_THREADS),
    img_view_("Image"),
    device_id_(device_id),
    mode_(mode),
    grabber_(device_id_, mode, mode),
    cloud_viewer_("PointCloud"),
    segmenting_(false),
    needs_redraw_(false),
    thresh_(0.01)
  {
    initializeGrabber();
    pcds_.reserve(1000);
    imgs_.reserve(1000);
    seed_imgs_.reserve(1000);
    segmentations_.reserve(1000);

    img_view_.setDelegate((OpenCVViewDelegate*)this);
    img_view_.scale_ = scale;

    if(mode == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz) {
      size_ = cv::Size(160, 120);
      seed_radius_ = 4;
    }
    else {
      ROS_ASSERT(mode == pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
      size_ = cv::Size(640, 480);
      seed_radius_ = 16;
    }
    seed_ = cv::Mat1b(size_, 127);
  }
  
  void RealTimeInterface::run()
  {
    grabber_.start();
    while(!cloud_viewer_.wasStopped(1)) {
      if(needs_redraw_) {
        drawVis();
      }
      usleep(3e4);
    }
    grabber_.stop();
  }

  void RealTimeInterface::drawVis()
  {
    lock();
    cv::Mat3b background;
    if(segmenting_) {
      if(imgs_.empty()) {
        unlock();
        return;
      }
      background = imgs_.back();
    }
    else
      background = current_img_;
      
    background.copyTo(vis_);
    for(int y = 0; y < seed_.rows; ++y) {
      for(int x = 0; x < seed_.cols; ++x) {
        switch(seed_(y, x)) {
        case 127:
          vis_(y, x) = background(y, x);
          break;
        case 0:
          vis_(y, x) = cv::Vec3b(0, 0, 0);
          break;
        case 255:
          vis_(y, x) = cv::Vec3b(255, 255, 255);
          break;
        default:
          break;
        }
      }
    }

    if(segmenting_)
      visualizeSegmentation(segmentations_.back(), vis_, vis_);
    
    img_view_.updateImage(vis_);
    char key = img_view_.cvWaitKey(8);
    switch(key) {
    case 's':
      saveSequence();
      break;
    case ' ':
      segmenting_ = !segmenting_;
      if(!segmenting_) {
        cout << "Segmented " << num_segmented_ << " frames." << endl;
        cout << hrt_.reportSeconds() << endl;
        cout << hrt_.getMilliseconds() / (double)num_segmented_ << " ms per frame on average." << endl;
      }
      else {
        sp_.reset();
        img_queue_.clear();
        img_stamp_queue_.clear();
        pcd_queue_.clear();
        pcds_.clear();
        imgs_.clear();
        seed_imgs_.clear();
        segmentations_.clear();
        pcd_results_.clear();

        num_segmented_ = 0;
        hrt_.reset("Total segmentation time");
      }
      break;
    case 'c':
      seed_ = 127;
      break;
    case 'q':
      exit(0);
      break;
    default:
      break;
    }
    unlock();
    
    needs_redraw_ = false;
  }

  void RealTimeInterface::saveSequence() const
  {
    ROS_ASSERT(!segmenting_);
    ROS_ASSERT(imgs_.size() == seed_imgs_.size());
    ROS_ASSERT(imgs_.size() == segmentations_.size());
    ROS_ASSERT(imgs_.size() == pcds_.size());
    
    KinectSequence seq;
    seq.images_ = imgs_;
    seq.seed_images_ = seed_imgs_;
    seq.segmentations_ = segmentations_;
    seq.pointclouds_.resize(pcds_.size());
    for(size_t i = 0; i < pcds_.size(); ++i) {
      seq.pointclouds_[i] = KinectCloud::Ptr(new KinectCloud(*pcds_[i]));
      cout << seq.pointclouds_[i]->header.stamp << endl;
    }
    ostringstream oss;
    time_t timestamp = time(0);
    oss << "realtime_interface_sequence-" << timestamp;
    string path = oss.str();
    seq.save(path);
    cout << "Saved KinectSequence to " << path << endl;
  }
  
  void RealTimeInterface::cloudCallback(const KinectCloud::ConstPtr& cloud)
  {
    KinectCloud::Ptr thresholded(new KinectCloud(*cloud));
    Point bad;
    bad.x = numeric_limits<float>::quiet_NaN();
    bad.y = numeric_limits<float>::quiet_NaN();
    bad.z = numeric_limits<float>::quiet_NaN();
    for(size_t i = 0; i < thresholded->size(); ++i)
      if(thresholded->at(i).z > 2.5)
        thresholded->at(i) = bad;
    
    lock();
    //cout << "Cloud timestamp: " << cloud->header.stamp.toSec() << endl;
    if(!segmenting_)
      cloud_viewer_.showCloud(thresholded);
    else {
      pcd_queue_.push_back(thresholded);
      processQueues();
    }
    unlock();
  }

  cv::Mat1b RealTimeInterface::irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const
  {
    cv::Mat1b img(ir->getHeight(), ir->getWidth());
    unsigned short data[img.rows * img.cols];
    ir->fillRaw(img.cols, img.rows, data);
    int i = 0;
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x, ++i) {
              img(y, x) = data[i];
      }
    }
    
    return img;
  }
  
  cv::Mat3b RealTimeInterface::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const
  {    
    cv::Mat3b img(oni->getHeight(), oni->getWidth());
    uchar data[img.rows * img.cols * 3];
    oni->fillRGB(img.cols, img.rows, data);
    int i = 0;
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x, i+=3) {
              img(y, x)[0] = data[i+2];
            img(y, x)[1] = data[i+1];
            img(y, x)[2] = data[i];
      }
    }

    if(mode_ == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz) {
      ROS_ASSERT(oni->getHeight() == 240);
      ROS_ASSERT(oni->getWidth() == 320);
      cv::Mat3b small;
      cv::resize(img, small, cv::Size(160, 120));
      return small;
    }
    else {
      ROS_ASSERT(mode_ == pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
      ROS_ASSERT(oni->getHeight() == 480);
      ROS_ASSERT(oni->getWidth() == 640);
      ROS_ASSERT(img.rows == 480);
      ROS_ASSERT(img.cols == 640);
      return img;
    }
  }

  void  RealTimeInterface::processQueues()
  {
    cout << "  Queue sizes: " << pcd_queue_.size() << " " << img_queue_.size() << endl;
    ROS_ASSERT(img_queue_.size() == img_stamp_queue_.size());
    if(img_queue_.empty() || pcd_queue_.empty())
      return;

    double its = img_stamp_queue_.back();
    double pts = pcd_queue_.back()->header.stamp * 1e-9;
    cout << "its: " << its << ", pts: " << pts << endl;
    if(fabs(its - pts) < thresh_) {
      cout << "Backs are equal. Adding pair at " << pts << " with delta " << fabs(its - pts) << endl;
      imgs_.push_back(img_queue_.back());
      pcds_.push_back(pcd_queue_.back());
      pcd_queue_.clear();
      img_stamp_queue_.clear();
      img_queue_.clear();
      segmentLatest();
    }
    else if(its < pts) {
      double delta = fabs(pcd_queue_.front()->header.stamp * 1e-9 - its);
      while(!pcd_queue_.empty()) {
        if(delta < thresh_) {
          cout << "PCD more recent. Adding pair at " << pcd_queue_.front()->header.stamp * 1e-9 << " with delta " << delta << endl;
          imgs_.push_back(img_queue_.back());
          pcds_.push_back(pcd_queue_.front());
          
          pcd_queue_.pop_front();
          img_queue_.clear();
          img_stamp_queue_.clear();
          
          segmentLatest();
          break;
        }
        else if(pcd_queue_.front()->header.stamp * 1e-9 < its)
          pcd_queue_.pop_front();
        else
          break;
      }
    }
    else {
      // image is more recent than pointcloud.
      // search for img that matches pcd timestamp.
      while(!img_queue_.empty()) {

        double delta = fabs(pts - img_stamp_queue_.front());
        if(delta < thresh_) {
          cout << "Img more recent. Adding pair at " << pts << " with delta: " << delta << endl;
          imgs_.push_back(img_queue_.front());
          pcds_.push_back(pcd_queue_.back());
          
          pcd_queue_.clear();
          img_queue_.pop_front();
          img_stamp_queue_.pop_front();
          
          segmentLatest();
          break;
        }
        else if(img_stamp_queue_.front() < pts) {
          img_queue_.pop_front();
          img_stamp_queue_.pop_front();
        }
        else
          break;
      }
    }
  }
  
  void RealTimeInterface::segmentLatest()
  {
    cout << "Segmenting pair with timestamp " << pcds_.back()->header.stamp * 1e-9  << endl;
    cout << "Aligned queue sizes: " << pcds_.size() << " " << imgs_.size() << endl;
    ROS_ASSERT(pcds_.size() == imgs_.size());
    ROS_ASSERT(segmentations_.size() == pcd_results_.size());

    segmentations_.push_back(cv::Mat1b(size_, 127));
    pcd_results_.push_back(KinectCloud::Ptr(new KinectCloud()));
    seed_imgs_.push_back(seed_.clone());

    hrt_.start();
        
    if(pcds_.size() == 1) {
      ROS_ASSERT(segmentations_.size() == 1 && pcd_results_.size() == 1);
      
      sp_.run(seed_,
              imgs_.back(),
              pcds_.back(),
              cv::Mat3b(),
              cv::Mat1b(),
              KinectCloud::Ptr(),
              segmentations_.back(),
              pcd_results_.back());

      seed_ = 127;
    }
    else { 
      sp_.run(seed_,
              imgs_.back(),
              pcds_.back(),
              imgs_[imgs_.size()-2],
                  segmentations_[segmentations_.size()-2],
              pcds_[pcds_.size()-2],
                  segmentations_.back(),
              pcd_results_.back());

      seed_ = 127;
    }
    hrt_.stop();

    
    // -- Visualize the segmentation.
    // double scale = 3;
    // cv::Size sz(segmentations_.back().cols * scale, segmentations_.back().rows * scale);
    // cv::resize(segmentations_.back(), seg_vis_, sz, cv::INTER_NEAREST);
    // cv::imshow("Segmentation", seg_vis_);
    //cv::imshow("Segmentation", segmentations_.back());
    ++num_segmented_;
  }
  
  void RealTimeInterface::imageCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_img)
  {
    lock();
    current_img_ = oniToCV(oni_img);
    needs_redraw_ = true;

    if(segmenting_) {
      img_queue_.push_back(current_img_);
      img_stamp_queue_.push_back(oni_img->getTimeStamp() / (double)1e6);
      processQueues();
    }
    unlock();
  }

  void RealTimeInterface::irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img)
  {
    ScopedTimer st("ir image callback");
    cv::Mat1b img = irToCV(oni_img);
    cv::namedWindow("IR", CV_WINDOW_NORMAL);
    cv::imshow("IR", img);
    cv::waitKey(10);
  }

  void RealTimeInterface::depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
  {
    cout << "Depth timestamp: " << oni->getTimeStamp() << endl;
  }
  
  void RealTimeInterface::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    
    // if(event.getKeyCode())
    //   cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
    // else
    //   cout << "the special key \'" << event.getKeySym() << "\' was";
    // if(event.keyDown())
    //   cout << " pressed" << endl;
    // else
    //   cout << " released" << endl;
    
  }

  void RealTimeInterface::initializeGrabber()
  {
    cloud_viewer_.registerKeyboardCallback(&RealTimeInterface::keyboardCallback, *this, NULL);
    
    if(SHOW_IR) { 
      boost::function<void (const boost::shared_ptr<openni_wrapper::IRImage>&)> ir_cb;
      ir_cb = boost::bind(&RealTimeInterface::irCallback, this, _1);
      grabber_.registerCallback(ir_cb);
    }
    else {
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> image_cb;
      image_cb = boost::bind(&RealTimeInterface::imageCallback, this, _1);
      grabber_.registerCallback(image_cb);

      boost::function<void (const KinectCloud::ConstPtr&)> cloud_cb;
      cloud_cb = boost::bind(&RealTimeInterface::cloudCallback, this, _1);
      grabber_.registerCallback(cloud_cb);
      grabber_.getDevice()->setSynchronization(true);
      ROS_ASSERT(grabber_.getDevice()->isSynchronized());
    }
  }

  void RealTimeInterface::mouseEvent(int event, int x0, int y0, int flags, void* param)
  {
    // if(segmenting_)
    //   return;

    cout << "event: " << event << ", x0: " << x0 << ", y0: " << y0 << ", flags: " << flags
         << ", CV_EVENT_FLAG_LBUTTON: " << CV_EVENT_FLAG_LBUTTON << endl;

    // -- Left click to add to source.
    if(flags & CV_EVENT_FLAG_LBUTTON) {
      cout << "Setting fg." << endl;
      for(int y = max(0, y0 - seed_radius_); y < seed_.rows && y <= y0 + seed_radius_; ++y)
        for(int x = max(0, x0 - seed_radius_); x < seed_.cols && x <= x0 + seed_radius_; ++x) {
          seed_(y, x) = 255;
        }
      
      needs_redraw_ = true;
    }

    // -- Right click to add to sink.
    else if(flags & CV_EVENT_FLAG_RBUTTON) {
      for(int y = max(0, y0 - seed_radius_); y < seed_.rows && y <= y0 + seed_radius_; ++y)
        for(int x = max(0, x0 - seed_radius_); x < seed_.cols && x <= x0 + seed_radius_; ++x)
          seed_(y, x) = 0;

      needs_redraw_ = true;
    }
  }

  
}
