#include <dst/kinect_sequence.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace dst
{

  KinectSequence::KinectSequence() :
    Serializable()
  {
  }

  void KinectSequence::serialize(std::ostream& out) const
  {
    ROS_FATAL_STREAM("Use save instead.");
  }

  void KinectSequence::deserialize(std::istream& in)
  {
    ROS_FATAL_STREAM("Use load instead.");
  }

  void KinectSequence::save(const std::string& dir) const
  {
    ROS_ASSERT(!bfs::exists(dir));
    bfs::create_directory(dir);

    for(size_t i = 0; i < images_.size(); ++i) {
      ostringstream oss;
      oss << "img" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), images_[i]);
    }

    for(size_t i = 0; i < pointclouds_.size(); ++i) {
      ostringstream oss;
      oss << "pcd" << setw(4) << setfill('0') << i << ".pcd";
      pcl::io::savePCDFileBinary(dir + "/" + oss.str(), *pointclouds_[i]);
    }
    
    ROS_ASSERT(seed_images_.empty() || seed_images_.size() == images_.size());
    for(size_t i = 0; i < seed_images_.size(); ++i) {
      ostringstream oss;
      oss << "seed" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), seed_images_[i]);
    }

    ROS_ASSERT(segmentations_.empty() || segmentations_.size() == images_.size());
    for(size_t i = 0; i < segmentations_.size(); ++i) {
      ostringstream oss;
      oss << "segmentation" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), segmentations_[i]);
    }
  }

  void KinectSequence::load(const std::string& dir)
  {
    // -- Get filenames.
    vector<string> img_names;
    vector<string> seed_names;
    vector<string> segmentation_names;
    vector<string> pcd_names;
    
    bfs::recursive_directory_iterator it(dir), eod;
    BOOST_FOREACH(bfs::path const & p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().string().substr(0, 4).compare("seed") == 0 &&  bfs::extension(p).compare(".png") == 0)
        seed_names.push_back(p.string());
      else if(p.leaf().string().substr(0, 4).compare("segm") == 0 &&  bfs::extension(p).compare(".png") == 0)
        segmentation_names.push_back(p.string());
      else if(p.leaf().string().substr(0, 3).compare("img") == 0 &&  bfs::extension(p).compare(".png") == 0)
        img_names.push_back(p.string());
      else if(bfs::extension(p).compare(".pcd") == 0)
        pcd_names.push_back(p.string());
    }
    ROS_ASSERT(img_names.size() == pcd_names.size());

    // -- Sort all filenames.
    sort(img_names.begin(), img_names.end());
    sort(pcd_names.begin(), pcd_names.end());
    sort(segmentation_names.begin(), segmentation_names.end());
    sort(seed_names.begin(), seed_names.end());

    // -- Load images and pointclouds.
    images_.resize(img_names.size());
    pointclouds_.resize(pcd_names.size());
    for(size_t i = 0; i < img_names.size(); ++i) {
      images_[i] = cv::imread(img_names[i], 1);
      pointclouds_[i] = KinectCloud::Ptr(new KinectCloud());
      pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_names[i], *pointclouds_[i]);
    }

    // -- Load segmentations.
    if(segmentation_names.size() == img_names.size()) { 
      segmentations_.resize(segmentation_names.size());
      for(size_t i = 0; i < segmentation_names.size(); ++i) { 
        //cout << "Loading " << segmentation_names[i] << endl;
        segmentations_[i] = cv::imread(segmentation_names[i], 0);
      }
    }
    else if(segmentation_names.empty()) {
      segmentations_.resize(img_names.size());
      for(size_t i = 0; i < segmentations_.size(); ++i)
        segmentations_[i] = cv::Mat1b(images_[0].size(), 127);
    }
    else
      ROS_FATAL("A KinectSequence must have either all segmentations or no segmentations.");

    // -- Load seed labels.
    if(seed_names.size() == img_names.size()) { 
      seed_images_.resize(seed_names.size());
      for(size_t i = 0; i < seed_names.size(); ++i)
        seed_images_[i] = cv::imread(seed_names[i], 0);
    }
    else if(seed_names.empty()) {
      seed_images_.resize(img_names.size());
      for(size_t i = 0; i < seed_images_.size(); ++i)
        seed_images_[i] = cv::Mat1b(images_[0].size(), 127);
    }
    else
      ROS_FATAL("A KinectSequence must have either all seed images or no seed images.");
  }
  
  KinectSequence::KinectSequence(const KinectSequence& seq)
  {
    ROS_ASSERT(seq.images_.size() == seq.pointclouds_.size());
    images_.resize(seq.images_.size());
    pointclouds_.resize(seq.images_.size());

    for(size_t i = 0; i < images_.size(); ++i) {
      images_[i] = seq.images_[i].clone();
      pointclouds_[i] = seq.pointclouds_[i]->makeShared();
    }
  }
  
  KinectSequence& KinectSequence::operator=(const KinectSequence& seq)
  {
    if(&seq == this)
      return *this;

    ROS_ASSERT(seq.images_.size() == seq.pointclouds_.size());
    images_.resize(seq.images_.size());
    pointclouds_.resize(seq.images_.size());
    
    for(size_t i = 0; i < images_.size(); ++i) {
      images_[i] = seq.images_[i].clone();
      pointclouds_[i] = seq.pointclouds_[i]->makeShared();
    }

    return *this;
  }  

  size_t KinectSequence::totalPixels() const
  {
    return images_.size() * pixelsPerFrame();
  }

  size_t KinectSequence::pixelsPerFrame() const
  {
    if(images_.empty())
      return 0;
    else
      return images_[0].rows * images_[0].cols;
  }
  
  void loadSequences(const std::string& path,
                     std::vector<KinectSequence::Ptr>* sequences)
  {
    size_t prev_num_seq = sequences->size();
    
    bfs::directory_iterator end_itr; // default construction yields past-the-end
    for(bfs::directory_iterator itr(path); itr != end_itr; ++itr) {
      string p = itr->path().string();
      if(!bfs::is_directory(p))
        continue;

      KinectSequence::Ptr seq(new KinectSequence());
      seq->load(p);
      ROS_DEBUG_STREAM("Loaded sequence " << p << " with  " << seq->images_.size() << " frames.");
      sequences->push_back(seq);
    }

    // -- If it was just a single sequence directory, load that.
    if(sequences->size() == prev_num_seq) { 
      KinectSequence::Ptr seq(new KinectSequence());
      seq->load(path);
      ROS_DEBUG_STREAM("Loaded sequence " << path << " with  " << seq->images_.size() << " frames.");
      sequences->push_back(seq);
    }
  }

  void visualizeSeedLabels(cv::Mat1b seed, cv::Mat3b img, cv::Mat3b vis)
  {    
    img.copyTo(vis);
    for(int y = 0; y < vis.rows; ++y) {
      for(int x = 0; x < vis.cols; ++x) {
        if(seed(y, x) == 0)
          vis(y, x) = cv::Vec3b(0, 0, 0);
        else if(seed(y, x) == 255)
          vis(y, x) = cv::Vec3b(255, 255, 255);
      }
    }
  }

  void visualizeSegmentation(cv::Mat1b seg, cv::Mat3b img, cv::Mat3b vis)
  {
    // -- Dull the colors of the background.
    cv::Mat3b dull_bg = img.clone();
    for(int y = 0; y < seg.rows; ++y) { 
      for(int x = 0; x < seg.cols; ++x) { 
        if(seg(y, x) != 255) { 
          dull_bg(y, x)[0] *= 0.5; 
          dull_bg(y, x)[1] *= 0.5;
          dull_bg(y, x)[2] *= 0.5;
        }
      }
    }

    // -- Add a red border around the object.    
    cv::Mat3b mask = dull_bg.clone();
    cv::Mat1b dilation;
    cv::dilate(seg, dilation, cv::Mat(), cv::Point(-1, -1), 4);
    for(int y = 0; y < seg.rows; ++y) 
      for(int x = 0; x < seg.cols; ++x)
        if(dilation(y, x) == 255 && seg(y, x) != 255)
          mask(y, x) = cv::Vec3b(0, 0, 255);
    cv::addWeighted(dull_bg, 0.5, mask, 0.5, 0.0, vis);
  }
  
  void KinectSequence::saveVisualization(const std::string& path) const
  {
    if(bfs::exists(path)) { 
      ROS_ERROR_STREAM("Save path " << path << " already exists.  Not saving visualization.");
      return;
    }

    if(segmentations_.size() != images_.size() || seed_images_.size() != images_.size()) {
      ROS_ERROR_STREAM("Sequence must have an equal number of images, seed image, and segmentations to call saveVisualization().");
      return;
    }

    bfs::create_directory(path);
    for(size_t i = 0; i < segmentations_.size(); ++i) {
      {
        cv::Mat3b vis(segmentations_[i].size());
        visualizeSegmentation(segmentations_[i], images_[i], vis);
        ostringstream oss;
        oss << path << "/segvis" << setw(4) << setfill('0') << i << ".png";
        cv::imwrite(oss.str(), vis);
      }
      
      {
        // -- Only save seed visualizations that have content.
        bool flag = false;
        for(int y = 0; y < seed_images_[i].rows; ++y)
          for(int x = 0; x < seed_images_[i].cols; ++x)
            if(seed_images_[i](y, x) == 0 || seed_images_[i](y, x) == 255)
              flag = true;
        if(flag) { 
          cv::Mat3b vis(seed_images_[i].size());
          visualizeSeedLabels(seed_images_[i], images_[i], vis);
          ostringstream oss;
          oss << path << "/seedvis" << setw(4) << setfill('0') << i << ".png";
          cv::imwrite(oss.str(), vis);
        }
      }
    }
  }
  
} // namespace dst
