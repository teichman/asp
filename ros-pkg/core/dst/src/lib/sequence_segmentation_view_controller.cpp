#include <eigen_extensions/eigen_extensions.h>
#include <dst/sequence_segmentation_view_controller.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::visualization;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define VWEIGHTS (getenv("VWEIGHTS"))

namespace dst
{

  SequenceSegmentationViewController::SequenceSegmentationViewController(KinectSequence::Ptr seq,
                                                                         SegmentationPipeline::Ptr sp) :
    OpenCVViewDelegate(),
    Lockable(),
    seq_(seq),
    seg_view_("Segmentation"),
    vis_("Visualizer"),
    img_view_("Image"),
    seed_radius_(3),
    sp_(sp),
    current_idx_(0),
    quitting_(false),
    needs_redraw_(true),
    state_(RAW),
    vis_type_(0),
    show_seg_3d_(false),
    max_range_(3.0),
    cluster_tol_(0.02),
    background_model_(new KinectCloud)
  {
    img_view_.setDelegate((OpenCVViewDelegate*)this);
    img_view_.message_scale_ = 0.25;
    img_view_.message_thickness_ = 1.0;
    
    segmented_pcds_.resize(seq->images_.size());
    for(size_t i = 0; i < segmented_pcds_.size(); ++i)
      segmented_pcds_[i] = generateForeground(seq->segmentations_[i], *seq->pointclouds_[i]);

    // -- Hardcode the camera for the Kinect.
    ROS_WARN("[SequenceSegmentationViewController] PCLVisualizer's camera param has been removed.  Initial camera pose needs to be set somehow...");
    // vis_.camera_.clip[0] = 0.00387244;
    // vis_.camera_.clip[1] = 3.87244;
    // vis_.camera_.focal[0] = -0.160878;
    // vis_.camera_.focal[1] = -0.0444743;
    // vis_.camera_.focal[2] = 1.281;
    // vis_.camera_.pos[0] = 0.0402195;
    // vis_.camera_.pos[1] = 0.0111186;
    // vis_.camera_.pos[2] = -1.7;
    // vis_.camera_.view[0] = 0;
    // vis_.camera_.view[1] = -1;
    // vis_.camera_.view[2] = 0;
    // vis_.camera_.window_size[0] = 1678;
    // vis_.camera_.window_size[1] = 525;
    // vis_.camera_.window_pos[0] = 2;
    // vis_.camera_.window_pos[1] = 82;
    // vis_.updateCamera();

    vis_.registerKeyboardCallback(&SequenceSegmentationViewController::keyboardEventOccurred, *this);
    vis_.setBackgroundColor(255, 255, 255);
    PointCloudColorHandlerCustom<PointXYZRGB> single_color(seq_->pointclouds_[current_idx_], 0, 0, 0);
    vis_.addPointCloud(seq_->pointclouds_[current_idx_], single_color, "original");
    vis_.addPointCloud(segmented_pcds_[current_idx_], "segmented");
    vis_.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "original");
    vis_.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 5, "segmented");
  }

  void SequenceSegmentationViewController::keyboardEventOccurred(const KeyboardEvent &event, void* data)
  {
    if(event.keyDown()) {
      //cout << event.getKeySym() << endl;
      if(event.getKeySym().size() == 1)
        handleKeypress(event.getKeySym());
      else if(event.getKeySym().compare("period") == 0)
        handleKeypress('.');
      else if(event.getKeySym().compare("comma") == 0)
        handleKeypress(',');
    }
  }
  
  KinectCloud::Ptr SequenceSegmentationViewController::generateForeground(cv::Mat1b seg, const KinectCloud& cloud) const
  {
    KinectCloud::Ptr fg(new KinectCloud);
    for(int y = 0; y < seg.rows; ++y) {
      for(int x = 0; x < seg.cols; ++x) {
        if(seg(y, x) == 255) {
          int idx = y * seg.cols + x;
          fg->push_back(cloud[idx]);
        }
      }
    }
    return fg;
  }

  SequenceSegmentationViewController::~SequenceSegmentationViewController()
  {
  }
  
  void SequenceSegmentationViewController::run( const std::string& cmd )
  {
    ROS_ASSERT(seq_);
    seed_vis_ = seq_->images_[current_idx_].clone();
    seg_vis_  = cv::Mat3b(seq_->images_[current_idx_].size(), 127);

    // Handle automated startup commands.
    std::string::const_iterator i = cmd.begin();
    while(i != cmd.end()) {
      vis_.spinOnce(50);
      if(needs_redraw_)
        draw();

      img_view_.cvWaitKey(300);
      handleKeypress(*i, true);

      if(quitting_)
        break;

      ++ i;
    }

    if(quitting_) 
      return;

    while(true) {
      vis_.spinOnce(5);
      if(needs_redraw_)
        draw();

      char key = img_view_.cvWaitKey(30);
      handleKeypress(key);

      if(quitting_)
        break;
    }
  }

  void SequenceSegmentationViewController::increaseSeedWeights()
  {
    ROS_INFO_STREAM("Increasing seed weights.  Assuming seed weight is the 0th node weight.");
    Eigen::VectorXd w = sp_->getWeights();
    w(sp_->getEdgeWeights().rows()) *= 10.0;
    sp_->setWeights(w);
  }
  
  void SequenceSegmentationViewController::useSegmentationAsSeed()
  {
    seq_->seed_images_[current_idx_] = seq_->segmentations_[current_idx_].clone();
    // cv::Mat1b seed = seq_->seed_images_[current_idx_];
    // for(int y = 0; y < seed.rows; ++y) {
    //   for(int x = 0; x < seed.cols; ++x) {
    //         if(seed(y, x) == 127)
    //           seed(y, x) = 0;
    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::toggleDebug()
  {
    sp_->toggleDebug();
    
    if(sp_->getDebug())
      cout << "Debug mode is on." << endl;
    else
      cout << "Debug mode is off." << endl;
  }
  
  void SequenceSegmentationViewController::saveGraphviz() const
  {
    string filename = "segmentation_pipeline_graphviz";
    ofstream f;
    f.open(filename.c_str());
    f << sp_->getGraphviz();
    f.close();

    cout << "Saved pipeline graphviz to " << filename << endl;
  }

  void SequenceSegmentationViewController::saveVisualization()
  {
    string path;
    cout << "Save path: " << flush;
    cin >> path;
    cout << "Saving visualization to " << path << endl;
    seq_->saveVisualization(path);
    
    for(current_idx_ = 0; current_idx_ < (int)seq_->segmentations_.size(); ++current_idx_) {
      draw();
      ostringstream oss;
      oss << path << "/pcdvis" << setw(4) << setfill('0') << current_idx_ << ".png";
      cout << "Saving to " << oss.str() << endl;
      vis_.saveScreenshot(oss.str());
    }
    --current_idx_;

    cout << "Done." << endl;
  }
  
  void SequenceSegmentationViewController::handleKeypress(string key, bool automated)
  {
    ROS_ASSERT(key.size() == 1);
    handleKeypress((char)(key[0]), automated);
  }
  
  void SequenceSegmentationViewController::handleKeypress(char key, bool automated)
  {
    lock();
    
    // -- Global keys.
    string retval;
    switch(key) {
    case '!':
      quitting_ = automated;
      break;
    case 'q':
      cout << "Really quit? " << endl;
      cin >> retval;
      if(retval.compare("y") == 0)
        quitting_ = true;
      break;
    case ',':
      advance(-1);
      break;
    case '.':
      advance(1);
      break;
    case '<':
      advance(-100);
      break;
    case '>':
      advance(100);
      break;
    case 'F':
      increaseSeedWeights();
      break;
    case 'l':
      transitionTo(SEED);
      break;
    case 'r':
      transitionTo(RAW);
      break;
    case 'g':
      saveGraphviz();
      break;
    case 'v':
      show_seg_3d_ = !show_seg_3d_;
      needs_redraw_ = true;
      break;
    case 'V':
      saveVisualization();
      break;
    case '[':
      max_range_ -= 0.33;
      needs_redraw_ = true;
      break;
    case ']':
      max_range_ += 0.33;
      needs_redraw_ = true;
      break;
    default:
      break;
    }

    // -- State-specific keys.
    switch(state_) {
    case SEED:
      switch(key) {
      case 'D':
        toggleDebug();
        break;
      case 'i':
        segmentImage();
        break;
      case 't':
        ++vis_type_;
        if(vis_type_ > 2)
          vis_type_ = 0;
               needs_redraw_ = true;
        break;
      case 's':
        segmentSequence();
        break;
      case 'R':
        segmentSequence(current_idx_);
        break;
      case 'S':
        saveSequence();
        break;
      case 'L':
        cout << "Using segmentation as seed image." << endl;
        useSegmentationAsSeed();
        break;
      case '+':
        ++seed_radius_;
        cout << "Seed radius: " << seed_radius_ << endl;
        break;
      case '-':
        --seed_radius_;
        if(seed_radius_ < 0)
          seed_radius_ = 0;
        cout << "Seed radius: " << seed_radius_ << endl;
        break;
      case 'C':
        clearHelperSeedLabels();
        cout << "Cleared seed labels for all frames but the first." << endl;
        break;
      case 'c':
        seq_->seed_images_[current_idx_] = 127;
        needs_redraw_ = true;
        break;
      case 'e':
        extractConnectedComponent();
        break;
      case 'E':
        for(current_idx_ = 0; current_idx_ < (int)seq_->segmentations_.size(); ++current_idx_) {
          cout << current_idx_ << " / " << seq_->segmentations_.size() << endl;
          useSegmentationAsSeed();
          extractConnectedComponent();
          segmentImage();
        }
        --current_idx_;
        break;
      case 'm':
        *background_model_ += *seq_->pointclouds_[current_idx_];
        cout << "Background model now has " << background_model_->size() << " points." << endl;
        break;
      case 'M':
        background_model_->clear();
        cout << "Background model cleared." << endl;
        break;
      case 'b':
        segmentUsingBackgroundModel();
        break;
      case 'B':
        segmentAllUsingBackgroundModel();
        break;
      case '*':
        cluster_tol_ *= 2.0;
        cout << "Cluster tolerance: " << cluster_tol_ << endl;
        break;
      case '/':
        cluster_tol_ /= 2.0;
        cout << "Cluster tolerance: " << cluster_tol_ << endl;
        break;
      case 'p':
        extractPlane();
        break;
      default:
        break;
      }
      break;
      
    case RAW:
      switch(key) {
      case 'v':
        cout << "Cloud is (wxh) " <<  seq_->pointclouds_[current_idx_]->width << " x "
             << seq_->pointclouds_[current_idx_]->height << endl;
        cout << "Number of points: " << seq_->pointclouds_[current_idx_]->points.size() << endl;
        cout << "is_dense: " << seq_->pointclouds_[current_idx_]->is_dense << endl;
        cout << "sensor origin: " << seq_->pointclouds_[current_idx_]->sensor_origin_.transpose() << endl;
        break;
      default:
        break;
      }
    default:
      break;
    }
    
    unlock();
  }

  void SequenceSegmentationViewController::extractPlane()
  {
    // -- Make a pointcloud with just the background seed labels.
    const KinectCloud& cloud = *seq_->pointclouds_[current_idx_];
    cv::Mat1b seed = seq_->seed_images_[current_idx_];
    vector<int> indices;
    for(size_t i = 0; i < cloud.size(); ++i) {
      if(!isFinite(cloud[i]))
        continue;
      int y = (int)i / cloud.width;
      int x = (int)i - y * cloud.width;
      if(seed(y, x) == 0)
        indices.push_back(i);
    }
    if(indices.empty()) {
      cout << "No bg pts." << endl;
      return;
    }

    // -- Fit the best plane.
    KinectCloud::Ptr bg(new KinectCloud);
    bg->resize(indices.size());
    for(size_t i = 0; i < indices.size(); ++i)
      (*bg)[i] = cloud[indices[i]];

    double tol = 0.005;
    SampleConsensusModelPlane<Point>::Ptr plane(new SampleConsensusModelPlane<Point>(bg));
    RandomSampleConsensus<Point> ransac(plane);
    ransac.setDistanceThreshold(tol);
    ransac.computeModel();
    std::vector<int> inliers;
    ransac.getInliers(inliers);  // inliers indexes into bg
    VectorXf raw_coefs;
    ransac.getModelCoefficients(raw_coefs);
    VectorXf coefs;
    plane->optimizeModelCoefficients(inliers, raw_coefs, coefs);

    cout << "Plane fit coefficients: " << coefs.transpose() << endl;
    cout << "Num inliers among bg points: " << inliers.size() << endl;
    for(size_t i = 0; i < inliers.size(); ++i)
      cout << "Pt: " << (*bg)[inliers[i]].getVector4fMap().transpose() << ", a^T pt: " << coefs.dot((*bg)[inliers[i]].getVector4fMap()) << endl;

    int num = 0;
    for(size_t i = 0; i < cloud.size(); ++i) {
      if(fabs(coefs.dot(cloud[i].getVector4fMap())) < tol) { 
        int y = (int)i / cloud.width;
        int x = (int)i - y * cloud.width;
        seed(y, x) = 0;
        ++num;
      }
    }
    cout << "Total num inliers set to bg: " << num << endl;
    
    needs_redraw_ = true;
  }

  void SequenceSegmentationViewController::segmentAllUsingBackgroundModel()
  {
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(background_model_);

    for(current_idx_ = 0; current_idx_ < (int)seq_->segmentations_.size(); ++current_idx_) {
      cout << current_idx_ << " / " << seq_->segmentations_.size() << endl;
      segmentUsingBackgroundModel(tree);
      draw();
      cv::waitKey(10);
    }
    --current_idx_;
    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::segmentUsingBackgroundModel(pcl::search::KdTree<Point>::Ptr tree)
  {
    if(background_model_->empty()) { 
      cout << "No background model yet." << endl;
      return;
    }

    if(!tree) { 
      tree = pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>);
      tree->setInputCloud(background_model_);
    }

    cout << "Segmenting using background model..." << endl;
    vector<int> indices;
    vector<float> distances;
    cv::Mat1b seed = seq_->seed_images_[current_idx_];
    KinectCloud& pcd = *seq_->pointclouds_[current_idx_];
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(isnan(pcd[i].x))
        continue;
      
      indices.clear();
      distances.clear();
      tree->radiusSearch(pcd[i], 0.01, indices, distances, 1);
      
      int y = i / pcd.width;
      int x = i - y * pcd.width;
      if(indices.empty())
        seed(y, x) = 255;
      else
        seed(y, x) = 0;
    }

    extractConnectedComponent();
    segmentImage();
  }
  
  void SequenceSegmentationViewController::extractConnectedComponent()
  {
    // -- Make a pointcloud with just the foreground seed labels.
    const KinectCloud& cloud = *seq_->pointclouds_[current_idx_];
    cv::Mat1b seed = seq_->seed_images_[current_idx_];
    KinectCloud::Ptr fg(new KinectCloud);
    vector<int> indices;
    for(size_t i = 0; i < cloud.size(); ++i) {
      int y = (int)i / cloud.width;
      int x = (int)i - y * cloud.width;
      if(seed(y, x) == 255 && isFinite(cloud[i])) { 
        fg->push_back(cloud[i]);
        indices.push_back(i);
      }
    }
    if(fg->empty()) {
      cout << "No fg pts." << endl;
      return;
    }
    
    // -- Run euclidean cluster extraction.
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(fg);
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(cluster_tol_);
    ec.setMinClusterSize(0);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(fg);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    cout << "Found " << cluster_indices.size() << " clusters." << endl;

    // -- Find the biggest cluster.
    int maxnum = -1;
    int biggest = -1;
    for(size_t i = 0; i < cluster_indices.size(); ++i) {
      vector<int>& ind = cluster_indices[i].indices;
      if((int)ind.size() > maxnum) {
        maxnum = ind.size();
        biggest = i;
      }
    }
    
    // -- Unlabel all points not in the biggest cluster.
    for(size_t i = 0; i < cluster_indices.size(); ++i) {
      if((int)i == biggest)
        continue;
      
      vector<int>& ind = cluster_indices[i].indices;
      for(size_t j = 0; j < ind.size(); ++j) {
        int idx = indices[ind[j]]; // Indexes into the original cloud.
        int y = idx / cloud.width;
        int x = idx - y * cloud.width;
        seed(y, x) = 0; // Explicitly set these to background.
      }
    }

    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::clearHelperSeedLabels()
  {
    for(size_t i = 1; i < seq_->seed_images_.size(); ++i)
      seq_->seed_images_[i] = 127;

    needs_redraw_ = true;
  }

  void SequenceSegmentationViewController::updatePCLVisualizer()
  {
    ostringstream oss;
    oss << "Frame " << setw(4) << setfill('0') << current_idx_;
    vis_.removeShape("frame_number");
    vis_.addText(oss.str(), 10, 10, 0, 0, 0, "frame_number");
    vis_.updatePointCloud(segmented_pcds_[current_idx_], "segmented");
    PointCloudColorHandlerCustom<PointXYZRGB> single_color(seq_->pointclouds_[current_idx_], 0, 0, 0);
    vis_.updatePointCloud(seq_->pointclouds_[current_idx_], single_color, "original");
    vis_.spinOnce(5);
  }
  
  void SequenceSegmentationViewController::draw()
  {
    updatePCLVisualizer();
    
    drawSegVis();
    seg_view_.updateImage(seg_vis_);

    ostringstream oss;
    oss << "raw - " << current_idx_;
    switch(state_) {
    case RAW:
      
      img_view_.message_ = oss.str();
      img_view_.updateImage(seq_->images_[current_idx_]);
      break;
    case SEED:
      img_view_.message_ = "";
      drawSeedVis();
      img_view_.updateImage(seed_vis_);
    default:
      break;
    }

    needs_redraw_ = false;
  }

  void SequenceSegmentationViewController::drawSegVis()
  {
    for(int y = 0; y < seg_vis_.rows; ++y) {
      for(int x = 0; x < seg_vis_.cols; ++x) {
        switch(seq_->segmentations_[current_idx_](y, x)) {
        case 127:
          seg_vis_(y, x) = cv::Vec3b(127, 127, 127);
          break;
        case 0:
          seg_vis_(y, x) = cv::Vec3b(0, 0, 0);
          break;
        case 255:
          seg_vis_(y, x) = cv::Vec3b(255, 255, 255); //seq_->images_[current_idx_](y, x);
          break;
        default:
          break;
        }
      }
    }
  }

  void SequenceSegmentationViewController::drawSeedVis()
  {
    
    cv::Mat3b vis;
        
    if(vis_type_ == 0)
      vis = seq_->images_[current_idx_];
    else if(vis_type_ == 1)
      vis = sp_->getZBuffer(*seq_->pointclouds_[current_idx_], 0, 0.5, max_range_);
    else if(vis_type_ == 2)
      vis = sp_->getSurfNorm(*seq_->pointclouds_[current_idx_]);
    else
      abort();
    
    for(int y = 0; y < seed_vis_.rows; ++y) {
      for(int x = 0; x < seed_vis_.cols; ++x) {
        switch(seq_->seed_images_[current_idx_](y, x)) {
        case 127:
          seed_vis_(y, x) = vis(y, x);
          break;
        case 0:
          seed_vis_(y, x) = cv::Vec3b(0, 0, 0);
          break;
        case 255:
          seed_vis_(y, x) = cv::Vec3b(255, 255, 255);
          break;
        default:
          break;
        }
      }
    }
  }

  void SequenceSegmentationViewController::segmentImage()
  {
    assert( seq_->images_[current_idx_].rows > 0);

    sp_->reset();
    sp_->run(seq_->seed_images_[current_idx_],
            seq_->images_[current_idx_],
            seq_->pointclouds_[current_idx_],
            cv::Mat3b(),
            cv::Mat1b(),
            KinectCloud::Ptr(),
            seq_->segmentations_[current_idx_],
            segmented_pcds_[current_idx_]);
    needs_redraw_ = true;
  }

  void SequenceSegmentationViewController::segmentSequence(int idx)
  {
    cout << "Segmenting sequence starting with idx " << idx << endl;
    
    // -- Erase all segmentations after idx.
    for(size_t i = idx + 1; i < seq_->segmentations_.size(); ++i)
      seq_->segmentations_[i] = 127;
    
    current_idx_ = idx;
    draw();
    cv::waitKey(10);
    
    sp_->reset();
    for(; current_idx_ < (int)seq_->images_.size(); ++current_idx_) {
      // First in the sequence doesn't get passed in 'prev' data.
      if(current_idx_ == 0) {
        sp_->run(seq_->seed_images_[current_idx_],
                seq_->images_[current_idx_],
                seq_->pointclouds_[current_idx_],
                cv::Mat3b(),
                cv::Mat1b(),
                KinectCloud::Ptr(),
                seq_->segmentations_[current_idx_],
                segmented_pcds_[current_idx_]);
      }
      else if(current_idx_ == idx) {
        sp_->run(seq_->segmentations_[current_idx_].clone(),  // Seed it with the segmentation previously computed.
                 seq_->images_[current_idx_],
                 seq_->pointclouds_[current_idx_],
                 cv::Mat3b(),
                 cv::Mat1b(),
                 KinectCloud::Ptr(),
                 seq_->segmentations_[current_idx_],
                 segmented_pcds_[current_idx_]);
      }
      else {
        ROS_ASSERT(seq_->pointclouds_[current_idx_ - 1]);
        ROS_ASSERT(seq_->pointclouds_[current_idx_]);
        sp_->run(seq_->seed_images_[current_idx_],
                 seq_->images_[current_idx_],
                 seq_->pointclouds_[current_idx_],
                 seq_->images_[current_idx_-1],
                 seq_->segmentations_[current_idx_-1],
                 seq_->pointclouds_[current_idx_-1],
                 seq_->segmentations_[current_idx_],
                 segmented_pcds_[current_idx_]);
      }

      draw();
      int wait_time = 10;
      if(sp_->getDebug()) {
        cout << "Press s to stop, any other key to continue." << endl;
        //wait_time = 0;
        
        string filename;
        filename = generateFilename("debug", "segmented_pointcloud.pcd", 4);
        // Writer fails if there are no points?
        if(segmented_pcds_[current_idx_]->size() == 0) {
          pcl::PointXYZRGB pt;
          pt.x = 0; pt.y = 0; pt.z = -20;
          segmented_pcds_[current_idx_]->push_back(pt);
          segmented_pcds_[current_idx_]->push_back(pt);
          segmented_pcds_[current_idx_]->push_back(pt);
        }
        pcl::io::savePCDFileBinary(filename, *segmented_pcds_[current_idx_]);
        filename = generateFilename("debug", "original_pointcloud.pcd", 4);
        pcl::io::savePCDFileBinary(filename, *seq_->pointclouds_[current_idx_]);
        filename = generateFilename("debug", "segmentation_mask.png", 4);
        cv::imwrite(filename, seq_->segmentations_[current_idx_]);
        filename = generateFilename("debug", "original_image.png", 4);
        cv::imwrite(filename, seq_->images_[current_idx_]);
        filename = generateFilename("debug", "segmented_image.png", 4);
        cv::imwrite(filename, seg_vis_);
      }
      
      char key = img_view_.cvWaitKey(wait_time);
      if(key == 's')
        break;
    }
    cout << "Sequence segmentation ended." << endl;

    if((size_t)current_idx_ == seq_->images_.size())
      current_idx_ = seq_->images_.size() - 1;
  }

  void SequenceSegmentationViewController::saveSequence()
  {
    // bool flag = true;
    // for(size_t i = 1; flag && i < seq_->seed_images_.size(); ++i)
    //   for(int y = 0; flag && y < seq_->seed_images_[i].rows; ++y)
    //         for(int x = 0; flag && x < seq_->seed_images_[i].cols; ++x)
    //           if(seq_->seed_images_[i](y, x) != 127)
    //             flag = false;
    
    // if(!flag) { 
    //   cout << "This will clear all seed labels except those in the first frame." << endl;
    //   cout << "Continue?  (y/n): " << endl;
    //   string retval;
    //   cin >> retval;
    //   if(retval.compare("y") != 0) {
    //         cout << "Aborted." << endl;
    //         return;
    //   }
    //   clearHelperSeedLabels();
    //   draw();
    //   cv::waitKey(10);
    // }
    
    cout << "Dir name for new sequence: " << endl;
    string dirname;
    cin >> dirname;
    seq_->save(dirname);
    cout << "Saved to " << dirname << endl;
  }
  
  void SequenceSegmentationViewController::advance(int num)
  {
    current_idx_ += num;
    if(current_idx_ < 0)
      current_idx_ = 0;
    if((size_t)current_idx_ >= seq_->images_.size())
      current_idx_ = seq_->images_.size() - 1;

    needs_redraw_ = true;
    cout << "Timestamp of pcd: " << seq_->pointclouds_[current_idx_]->header.stamp << endl;
  }
  
  void SequenceSegmentationViewController::mouseEvent(int event, int x, int y, int flags, void* param)
  {
    //lock();
    if(state_ != SEED) {
      //unlock();
      return;
    }

    // -- Left click to add to source.
    if(flags & CV_EVENT_FLAG_LBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) { 
        for(int j = y - seed_radius_; j <= y + seed_radius_; ++j) {
          if(i >= 0 && i < seed_vis_.cols &&
             j >= 0 && j < seed_vis_.rows) {

            seq_->seed_images_[current_idx_](j, i) = 255;
          }
        }
      }
      needs_redraw_ = true;
    }

    // -- Right click to add to sink.
    else if(flags & CV_EVENT_FLAG_RBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) { 
        for(int j = y - seed_radius_; j <= y + seed_radius_; ++j) {
          if(i >= 0 && i < seed_vis_.cols &&
             j >= 0 && j < seed_vis_.rows) { 

            seq_->seed_images_[current_idx_](j, i) = 0;
          }
        }
      }
      needs_redraw_ = true;
    }

    //unlock();
  }
  
  void SequenceSegmentationViewController::transitionTo(state_t state) { 
    state_ = state;
    needs_redraw_ = true;
  }
  
} // namespace dst
