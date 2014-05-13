#include <dst/segmentation_pipeline.h>

#define NUM_GC_THREADS (getenv("NUM_GC_THREADS") ? atoi(getenv("NUM_GC_THREADS")) : 8)
#define VISUALIZE (getenv("VISUALIZE"))
#define BIL_SIGMA_DIST (getenv("BIL_SIGMA_DIST") ? atof(getenv("BIL_SIGMA_DIST")) : 0.05)
#define BIL_SIGMA_COLOR (getenv("BIL_SIGMA_COLOR") ? atof(getenv("BIL_SIGMA_COLOR")) : 5.0)
#define DEPG_SIGMA_NORM (getenv("DEPG_SIGMA_NORM") ? atof(getenv("DEPG_SIGMA_NORM")) : 0.010)
#define DEPG_SIGMA_EUC (getenv("DEPG_SIGMA_EUC") ? atof(getenv("DEPG_SIGMA_EUC")) : 0.01)

#define SKIP (getenv("SKIP") ? atoi(getenv("SKIP")) : 1)

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;
using namespace pipeline2;

namespace dst
{

  SegmentationPipeline::SegmentationPipeline(int num_threads) :
    pipeline_(num_threads),
    verbose_(true),
    num_threads_(num_threads)
{
    image_ep_ = new EntryPoint<cv::Mat3b>("CurrentImage");
    seed_ep_ = new EntryPoint<cv::Mat1b>("SeedImage");
    previous_image_ep_  = new EntryPoint<cv::Mat3b>("PreviousImage");
    previous_segmentation_ep_  = new EntryPoint<cv::Mat1b>("PreviousSegmentation");
    cloud_ep_ = new EntryPoint<KinectCloud::ConstPtr>("Cloud");
    previous_cloud_ep_ = new EntryPoint<KinectCloud::ConstPtr>("PreviousCloud");
    graph_ep_ = new EntryPoint<Graph3dPtr>("Graph");                                                   

    boundary_mask_node_ = new BoundaryMaskNode(&previous_segmentation_ep_->outlet_,
                                               &cloud_ep_->outlet_,
                                               &previous_cloud_ep_->outlet_,
                                               &image_ep_->outlet_,
                                               &seed_ep_->outlet_, 12, 0.2);

    depth_projector_ = new DepthProjector(&image_ep_->outlet_, &cloud_ep_->outlet_);
    OpticalFlowNode* optflow = new OpticalFlowNode(&depth_projector_->index_otl_);
                                                                                                      
    KdTreeNode* kdtree = new KdTreeNode(&cloud_ep_->outlet_,
                                        &boundary_mask_node_->pcd_indices_otl_, 0.03);
    scene_alignment_node_ = new SceneAlignmentNode(&optflow->optflow_otl_,
                                                   &depth_projector_->index_otl_);
                                                                 
    normals_node_ = new OrganizedSurfaceNormalNode(&cloud_ep_->outlet_,
                                                   &boundary_mask_node_->mask_otl_, 1);
    ForegroundKdTreeNode* fg_kdtree_node;
    fg_kdtree_node = new ForegroundKdTreeNode(&previous_segmentation_ep_->outlet_,
                                              &depth_projector_->index_otl_);
    
    
    // ----------------------------------------
    // -- Edge potentials.
    // ----------------------------------------

    CannyEPG* canny = new CannyEPG(&image_ep_->outlet_, 75, 100);
    ColorDeltaEPG* color_delta = new ColorDeltaEPG(&image_ep_->outlet_);
    // ROS_DEBUG_STREAM("DEPG_SIGMA_NORM: " << DEPG_SIGMA_NORM);
    // ROS_DEBUG_STREAM("DEPG_SIGMA_EUC: " << DEPG_SIGMA_EUC);
    DepthEPG* depth = new DepthEPG(&depth_projector_->index_otl_,
                                   &normals_node_->normals_otl_,
                                   &boundary_mask_node_->mask_otl_,
                                   1, DEPG_SIGMA_NORM, DEPG_SIGMA_EUC);
    SurfaceNormalEPG* normal_epg = new SurfaceNormalEPG(&normals_node_->normals_otl_,
                                                            &depth_projector_->index_otl_,
                                                        &boundary_mask_node_->mask_otl_,
                                                            &image_ep_->outlet_, 0.32);

    // Edge products.

    vector<EdgePotentialGenerator*> eg0;
    eg0.push_back(canny);
    eg0.push_back(color_delta);
    eg0.push_back(depth);
    eg0.push_back(normal_epg);
    EdgePotentialProduct* epp0 = new EdgePotentialProduct(&image_ep_->outlet_, eg0);
    
    vector<EdgePotentialGenerator*> eg1;
    eg1.push_back(canny);
    eg1.push_back(depth);
    EdgePotentialProduct* epp1 = new EdgePotentialProduct(&image_ep_->outlet_, eg1);

    vector<EdgePotentialGenerator*> eg2;
    eg2.push_back(color_delta);
    eg2.push_back(depth);
    EdgePotentialProduct* epp2 = new EdgePotentialProduct(&image_ep_->outlet_, eg2);

    vector<EdgePotentialGenerator*> eg3;
    eg3.push_back(normal_epg);
    eg3.push_back(depth);
    EdgePotentialProduct* epp3 = new EdgePotentialProduct(&image_ep_->outlet_, eg3);
    

    // Edge aggregator.
    vector<EdgePotentialGenerator*> edge_generators;
    edge_generators.push_back(canny);
    edge_generators.push_back(color_delta);
    edge_generators.push_back(depth);
    edge_generators.push_back(normal_epg);
    edge_generators.push_back(epp0);
    edge_generators.push_back(epp1);
    edge_generators.push_back(epp2);
    edge_generators.push_back(epp3);
    
    VectorXd eweights = VectorXd::Ones(edge_generators.size());
    edge_aggregator_ = new EdgePotentialAggregator(&graph_ep_->outlet_,
                                                   &image_ep_->outlet_,
                                                   &depth_projector_->index_otl_,
                                                   edge_generators,
                                                   eweights,
                                                   true,
                                                   NULL);

    // ----------------------------------------
    // -- Node potentials.
    // ----------------------------------------

    
    // SceneAlignmentNPG* sanpg = new SceneAlignmentNPG(&scene_alignment->transformed_otl_,
    //                                                      &kdtree->kdtree_otl_,
    //                                                      &previous_segmentation_ep_->outlet_,
    //                                                      &depth_projector_->index_otl_);

    // SKIP: 1, 3, 5
    // sigma_dist: 0.005, 0.01, 0.05
    // sigma_color: 50.0, 5.0, 100.0
    
    BilateralNPG* bilateral = new BilateralNPG(&kdtree->kdtree_otl_,
                                               &scene_alignment_node_->transformed_otl_,
                                               &previous_segmentation_ep_->outlet_,
                                               &depth_projector_->index_otl_,
                                               &boundary_mask_node_->mask_otl_,
                                               BIL_SIGMA_DIST, BIL_SIGMA_COLOR, 5.0, SKIP, SKIP);  
    
    SeedNPG* seed = new SeedNPG(&seed_ep_->outlet_);
    // SeedDistanceNPG* sd = new SeedDistanceNPG(&seed_ep_->outlet_,
    //                                               &depth_projector_->index_otl_,
    //                                               1.0);

    LabelFlowNPG* label_flow = new LabelFlowNPG(&optflow->optflow_otl_, &previous_segmentation_ep_->outlet_);


    IntensityImageNode* intensity_node = new IntensityImageNode(&image_ep_->outlet_);
    IntegralImageNode* integral_node = new IntegralImageNode(&intensity_node->outlet_);
    HSVImageNode* hsv_node = new HSVImageNode(&image_ep_->outlet_);
    PatchClassifierNPG* patch_classifier5 = new PatchClassifierNPG(&image_ep_->outlet_,
                                                                   &intensity_node->outlet_,
                                                                   &integral_node->outlet_,
                                                                   &hsv_node->outlet_,
                                                                   &previous_segmentation_ep_->outlet_,
                                                                   &boundary_mask_node_->mask_otl_,
                                                                   7, SKIP); // 1 results in small variance.

    DistanceNPG* distance_npg = new DistanceNPG(&scene_alignment_node_->transformed_otl_,
                                                &fg_kdtree_node->kdtree_otl_,
                                                &image_ep_->outlet_,
                                                &boundary_mask_node_->pcd_indices_otl_,
                                                0.15);
                                                    
    IcpNPG* icp_npg0 = new IcpNPG(&depth_projector_->index_otl_,
                                      &kdtree->kdtree_otl_,
                                      &scene_alignment_node_->prev_to_curr_otl_,
                                      &previous_segmentation_ep_->outlet_,
                                      &boundary_mask_node_->pcd_indices_otl_,
                                      0.75, 0.05, 0.05, 0.02, 75, 0.001, true, SKIP);

    PriorNPG* prior_npg = new PriorNPG(&image_ep_->outlet_);

    TemplateMatcherNPG* template_matcher = new TemplateMatcherNPG(&depth_projector_->index_otl_,
                                                                  &image_ep_->outlet_,
                                                                  &boundary_mask_node_->mask_otl_);
                                                                  
    vector<NodePotentialGenerator*> node_generators;
    node_generators.push_back(seed);
    node_generators.push_back(bilateral);
    //node_generators.push_back(sd);
    node_generators.push_back(label_flow);
    node_generators.push_back(patch_classifier5);
    node_generators.push_back(distance_npg);
    node_generators.push_back(icp_npg0);
    node_generators.push_back(boundary_mask_node_);
    node_generators.push_back(prior_npg);
    node_generators.push_back(template_matcher);

    
    VectorXd nweights = VectorXd::Ones(node_generators.size());
    nweights(0) = 10;
    nweights(node_generators.size() - 1) = 0.0001;
    node_aggregator_ = new NodePotentialAggregator(&graph_ep_->outlet_,
                                                   &seed_ep_->outlet_,
                                                   &image_ep_->outlet_,
                                                   &depth_projector_->index_otl_,
                                                   edge_aggregator_,
                                                   node_generators,
                                                   nweights);
    
    pipeline_.addComponent(image_ep_);
  }
  
  void SegmentationPipeline::reset()
  {
    pipeline_.reset();
  }
  
  std::string SegmentationPipeline::getGraphviz() const
  {
    return pipeline_.getGraphviz();
  }

  Eigen::VectorXd SegmentationPipeline::getNodeWeights() const
  {
    return node_aggregator_->getWeights();
  }
  
  Eigen::VectorXd SegmentationPipeline::getEdgeWeights() const
  {
    return edge_aggregator_->getWeights();
  }

  std::string SegmentationPipeline::weightsStatus() const
  {
    ostringstream oss;
    oss << edge_aggregator_->weightsStatus();
    oss << node_aggregator_->weightsStatus();
    return oss.str();
  }
  
  void SegmentationPipeline::setWeightsWithClipping(Eigen::VectorXd weights, bool verbose)
  {
    ROS_ASSERT(weights.rows() == getEdgeWeights().rows() + getNodeWeights().rows());
    for(int i = 0; i < getEdgeWeights().rows(); ++i) {
      if(weights(i) < 0.0) {
        //ROS_DEBUG_STREAM_ONCE(30, "Weight " << i << " = " << weights(i) << ", clipping to zero.");
        weights(i) = 0;
      }
    }
    setWeights(weights, verbose);
  }
  
  void SegmentationPipeline::setWeights(const Eigen::VectorXd& weights, bool verbose)
  {
    ROS_ASSERT(weights.rows() == getEdgeWeights().rows() + getNodeWeights().rows());
    edge_aggregator_->setWeights(weights.head(getEdgeWeights().rows()), verbose);
    node_aggregator_->setWeights(weights.tail(getNodeWeights().rows()), verbose);
  }
  
  Eigen::VectorXd SegmentationPipeline::getWeights() const
  {
    VectorXd nw = getNodeWeights();
    VectorXd ew = getEdgeWeights();
    VectorXd weights(nw.rows() + ew.rows());
    weights.head(ew.rows()) = ew;
    weights.tail(nw.rows()) = nw;
    return weights;
  }

  PotentialsCache::Ptr SegmentationPipeline::cacheUnweightedPotentialsWithOracle(KinectSequence::Ptr seq)
  {
    reset();
    PotentialsCache::Ptr cache(new PotentialsCache());

    // TODO: Assert that there is ground truth.
    //for(size_t i = 0; i < seq->segmentations_.size(); ++i) {

    // -- Run segmentation on all frames.
    //    But pass in ground truth for prev_seg.
    //    Also, ignore seed for all following frames; we're assuming that
    //    seed labels in following frames were used to create
    //    ground truth info.
    KinectCloud::Ptr seg_pcd(new KinectCloud());
    cv::Mat1b throwaway_img_seg(seq->seed_images_[0].size(), 127);
    cv::Mat1b empty_seed(seq->seed_images_[0].size(), 127);
    for(size_t i = 0; i < seq->segmentations_.size(); ++i) {
      // Run segmentation.
      if(i == 0) {
        run(seq->seed_images_[i],
            seq->images_[i],
            seq->pointclouds_[i],
            cv::Mat3b(),
            cv::Mat1b(),
            KinectCloud::Ptr(),
            throwaway_img_seg, // Don't overwrite ground truth.
            seg_pcd);
      }
      else { 
        run(empty_seed,
            seq->images_[i],
            seq->pointclouds_[i],
            seq->images_[i-1],
            seq->segmentations_[i-1], // Pass in ground truth as prev seg.
            seq->pointclouds_[i-1],
            throwaway_img_seg, // Don't overwrite ground truth.
            seg_pcd);
      }

      // Get out the potentials.
      cache->framecaches_.push_back(getFrameCache());

      // Debugging
      if(VISUALIZE && NUM_GC_THREADS == 1) { 
        cv::imshow("OracleSegmentation", throwaway_img_seg);
        cv::imshow("Depth index", cache->framecaches_.back()->depth_index_);
        cv::waitKey(50);
      }
    }
    
    return cache;
  }

  FramePotentialsCache::Ptr SegmentationPipeline::getFrameCache() const
  {
    FramePotentialsCache::Ptr framecache(new FramePotentialsCache());
    edge_aggregator_->cacheUnweightedPotentials(framecache);
    node_aggregator_->cacheUnweightedPotentials(framecache);
    framecache->depth_index_ = depth_projector_->index_otl_.pull().current_index_.clone();
    
    return framecache;
  }

  cv::Mat1b SegmentationPipeline::findMostViolating(const FramePotentialsCache& fc,
                                                    cv::Mat1b labels,
                                                    bool hamming)
  {
    ROS_ASSERT(fc.depth_index_.rows > 0);

    // -- Allocate the graph.
    int num_nodes = labels.rows * labels.cols;
    int max_num_edges = 10 * labels.rows * labels.cols;
    graph_ = Graph3dPtr(new Graph3d(num_nodes, max_num_edges));
    graph_->add_node(num_nodes);
    
    // -- Get weighted source, sink, and edge potentials.
    VectorXd node_weights = getNodeWeights();
    ROS_ASSERT((size_t)node_weights.rows() == fc.source_potentials_.size());
    ROS_ASSERT((size_t)node_weights.rows() == fc.sink_potentials_.size());
    MatrixXd srcpot = MatrixXd::Zero(fc.source_potentials_[0].rows(),
                                     fc.source_potentials_[0].cols());
    for(size_t i = 0; i < fc.source_potentials_.size(); ++i)
      srcpot += node_weights(i) * fc.source_potentials_[i];

    MatrixXd snkpot = MatrixXd::Zero(fc.sink_potentials_[0].rows(),
                                     fc.sink_potentials_[0].cols());
    for(size_t i = 0; i < fc.sink_potentials_.size(); ++i)
      snkpot += node_weights(i) * fc.sink_potentials_[i];

    SparseMatrix<double, Eigen::RowMajor> epot(fc.edge_potentials_[0].rows(),
                                               fc.edge_potentials_[0].cols());
    VectorXd edge_weights = getEdgeWeights();
    ROS_ASSERT((size_t)edge_weights.rows() == fc.edge_potentials_.size());
    for(size_t i = 0; i < fc.edge_potentials_.size(); ++i)
      epot += edge_weights(i) * fc.edge_potentials_[i];

    // -- Add the node potentials corresponding to the Hamming loss.
    if(hamming) { 
      for(int y = 0; y < labels.rows; ++y) {
        for(int x = 0; x < labels.cols; ++x) {
          
          if(fc.depth_index_(y, x) == -1)
            continue;
          
          if(labels(y, x) == 255)
            snkpot(y, x) += 1.0;
          else if(labels(y, x) == 0)
            srcpot(y, x) += 1.0;
        }
      }
    }

    // -- Fill the graph with node potentials.
    ROS_ASSERT(srcpot.rows() == snkpot.rows());
    ROS_ASSERT(srcpot.cols() == snkpot.cols());
    for(int i = 0; i < srcpot.rows(); ++i) {
      for(int j = 0; j < srcpot.cols(); ++j) {
        int idx = i * srcpot.cols() + j;
        graph_->add_tweights(idx, srcpot(i, j), snkpot(i, j));
      }
    }

    // -- Fill the graph with edge potentials.
    //    TODO: Should this use symmetric or asymmetric edge potentials?
    //    If this is changed, then change the svm_struct_api.c's psi() and
    //    find_most_violated_constraint_marginrescaling() functions.
    SparseMatrix<double, Eigen::RowMajor> trans(epot.transpose());  // Unfortunately, yes.
    SparseMatrix<double, Eigen::RowMajor> sym = (epot + trans) / 2.0;
    for(int i = 0; i < sym.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, i); it; ++it) {
        if(it.col() <= it.row())
          continue;
        
        graph_->add_edge(it.col(), it.row(), it.value(), it.value());
      }
    }

    // -- Solve.
    HighResTimer hrt("maxflow");
    hrt.start();
    graph_->maxflow();
    hrt.stop();
    // cout << hrt.reportMilliseconds() << endl;
    // cout << "Maxflow result: " << flow << endl;

    // -- Fill the output.
    cv::Mat1b pred(labels.size(), 127);
    generateSegmentationFromGraph(*graph_, fc.depth_index_, pred);

    cv::Mat1b indvis(fc.depth_index_.size(), 0);
    bool error = false;
    for(int x = 0; x < indvis.cols; ++x) {
      for(int y = 0; y < indvis.rows; ++y) {
        if(fc.depth_index_(y, x) != -1)
          indvis(y, x) = 255;

        if(!((labels(y, x) == 127 && pred(y, x) == 127) ||
             (labels(y, x) == 255 && pred(y, x) == 255) ||
             (labels(y, x) == 0 && pred(y, x) == 0) ||
             (labels(y, x) == 0 && pred(y, x) == 255) ||
             (labels(y, x) == 255 && pred(y, x) == 0)))
        {
          ROS_ERROR_STREAM("label: " << (int)labels(y, x) << ", pred: " << (int)pred(y, x) << flush);
          error = true;
        }
        
        // TODO: Investigate why this fails when training on the test set.  label 127, pred 0.  1/31.
        // ROS_ASSERT((labels(y, x) == 127 && pred(y, x) == 127) ||
        //            (labels(y, x) == 255 && pred(y, x) == 255) ||
        //            (labels(y, x) == 0 && pred(y, x) == 0) ||
        //            (labels(y, x) == 0 && pred(y, x) == 255) ||
        //            (labels(y, x) == 255 && pred(y, x) == 0));
      }
    }
    if(VISUALIZE) { 
      cv::imshow("depth index", indvis);
      cv::imshow("ground truth", labels);
      cv::imshow("most violating", pred);
      cv::waitKey(50);
      if(error)
        cv::waitKey();
    }
    
    return pred;
  }

  cv::Mat3b SegmentationPipeline::getZBuffer(const KinectCloud& cloud,
                                             int spread,
                                             float min_range,
                                             float max_range) const
  {
    return depth_projector_->getZBuffer(cloud, spread, min_range, max_range);
  }

  cv::Mat3b SegmentationPipeline::getSurfNorm(const KinectCloud& cloud) const
  {
    return normals_node_->getSurfNorm(cloud);
  }
   
  void SegmentationPipeline::setDebug(bool debug)
  {
    pipeline_.setDebug(debug);
    if(debug) {
      ROS_ASSERT(!bfs::exists("debug") || bfs::is_directory("debug"));
      if(!bfs::exists("debug"))
        bfs::create_directory("debug");
    }
  }

  bool SegmentationPipeline::getDebug() const
  {
    return pipeline_.getDebug();
  }
  
  void SegmentationPipeline::toggleDebug()
  {
    setDebug(!getDebug());
  }
  
  void SegmentationPipeline::run(cv::Mat1b seed,
                                 cv::Mat3b image,
                                 KinectCloud::ConstPtr cloud,
                                 cv::Mat3b prev_image,
                                 cv::Mat1b prev_seg,
                                 KinectCloud::ConstPtr prev_cloud,
                                 cv::Mat1b img_seg,
                                 KinectCloud::Ptr pcd_seg)
  {
    ROS_ASSERT(img_seg.rows == image.rows);
    ROS_ASSERT(img_seg.cols == image.cols);
    ROS_ASSERT(seed.rows > 0);
    ROS_ASSERT(image.rows > 0);
    
    // -- Strip off NaNs.
    // KinectCloud::Ptr cleaned(new KinectCloud());
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*cloud, *cleaned, indices);
    // cout << "Cleaned is (wxh) " <<  cleaned->width << " x "
    //          << cleaned->height << endl;
    // cout << "Number of points: " << cleaned->points.size() << endl;
    // cout << "is_dense: " << cleaned->is_dense << endl;
    // cout << "sensor origin: " << cleaned->sensor_origin_.transpose() << endl;
    
    // -- For now, initialize a new graph for every run.  This might be slow.
    HighResTimer hrt("graph allocation");
    hrt.start();
    int num_nodes = image.rows * image.cols;
    int max_num_edges = 10 * image.rows * image.cols;
    graph_ = Graph3dPtr(new Graph3d(num_nodes, max_num_edges));
    graph_->add_node(num_nodes);
    hrt.stop();
    if(verbose_)
      cout << hrt.reportMilliseconds() << endl;

    hrt.reset("flush");
    hrt.start();
    pipeline_.flush();
    hrt.stop();
    if(verbose_)
      cout << hrt.reportMilliseconds() << endl;

    // -- Feed pipeline and run.
    image_ep_->setData(image);
    seed_ep_->setData(seed);
    cloud_ep_->setData(cloud);
    previous_cloud_ep_->setData(prev_cloud);
    graph_ep_->setData(graph_);
    previous_image_ep_->setData(prev_image);
    previous_segmentation_ep_->setData(prev_seg);
    
    hrt.reset("Total feature computation");
    hrt.start();
    pipeline_.compute();
    hrt.stop();
    if(verbose_) { 
      cout << pipeline_.reportTiming() << endl;
      cout << hrt.reportMilliseconds() << endl;
    }
    
    // -- Fill the graph with node potentials.
    MatrixXd& srcpot = *node_aggregator_->source_otl_.pull();
    MatrixXd& snkpot = *node_aggregator_->sink_otl_.pull();
    ROS_ASSERT(srcpot.rows() == snkpot.rows());
    ROS_ASSERT(srcpot.cols() == snkpot.cols());
    for(int i = 0; i < srcpot.rows(); ++i) {
      for(int j = 0; j < srcpot.cols(); ++j) {
        int idx = i * srcpot.cols() + j;
        graph_->add_tweights(idx, srcpot(i, j), snkpot(i, j));
      }
    }
    
    // -- Fill the graph with edge potentials.
    //    TODO: Should this use symmetric or asymmetric edge potentials?
    //    If this is changed, then change the svm_struct_api.c's psi() and
    //    find_most_violated_constraint_marginrescaling() functions.
    SparseMatrix<double, Eigen::RowMajor>& epot = *edge_aggregator_->edge_otl_.pull();
    SparseMatrix<double, Eigen::RowMajor> trans(epot.transpose());  // Unfortunately, yes.
    SparseMatrix<double, Eigen::RowMajor> sym = (epot + trans) / 2.0;
    for(int i = 0; i < sym.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, i); it; ++it) {
        if(it.col() <= it.row())
          continue;

        ROS_WARN_STREAM_COND(it.value() < 0, "Edgepot weighted sum is negative: " << it.value());
        ROS_FATAL_STREAM_COND(isnan(it.value()), "NaN in edgepot.");
        graph_->add_edge(it.col(), it.row(), it.value(), it.value());
      }
    }
    
    hrt.reset("maxflow");
    hrt.start();
    graph_->maxflow();
    hrt.stop();
    if(verbose_)
      cout << hrt.reportMilliseconds() << endl;
    
    generateSegmentationFromGraph(*graph_, 
                                  depth_projector_->index_otl_.pull().current_index_,
                                  img_seg, cloud, pcd_seg);
  }
  
  void SegmentationPipeline::generateSegmentationFromGraph(Graph3d& graph,
                                                           cv::Mat1i index,
                                                           cv::Mat1b img_seg,
                                                           KinectCloud::ConstPtr cloud,
                                                           KinectCloud::Ptr pcd_seg) const
  {
    ROS_ASSERT(img_seg.rows == index.rows);
    ROS_ASSERT(img_seg.cols == index.cols);

    img_seg = 127;
    if(pcd_seg)
      pcd_seg->clear();
    for(int y = 0; y < img_seg.rows; ++y) {
      for(int x = 0; x < img_seg.cols; ++x) {
        int img_idx = getIdx(y, x, img_seg.cols);
        int pcd_idx = index(y, x);

        // If no depth information, don't say anything.
        // NPA & EPA should have disconnected these pixels entirely.
        if(pcd_idx == -1) {
          
          ROS_ASSERT((graph.what_segment(img_idx, Graph<double, double, double>::SINK) == Graph<double, double, double>::SINK));

                     
          img_seg(y, x) = 127;
        }
        else { 
          if(graph.what_segment(img_idx, Graph<double, double, double>::SINK)
             == Graph<double, double, double>::SOURCE) {
            img_seg(y, x) = 255;
            if(pcd_seg)
              pcd_seg->push_back(cloud->at(pcd_idx));
          }
          else
            img_seg(y, x) = 0;
        }
      }
    }
  }

  SegmentationPipeline::Ptr SegmentationPipeline::mitosis(int num_threads) const
  {
    return SegmentationPipeline::Ptr(new SegmentationPipeline(num_threads));
  }

  void SegmentationPipeline::getNameMappings(NameMapping* epot_names, NameMapping* npot_names) const
  {
    *epot_names = edge_aggregator_->getNameMapping();
    *npot_names = node_aggregator_->getNameMapping();
  }
  
} // namespace dst
