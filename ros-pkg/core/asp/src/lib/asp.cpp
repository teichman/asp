#include <asp/asp.h>

using namespace std;
using namespace pl;
using namespace Eigen;

namespace asp
{

  double sigmoid(double z)
  {
    return 1.0 / (1.0 + exp(-z));
  }
  
  Asp::Asp(int num_threads) :
    Pipeline(num_threads)
  {
    registerPodTypes();
    
    createPod("EntryPoint<cv::Mat1b>", "MaskEntryPoint");
    createPod("EntryPoint<cv::Mat3b>", "ImageEntryPoint");
    createPod("EntryPoint<cv::Mat1b>", "SeedEntryPoint");
    createPod("NodePotentialAggregator", "NodePotentialAggregator");
    createPod("EdgePotentialAggregator", "EdgePotentialAggregator");
    connect("NodePotentialAggregator.Image <- ImageEntryPoint.Output");
    connect("EdgePotentialAggregator.Image <- ImageEntryPoint.Output");
    createPod("GraphcutsPod", "GraphcutsPod");
    connect("GraphcutsPod.AggregatedNodePotentials <- NodePotentialAggregator.Node");
    connect("GraphcutsPod.AggregatedEdgePotentials <- EdgePotentialAggregator.Edge");
    connect("GraphcutsPod.Image <- ImageEntryPoint.Output");
    createPod("SeedNPG", "SeedNPG");
    connect("SeedNPG.Image <- ImageEntryPoint.Output");
    connect("SeedNPG.SeedImage <- SeedEntryPoint.Output");
    connect("NodePotentialAggregator.UnweightedNode <- SeedNPG.Node");
    createPod("PriorNPG", "PriorNPG");
    connect("PriorNPG.Image <- ImageEntryPoint.Output");
    connect("NodePotentialAggregator.UnweightedNode <- PriorNPG.Node");
  }

  void Asp::registerPodTypes()
  {
    REGISTER_POD_TEMPLATE(EntryPoint, cv::Mat3b);
    REGISTER_POD_TEMPLATE(EntryPoint, cv::Mat1b);
    REGISTER_POD(NodePotentialAggregator);
    REGISTER_POD(EdgePotentialAggregator);
    REGISTER_POD(EdgeStructureGenerator);
    REGISTER_POD(GraphcutsPod);
    REGISTER_POD(SeedNPG);
    REGISTER_POD(PriorNPG);
    REGISTER_POD(SmoothnessEPG);
    REGISTER_POD(SimpleColorDifferenceEPG);
  }

  Asp::Asp(int num_threads, YAML::Node config) :
    Pipeline(num_threads)
  {
    deYAMLize(config);
  }
  
  Model Asp::defaultModel() const
  {
    Model mod = model();
    mod.nweights_ = VectorXd::Ones(mod.nameMapping("nmap").size());
    mod.eweights_ = VectorXd::Ones(mod.nameMapping("emap").size());
    return mod;
  }

  Model Asp::model() const
  {
    Model mod;
    NodePotentialAggregator* npa = (NodePotentialAggregator*)pod("NodePotentialAggregator");
    npa->fillModel(&mod);
    EdgePotentialAggregator* epa = (EdgePotentialAggregator*)pod("EdgePotentialAggregator");
    epa->fillModel(&mod);
    return mod;
  }
  
  void Asp::setModel(const Model& mod)
  {
    NodePotentialAggregator* npa = (NodePotentialAggregator*)pod("NodePotentialAggregator");
    npa->setWeights(mod);
    EdgePotentialAggregator* epa = (EdgePotentialAggregator*)pod("EdgePotentialAggregator");
    epa->setWeights(mod);
  }

  void Asp::segment(cv::Mat1b* seg_img, PotentialsCache* pc)
  {
    compute();
    *seg_img = pull<cv::Mat1b>("GraphcutsPod", "Segmentation");

    if(pc) {     
      pod<NodePotentialAggregator>()->fillPotentialsCache(pc);
      pod<EdgePotentialAggregator>()->fillPotentialsCache(pc);
    }
  }

  void NodePotentialGenerator::initializeStorage()
  {
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    if(node_.rows() != img.rows || node_.cols() != img.cols)
      node_.resize(img.rows, img.cols);

    node_.setZero();
  }
  
  void NodePotentialGenerator::writeNodePotentialVisualization() const
  {
    // Node potentials should be in [-1, 1].
    // NodePotentialAggregator is exempted because that is after weighting and summing.
    if(!dynamic_cast<const NodePotentialAggregator*>(this)) {
      ROS_ASSERT(node_.maxCoeff() <= 1);
      ROS_ASSERT(node_.minCoeff() >= -1);
    }
    
    // -- Just the potentials.
    cv::Mat3b raw;
    raw = cv::Mat3b(node_.rows(), node_.cols(), cv::Vec3b(0, 0, 0));
    
    for(int y = 0; y < raw.rows; ++y) { 
      for(int x = 0; x < raw.cols; ++x) {
        // -1 for bg, +1 for fg.
        double val = 2.0 * sigmoid(1.1 * (node_(y, x))) - 1.0;
        val = min(0.9, max(-0.9, val));
        if(val < 0)
          raw(y, x)[1] = 255 * -val;
        else
          raw(y, x)[2] = 255 * val;          
      }
    }

    double scale = 3;
    cv::Mat3b scaled_raw;
    cv::Size sz;
    sz.width = raw.cols * scale;
    sz.height = raw.rows * scale;
    cv::resize(raw, scaled_raw, sz);
    string raw_path = debugBasePath() + "-raw.png";
    cv::imwrite(raw_path, scaled_raw);
    cout << "Wrote node potential visualization to " << raw_path << endl;

    // -- Overlay.
    if(numIncoming("Image") == 0)
      return;

    cv::Mat3b img = pull<cv::Mat3b>("Image");
    cv::Mat3b vis;
    vis = img.clone();
    ROS_ASSERT(vis.rows > 0 && vis.cols > 0);
    for(int y = 0; y < vis.rows; ++y) { 
      for(int x = 0; x < vis.cols; ++x) {
        double val = sigmoid(5.0 * (node_(y, x))); // 1.0 for foreground.
        val = min(0.9, max(0.1, val));
        vis(y, x)[0] = vis(y, x)[0] * val;
        vis(y, x)[1] = vis(y, x)[1] * val;
        vis(y, x)[2] = vis(y, x)[2] * val;
      }
    }

    cv::Mat3b scaled;
    sz.width = vis.cols * scale;
    sz.height = vis.rows * scale;
    cv::resize(vis, scaled, sz);
    string overlay_path = debugBasePath() + "-overlay.png";
    cv::imwrite(overlay_path, scaled);
    cout << "Wrote node potential visualization to " << overlay_path << endl;
  }

  NameMapping NodePotentialAggregator::generateNameMapping() const
  {
    vector<string> names = upstreamOutputNames("UnweightedNode");
    string cruft = ".Node";
    NameMapping nmap;
    for(size_t i = 0; i < names.size(); ++i)
      nmap.addName(names[i].substr(0, names[i].size() - cruft.size()));

    return nmap;
  }
  
  void NodePotentialAggregator::setWeights(Model model)
  {
    // The model must have the correct number of weights otherwise 
    // this makes no sense.
    NameMapping nmap = generateNameMapping();
    ROS_ASSERT(nmap.size() == (size_t)model.nweights_.rows());

    // Check that the model has the same set of node potentials.
    for(size_t i = 0; i < nmap.size(); ++i)
      model.nameMapping("nmap").hasName(nmap.toName(i));
    
    // Make the numbers in the model have the same ordering as the
    // node potentials in this aggregator.  Apply them.
    model.applyNameMapping("nmap", nmap);
    nweights_ = model.nweights_;
  }

  void NodePotentialAggregator::fillModel(Model* model) const
  {        
    model->applyNameMapping("nmap", generateNameMapping());
    model->nweights_ = nweights_;
  }

  void NodePotentialAggregator::fillPotentialsCache(PotentialsCache* pc) const
  {
    pc->node_.clear();
    pc->applyNameMapping("nmap", generateNameMapping());
    
    vector<const MatrixXd*> node;
    multiPull("UnweightedNode", &node);
    pc->node_.resize(node.size());
    for(size_t i = 0; i < node.size(); ++i) {
      pc->node_[i].resize(node[i]->rows() * node[i]->cols());
      int idx = 0;
      for(int y = 0; y < node[i]->rows(); ++y)
        for(int x = 0; x < node[i]->cols(); ++x, ++idx)
          pc->node_[i].coeffRef(idx) = node[i]->coeffRef(y, x);
    }
  }

  void NodePotentialAggregator::compute()
  {
    initializeStorage();
    
    vector<const MatrixXd*> node;
    multiPull("UnweightedNode", &node);
    ROS_ASSERT((size_t)nweights_.rows() == node.size());
    for(size_t i = 0; i < node.size(); ++i)
      node_ += (*node[i]) * nweights_[i];
    
    push<const MatrixXd*>("Node", &node_);
  }

  void NodePotentialAggregator::debug() const
  {
    writeNodePotentialVisualization();
  }
  
  void EdgePotentialGenerator::initializeStorage(double reserve_per_node)
  {
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    initializeSparseMat(img.rows, img.cols, reserve_per_node, &edge_);
    
    // const SparseMat& structure = *pull<const SparseMat*>("EdgeStructure");
    // initializeSparseMat(structure.rows(), structure.cols(),
    //                         (double)structure.nonZeros() / (structure.rows() * structure.cols()),
    //                         &edge_);
  }

  void EdgePotentialGenerator::writeEdgePotentialVisualization(bool white_background) const
  {
    cv::Mat3b img = pull<cv::Mat3b>("Image").clone();
    if(white_background)
      img = cv::Vec3b(255, 255, 255);
    cv::Mat3b vis = drawEdgeVisualization(img, edge_);
    double minval = std::numeric_limits<double>::max();
    double maxval = -std::numeric_limits<double>::max();
    for(int i = 0; i < edge_.rows(); ++i) {
      SparseMatrix<double, RowMajor>::InnerIterator it(edge_, i);
      for(; it; ++it) {
        minval = min(minval, it.value());
        maxval = max(maxval, it.value());
      }
    }
    // Unweighted edge potentials should be in [0, 1].
    // EdgePotentialAggregator is exempted because that is after weighting and summing.
    if(!dynamic_cast<const EdgePotentialAggregator*>(this)) {
      ROS_ASSERT(maxval <= 1);  
      ROS_ASSERT(minval >= 0);
    }

    cout << name() << ": range of edge weights is " << minval << " to " << maxval << endl;
    string overlay_path = debugBasePath() + ".png";
    if(white_background)
      overlay_path = debugBasePath() + "-white_background.png";
    
    cv::imwrite(overlay_path, vis);
  }
  
  void EdgePotentialAggregator::compute()
  {
    initializeStorage();

    vector<const SparseMat*> unweighted;
    multiPull("UnweightedEdge", &unweighted);
    ROS_ASSERT((size_t)eweights_.rows() == unweighted.size());
    for(size_t i = 0; i < unweighted.size(); ++i) {
      edge_ += eweights_(i) * (*unweighted[i]);
    }

    push<const SparseMat*>("Edge", &edge_);
  }

  void EdgePotentialAggregator::debug() const
  {
    writeEdgePotentialVisualization();
    writeEdgePotentialVisualization(true);
  }

  NameMapping EdgePotentialAggregator::generateNameMapping() const
  {
    vector<string> names = upstreamOutputNames("UnweightedEdge");
    string cruft = ".Edge";
    NameMapping emap;
    for(size_t i = 0; i < names.size(); ++i)
      emap.addName(names[i].substr(0, names[i].size() - cruft.size()));
    
    return emap;
  }

  void EdgePotentialAggregator::setWeights(Model model)
  {
    // The model must have the correct number of weights otherwise 
    // this makes no sense.
    NameMapping emap = generateNameMapping();
    ROS_ASSERT(emap.size() == (size_t)model.eweights_.rows());

    // Check that the model has the same set of edge potentials.
    for(size_t i = 0; i < emap.size(); ++i)
      model.nameMapping("emap").hasName(emap.toName(i));
    
    // Make the numbers in the model have the same ordering as the
    // edge potentials in this aggregator.  Apply them.
    model.applyNameMapping("emap", emap);
    eweights_ = model.eweights_;
  }

  void EdgePotentialAggregator::fillModel(Model* model) const
  {
    model->applyNameMapping("emap", generateNameMapping());
    model->eweights_ = eweights_;
  }

  void EdgePotentialAggregator::fillPotentialsCache(PotentialsCache* pc) const
  {
    pc->edge_.clear();
    pc->applyNameMapping("emap", generateNameMapping());

    vector<const SparseMat*> edge;
    multiPull("UnweightedEdge", &edge);
    pc->edge_.resize(edge.size());
    for(size_t i = 0; i < edge.size(); ++i)
      pc->edge_[i] = *edge[i];
  }

  void GraphcutsPod::compute()
  {
    const MatrixXd* node;
    const SparseMat* edge;
    pull("AggregatedNodePotentials", &node);
    pull("AggregatedEdgePotentials", &edge);

    // TODO: Could probably do this just once.  Does it matter?
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    int num_nodes = img.rows * img.cols;
    int max_num_edges = param<int>("ExpectedNumEdges") * img.rows * img.cols;
    Graph3d graph(num_nodes, max_num_edges);
    graph.add_node(num_nodes);

    // -- Fill the graph with node potentials.
    for(int i = 0; i < node->rows(); ++i) {
      for(int j = 0; j < node->cols(); ++j) {
        int idx = i * node->cols() + j;
        graph.add_tweights(idx, node->coeffRef(i, j), 0);
      }
    }

    // -- Fill the graph with edge potentials.
    //    TODO: Should this use symmetric or asymmetric edge potentials?
    SparseMatrix<double, Eigen::RowMajor> trans(edge->transpose());  // Unfortunately, yes.
    SparseMatrix<double, Eigen::RowMajor> sym = (*edge + trans) / 2.0;
    for(int i = 0; i < sym.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, i); it; ++it) {
        if(it.col() <= it.row())
          continue;

        ROS_WARN_STREAM_COND(it.value() < 0, "Edgepot weighted sum is negative: " << it.value());
        ROS_FATAL_STREAM_COND(isnan(it.value()), "NaN in edgepot.");
        graph.add_edge(it.col(), it.row(), it.value(), it.value());
      }
    }

    // -- Compute the segmentation.
    graph.maxflow();

    // -- Pull it out from the maxflow library.
    if(seg_.rows != node->rows() || seg_.cols != node->cols())
      seg_ = cv::Mat1b(cv::Size(node->cols(), node->rows()));

    for(int y = 0; y < seg_.rows; ++y) {
      for(int x = 0; x < seg_.cols; ++x) {
        int idx = index(y, x, seg_.cols);

        if(graph.what_segment(idx, Graph3d::SINK) == Graph3d::SOURCE)
          seg_(y, x) = 255;
        else if(graph.what_segment(idx, Graph3d::SOURCE) == Graph3d::SINK)
          seg_(y, x) = 0;
        else
          seg_(y, x) = 127;
      }
    }

    push<cv::Mat1b>("Segmentation", seg_);
  }

  void GraphcutsPod::debug() const
  {
    // -- Just the segmentation.
    string seg_path = debugBasePath() + "-segmentation-raw.png";
    cv::imwrite(seg_path, seg_);

    // -- Overlay.
    if(numIncoming("Image") == 0)
      return;

    cv::Mat3b img = pull<cv::Mat3b>("Image");
    cv::Mat3b vis;
    vis = img.clone();
    ROS_ASSERT(vis.rows > 0 && vis.cols > 0);
    visualizeSegmentation(seg_, img, vis);

    string overlay_path = debugBasePath() + "-segmentation-outline.png";
    cv::imwrite(overlay_path, vis);
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

  void SeedNPG::compute()
  {
    initializeStorage();
    
    cv::Mat1b seed = pull<cv::Mat1b>("SeedImage");
    for(int y = 0; y < seed.rows; ++y) {
      for(int x = 0; x < seed.cols; ++x) {
        if(seed(y, x) == 255)
          node_(y, x) = 1;
        else if(seed(y, x) == 0)
          node_(y, x) = -1;
      }
    }

    push<const MatrixXd*>("Node", &node_);
  }

  void SeedNPG::debug() const
  {
    writeNodePotentialVisualization();
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    string path = debugBasePath() + "-original_image.png";
    cv::imwrite(path, img);
  }

  void PriorNPG::compute()
  {
    initializeStorage();
    node_.setConstant(-1);
    push<const MatrixXd*>("Node", &node_);
  }
  
  void EdgeStructureGenerator::compute()
  {
    // -- Initialize the structure.
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    cv::Mat1b mask = pull<cv::Mat1b>("Mask");
    ROS_ASSERT(img.rows == mask.rows && img.cols == mask.cols);
    
    double reserve_per_pixel = 0;
    if(param<bool>("Grid"))
      reserve_per_pixel += 4;
    if(param<bool>("Diagonal"))
      reserve_per_pixel += 4;
    if(param<bool>("Web"))
      reserve_per_pixel += param<double>("WebNumOutgoing");
    initializeSparseMat(img.rows, img.cols, reserve_per_pixel, &structure_);

    // -- Generate edges.
    if(param<bool>("Grid")) {
      int idx = 0;
      for(int y = 0; y < img.rows; ++y) {
        for(int x = 0; x < img.cols; ++x, ++idx) {
          if(mask(y, x) == 0)
            continue;
          if(x < img.cols - 1)
            structure_.insert(idx, index(y, x + 1, img.cols)) = 1;
          if(y < img.rows - 1)
            structure_.insert(idx, index(y + 1, x, img.cols)) = 1;
        }
      }
    }
    if(param<bool>("Diagonal")) {
      initializeSparseMat(img.rows, img.cols, 4, &diag_);
      int idx = 0;
      for(int y = 0; y < img.rows - 1; ++y) {
        for(int x = 0; x < img.cols; ++x, ++idx) {
          if(mask(y, x) == 0)
            continue;
          if(x > 0)
            diag_.insert(idx, index(y + 1, x - 1, img.cols)) = 1;
          if(x < img.cols - 1)
            diag_.insert(idx, index(y + 1, x + 1, img.cols)) = 1;
        }
      }
      structure_ += diag_;
    }
    if(param<bool>("Web")) {
      initializeSparseMat(img.rows, img.cols, param<double>("WebNumOutgoing"), &web_);

      // web_.coeffRef(index(0, 0, img.cols), index(50, 50, img.cols)) = 1;
      // web_.coeffRef(index(50, 50, img.cols), index(50, 75, img.cols)) = 1;
      // web_.coeffRef(index(25, 47, img.cols), index(50, 50, img.cols)) = 1;
      // web_.coeffRef(index(25, 75, img.cols), index(50, 50, img.cols)) = 1;
      // dyn.coeffRef(index(49, 80, img.cols), index(50, 50, img.cols)) = 1;
      // dyn.coeffRef(index(50, 50, img.cols), index(51, 80, img.cols)) = 1;

      // int y, x;
      // int idx = index(50, 50, img.cols);
      // x = 50;
      // y = 55;
      // for(int dx = -10; dx <= 10; ++dx)
      //         dyn.coeffRef(idx, index(y, x + dx, img.cols)) = 1;
      // x = 50;
      // y = 45;
      // for(int dx = -10; dx <= 10; ++dx)
      //         dyn.coeffRef(index(y, x + dx, img.cols), idx) = 1;

      int idx = 0;
      int num_outgoing = param<double>("WebNumOutgoing");
      float max_radius = param<double>("WebMaxRadius");
      eigen_extensions::Uniform01Sampler uniform;
      for(int y = 0; y < img.rows; ++y) {
        for(int x = 0; x < img.cols; ++x, ++idx) {
          if(mask(y, x) == 0)
            continue;
          
          int num_edges = 0;
          int num_attempts = 0;
          int max_num_attempts = num_outgoing * 2;
          while(num_edges < num_outgoing && num_attempts < max_num_attempts) {
            ++num_attempts;
            double radius = uniform.sample() * max_radius;
            double theta = uniform.sample() * 2 * M_PI;
            int dx = radius * cos(theta);
            int dy = radius * sin(theta);
            int x0 = x + dx;
            int y0 = y + dy;
            if(y0 < 0 || y0 >= img.rows || x0 < 0 ||
               x0 >= img.cols || (dx == 0 && dy == 0) ||
               mask(y0, x0) == 0)
            {
              // cout << "Edge from (" << x << ", " << y << ") to (" << x0 << ", " << y0 << ") is invalid.";
              // cout << "  Size: " << img.cols << " x " << img.rows << ".  radius: " << radius << ", theta: " << theta << endl;
              continue;
            }
            // cout << "*** Edge from (" << x << ", " << y << ") to (" << x0 << ", " << y0 << ") is valid." << endl;
            // cout << "radius: " << radius << ", theta: " << theta << ", dx: " << dx << ", dy: " << dy << endl;
            
            int idx0 = index(y0, x0, img.cols);
            if(idx < idx0)
              web_.coeffRef(idx, idx0) = 1;
            else
              web_.coeffRef(idx0, idx) = 1;

            ++num_edges;
          }
        }
      }

      structure_ += web_;
    }
    
    push<const SparseMat*>("EdgeStructure", &structure_);
  }

  void EdgeStructureGenerator::debug() const
  {
    for(int i = 0; i < structure_.rows(); ++i) {
      SparseMat::InnerIterator it(structure_, i);
      for(; it; ++it)
        ROS_ASSERT(it.col() >= it.row());
    }
    
    cout << "EdgeStructureGenerator: " << structure_.nonZeros() << " edges with average weight of nonzeros of " << structure_.sum() / (structure_.nonZeros()) << endl;

    // MatrixXd dense(structure_);
    // cout << "Middle 20 x 20: " << endl;
    // cout << dense.block(dense.rows() / 2, dense.cols() / 2, 20, 20) << endl;
    
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    cv::Mat3b vis = drawEdgeVisualization(img, structure_);
    string overlay_path = debugBasePath() + ".png";
    cv::imwrite(overlay_path, vis);
  }

  void SimpleColorDifferenceEPG::compute()
  {
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    const SparseMat& structure = *pull<const SparseMat*>("EdgeStructure");
    double sigma = param<double>("Sigma");

    initializeStorage();
    //edge_ = structure;
    
    int idx = 0;
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x, ++idx) {
        SparseMat::InnerIterator it(structure, idx);
        for(; it; ++it) {
          int y1, x1;
          pixel(it.col(), img.cols, &y1, &x1);
          cv::Vec3b p = img(y, x);
          cv::Vec3b q = img(y1, x1);
          double norm = sqrt((p[0] - q[0])*(p[0] - q[0]) +
                             (p[1] - q[1])*(p[1] - q[1]) +
                             (p[2] - q[2])*(p[2] - q[2]));
          edge_.insert(it.row(), it.col()) = exp(-norm / sigma);  // TODO: Is there a faster way to fill this?
        }
      }
    }

    push<const SparseMat*>("Edge", &edge_);
  }

  void SimpleColorDifferenceEPG::debug() const
  {
    writeEdgePotentialVisualization();
  }

  void SmoothnessEPG::compute()
  {
    push("Edge", pull<const SparseMat*>("EdgeStructure"));
  }

  void SmoothnessEPG::debug() const
  {
  }

    
  /************************************************************
   * Miscellaneous functions
   ************************************************************/
  
  int sign(int x)
  {
    return (x > 0) - (x < 0);
  }

  void drawLine(cv::Point2d pt, const cv::Point2d& pt1, double potential, cv::Mat3b vis)
  {
    ROS_ASSERT(pt.x != pt1.x || pt.y != pt1.y);
    
    if(pt1.x < 0 || pt1.x >= vis.cols)
      return;
    if(pt1.y < 0 || pt1.y >= vis.rows)
      return;

    double dx = pt1.x - pt.x;
    double dy = pt1.y - pt.y;
    double maxabs = max(fabs(dx), fabs(dy));
    dx /= maxabs;
    dy /= maxabs;
    double x = pt.x;
    double y = pt.y;
    double sdx = sign(dx);
    double sdy = sign(dy);
    //cout << pt << ", " << pt1 << ", " << dx << ", " << dy << ", " << sdx << ", " << sdy << endl;
    while(true) {
      vis(y, x)[0] = vis(y, x)[0] * (1.0 - potential);
      vis(y, x)[1] = vis(y, x)[1] * (1.0 - potential);
      vis(y, x)[2] = vis(y, x)[2] * (1.0 - potential);
      
      x += dx;
      y += dy;
      
      if(sdx * x > sdx * pt1.x || sdy * y > sdy * pt1.y)
        break;
      if(x < 0 || y < 0 || x >= vis.cols || y >= vis.rows)
        break;
    }
  }

  cv::Mat3b drawEdgeVisualization(cv::Mat3b img, const SparseMat& edge)
  {
    ROS_ASSERT(img.rows > 0 && img.cols > 0);
    double scale = 7;
    cv::Size sz(img.cols * scale, img.rows * scale);
    cv::Mat3b vis;
    cv::resize(img, vis, sz, cv::INTER_NEAREST);
    for(int y = 0; y < vis.rows; ++y)
      for(int x = 0; x < vis.cols; ++x)
        vis(y, x) = img(floor(y / scale), floor(x / scale));

    // -- Get min and max.
    double minval = std::numeric_limits<double>::max();
    double maxval = -std::numeric_limits<double>::max();
    for(int i = 0; i < edge.rows(); ++i) {
      SparseMatrix<double, RowMajor>::InnerIterator it(edge, i);
      for(; it; ++it) {
        minval = min(minval, it.value());
        maxval = max(maxval, it.value());
      }
    }
          
    // -- Draw non-zero edges.
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x) {
        int idx0 = index(y, x, img.cols);

        SparseMat::InnerIterator it(edge, idx0);
        for(; it; ++it) {
          int idx1 = it.col();
          if(idx1 == idx0) {
            ROS_WARN_ONCE("At least one edge from a node to itself encountered in drawEdgeVisualization.");
            continue;
          }
          int y1 = idx1 / img.cols;
          int x1 = idx1 - y1 * img.cols;

          cv::Point pt(x * scale + scale * 0.5, y * scale + scale * 0.5);
          cv::Point pt1(x1 * scale + scale * 0.5, y1 * scale + scale * 0.5);
          drawLine(pt, pt1, it.value() / maxval, vis);
        }
      }
    }

    return vis;
  }

  void imageToVectorSegmentation(cv::Mat1b seg_img, Eigen::VectorXi* seg_vec)
  {
    seg_vec->resize(seg_img.rows * seg_img.cols);
    int idx = 0;
    for(int y = 0; y < seg_img.rows; ++y) {
      for(int x = 0; x < seg_img.cols; ++x, ++idx) {
        if(seg_img(y, x) == 255)
          seg_vec->coeffRef(idx) = 1;
        else if(seg_img(y, x) == 0)
          seg_vec->coeffRef(idx) = -1;
        else
          seg_vec->coeffRef(idx) = 0;
      }
    }
  }

  
}  // namespace asp


