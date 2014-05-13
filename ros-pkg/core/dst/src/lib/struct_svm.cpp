#include <dst/struct_svm.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 40)
#define NUM_GC_THREADS (getenv("NUM_GC_THREADS") ? atoi(getenv("NUM_GC_THREADS")) : 8)
#define DEBUG_LEVEL (getenv("DEBUG_LEVEL") ? atoi(getenv("DEBUG_LEVEL")) : 0)
#define TOL (getenv("TOL") ? atof(getenv("TOL")) : 0.2)
#define MAX_NORMALIZED_LOSS (getenv("MAX_NORMALIZED_LOSS") ? atof(getenv("MAX_NORMALIZED_LOSS")) : 0.1)
#define MIN_NORMALIZED_LOSS (getenv("MIN_NORMALIZED_LOSS") ? atof(getenv("MIN_NORMALIZED_LOSS")) : 0.0)
#define PRINT_MATRICES (getenv("PRINT_MATRICES"))
#define HYBRID (getenv("HYBRID"))
#define ROBUST (getenv("ROBUST"))
#define NSLACK (getenv("NSLACK"))
#define DEBUG (getenv("DEBUG"))
#define SAVE_INNER_WEIGHTS (getenv("SAVE_INNER_WEIGHTS"))
#define RETURN_BEST (getenv("RETURN_BEST") ? atoi(getenv("RETURN_BEST")) : 1)
#define CLEAR_CONSTRAINTS (getenv("CLEAR_CONSTRAINTS") ? atoi(getenv("CLEAR_CONSTRAINTS")) : 0)
#define ALL_NONNEG (getenv("ALL_NONNEG") ? atoi(getenv("ALL_NONNEG")) : 1)

using namespace std;
namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;
using namespace Eigen;
using namespace pipeline2;

namespace dst
{

  StructSVM::StructSVM(double c,
                       double precision,
                       bool margin_rescaling) :
    c_(c),
    precision_(precision),
    margin_rescaling_(margin_rescaling)
  {
  }
  
  void StructSVM::loadSequences(const std::string& path)
  {
    dst::loadSequences(path, &sequences_);
  }
  
  Eigen::VectorXd StructSVM::trainRobust2()
  {
    ROS_DEBUG_STREAM("MAX_NORMALIZED_LOSS: " << MAX_NORMALIZED_LOSS);
    ROS_DEBUG_STREAM("MIN_NORMALIZED_LOSS: " << MIN_NORMALIZED_LOSS);
    ROS_DEBUG_STREAM("Returning best weights: " << RETURN_BEST);
    
    // -- Initialize the training set with the oracle examples.
    // Caches must accumulate from one frame to the next or it can easily forget all its
    // progress in one bad round.
    vector<FramePotentialsCache::Ptr> caches;
    vector<cv::Mat1b> labels;

    if(HYBRID) {
      ROS_DEBUG_STREAM("Initializing with oracle training examples.");
      for(size_t i = 0; i < sequences_.size(); ++i) {
            KinectSequence::Ptr seq = sequences_[i];
            SegmentationPipeline sp(NUM_THREADS);
            PotentialsCache::Ptr cache = sp.cacheUnweightedPotentialsWithOracle(seq);
        
            for(size_t j = 0; j < cache->size(); ++j) { 
              caches.push_back(cache->framecaches_[j]);
              labels.push_back(seq->segmentations_[j]);
            }
      }
    }

    Eigen::VectorXd weights = VectorXd::Ones(getNumWeights());
    weights.normalize();
    Eigen::VectorXd prev_weights = weights;
    Eigen::VectorXd best_weights;
    int iter = 0;
    double best_total_normalized_loss = std::numeric_limits<double>::max();
    int best_total_segmented_frames = 0;
    int num_iters_since_improvement = 0;
    vector<Constraint> constraints;
    while(true) {
      ROS_DEBUG_STREAM("============================================================");
      ROS_DEBUG_STREAM("=          Starting outer iteration " << iter);
      ROS_DEBUG_STREAM("============================================================");
      int total_segmented_frames = 0;
      int total_frames = 0;
      int num_training_examples_added = 0;
      double total_normalized_loss = 0;

      // -- Save the weights on every iteration.
      ostringstream oss;
      oss << "weights" << setw(4) << setfill('0') << iter << ".eig.txt";
      eigen_extensions::saveASCII(weights, oss.str());
      
      // -- Build the training set in a way that will produce some non-optimal,
      //    realistic sets of potentials.
      for(size_t i = 0; i < sequences_.size(); ++i) {
        KinectSequence::Ptr seq = sequences_[i];
        total_frames += seq->segmentations_.size();

        SegmentationPipeline sp(NUM_THREADS);
        sp.verbose_ = false;
        sp.setWeightsWithClipping(weights);
        if(i == 0)
          ROS_DEBUG_STREAM("Weights: " << endl << sp.weightsStatus() << flush);
        cv::Mat1b working_img_seg(seq->seed_images_[0].size(), 127);
        cv::Mat1b empty_seed(seq->seed_images_[0].size(), 127);
        double seq_normalized_loss = 0;
        for(size_t j = 0; j < seq->segmentations_.size(); ++j) { 
          // Segment using current weights.
          KinectCloud::Ptr seg_pcd(new KinectCloud());  // Not used.
          if(j == 0) {
            sp.run(seq->seed_images_[j],
                   seq->images_[j],
                   seq->pointclouds_[j],
                   cv::Mat3b(),
                   cv::Mat1b(),
                   KinectCloud::Ptr(),
                   working_img_seg,
                   seg_pcd);
            continue; // Don't add training examples for seed frames.
          }
          else {
            sp.run(empty_seed,
                   seq->images_[j],
                   seq->pointclouds_[j],
                   seq->images_[j-1],
                   working_img_seg,
                   seq->pointclouds_[j-1],
                   working_img_seg,
                   seg_pcd);
            ++total_segmented_frames;
          }

          double normalized_loss = fgNormalizedHammingLoss(seq->segmentations_[j], working_img_seg);
          total_normalized_loss += normalized_loss;
          seq_normalized_loss += normalized_loss;
          
          // -- Check to see if the boundary mask includes the entire ground truth boundary.
          if(DEBUG) { 
            cv::Mat1b boundary = sp.boundary_mask_node_->mask_otl_.pull();
            cv::namedWindow("Boundary", 0);
            cv::namedWindow("Image", 0);
            cv::namedWindow("Segmentation", 0);
            cv::namedWindow("Ground truth", 0);
            cvMoveWindow("Boundary", 100, 600);
            cvMoveWindow("Image", 500, 600);
            cvMoveWindow("Segmentation", 100, 1000);
            cvMoveWindow("Ground truth", 500, 1000);
            cv::imshow("Boundary", boundary);
            cv::imshow("Image", seq->images_[j]);
            cv::imshow("Segmentation", working_img_seg);
            cv::imshow("Ground truth", seq->segmentations_[j]);
            cv::waitKey(10);
            cout << "normalized loss: " << normalized_loss << endl;
            cv::waitKey(0);
          }
          
          // If we've done really well on this frame, don't bother adding it to the training set.
          if(normalized_loss < MIN_NORMALIZED_LOSS) {
            if(j == seq->images_.size() - 1)
              ROS_DEBUG_STREAM("Sequence " << i << ": Completed " << j << " frames without making serious errors.  Mean normalized loss: "
                               << seq_normalized_loss / (double)j << flush);
            continue;
          }

          ++num_training_examples_added;
          labels.push_back(seq->segmentations_[j]);
          caches.push_back(sp.getFrameCache());


          // -- Make sure sp is consistent.
          if(DEBUG) { 
            SegmentationPipeline sp(1);
            sp.verbose_ = false;
            sp.setWeightsWithClipping(weights);
            ROS_ASSERT(!margin_rescaling_);
            cv::Mat1b ymv = sp.findMostViolating(*caches.back(), labels.back(), margin_rescaling_);

            cv::Mat1b diff(ymv.size());
            diff = 0;
            bool show = false;
            for(int y = 0; y < ymv.rows; ++y)
              for(int x = 0; x < ymv.cols; ++x)
                if(ymv(y, x) != working_img_seg(y, x)) {
                  ROS_ERROR_STREAM("SegPipe is inconsistent.");
                  show = true;
                  diff(y, x) = 255;
                }

            cv::imshow("Seg differences", diff);
            cv::waitKey(10);
            if(show) { 
              cv::waitKey();
            }
          }
          
          
          // Memory limit.
          if(caches.size() > 1500) {
            caches.erase(caches.begin(), caches.begin()+1);
            labels.erase(labels.begin(), labels.begin()+1);
          }
          
          // If we're starting to do really poorly or if we're done,
          // print out results and move on to the next sequence.
          if(normalized_loss >= MAX_NORMALIZED_LOSS) {
            ROS_DEBUG_STREAM("Sequence " << i << ": Normalized loss for frame " << j + 1 << " / " << seq->images_.size()
                             << " is " << normalized_loss << ".  Mean normalized loss: " << seq_normalized_loss / (double)j
                             << flush);
                             
            break;
          }
          else if(j == seq->images_.size() - 1)
            ROS_DEBUG_STREAM("Sequence " << i << ": Completed " << j + 1 << " frames without making serious errors.  Mean normalized loss: "
                             << seq_normalized_loss / (double)j << flush);
        }
      }

      ROS_DEBUG_STREAM("Added " << num_training_examples_added << " training examples.");
      ROS_DEBUG_STREAM(total_segmented_frames << " frames segmented, mean normalized loss of "
                       << total_normalized_loss / (double)total_segmented_frames);
      
      // -- Determine if this is an improvement.
      if(total_segmented_frames > best_total_segmented_frames ||
         (total_segmented_frames == best_total_segmented_frames &&
          total_normalized_loss < best_total_normalized_loss))
      { 
        best_total_segmented_frames = total_segmented_frames;
        best_total_normalized_loss = total_normalized_loss;
        ROS_DEBUG_STREAM("Best performance so far this iter.");
        best_weights = weights;
        num_iters_since_improvement = 0;
        
        string filename = "autosave_best_weights.eig.txt";
        ROS_DEBUG_STREAM("Saving best weights to " << filename);
        eigen_extensions::saveASCII(weights, filename);
      }
      else
        ++num_iters_since_improvement;
      ROS_DEBUG_STREAM(num_iters_since_improvement << " iterations since last improvement.");
        
      
      // -- Run structural SVM.
      ROS_DEBUG_STREAM("Total number of segmented frames this outer iteration: " << total_segmented_frames);
      ROS_DEBUG_STREAM("Total number of frames in all sequences: " << total_frames);
      ROS_DEBUG_STREAM("Learning new weights using " << caches.size() << " training examples.");
      prev_weights = weights;
      weights = runSolver(caches, labels, &constraints);
      if(CLEAR_CONSTRAINTS) {
        ROS_DEBUG_STREAM("Clearing constraints.");
        constraints.clear();
      }

      if(iter > 0) {
              // cout << "weights norm: " << weights.norm() << endl;
              // cout << "prev weights norm: " << prev_weights.norm() << endl;
              ROS_ASSERT(fabs(weights.norm() - 1.0) < 1e-3);
              ROS_ASSERT(fabs(prev_weights.norm() - 1.0) < 1e-3);
      }
      
      // -- Stop if we're done.
      ROS_DEBUG_STREAM("Angle change between weights: " << fabs(acos(weights.dot(prev_weights))) << flush);
      if(fabs(acos(weights.dot(prev_weights))) < 1e-3) {
        ROS_DEBUG_STREAM("Breaking outer iteration because weights have not changed much." << flush);
        break;
      }
      if(num_iters_since_improvement > 10) {
        ROS_DEBUG_STREAM("Breaking outer iteration because performance has not improved recently.");
        break;
      }
    
      ++iter;
    }

    if(RETURN_BEST != 0) 
      return best_weights;  // Not clear this is the right thing to do.
    else
      return weights;
  }

  void StructSVM::precache(std::vector<FramePotentialsCache::Ptr>* caches,
                           std::vector<cv::Mat1b>* labels) const
  {
    for(size_t i = 0; i < sequences_.size(); ++i) {
      KinectSequence::Ptr seq = sequences_[i];
      SegmentationPipeline sp(NUM_THREADS);
      PotentialsCache::Ptr cache = sp.cacheUnweightedPotentialsWithOracle(seq);
      
      for(size_t j = 0; j < cache->size(); ++j) { 
        caches->push_back(cache->framecaches_[j]);
        labels->push_back(seq->segmentations_[j]);
      }
    }
  }
  
  Eigen::VectorXd StructSVM::train()
  {
    // -- Build the training set assuming that segmentation is always correct.
    vector<FramePotentialsCache::Ptr> caches;
    vector<cv::Mat1b> labels;
    precache(&caches, &labels);
    
    // -- Run structural SVM.
    vector<Constraint> constraints;
    return runSolver(caches, labels, &constraints);
  }

  Eigen::VectorXd StructSVM::runOneSlackSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
                                               const std::vector<cv::Mat1b>& labels,
                                               std::vector<Constraint>* constraints) const
  {
    ROS_ASSERT(caches.size() == labels.size());
    ROS_ASSERT(constraints);
    ROS_DEBUG_STREAM("Using 1-slack formulation.");
    ROS_ASSERT(!caches.empty() && !labels.empty());
    
    VectorXd weights = 0.001 * VectorXd::Ones(caches[0]->getNumWeights()); // Start slightly away from the boundary.
    VectorXd slacks = 0.001 * VectorXd::Ones(1);
    int iter = 0;
    SegmentationPipeline sp(NUM_THREADS);
    int num_edge_weights = sp.getEdgeWeights().rows();
    double best_normalized_loss = std::numeric_limits<double>::max();
    VectorXd best_weights = weights;
    
    while(true) {
      ROS_DEBUG_STREAM("==================== Starting iteration " << iter << flush);

      if(SAVE_INNER_WEIGHTS) {
        ROS_ASSERT(!ROBUST && !HYBRID);
        ostringstream oss;
        oss << "weights" << setw(4) << setfill('0') << iter << ".eig.txt";
        eigen_extensions::saveASCII(weights, oss.str());
      }
      
      VectorXd prev_weights = weights;
      double normalized_loss = 0;
      vector<cv::Mat1b> ymvs;
      vector<cv::Mat1b> ys;

      // -- Compute the most violating labeling for each framecache.
      vector<ComputeNode*> nodes(caches.size(), NULL);
      for(size_t i = 0; i < caches.size(); ++i)
        nodes[i] = new ConstraintGenerator(this, weights, i, caches[i], labels[i]);
      Pipeline2 pl(NUM_GC_THREADS, nodes);
      HighResTimer hrt("Computing ymvs");
      hrt.start();
      pl.compute();
      hrt.stop();

      // -- Compute the one new constraint.
      ROS_ASSERT(!margin_rescaling_);
      Constraint c;
      c.loss_ = 0;
      c.tr_ex_id_ = 0; // For all; this indexes into which slack variable, and there's only one.
      c.dpsi_ = VectorXd::Zero(weights.rows());
      for(size_t i = 0; i < nodes.size(); ++i) {
        ConstraintGenerator& cg = *(ConstraintGenerator*)nodes[i];
        ROS_ASSERT(cg.hamming_loss_ >= 0);  // Make sure all nodes computed.
        normalized_loss += cg.normalized_loss_ / (double)caches.size();
        c.loss_ += cg.con_.loss_ / (double)nodes.size(); // mean 0-1 loss
        c.dpsi_ += cg.con_.dpsi_ / (double)nodes.size(); // mean dpsi
      }
      constraints->push_back(c);
      ROS_DEBUG_STREAM("Mean normalized loss: " << normalized_loss);
      if(normalized_loss < best_normalized_loss) {
        ROS_DEBUG_STREAM("Best normalized loss so far (previous best: " << best_normalized_loss << ").");
        best_normalized_loss = normalized_loss;
        best_weights = weights;
      }
      
      // -- Check if we're done.
      double margin = weights.dot(c.dpsi_);
      ROS_DEBUG_STREAM("loss - margin: " << c.loss_ - margin);
      ROS_DEBUG_STREAM("Slack: " << slacks(0));
      if(c.loss_ - margin <= slacks(0) + 1e-6) {
        ROS_DEBUG_STREAM("Breaking because the newly added constraint is already satisfied.");
        break;
      }
      
      // -- Run the solver.
      ROS_DEBUG_STREAM("Total constraints: " << constraints->size());
      hrt.reset("Learning new weights (NIPS)");
      hrt.start();
      updateWeights2(*constraints, 1,
                     num_edge_weights,
                     &weights, &slacks);
      hrt.stop();
      ROS_DEBUG_STREAM(hrt.report() << flush);

      ++iter;
    }

    // return best_weights;  // Not clear if this the right thing to do.  The margin...
    return weights;
  }

  Eigen::VectorXd StructSVM::runSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
                                       const std::vector<cv::Mat1b>& labels,
                                       std::vector<Constraint>* constraints) const
  {
    // -- Report on memory usage.
    long int bytes = 0;
    for(size_t i = 0; i < caches.size(); ++i)
      bytes += caches[i]->bytes();
    int mb = bytes / (1024.0 * 1024.0);
    ROS_DEBUG_STREAM(caches.size() << " framecaches: " << mb << " mb");
    
    if(NSLACK)
      return runNSlackSolver(caches, labels);
    else
      return runOneSlackSolver(caches, labels, constraints);
  }
    
  
  Eigen::VectorXd StructSVM::runNSlackSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
                                             const std::vector<cv::Mat1b>& labels) const
  {
    ROS_ASSERT(caches.size() == labels.size());
    ROS_DEBUG_STREAM("Using n-slack formulation.");
    
    std::vector<Constraint> working;
    VectorXd weights = 0.001 * VectorXd::Ones(caches[0]->getNumWeights()); // Start slightly away from the boundary.
    VectorXd slacks = 0.001 * VectorXd::Ones(caches.size());
    int iter = 0;
    double best_hamming_loss = numeric_limits<double>::max();
    VectorXd best_weights = weights;
    int num_iters_since_improvement = 0;
    while(true) {
      VectorXd prev_weights = weights;
      size_t prev_working_size = working.size();
      double hamming_loss = 0;

      ROS_DEBUG_STREAM("==================== Starting iteration " << iter << flush);
      
      // -- Add constraints for all training examples.
      vector<ComputeNode*> nodes(caches.size(), NULL);
      for(size_t i = 0; i < caches.size(); ++i)
        nodes[i] = new ConstraintGenerator(this, weights, i, caches[i], labels[i]);
      Pipeline2 pl(NUM_GC_THREADS, nodes);
      ROS_DEBUG_STREAM("Using parallel constraint finder.");
      HighResTimer hrt("Computing constraints");
      hrt.start();
      pl.compute();
      hrt.stop();
      ROS_DEBUG_STREAM(hrt.report());
      for(size_t i = 0; i < nodes.size(); ++i) {
        ConstraintGenerator& cg = *(ConstraintGenerator*)nodes[i];
        ROS_ASSERT(cg.hamming_loss_ >= 0);  // Make sure all nodes computed.
        hamming_loss += cg.hamming_loss_ / (double)caches.size();
        if(cg.hamming_loss_ > 0)
          working.push_back(cg.con_);
      }
      
      // -- Keep track of the best weights vector.
      ROS_DEBUG_STREAM("Mean hamming loss: " << hamming_loss);
      ROS_DEBUG_STREAM("Best so far: " << best_hamming_loss);
      if(hamming_loss < best_hamming_loss) {
        ROS_DEBUG_STREAM("This iteration has best mean hamming loss so far: " << hamming_loss << " vs " << best_hamming_loss);
        best_hamming_loss = hamming_loss;
        best_weights = weights;
        num_iters_since_improvement = 0;
        
        string filename = "autosave_best_weights.eig.txt";
        ROS_DEBUG_STREAM("Saving best weights to " << filename);
        eigen_extensions::saveASCII(weights, filename);
      }
      else
        ++num_iters_since_improvement;
      
      // -- Check if we're done.
      if(working.size() == prev_working_size) {
        ROS_DEBUG_STREAM("Breaking due to lack of additions to working set.");
        break;
      }
      
      // -- Update the weights.
      bool feas = true;
      ROS_DEBUG_STREAM("Added " << working.size() - prev_working_size << " constraints.");
      ROS_DEBUG_STREAM("Total constraints: " << working.size());
      SegmentationPipeline sp(NUM_THREADS);
      int num_edge_weights = sp.getEdgeWeights().rows();
      
      bpt::ptime now = bpt::second_clock::local_time();
      ROS_DEBUG_STREAM("Starting to learn new weights at " << bpt::to_iso_extended_string(now));

      // -- Nesterov interior point solver.
      hrt.reset("Learning new weights (NIPS)");
      hrt.start();
      updateWeights2(working, caches.size(),
                           num_edge_weights,
                           &weights, &slacks);
      hrt.stop();
      ROS_DEBUG_STREAM(hrt.report() << flush);

      ROS_ASSERT(fabs(weights.norm() - 1.0) < 1e-6);
      if(iter > 0) { 
        ROS_ASSERT(fabs(prev_weights.norm() - 1.0) < 1e-6);
        ROS_DEBUG_STREAM("angle change between current weights and old weights = " << acos(weights.dot(prev_weights)));
      }
      ROS_DEBUG_STREAM("new weights: ");
      ROS_DEBUG_STREAM(weights.transpose());

      // -- Check if we're done.
      if(!feas) {
        ROS_WARN_STREAM("Breaking due to infeasibility.");
        break;
      }
      if(fabs(acos(weights.dot(prev_weights))) < 1e-3) {
        ROS_DEBUG_STREAM("Breaking because weights have not changed much.");
        break;
      }
      if(num_iters_since_improvement > 20) {
        ROS_DEBUG_STREAM("Breaking because best hamming loss has not improved recently.");
        break;
      }

      ++iter;
    }

    ROS_DEBUG_STREAM("Best hamming loss for this run: " << best_hamming_loss);
    return best_weights;
  }

  double* StructSVM::eigToQPO(const Eigen::MatrixXd& eig)
  {
    double* qpo = new double[eig.rows() * eig.cols()];
    for(int i = 0; i < eig.rows(); ++i)
      for(int j = 0; j < eig.cols(); ++j) 
        qpo[j + i*eig.cols()] = eig(i, j);

    return qpo;
  }

  double* StructSVM::eigToQPO(const Eigen::VectorXd& eig)
  {
    double* qpo = new double[eig.rows()];
    for(int i = 0; i < eig.rows(); ++i)
      qpo[i] = eig(i);

    return qpo;
  }

  double StructSVM::updateWeights2(const std::vector<Constraint>& constraints,
                                   int num_tr_ex,
                                   int num_edge_weights,
                                   Eigen::VectorXd* weights,
                                   Eigen::VectorXd* slacks) const
  {
    // -- Warm start from previous solution.
    VectorXd x = VectorXd::Zero(weights->rows() + num_tr_ex);
    x.head(weights->rows()) = *weights;
    x.tail(slacks->rows()) = *slacks;

    // ... but make sure it's a feasible starting point.
    for(size_t i = 0; i < constraints.size(); ++i) {
      const Constraint& c = constraints[i];
      double& slack = x.coeffRef(weights->rows() + c.tr_ex_id_);
      if(weights->dot(c.dpsi_) < c.loss_ - slack) { 
        // Strict equality won't work with this solver,
        // so add a bit to the minimum amount of slack.
        slack = c.loss_ - weights->dot(c.dpsi_) + 1.0;
      }
    }
    
    // -- Generate the objective function and gradient.
    SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
    for(int i = 0; i < weights->rows(); ++i) {
      A->startVec(i);
      A->insertBack(i, i) = 1.0;
    }
    A->finalize();

    SVPtr b(new SparseVector<double>(x.rows()));
    b->startVec(0);
    for(int i = 0; i < x.rows(); ++i)
      if(i >= weights->rows())
        b->insertBack(i) = c_ / (double)num_tr_ex;
    b->finalize();

    SparseQuadraticFunction::Ptr objective(new SparseQuadraticFunction(A, b, 0));
    SparseLinearFunction::Ptr gradient(new SparseLinearFunction(A, b));
    
    // -- Set up the optimizer.
    double initial_mu = 1.0;
    double tol = TOL; // 1e-3 works with c=1.  With smaller c, tol can decrease, though it's not clear that it helps.
    double alpha = 0.3;
    double beta = 0.5;
    int max_num_iters = 0;
    double stepsize = 1;
    int restart = 0;
    ROS_DEBUG_STREAM("Solving with tolerance " << TOL);
    NesterovInteriorPointSolver nips(objective, gradient,
                                     initial_mu, tol, alpha, beta,
                                     max_num_iters, stepsize, restart, DEBUG_LEVEL);

    // -- Generate the margin constraints.
    for(size_t i = 0; i < constraints.size(); ++i) {
      SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
      A->finalize();

      SVPtr b(new Eigen::SparseVector<double>(x.rows()));
      b->startVec(0);
      for(int j = 0; j < weights->rows(); ++j)
        b->insertBack(j) = -constraints[i].dpsi_(j);
      b->insertBack(weights->rows() + constraints[i].tr_ex_id_) = -1;

      SparseQuadraticFunction::Ptr c(new SparseQuadraticFunction(A, b, constraints[i].loss_));
      SparseLinearFunction::Ptr gc(new SparseLinearFunction(A, b));
      nips.addConstraint(c, gc);
    }
    
    // -- Generate the non-negativity constraints for the edge weights
    //    and the slack variables.
    for(int i = 0; i < x.rows(); ++i) {
      if(ALL_NONNEG || (i < num_edge_weights || i >= weights->rows())) { 
        SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
        A->finalize();
        
        SVPtr b(new Eigen::SparseVector<double>(x.rows()));
        b->startVec(0);
        b->insertBack(i) = -1.0;
        b->finalize();

        SparseQuadraticFunction::Ptr c(new SparseQuadraticFunction(A, b, 0));
        SparseLinearFunction::Ptr gc(new SparseLinearFunction(A, b));
        nips.addConstraint(c, gc);
      }
    }

    // -- Solve.
    long int ns = 0;
    VectorXd xstar = nips.solve(x, &ns);
    ROS_DEBUG_STREAM("Solving with Nesterov took " << ns << " steps, total.");
    if(slacks)
      *slacks = xstar.tail(num_tr_ex);
    *weights = xstar.head(weights->rows());
    
    for(int i = 0; i < num_edge_weights; ++i)
      if(weights->coeffRef(i) < 0)
        weights->coeffRef(i) = 0;
    weights->normalize();

    return objective->eval(xstar);
  }

  double StructSVM::fgNormalizedHammingLoss(cv::Mat1b label, cv::Mat1b pred) const
  {
    ROS_ASSERT(label.rows == pred.rows);
    ROS_ASSERT(label.cols == pred.cols);

    double loss = 0;
    double num_fg = 0;
    for(int y = 0; y < label.rows; ++y) {
      for(int x = 0; x < label.cols; ++x) {
        
        if(label(y, x) == 255)
          ++num_fg;
        // Ignore pixels without depth.
        if(label(y, x) == 255 && pred(y, x) == 0)
          ++loss;
        else if(label(y, x) == 0 && pred(y, x) == 255)
          ++loss;
      }
    }

    double output;
    if(num_fg == 0)
      output = loss;
    else
      output = loss / num_fg;

    // cout << "Normalized loss: " << output << endl;
    // cv::imshow("Prediction", pred);
    // cv::imshow("Ground truth", label);
    // cv::waitKey(0);

    return output;
  }
  
  double StructSVM::loss(cv::Mat1b label, cv::Mat1b pred, bool hamming) const
  {
    ROS_ASSERT(label.rows == pred.rows);
    ROS_ASSERT(label.cols == pred.cols);

    if(hamming) {
      double loss = 0;
      for(int y = 0; y < label.rows; ++y) {
        for(int x = 0; x < label.cols; ++x) {
          // Ignore pixels without depth.
          if(label(y, x) == 255 && pred(y, x) == 0)
            ++loss;
          else if(label(y, x) == 0 && pred(y, x) == 255)
            ++loss;
        }
      }
      return loss;
      //return 1000.0 * loss / (double)(label.rows * label.cols);
      //return loss / (double)(label.rows * label.cols);
    }
    else { 
      bool equal = true;
      for(int y = 0; y < label.rows && equal; ++y) {
        for(int x = 0; x < label.cols && equal; ++x) {
          // Ignore pixels without depth.
          if((label(y, x) == 255 && pred(y, x) == 0) ||
             (label(y, x) == 0 && pred(y, x) == 255))
            equal = false;
        }
      }
      if(!equal)
        return 1.0;
      else
        return 0.0;
    }

    ROS_ASSERT(0);
    return 0;
  }

  ConstraintGenerator::ConstraintGenerator(const StructSVM* svm,
                                           const Eigen::VectorXd& weights,
                                           int tr_ex_id,
                                           FramePotentialsCache::Ptr cache,
                                           cv::Mat1b labels) :
    ComputeNode(),
    hamming_loss_(-1),
    normalized_loss_(-1),
    svm_(svm),
    weights_(weights),
    tr_ex_id_(tr_ex_id),
    cache_(cache),
    labels_(labels)
  {
  }
     
  void ConstraintGenerator::_compute()
  {
    SegmentationPipeline sp(1);
    sp.verbose_ = false;
    sp.setWeightsWithClipping(weights_);
    ROS_ASSERT(svm_->margin_rescaling_ == false);
    cv::Mat1b ymv = sp.findMostViolating(*cache_, labels_, svm_->margin_rescaling_);
    double l = svm_->loss(labels_, ymv, svm_->margin_rescaling_);
    hamming_loss_ = svm_->loss(labels_, ymv, true);
    normalized_loss_ = svm_->fgNormalizedHammingLoss(labels_, ymv);

    VectorXd psi_gt;
    double gt_score = cache_->computeScore(labels_,
                                           sp.getEdgeWeights(),
                                           sp.getNodeWeights(),
                                           &psi_gt);
        
    VectorXd psi_pred;
    double pred_score = cache_->computeScore(ymv,
                                             sp.getEdgeWeights(),
                                             sp.getNodeWeights(),
                                             &psi_pred);
    VectorXd dpsi = psi_gt - psi_pred;

    // ymv should the best possible labeling for these weights.
    if(!svm_->margin_rescaling_ && pred_score < gt_score) { 
      ROS_FATAL_STREAM("Prediction score (" << pred_score << ") is less than ground truth score (" << gt_score << ")!");
      cv::imshow("ground truth", labels_);
      cv::imshow("prediction", ymv);
      cv::waitKey();
      ROS_ASSERT(pred_score >= gt_score);  // This failed once.  Why?
    }
        
    con_.tr_ex_id_ = tr_ex_id_;
    con_.dpsi_ = dpsi;
    con_.loss_ = l;
  }

  void ConstraintGenerator::_flush()
  {
    con_ = Constraint();
    hamming_loss_ = -1;
  }
  
} // namespace dst
