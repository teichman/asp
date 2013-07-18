#include <graphcuts/structural_svm.h>

using namespace std;
namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;
using namespace Eigen;

namespace gc
{

  StructuralSVM::StructuralSVM(double c,
                               double precision,
                               int num_threads,
                               int debug_level) :
    c_(c),
    precision_(precision),
    num_threads_(num_threads),
    debug_level_(debug_level)
  {
  }
  
  Model StructuralSVM::train(const std::vector<PotentialsCache::Ptr>& caches,
                             const std::vector<VecXiPtr>& labels) const
                                                   
  {
    ROS_ASSERT(caches.size() == labels.size());
    ROS_ASSERT(!caches.empty());
    for(size_t i = 1; i < caches.size(); ++i)
      ROS_ASSERT(caches[i]->nameMappingsAreEqual(*caches[i-1]));
    for(size_t i = 0; i < labels.size(); ++i)
      for(int j = 0; j < labels[i]->rows(); ++j)
        ROS_ASSERT(labels[i]->coeffRef(j) == -1 || labels[i]->coeffRef(j) == 1);
    
    // Start slightly away from the boundary.
    Model model;
    model.applyNameMappings(*caches[0]);
    model.eweights_ = 0.001 * VectorXd::Ones(caches[0]->numEdgePotentials());
    model.nweights_ = 0.001 * VectorXd::Ones(caches[0]->numNodePotentials());
    double slack = 0.001;
    int iter = 0;
    vector<Constraint> constraints;
    
    while(true) {
      ROS_DEBUG_STREAM("==================== Starting iteration " << iter << flush);
      double loss = 0;

      // -- Compute the most violating labeling for each framecache.
      //    Could probably do this with omp but no point in switching it over now.
      vector<ThreadPtr> threads;
      vector<ConstraintGenerator*> cgs;
      HighResTimer hrt("Computing most violating constraints");
      hrt.start();
      for(size_t i = 0; i < caches.size(); ++i) {
        cgs.push_back(new ConstraintGenerator(model, caches[i], labels[i]));
        threads.push_back(cgs[i]->launch());
      }
      for(size_t i = 0; i < threads.size(); ++i) {
        threads[i]->join();
      }
      hrt.stop();

      // -- Compute the one new constraint.
      Constraint c;
      c.loss_ = 0;
      c.dpsi_ = VectorXd::Zero(model.size());
      for(size_t i = 0; i < cgs.size(); ++i) {
        ConstraintGenerator& cg = *cgs[i];
        ROS_ASSERT(cg.hamming_loss_ >= 0);  // Make sure all nodes computed.
        loss += cg.hamming_loss_ / (double)caches.size();
        c.loss_ += cg.con_.loss_ / (double)caches.size(); // mean 0-1 loss
        c.dpsi_ += cg.con_.dpsi_ / (double)caches.size(); // mean dpsi
        delete cgs[i];
      }
      constraints.push_back(c);
      ROS_DEBUG_STREAM("Mean loss: " << loss);
      
      // -- Check if we're done.
      double margin = model.score(c.dpsi_);
      ROS_DEBUG_STREAM("loss - margin: " << c.loss_ - margin);
      ROS_DEBUG_STREAM("Slack: " << slack);
      if(c.loss_ - margin <= slack + 1e-6) {
        ROS_DEBUG_STREAM("Breaking because the newly added constraint is already satisfied.");
        break;
      }
      
      // -- Run the solver.
      ROS_DEBUG_STREAM("Total constraints: " << constraints.size());
      hrt.reset("Learning new weights");
      hrt.start();
      updateModel(constraints, &model, &slack);
                    
      hrt.stop();
      ROS_DEBUG_STREAM(hrt.report() << flush);
      cout << model << endl;

      ++iter;
    }

    return model;
  }

  double StructuralSVM::updateModel(const std::vector<Constraint>& constraints,
                                    Model* model, double* slack) const
                                      
  {
    // -- Warm start from previous solution.
    VectorXd x = VectorXd::Zero(model->size() + 1);
    x.head(model->size()) = model->concatenate();
    x(x.rows() - 1) = *slack;

    // ... but make sure it's a feasible starting point.
    for(size_t i = 0; i < constraints.size(); ++i) {
      const Constraint& c = constraints[i];
      double& sl = x.coeffRef(model->size());
      if(model->score(c.dpsi_) < c.loss_ - sl) { 
        // Strict equality won't work with this solver,
        // so add a bit to the minimum amount of slack.
        sl = c.loss_ - model->score(c.dpsi_) + 1.0;
      }
    }
    
    // -- Generate the objective function and gradient.
    SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
    for(int i = 0; i < model->size(); ++i) {
      A->startVec(i);
      A->insertBack(i, i) = 1.0;
    }
    A->finalize();

    SVPtr b(new SparseVector<double>(x.rows()));
    b->startVec(0);
    for(int i = 0; i < x.rows(); ++i)
      if(i >= model->size())
        b->insertBack(i) = c_;
    b->finalize();

    SparseQuadraticFunction::Ptr objective(new SparseQuadraticFunction(A, b, 0));
    SparseLinearFunction::Ptr gradient(new SparseLinearFunction(A, b));
    
    // -- Set up the optimizer.
    double initial_mu = 1.0;
    double alpha = 0.3;
    double beta = 0.5;
    int max_num_iters = 0;
    double stepsize = 1;
    int restart = 0;
    ROS_DEBUG_STREAM("Solving with tolerance " << precision_);
    NesterovInteriorPointSolver nips(objective, gradient,
                                     initial_mu, precision_, alpha, beta,
                                     max_num_iters, stepsize, restart, debug_level_);

    // -- Generate the margin constraints.
    for(size_t i = 0; i < constraints.size(); ++i) {
      SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
      A->finalize();

      SVPtr b(new Eigen::SparseVector<double>(x.rows()));
      b->startVec(0);
      for(int j = 0; j < model->size(); ++j)
        b->insertBack(j) = -constraints[i].dpsi_(j);
      b->insertBack(model->size()) = -1;

      SparseQuadraticFunction::Ptr c(new SparseQuadraticFunction(A, b, constraints[i].loss_));
      SparseLinearFunction::Ptr gc(new SparseLinearFunction(A, b));
      nips.addConstraint(c, gc);
    }
    
    // -- Generate the non-negativity constraints for the edge weights
    //    and the slack variables.
    for(int i = 0; i < x.rows(); ++i) {
      if(i < model->eweights_.rows() || i >= model->size()) { 
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
    *slack = xstar(xstar.rows() - 1);
    model->eweights_ = xstar.head(model->eweights_.rows());
    model->nweights_ = xstar.segment(model->eweights_.rows(), model->nweights_.rows());

    return objective->eval(xstar);
  }  
  
  ConstraintGenerator::ConstraintGenerator(const Model& model,
                                           PotentialsCache::ConstPtr cache,
                                           VecXiConstPtr labels) :
    hamming_loss_(-1),
    model_(model),
    cache_(cache),
    labels_(labels)
  {
  }
    
  void ConstraintGenerator::_run()
  {
    MaxflowInference mfi(model_);
    VecXi seg;
    mfi.segment(cache_, &seg);
    hamming_loss_ = hammingLoss(*labels_, seg);
    double zero_one_loss = zeroOneLoss(*labels_, seg);

    VectorXd psi_gt = cache_->psi(*labels_);

    double gt_score = model_.score(psi_gt);
    VectorXd psi_pred = cache_->psi(seg);
    double pred_score = model_.score(psi_pred);
    VectorXd dpsi = psi_gt - psi_pred;

    // The segmentation provided by MaxflowInference should have the
    // lowest score possible for these weights.
    ROS_FATAL_STREAM_COND(pred_score < gt_score,
                          "Prediction score (" << pred_score
                          << ") is less than ground truth score ("
                          << gt_score << ")!");
    
    con_.dpsi_ = dpsi;
    con_.loss_ = zero_one_loss;
  }

  double hammingLoss(const Eigen::VectorXi& label,
                     const Eigen::VectorXi& pred)
  {
    ROS_ASSERT(label.rows() == pred.rows());
    ROS_ASSERT(label.cols() == pred.cols());

    double loss = 0;
    for(int i = 0; i < label.rows(); ++i) {
      ROS_ASSERT(label.coeffRef(i) == -1 || label.coeffRef(i) == 1);
      ROS_ASSERT(pred.coeffRef(i) == -1 || pred.coeffRef(i) == 1);
      if(label.coeffRef(i) != pred.coeffRef(i))
        ++loss;
    }

    return loss;
  }

  double zeroOneLoss(const Eigen::VectorXi& label,
                     const Eigen::VectorXi& pred)
  {
    ROS_ASSERT(label.rows() == pred.rows());
    ROS_ASSERT(label.cols() == pred.cols());

    for(int i = 0; i < label.rows(); ++i) {
      ROS_ASSERT(label.coeffRef(i) == -1 || label.coeffRef(i) == 1);
      ROS_ASSERT(pred.coeffRef(i) == -1 || pred.coeffRef(i) == 1);
      if(label.coeffRef(i) != pred.coeffRef(i))
        return 1.0;
    }

    return 0.0;
  }
  
} // namespace dst

