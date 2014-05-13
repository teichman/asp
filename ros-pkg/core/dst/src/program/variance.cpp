#include <dst/struct_svm.h>
#include <dst/evaluator.h>
#include <eigen_extensions/eigen_extensions.h>

#define C_VALUE (getenv("C_VALUE") ? atof(getenv("C_VALUE")) : 10)

using namespace std;
using namespace dst;

string usageString()
{
  ostringstream oss;
  oss << "Usage: variance TRAINING_DIR [TRAINING_DIR ...]" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 2) {
    cout << usageString() << endl;
    return 0;
  }

  cout << "Using frame-to-frame training." << endl;    
  cout << "Using c = " << C_VALUE << endl;
  cout << "Not using margin rescaling." << endl;

  // -- Load data and precache potentials.
  StructSVM learner(C_VALUE, 1, false);
  for(int i = 1; i < argc; ++i) 
    learner.loadSequences(argv[i]);

  vector<FramePotentialsCache::Ptr> caches;
  vector<cv::Mat1b> labels;
  learner.precache(&caches, &labels);

  // -- Run learning and evaluate.
  for(int i = 0; i < 30; ++i) {
    vector<Constraint> constraints;
    Eigen::VectorXd weights = learner.runSolver(caches, labels, &constraints);
    cout << "Learned weights " << endl << weights.transpose() << endl;

    Evaluator evaluator(weights, false);
    for(size_t j = 0; j < learner.sequences_.size(); ++j)
      evaluator.evaluate(learner.sequences_[j], "name");
    
    cout << evaluator.status() << endl;
  }

  return 0;
}
