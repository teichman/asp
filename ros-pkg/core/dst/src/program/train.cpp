#include <dst/struct_svm.h>
#include <eigen_extensions/eigen_extensions.h>

#define MARGIN_RESCALING getenv("MARGIN_RESCALING")
#define C_VALUE (getenv("C_VALUE") ? atof(getenv("C_VALUE")) : 10)

using namespace std;
using namespace dst;

string usageString()
{
  ostringstream oss;
  oss << "Usage: ./train TRAINING_DIR [TRAINING_DIR ...] OUTPUT_WEIGHTS" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    cout << usageString() << endl;
    return 0;
  }

  string savepath = argv[argc-1];
  ROS_ASSERT(savepath.size() > 7);
  ROS_ASSERT(savepath.substr(savepath.size() - 8).compare(".eig.txt") == 0);

  if(getenv("HYBRID"))
    cout << "Using hybrid training." << endl;
  else if(getenv("ROBUST"))
    cout << "Using robust training." << endl;
  else
    cout << "Using frame-to-frame training." << endl;    

  cout << "Using c = " << C_VALUE << endl;

  if(MARGIN_RESCALING)
    cout << "Using margin rescaling." << endl;
  else
    cout << "Not using margin rescaling." << endl;
  
  StructSVM learner(C_VALUE, 1, MARGIN_RESCALING);
  for(int i = 1; i < argc - 1; ++i) 
    learner.loadSequences(argv[i]);

  Eigen::VectorXd weights;
  if(getenv("ROBUST") || getenv("HYBRID"))
    weights = learner.trainRobust2();
  else
    weights = learner.train();
  
  cout << "Learned weights " << endl << weights.transpose() << endl;
  eigen_extensions::saveASCII(weights, savepath);
  cout << "Saved to " << savepath << endl;
  
  return 0;
}
