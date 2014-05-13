#include <dst/evaluator.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace dst;
namespace bfs = boost::filesystem;

#define FTF (getenv("FTF"))

string usageString()
{
  ostringstream oss;
  oss << "Usage: ./evaluate WEIGHTS SEQ [SEQ ...] OUTPUT" << endl;
  oss << "  where SEQ can be a directory with a sequence or a directory with directories of sequences." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 0;
  }

  string output_path = argv[argc-1];
  ROS_ASSERT(!bfs::exists(output_path));
  cout << "Saving results to " << output_path << endl;
  
  Eigen::VectorXd weights;
  eigen_extensions::loadASCII(argv[1], &weights);
  cout << "Loaded weights: " << endl << weights.transpose() << endl;

  if(FTF)
    cout << "Evaluating using frame-to-frame method, assuming previous segmentation is always correct." << endl;
  else
    cout << "Evaluating using full sequence segmentation method." << endl;
      
  Evaluator evaluator(weights, FTF);
  std::vector<KinectSequence::Ptr> sequences;
  for(int i = 2; i < argc - 1; ++i) {
    loadSequences(argv[i], &sequences);
    for(size_t j = 0; j < sequences.size(); ++j)
      evaluator.evaluate(sequences[j], argv[i]);

    sequences.clear();
  }

  ofstream file;
  file.open(output_path.c_str());
  file << evaluator.status() << endl;
  file.close();
  
  cout << evaluator.status() << endl;
  
  return 0;
}
