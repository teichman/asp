#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <dst/evaluator.h>
#include <eigen_extensions/eigen_extensions.h>
#include <eigen_extensions/random.h>

using namespace std;
using namespace Eigen;
using namespace dst;
namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  int num;
  string path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("num,n", bpo::value<int>(&num)->required(), "Number of weights")
    ("output,o", bpo::value<string>(&path)->required(), "Destination file for weights")
    ;

  p.add("num", 1);
  p.add("output", 2);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: generate_random_weights NUM OUTPUT" << endl;  
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Generating " << num << " random weights, saving to " << path << endl;
  VectorXd weights(num);
  eigen_extensions::GaussianSampler gs(0, 1, time(0));
  gs.sample(&weights);
  weights = weights.array().abs().matrix();
  cout << weights.transpose() << endl;
  eigen_extensions::saveASCII(weights, path);
  
  return 0;
}
