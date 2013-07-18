#include <boost/program_options.hpp>
#include <asp/simple_segmentation_pipeline.h>
#include <graphcuts/structural_svm.h>

using namespace asp::example;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("dir", bpo::value<string>(&dir)->required(), "Directory of training data")
    ("model-output", bpo::value<string>()->required(), "Location to put learned model")
    ("ssvm-c", bpo::value<double>()->default_value(1))
    ("ssvm-precision", bpo::value<double>()->default_value(1e-3))
    ("ssvm-num-threads", bpo::value<int>()->default_value(1))
    ("ssvm-debug-level", bpo::value<int>()->default_value(0))
    ;

  p.add("dir", 1).add("model-output", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: simple_segmenter [OPTS] DIR MODEL-OUTPUT" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Start with something really dumb that definitely won't work.
  Asp asp(1);
  generateSimpleSegmentationPipeline(&asp);
  asp.setModel(asp.defaultModel());
  
  // -- Index training data.
  vector<string> seed_paths;
  vector<string> img_paths;
  vector<string> seg_paths;
  bfs::directory_iterator it(dir), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    string path = dir + "/" + p.leaf().string();
    if(p.extension() != ".png")
      continue;

    if(p.string().find("image") != string::npos)
      img_paths.push_back(path);
    else if(p.string().find("seed") != string::npos)
      seed_paths.push_back(path);
    else if(p.string().find("seg") != string::npos)
      seg_paths.push_back(path);
  }
  sort(img_paths.begin(), img_paths.end());
  sort(seed_paths.begin(), seed_paths.end());
  sort(seg_paths.begin(), seg_paths.end());

  cout << "img_paths: " << endl;
  for(size_t i = 0; i < img_paths.size(); ++i)
    cout << img_paths[i] << endl;
  cout << "seed_paths: " << endl;
  for(size_t i = 0; i < seed_paths.size(); ++i)
    cout << seed_paths[i] << endl;
  cout << "seg_paths: " << endl;
  for(size_t i = 0; i < seg_paths.size(); ++i)
    cout << seg_paths[i] << endl;

  ROS_ASSERT(img_paths.size() == seed_paths.size());
  ROS_ASSERT(img_paths.size() == seg_paths.size());
  
  // -- Construct the training set.
  vector<PotentialsCache::Ptr> caches;
  std::vector<VecXiPtr> labels;
  
  for(size_t i = 0; i < img_paths.size(); ++i) {
    // -- Add the ground truth to the training set.
    VecXiPtr groundtruth(new VectorXi);
    cv::Mat1b seg = cv::imread(seg_paths[i], 0);
    imageToVectorSegmentation(seg, groundtruth.get());
    labels.push_back(groundtruth);

    // -- Load the pipeline.
    cv::Mat3b img = cv::imread(img_paths[i], 1);
    cv::Mat1b seed = cv::imread(seed_paths[i], 0);
    asp.pod< EntryPoint<cv::Mat3b> >("ImageEntryPoint")->setData(img);
    asp.pod< EntryPoint<cv::Mat1b> >("MaskEntryPoint")->setData(cv::Mat1b(img.size(), 255));
    asp.pod< EntryPoint<cv::Mat1b> >("SeedEntryPoint")->setData(seed);

    // -- Segment and add the potentials cache to the training set.
    cv::Mat1b seg_img;
    PotentialsCache::Ptr pc(new PotentialsCache);
    asp.segment(&seg_img, pc.get());
    caches.push_back(pc);
  }

  // -- Run structural SVM training.
  StructuralSVM ssvm(opts["ssvm-c"].as<double>(),
                     opts["ssvm-precision"].as<double>(),
                     opts["ssvm-num-threads"].as<int>(),
                     opts["ssvm-debug-level"].as<int>());

  Model model = ssvm.train(caches, labels);
  cout << "Learned new model: " << endl;
  cout << model << endl;

  model.save(opts["model-output"].as<string>());
  cout << "Saved to " << opts["model-output"].as<string>() << endl;
                       
  return 0;
}
