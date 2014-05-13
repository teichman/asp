#include <boost/program_options.hpp>
#include <asp/aspvis.h>
#include <asp/simple_segmentation_pipeline.h>

using namespace asp::example;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("img", bpo::value<string>()->required(), "Image to segment")
    ("scale", bpo::value<double>()->default_value(1.0), "Scale factor to apply when visualizing")
    ("savedir", bpo::value<string>()->default_value("."), "Where to save training data")
    ("default-model", "Use a really stupid default model that is guaranteed to fail")
    ("model",  bpo::value<string>(), "Trained model to use")
    ;

  p.add("img", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs || (opts.count("default-model") && opts.count("model"))) {
    cout << "Usage: simple_segmenter [OPTS] IMG" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cv::Mat3b img = cv::imread(opts["img"].as<string>());
  Asp asp(1);
  generateSimpleSegmentationPipeline(&asp);

  string graphviz_path = "graphviz";
  cout << "Wrote graphviz to \"" << graphviz_path << "\"." << endl;
  asp.writeGraphviz(graphviz_path);
  
  if(opts.count("default-model")) {
    cout << "Default model: " << endl;
    cout << asp.defaultModel() << endl;
    cout << endl;
    asp.setModel(asp.defaultModel());
  }
  else if(opts.count("model")) {
    Model model;
    model.load(opts["model"].as<string>());
    asp.setModel(model);
  }
  cout << endl;
  cout << "Using model: " << endl;
  cout << asp.model() << endl;
  
  AspVis vis(&asp, img, opts["scale"].as<double>(), opts["savedir"].as<string>());
  vis.run();

  return 0;
}
