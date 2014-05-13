#include <signal.h>
#include <boost/program_options.hpp>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <iostream>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <ros/assert.h>
#include <asp/rgbd.h>

using namespace std;
using namespace asp;

class OniVisualizer : public OpenNI2Handler
{
public:
  OpenNI2Interface oni_;
  Asp asp_;
  
  OniVisualizer(OpenNI2Interface::Resolution color_res,
                OpenNI2Interface::Resolution depth_res,
                int num_threads, YAML::Node config) :
    oni_(color_res, depth_res),
    asp_(num_threads, config)
  {
    asp_.setModel(asp_.defaultModel());
    asp_.writeGraphviz("graphviz");
  }
  
  void run()
  {
    oni_.setHandler(this);
    oni_.run();
  }

  void rgbdCallback(openni::VideoFrameRef oni_color,
                    openni::VideoFrameRef oni_depth,
                    size_t frame_id, double timestamp)
  {
    ROS_ASSERT(oni_depth.getHeight() > 0 && oni_depth.getWidth() > 0);
    cv::Mat3b color = oniToCV(oni_color);
    DepthMatConstPtr depth = oniDepthToEigenPtr(oni_depth);
    cv::imshow("Depth", colorize(*depth, 0, 10));
    cv::imshow("Color", oniToCV(oni_color));
    cv::imshow("Visualization", visualize(oni_color, oni_depth));
    char key = cv::waitKey(3);

    if(key == ' ') {
      cout << "Segmenting" << endl;
      asp_.push("ColorImageEntryPoint", color);
      asp_.push("DepthMatEntryPoint", depth);
      asp_.push("MaskEntryPoint", cv::Mat1b(color.size(), 255));
      asp_.push("SeedEntryPoint", cv::Mat1b(color.size(), 127));

      asp_.setDebug(false);
      asp_.compute();
      cout << asp_.reportTiming() << endl;

      // asp_.setDebug(true);
      // asp_.compute();
      // cout << "Done." << endl;

      const Normals& normals = *asp_.pull<Normals::ConstPtr>("OrganizedSurfaceNormalPod", "Normals");
      cv::Mat3b vis = asp_.pod<OrganizedSurfaceNormalPod>()->visualize(normals);
      cv::imshow("Normals", vis);
    }
    else if(key == 'q') {
      oni_.stop();
    }
  }
  
};

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  string color_resolution;
  string depth_resolution;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value(&config_path)->required(), "")
    ("color-res", bpo::value(&color_resolution), "")
    ("depth-res", bpo::value(&depth_resolution), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

      
  REGISTER_POD_TEMPLATE(EntryPoint, cv::Mat3b);
  REGISTER_POD_TEMPLATE(EntryPoint, cv::Mat1b);
  REGISTER_POD_TEMPLATE(EntryPoint, DepthMatConstPtr);
  REGISTER_POD(EdgeStructureGenerator);
  REGISTER_POD(SimpleColorDifferenceEPG);
  REGISTER_POD(EdgePotentialAggregator);
  REGISTER_POD(NodePotentialAggregator);
  REGISTER_POD(SeedNPG);
  REGISTER_POD(PriorNPG);
  REGISTER_POD(DepthMatProjector);
  REGISTER_POD(OrganizedSurfaceNormalPod);
  
  // -- Load the config.
  YAML::Node config = YAML::LoadFile(config_path);
  ROS_ASSERT(config["Pipeline"]);
  
  
  OpenNI2Interface::Resolution color_res = OpenNI2Interface::VGA;
  if(opts.count("color-res")) {
    if(color_resolution == "QVGA" || color_resolution == "qvga")
      color_res = OpenNI2Interface::QVGA;
    else if(color_resolution == "VGA" || color_resolution == "vga")
      color_res = OpenNI2Interface::VGA;
    else {
      cout << "Unrecognized resolution \"" << color_res << "\"." << endl;
      return 1;
    }
  }
  OpenNI2Interface::Resolution depth_res = OpenNI2Interface::VGA;
  if(opts.count("depth-res")) {
    if(depth_resolution == "QVGA" || depth_resolution == "qvga")
      depth_res = OpenNI2Interface::QVGA;
    else if(depth_resolution == "VGA" || depth_resolution == "vga")
      depth_res = OpenNI2Interface::VGA;
    else {
      cout << "Unrecognized resolution \"" << depth_res << "\"." << endl;
      return 1;
    }
  }

  OniVisualizer vis(color_res, depth_res, 12, config["Pipeline"]);
  vis.run();
  
  return 0;
}
