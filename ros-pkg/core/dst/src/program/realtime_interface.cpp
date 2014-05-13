#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <dst/realtime_interface.h>

using namespace std;
using namespace Eigen;
using namespace dst;

int main(int argc, char** argv)
{
  cout << "Starting daito." << endl;
  cout << "Realtime, ONline, INteractive segementer" << endl;

  
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("weights", bpo::value<string>()->required())
    ("vga", "Use 640x480 rather than 160x120.")
    ("scale", bpo::value<double>()->default_value(3.0))
    ;

  p.add("weights", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: real_time_interface WEIGHTS [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  int retval = system("killall XnSensorServer");
  cout << "killall XnSensorServer returned: " << retval << endl;
  
  VectorXd weights;
  eigen_extensions::loadASCII(opts["weights"].as<string>(), &weights);

  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz;
  if(opts.count("vga"))
    mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;

  RealTimeInterface rti("", mode, opts["scale"].as<double>());
  rti.sp_.setWeights(weights, true);
  rti.run();
  
  return 0;
}
