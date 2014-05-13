#include <dst/kinect_sequence.h>

using namespace std;
using namespace dst;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;
  oss << "Usage: translate_sequence SEQ NEW_NAME" << endl;;
  return oss.str();
}


int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  KinectSequence seq;
  seq.load(argv[1]);
  ROS_ASSERT(!seq.segmentations_.empty());

  string path = argv[2];
  ROS_ASSERT(!bfs::exists(path));
  bfs::create_directory(path);

  for(size_t i = 0; i < seq.images_.size(); ++i) {
    ostringstream oss;
    oss << setw(5) << setfill('0') << i;
    string id = oss.str();
    cv::imwrite(path + "/img_" + id + ".png", seq.images_[i]);
    cv::imwrite(path + "/mask_" + id + ".png", seq.segmentations_[i]);

    // -- Make the depth image.
    KinectCloud::Ptr pcd = seq.pointclouds_[i];
    ROS_ASSERT((int)pcd->height == seq.images_[i].rows);
    cv::Mat1b mask(seq.images_[i].rows, seq.images_[i].cols);
    mask = 0;
    for(size_t y = 0; y < pcd->height; ++y) {
      for(size_t x = 0; x < pcd->width; ++x) {
        int idx = y * pcd->width + x;
        if(!isnan((*pcd)[idx].z))
          mask(y, x) = min((double)(*pcd)[idx].z, 2.0) / 2.0 * 255;
      }
    }
    cv::imwrite(path + "/depth_" + id + ".png", mask);
  }

    
  return 0;
}
