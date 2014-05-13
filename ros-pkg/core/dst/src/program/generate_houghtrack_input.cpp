#include <dst/kinect_sequence.h>

using namespace std;
using namespace dst;

string usageString()
{
  ostringstream oss;
  oss << "Usage: generate_houghtrack_input SEQ" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  string seqpath = argv[1];
  if(seqpath.substr(seqpath.size() - 1).compare("/") == 0)
    seqpath = seqpath.substr(0, seqpath.size() - 1);
  string seed_path = seqpath + "/seed0000.png";
  
  KinectSequence seq;
  seq.load(seqpath);
  cv::Mat1b seg = seq.segmentations_[0];

  int minx = seg.cols;
  int miny = seg.rows;
  int maxx = 0;
  int maxy = 0;
  for(int y = 0; y < seg.rows; ++y) {
    for(int x = 0; x < seg.cols; ++x) {
      if(seg(y, x) == 255) {
        if(x < minx)
          minx = x;
        if(x > maxx)
          maxx = x;
        if(y < miny)
          miny = y;
        if(y > maxy)
          maxy = y;
      }
    }
  }

  int width = maxx - minx;
  int height = maxy - miny;
  
  cout << "Tracker:" << endl;
  cout << "  {" << endl;
  cout << "    //      Track-Name and Initial Position" << endl;
  cout << "    Name = \"result\";                 // folder name for output images, folder will be created" << endl;
  cout << "    maxScale = 1.0;                                 // maximum scaling factor object may reach from initial size" << endl;
  cout << "    startRegion = [" << minx << "," << miny << "," << width << "," << height << "]; // comment parameter to select object by hand!" << endl;
  cout << "  };" << endl;
  cout << "" << endl;
  cout << "Input:" << endl;
  cout << "  {" << endl;
  cout << "    //      Path and image format of sequence" << endl;
  cout << "    path = \"" << seqpath << "\";                  // path of image data" << endl;
  cout << "    seedPath = \"" << seed_path << "\";                  // path of seed image" << endl;
  cout << "    imageFormat = \"png\";                    // suffix of accepted images" << endl;
  cout << "" << endl;
  cout << "    //      Additional Parameters and Default values" << endl;
  cout << "    //      increment = 1;                                  // load only every n-th image" << endl;
  cout << "    //      scaling = 1.0;                                  // scale images by factor" << endl;
  cout << "    //      loopImages = 0;                                 // loop over images" << endl;
  cout << "    //      offset = 0;                                             // ignore first n images" << endl;
  cout << "    //      flipVertical = 0;                               // mirror images" << endl;
  cout << "" << endl;
  cout << "  };" << endl;
  
  return 0;
}
