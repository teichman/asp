#include <dst/kinect_sequence.h>
#include <dst/evaluator.h>

#define VISUALIZE (getenv("VISUALIZE"))

using namespace std;
namespace bfs = boost::filesystem;
using namespace dst;

string usageString()
{
  ostringstream oss;
  oss << "Usage: evaluate_houghtrack SEQ RESULTS" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  string seqpath = argv[1];
  if(seqpath.find_last_of('/') == seqpath.size() - 1)
    seqpath = seqpath.substr(0, seqpath.size() - 1);
  string name = seqpath.substr(seqpath.find_last_of('/') + 1);
  
  KinectSequence seq;
  seq.load(seqpath);
  
  vector<string> masks;
  string path = argv[2];
  bfs::directory_iterator end_itr; // default construction yields past-the-end
  for(bfs::directory_iterator itr(path); itr != end_itr; ++itr) {
    string p = itr->path().string();
    if(p.substr(p.size() - 4).compare(".png") == 0) {
      masks.push_back(p);
    }
  }

  cout << masks.size() << " predictions, " << seq.segmentations_.size() << " ground truth labels." << endl;
  assert(masks.size() == seq.segmentations_.size() - 1); // There is no prediction for the first frame.
  sort(masks.begin(), masks.end());
  SequenceResults sr(name);

  for(size_t i = 0; i < masks.size(); ++i) {
    cv::Mat3b mask = cv::imread(masks[i]);
    cv::Mat1b mask1b(mask.size(), 0);
    cv::Mat3b vis(mask.size(), cv::Vec3b(0, 0, 0));
    for(int y = 0; y < mask.rows; ++y) { 
      for(int x = 0; x < mask.cols; ++x) { 
        if(mask(y, x)[0] != 0) { 
          mask1b(y, x) = 255;
          vis(y, x)[2] = 255;
        }
        if(seq.segmentations_[i+1](y, x) == 255)
          vis(y, x)[1] = 255;
      }
    }

    if(VISUALIZE) { 
      cv::imshow("img", seq.images_[i]);
      cv::imshow("mask", mask1b);
      cv::imshow("gndtruth", seq.segmentations_[i+1]);
      cv::imshow("vis", vis);
      sr.update(seq.segmentations_[i+1], mask1b, true);
      cout << sr.status() << endl;
      cv::waitKey(0);
    }
    else
      sr.update(seq.segmentations_[i+1], mask1b);
  }

  cout << sr.status() << endl;
  return 0;
}
