#include <dst/kinect_sequence.h>

using namespace std;
using namespace dst;

int main(int argc, char** argv)
{

  int num_sequences = argc - 1;
  int num_frames = 0;
  int num_seed_frames = 0;
  for(int i = 1; i < argc; ++i) {
    cout << "Loading " << argv[i] << endl;
    KinectSequence seq;
    seq.load(argv[i]);
    num_frames += seq.images_.size();
    for(size_t j = 0; j < seq.seed_images_.size(); ++j) {
      cv::Mat1b seed = seq.seed_images_[j];
      bool flag = false;
      for(int y = 0; y < seed.rows; ++y)
        for(int x = 0; x < seed.cols; ++x)
          if(seed(y, x) != 127)
            flag = true;

      if(flag)
        ++num_seed_frames;
    }
  }

  cout << "Num sequences: " << num_sequences << endl;
  cout << "Num frames: " << num_frames << endl;
  cout << "Num seed frames: " << num_seed_frames << endl;
  return 0;
}
