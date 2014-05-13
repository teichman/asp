#include <planar_cut/CutSegment.h>
#include <image_labeler/opencv_view.h>
#include <timer/timer.h>
#include <sstream>
#include <iostream>
#include <set>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: planar_image_cut IMAGE" << endl;
  return oss.str();
}

class ViewController : public OpenCVViewDelegate
{
public:
  ViewController(cv::Mat img);
  void run();

private:
  //! y, x (i.e. row, col).
  std::set< std::pair<int, int> > sink_points_;
  //! y, x (i.e. row, col).
  std::set< std::pair<int, int> > source_points_;
  int radius_;
  cv::Mat img_;
  cv::Mat vis_;
  OpenCVView view_;
  
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void cut();
  //! row, col
  int getIndex(int y, int x) const;
  double computeCost(const cv::Vec3b& p, const cv::Vec3b& q) const;
};

ViewController::ViewController(cv::Mat img) :
  radius_(3),
  img_(img),
  view_("test")
{
  view_.setDelegate(this);
  vis_ = img_.clone();
  view_.updateImage(vis_);
}
  
void ViewController::mouseEvent(int event, int x, int y, int flags, void* param)
{
  // -- Left click to add to source.
  if(flags == 1) {
    for(int i = x - radius_; i <= x + radius_; ++i) { 
      for(int j = y - radius_; j <= y + radius_; ++j) {
        if(i >= 0 && i < vis_.cols &&
           j >= 0 && j < vis_.rows) { 
          source_points_.insert(pair<int, int>(j, i));
          cv::circle(vis_, cv::Point(i, j), 1, cv::Scalar(255, 255, 255)); // OpenCV consistency fail.
        }
      }
    }
    view_.updateImage(vis_);
  }

  // -- Right click to add to sink.
  else if(flags == 2) {
    for(int i = x - radius_; i <= x + radius_; ++i) { 
      for(int j = y - radius_; j <= y + radius_; ++j) {
        if(i >= 0 && i < vis_.cols &&
           j >= 0 && j < vis_.rows) { 
          sink_points_.insert(pair<int, int>(j, i));
          cv::circle(vis_, cv::Point(i, j), 1, cv::Scalar(0, 0, 0));
        }
      }
    }
    view_.updateImage(vis_);
  }
}

void ViewController::run()
{
  while(true) { 
    switch(view_.cvWaitKey(5)) {
    case 'q':
      exit(0);
      break;
    case 'c':
      cout << "Running graph cuts..." << endl;
      cut();
      break;
    default:
      break;
    }
  }
}

int ViewController::getIndex(int y, int x) const
{
  return x + y * img_.cols;
}

void ViewController::cut()
{
  cout << "Image is " << img_.rows << " rows and " << img_.cols << " cols." << endl;

  HighResTimer hrt("Graph setup");
  hrt.start();

  // -- Put image data into the graph.
  CutSegment graph(img_.cols, img_.rows);
  int num_nodes = img_.rows * img_.cols;
  uchar rchan[num_nodes];
  uchar gchan[num_nodes];
  uchar bchan[num_nodes];
  uchar gray[num_nodes];
  int idx = 0;
  for(int y = 0; y < img_.rows; ++y) {
    for(int x = 0; x < img_.cols; ++x) {
      bchan[idx] = img_.at<cv::Vec3b>(y, x)[0];
      gchan[idx] = img_.at<cv::Vec3b>(y, x)[1];
      rchan[idx] = img_.at<cv::Vec3b>(y, x)[2];
      gray[idx] = img_.at<cv::Vec3b>(y, x)[2];

      ++idx;
    }
  }
  //graph.setImageData(rchan, gchan, bchan);
  graph.setImageData(gray);

  // -- Put seed labels into the graph.
  uchar unl = 0;
  uchar source_id = 1;
  uchar sink_id = 2;
  uchar mask[num_nodes];
  idx = 0;
  for(int y = 0; y < img_.rows; ++y) { 
    for(int x = 0; x < img_.cols; ++x) {
      if(source_points_.count(pair<int, int>(y, x)))
        mask[idx] = source_id;
      else if(sink_points_.count(pair<int, int>(y, x)))
        mask[idx] = sink_id;
      else
        mask[idx] = unl;

      ++idx;
    }
  }
  graph.setSourceSink(mask, source_id, sink_id);

  hrt.stop();
  cout << hrt.reportMilliseconds() << endl;
  
  hrt.reset("planar cut");
  hrt.start();
  graph.segment();
  hrt.stop();
  cout << hrt.reportMilliseconds() << endl;

  cv::Mat output = img_.clone();
  for(int y = 0; y < output.rows; ++y) {
    for(int x = 0; x < output.cols; ++x) {
      if(graph.getLabel(y, x) == CutPlanar::LABEL_SOURCE)
        output.at<cv::Vec3b>(y, x) = img_.at<cv::Vec3b>(y, x);
      else if(graph.getLabel(y, x) == CutPlanar::LABEL_SINK)
        output.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
      else
        output.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
    }
  }

  cv::imshow("output", output);
  view_.cvWaitKey(0);
  cv::destroyWindow("output");
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  cv::Mat img = cv::imread(argv[1]);
  ViewController vc(img);
  vc.run();
  
  return 0;
}
