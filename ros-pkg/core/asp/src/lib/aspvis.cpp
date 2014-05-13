#include <asp/aspvis.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace asp
{

  AspVis::AspVis(Asp* asp, cv::Mat3b img, double scale, string savedir) :
    asp_(asp),
    img_(img),
    img_view_("Image", scale),
    seed_(cv::Mat1b(img_.size(), 127)),
    show_seed_(true),
    needs_redraw_(true),
    seed_radius_(2),
    seg_view_("Segmentation", scale),
    seg_(cv::Mat1b(img_.size(), 0)),
    debug_(true),
    savedir_(savedir)
  {
    img_view_.setDelegate((OpenCVViewDelegate*)this);
    img_view_.message_scale_ = 0.25;
    img_view_.message_thickness_ = 1.0;

    //seg_view_.setDelegate((OpenCVViewDelegate*)this);
  }

  void AspVis::_run()
  {
    img_view_.updateImage(img_);
    while(!quitting_) {

      lockWrite();
      if(needs_redraw_)
        draw();
      unlockWrite();
                  
      char key = img_view_.cvWaitKey(10);
      handleKeypress(key);
    }
  }

  void AspVis::draw()
  {
    cv::Mat3b vis = img_.clone();
    if(show_seed_) { 
      for(int y = 0; y < vis.rows; ++y) {
        for(int x = 0; x < vis.cols; ++x) {
          if(seed_(y, x) == 255)
            vis(y, x) = cv::Vec3b(255, 255, 255);
          else if(seed_(y, x) == 0)
            vis(y, x) = cv::Vec3b(0, 0, 0);
        }
      }
    }
    img_view_.updateImage(vis);

    cv::Mat3b segvis = cv::Mat3b(img_.size(), cv::Vec3b(0, 0, 0));
    visualizeSegmentation(seg_, img_, segvis);
    seg_view_.updateImage(segvis);
    
    needs_redraw_ = false;
  }
  
  void AspVis::handleKeypress(char key)
  {
    switch(key) {
    case 'd':
      lockWrite();
      debug_ = !debug_;
      cout << "debug_: " << debug_ << endl;
      unlockWrite();
      break;
    case 'c':
      lockWrite();
      seed_ = 127;
      needs_redraw_ = true;
      unlockWrite();
      break;
    case 'C':
      lockWrite();
      seed_ = 127;
      seg_ = 127;
      needs_redraw_ = true;
      unlockWrite();
      break;
    case 'q':
      quit();
      break;
    case 'S':
      save();
      break;
    case 'v':
      lockWrite();
      show_seed_ = !show_seed_;
      cout << "show_seed_: " << show_seed_ << endl;
      needs_redraw_ = true;
      unlockWrite();
      break;
    case 's':
      segment();
      break;
    case '+':
      lockWrite();
      ++seed_radius_;
      unlockWrite();
      break;
    case '-':
      lockWrite();
      --seed_radius_;
      seed_radius_ = max(0, seed_radius_);
      unlockWrite();
      break;
    default:
      break;
    }
  }

  void AspVis::save()
  {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%Y.%m.%d-%H.%M.%S", timeinfo);
    string basename = savedir_ + "/" + string(buffer);

    if(!bfs::exists(savedir_))
      bfs::create_directory(savedir_);
    
    cv::imwrite(basename + "-image.png", img_);
    cv::imwrite(basename + "-seed.png", seed_);
    cv::imwrite(basename + "-seg.png", seg_);
    cout << "Saved to " << basename << "-*.png" << endl;
  }

  void AspVis::segment()
  {
    scopeLockWrite;
    
    asp_->pod< EntryPoint<cv::Mat3b> >("ImageEntryPoint")->setData(img_);
    asp_->pod< EntryPoint<cv::Mat1b> >("MaskEntryPoint")->setData(cv::Mat1b(img_.size(), 255));
    asp_->pod< EntryPoint<cv::Mat1b> >("SeedEntryPoint")->setData(seed_);
    asp_->setDebug(debug_);
    asp_->compute();
    cout << asp_->reportTiming() << endl;

    asp_->setDebug(false);
    asp_->segment(&seg_);
    needs_redraw_ = true;
  }

  void AspVis::mouseEvent(int event, int x, int y, int flags, void* param)
  {
    if(!show_seed_)
      return;

    // -- Left click to add to source.
    if(flags & CV_EVENT_FLAG_LBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i)
        for(int j = y - seed_radius_; j <= y + seed_radius_; ++j)
          if(i >= 0 && i < seed_.cols && j >= 0 && j < seed_.rows)
            seed_(j, i) = 255;

      scopeLockWrite;
      needs_redraw_ = true;
    }

    // -- Right click to add to sink.
    else if(flags & CV_EVENT_FLAG_RBUTTON) {

      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) 
        for(int j = y - seed_radius_; j <= y + seed_radius_; ++j)
          if(i >= 0 && i < seed_.cols && j >= 0 && j < seed_.rows)
            seed_(j, i) = 0;

      scopeLockWrite;
      needs_redraw_ = true;
    }
  }

}  // namespace asp
 
