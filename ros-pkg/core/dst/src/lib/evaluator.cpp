#include <dst/evaluator.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define VISUALIZE (getenv("VISUALIZE"))

using namespace std;

namespace dst
{

  SequenceResults::SequenceResults(const std::string& name) :
    name_(name),
    total_frames_(0),
    hamming_loss_(0),
    hamming_accuracy_(0),
    normalized_loss_(0),
    capped_normalized_loss_(0),
    failure_criterion_(0.75),
    frames_till_failure_(0),
    failed_(false)
  {
  }

  void SequenceResults::update(cv::Mat1b label, cv::Mat1b pred, bool verbose)
  {
    ROS_ASSERT(label.rows == pred.rows);
    ROS_ASSERT(label.cols == pred.cols);
    ++total_frames_;
    
    // -- Hamming loss.
    double hamming_loss = 0;
    double num_fg = 0;
    for(int y = 0; y < pred.rows; ++y) { 
      for(int x = 0; x < pred.cols; ++x) {
        if(label(y, x) != 255 && label(y, x) != 0)
          continue;
        
        if(pred(y, x) != label(y, x))
          ++hamming_loss;
        if(label(y, x) == 255)
          ++num_fg;
      }
    }
    double hamming_accuracy = (double)(pred.rows * pred.cols - hamming_loss) / (double)(pred.rows * pred.cols);
    hamming_loss_ += hamming_loss;
    hamming_accuracy_ += hamming_accuracy;

    // -- Normalized loss.
    double normalized_loss = hamming_loss / num_fg;
    normalized_loss_ += normalized_loss;
    capped_normalized_loss_ += min(1.0, normalized_loss);

    // -- Frames till failure.
    if(!failed_) {
      ++frames_till_failure_;
      if(normalized_loss >= failure_criterion_)
        failed_ = true;
    }

    if(verbose)
      cout << "SequenceResults::update(): Hamming loss = " << hamming_loss
           << ", total fg: " << num_fg << endl;
  }

  
  std::string SequenceResults::status() const
  {
    ostringstream oss;
    oss << "Results for sequence " << name_ << endl;
    oss << "Total frames: " << total_frames_ << endl;
    oss << "Mean Hamming loss: " << hamming_loss_ / total_frames_ << endl;
    oss << "Mean Hamming accuracy: " << hamming_accuracy_ / total_frames_ << endl;
    oss << "Mean capped normalized loss: " << capped_normalized_loss_ / total_frames_ << endl;
    oss << "Mean capped normalized accuracy: " << 1.0 - (capped_normalized_loss_ / total_frames_) << endl;
    oss << "Failure criterion: " << failure_criterion_ << endl;
    oss << "Failed?: " << failed_ << endl;
    oss << "Frames until failure: " << frames_till_failure_ << endl;
    oss << "Pct frames until failure: " << 100.0 * (double)frames_till_failure_ / total_frames_ << endl;
    return oss.str();
  }
  
  Evaluator::Evaluator(const Eigen::VectorXd& weights,
                       bool frame_to_frame) :
    weights_(weights),
    frame_to_frame_(frame_to_frame)
  {
  }
  
  void Evaluator::evaluate(KinectSequence::ConstPtr seq, const std::string& name)
  {
    hrt_.start();
    ROS_ASSERT(!seq->segmentations_.empty());

    SegmentationPipeline sp(NUM_THREADS);
    sp.verbose_ = true;
    sp.setWeights(weights_);
    cv::Mat1b pred(seq->segmentations_[0].size(), 127);
    cv::Mat1b empty_seed(seq->seed_images_[0].size(), 127);
    SequenceResults sr(name);
    for(size_t i = 0; i < seq->segmentations_.size(); ++i) {
      //cout << "Evaluating frame " << i+1 << " / " << seq->segmentations_.size() << endl;      
      // -- Segment the frame.
      if(i == 0) {
        sp.run(seq->seed_images_[i],
               seq->images_[i],
               seq->pointclouds_[i],
               cv::Mat3b(),
               cv::Mat1b(),
               KinectCloud::Ptr(),
               pred,
               KinectCloud::Ptr());
        continue; // Do not count the segmentation of the first frame.
      }
      else {
        cv::Mat1b prev_seg = pred;
        if(frame_to_frame_)
          prev_seg = seq->segmentations_[i-1];
        
        sp.run(empty_seed,
               seq->images_[i],
               seq->pointclouds_[i],
               seq->images_[i-1],
               prev_seg,
               seq->pointclouds_[i-1],
               pred,
               KinectCloud::Ptr());
      }
      sr.update(seq->segmentations_[i], pred);
      
      if(VISUALIZE) { 
        cv::imshow("Original image", seq->images_[i]);
        cv::imshow("Prediction", pred);
        cvMoveWindow("Original image", 200, 100);
        cvMoveWindow("Prediction", 600, 100);
        cv::waitKey(10);
      }
    }

    seq_res_.push_back(sr);
    hrt_.stop();
  }

  std::string Evaluator::status() const
  {
    ostringstream oss;
    double capped_normalized_loss = 0;
    double hamming_loss = 0;
    double total_frames = 0;
    for(size_t i = 0; i < seq_res_.size(); ++i) {
      capped_normalized_loss += seq_res_[i].capped_normalized_loss_ / seq_res_[i].total_frames_;
      hamming_loss += seq_res_[i].hamming_loss_ / seq_res_[i].total_frames_;
      total_frames += seq_res_[i].total_frames_;
      
      oss << seq_res_[i].status() << endl;
    }
    
    oss << "Frame to frame: " << frame_to_frame_ << endl;
    oss << "Total number of sequences evaluated: " << seq_res_.size() << endl;
    oss << "Total number of frames evaluated: " << total_frames << endl;
    oss << "Overall Hamming loss: " << hamming_loss / seq_res_.size() << endl;
    oss << "Overall mean capped normalized loss: " << capped_normalized_loss / seq_res_.size() << endl;
    oss << "Total time to run evaluations (min): " << hrt_.getMinutes() << endl;
    oss << "Average time per frame (ms): " << hrt_.getMilliseconds() / total_frames << endl;

    return oss.str();
  }

} // namespace dst

