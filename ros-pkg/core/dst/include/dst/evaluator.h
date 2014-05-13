#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <dst/segmentation_pipeline.h>

namespace dst
{
  class SequenceResults
  {
  public:
    std::string name_;
    double total_frames_;
    double hamming_loss_;
    double hamming_accuracy_;
    double normalized_loss_;
    double capped_normalized_loss_;
    double failure_criterion_;
    int frames_till_failure_;
    bool failed_;

    SequenceResults(const std::string& name);
    void update(cv::Mat1b label, cv::Mat1b pred, bool verbose = false);
    std::string status() const;
  };
  
  class Evaluator
  {
  public:
    Evaluator(const Eigen::VectorXd& weights, bool frame_to_frame);
    void evaluate(KinectSequence::ConstPtr seq, const std::string& name);
    std::string status() const;
    
  private:
    Eigen::VectorXd weights_;
    bool frame_to_frame_;
    std::vector<SequenceResults> seq_res_;
    HighResTimer hrt_;
  };

}

#endif // EVALUATOR_H
