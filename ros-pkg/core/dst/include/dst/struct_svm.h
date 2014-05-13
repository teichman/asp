#ifndef STRUCT_SVM_H
#define STRUCT_SVM_H

#include <boost/thread.hpp>
#include <optimization/nips.h>
#include <optimization/common_functions.h>
#include <eigen_extensions/eigen_extensions.h>
#include <dst/segmentation_pipeline.h>

namespace dst
{

  class Constraint
  {
  public:
    Constraint() :
      loss_(-1),
      tr_ex_id_(-1)
      {
      }
    
    Eigen::VectorXd dpsi_;
    double loss_;
    int tr_ex_id_;
  };

  class ConstraintGenerator;
  
  class StructSVM
  {
  public:
    std::vector<KinectSequence::Ptr> sequences_;
    
    StructSVM(double c, double precision, bool margin_rescaling);
    void loadSequences(const std::string& path);
    //! Assumes that previous segmentation up until this point is perfect.
    Eigen::VectorXd train();
    //! Iteratively A) runs DS&T, adding constraints for each frame until segmentation
    //! gets really bad, then B) learns new weights.
    Eigen::VectorXd trainRobust();
    Eigen::VectorXd trainRobust2();
    void precache(std::vector<FramePotentialsCache::Ptr>* caches,
                  std::vector<cv::Mat1b>* labels) const;
    Eigen::VectorXd runSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
                              const std::vector<cv::Mat1b>& labels,
                              std::vector<Constraint>* constraints = NULL) const;
  protected:
    double c_;
    double precision_;
    bool margin_rescaling_;

    int getNumWeights() const { SegmentationPipeline sp(1); return sp.getWeights().rows(); } 
    Eigen::VectorXd runNSlackSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
                                    const std::vector<cv::Mat1b>& labels) const;
    Eigen::VectorXd runOneSlackSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
                                      const std::vector<cv::Mat1b>& labels,
                                      std::vector<Constraint>* constraints) const;

    void addConstraint(const SegmentationPipeline& sp, int id,
                       cv::Mat1b label, cv::Mat1b ymv, double loss,
                       std::vector<Constraint>* constraints) const;
      
    double loss(cv::Mat1b label, cv::Mat1b pred, bool hamming) const;
    double fgNormalizedHammingLoss(cv::Mat1b label, cv::Mat1b pred) const;
    
    
    static double* eigToQPO(const Eigen::VectorXd& eig);
    static double* eigToQPO(const Eigen::MatrixXd& eig);
    double updateWeights2(const std::vector<Constraint>& constraints,
                          int num_tr_ex,
                          int num_edge_weights,
                          Eigen::VectorXd* weights,
                          Eigen::VectorXd* slacks) const;
    
    friend class ConstraintGenerator;
  };

  class ConstraintGenerator : public pipeline2::ComputeNode
  {
  public:
    Constraint con_;
    double hamming_loss_;
    double normalized_loss_;
    
    ConstraintGenerator(const StructSVM* svm,
                        const Eigen::VectorXd& weights,
                        int tr_ex_id,
                        FramePotentialsCache::Ptr cache,
                        cv::Mat1b labels);
    void _compute();
    //void _display() const;
    void _flush();
    std::string _getName() const {return "ConstraintGenerator";}

  protected:
    const StructSVM* svm_;
    Eigen::VectorXd weights_;
    int tr_ex_id_;
    FramePotentialsCache::Ptr cache_;
    cv::Mat1b labels_;
  };
  
} 
  
#endif // STRUCT_SVM_H
