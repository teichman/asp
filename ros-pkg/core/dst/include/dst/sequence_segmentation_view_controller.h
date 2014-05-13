#ifndef SEQUENCE_SEGMENTATION_VIEW_CONTROLLER_H
#define SEQUENCE_SEGMENTATION_VIEW_CONTROLLER_H

#include <image_labeler/opencv_view.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <agent/lockable.h>
#include <dst/kinect_sequence.h>
#include <dst/segmentation_pipeline.h>
#include <dst/segmentation_visualizer.h>

namespace dst
{  
  class SequenceSegmentationViewController : public OpenCVViewDelegate, public Lockable
  {
  public:
    typedef enum {RAW, SEED, SEGMENTED} state_t;
    
    KinectSequence::Ptr seq_;
    OpenCVView seg_view_;
    pcl::visualization::PCLVisualizer vis_;
    OpenCVView img_view_;
    int seed_radius_;

    SequenceSegmentationViewController(KinectSequence::Ptr seq, SegmentationPipeline::Ptr sp);
    ~SequenceSegmentationViewController();
    void run( const std::string& cmd = "" );
    void setWeights(const Eigen::VectorXd& weights) { sp_->setWeights(weights, true); }

  private:
    SegmentationPipeline::Ptr sp_;
    int current_idx_;
    bool quitting_;
    bool needs_redraw_;
    state_t state_;
    cv::Mat3b seed_vis_;
    cv::Mat3b seg_vis_;
    int vis_type_;
    bool show_seg_3d_;
    std::vector<KinectCloud::Ptr> segmented_pcds_;
    float max_range_;
    double cluster_tol_;
    KinectCloud::Ptr background_model_;

    KinectCloud::Ptr generateForeground(cv::Mat1b seg, const KinectCloud& cloud) const;
    void increaseSeedWeights();
    void useSegmentationAsSeed();
    void toggleDebug();
    void saveGraphviz() const;
    void saveVisualization();
    void handleKeypress(std::string key, bool automated = false);
    void handleKeypress(char key, bool automated = false);
    void mouseEvent(int event, int x, int y, int flags, void* param);
    void advance(int num);
    void segmentUsingBackgroundModel(pcl::search::KdTree<Point>::Ptr tree = pcl::search::KdTree<Point>::Ptr());
    void segmentAllUsingBackgroundModel();
    void extractConnectedComponent();
    void extractPlane();
    void clearHelperSeedLabels();
    void draw();
    void drawSegVis();
    void drawSeedVis();
    void segmentImage();
    void segmentSequence(int idx = 0);
    void saveSequence();
    size_t size() const;
    void transitionTo(state_t state);
    void updatePCLVisualizer();
    //! For PCL.
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* data);
    
    friend class OpenCVView;
  };
    
  
} // namespace dst

#endif // SEQUENCE_SEGMENTATION_VIEW_CONTROLLER_H


