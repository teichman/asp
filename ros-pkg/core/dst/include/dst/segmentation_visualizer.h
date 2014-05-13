#ifndef SEGMENTATION_VISUALIZER_H
#define SEGMENTATION_VISUALIZER_H

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <dst/typedefs.h>
#include <dst/helper_functions.h>

namespace dst
{
  class SegmentationVisualizer
  {
  public:
    bool quit_at_end_;

    SegmentationVisualizer(const std::vector<KinectCloud::Ptr>& original,
                           const std::vector<KinectCloud::Ptr>& segmented);
    void spin();

  private:
    pcl::visualization::PCLVisualizer vis_;
    std::vector<KinectCloud::Ptr> original_;
    std::vector<KinectCloud::Ptr> segmented_;
    int idx_;
    bool stepping_;
    bool recording_;


    void increment(int num);
    void draw();
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* data);
  };
}

#endif // SEGMENTATION_VISUALIZER_H
