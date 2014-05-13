#ifndef TEMPLATE_MATCHER_NPG_H
#define TEMPLATE_MATCHER_NPG_H

#include <dst/node_potential_generator.h>
#include <dst/depth_projector.h>

namespace dst
{

  class TemplateMatcherNPG : public NodePotentialGenerator
  {
  public:
    TemplateMatcherNPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
                       pipeline2::Outlet<cv::Mat3b>* img_otl,
                       pipeline2::Outlet<cv::Mat1b>* mask_otl);

  protected:
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    pipeline2::Outlet<cv::Mat1b>* mask_otl_;

    //! Gets input, computes node potentials, pushes them out via NodePotentialGenerator's source_otl_ and sink_otl_.
    void _compute();
    //! Only called when in debugging mode.  Here you can save debugging images to disk and look at them after a run.
    void _display() const;
    //! Called after processing every frame.
    void _flush();
    //! Called when starting to track a new object.  Should clear all saved templates, probably some other things.
    void _reset();
    //! Returns a unique name to identify this node and its params.  If a param setting changes, this should also change.
    //! This lets you confirm that you're using a particular computation pipeline to generate a result.
    std::string _getName() const;
  };
  
}

#endif // TEMPLATE_MATCHER_NPG_H
