#ifndef EDGE_POTENTIAL_AGGREGATOR_H
#define EDGE_POTENTIAL_AGGREGATOR_H

#include <dst/edge_potential_generator.h>
#include <dst/potentials_cache.h>
#include <dst/depth_projector.h>

namespace dst
{

  //! Adds up a weighted combination of all edge potentials.
  class EdgePotentialAggregator : public EdgePotentialGenerator
  {
  public:
    EdgePotentialAggregator(pipeline2::Outlet<Graph3dPtr>* graph_otl,
                            pipeline2::Outlet<cv::Mat3b>* img_otl,
                            pipeline2::Outlet<DepthProjector::Output>* index_otl,
                            const std::vector<EdgePotentialGenerator*>& generators,
                            const Eigen::VectorXd& weights,
                            bool ignore_depthless = true,
                            pipeline2::Outlet< Eigen::SparseMatrix<double, Eigen::RowMajor>* >* weights_otl = NULL);

    std::string weightsStatus() const;
    Eigen::VectorXd getWeights() const { return weights_; }
    void setWeights(const Eigen::VectorXd& weights, bool verbose = false);
    void cacheUnweightedPotentials(FramePotentialsCache::Ptr framecache) const;
    NameMapping getNameMapping() const;
    
  protected:
    pipeline2::Outlet<Graph3dPtr>* graph_otl_;
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    std::vector<EdgePotentialGenerator*> generators_;
    Eigen::VectorXd weights_;
    bool ignore_depthless_;
    pipeline2::Outlet< Eigen::SparseMatrix<double, Eigen::RowMajor>* >* weights_otl_;

    //! sanitized is a version of pot.  Edge potentials that touch at least one depthless
    //! pixel are removed.
    void ignoreDepthless(const Eigen::SparseMatrix<double, Eigen::RowMajor>& pot,
                         Eigen::SparseMatrix<double, Eigen::RowMajor>* sanitized,
                         const std::string& name = "") const;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // EDGE_POTENTIAL_AGGREGATOR_H
