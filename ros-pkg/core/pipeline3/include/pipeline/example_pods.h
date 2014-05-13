#ifndef EXAMPLE_PODS_H
#define EXAMPLE_PODS_H

#include <pipeline/pipeline.h>

namespace pl
{
  namespace example
  {

    class Vec : public std::vector<double>
    {
    public:
      typedef boost::shared_ptr<Vec> Ptr;
      typedef boost::shared_ptr<const Vec> ConstPtr;
      Vec() {}
      Vec(size_t num) : std::vector<double>(num)
      {
      }
    };
    
    //! Sorts a set of points.
    class Sorter : public Pod
    {
    public:
      DECLARE_POD(Sorter);
      Sorter(std::string name) :
        Pod(name),
        sorted_(new Vec)
      {
        declareInput<Vec::ConstPtr>("Points");
        declareOutput<Vec::ConstPtr>("Sorted");
      }

      Vec::Ptr sorted_;
      void compute();
    };

    //! Computes descriptors on a set of points.
    //! Requires the input to be sorted.
    class Summarizer : public Pod
    {
    public:
      DECLARE_POD(Summarizer);
      Summarizer(std::string name) :
        Pod(name)
      {
        declareInput<Vec::ConstPtr>("Points"); // Must be sorted.
        declareOutput<double>("Mean");
        declareOutput<double>("Stdev");
        declareOutput<double>("MeanNeighborSeparation");
      }

      void compute();
      //! This demonstrates one method of making Pod functionality available outside of a Pipeline.
      static void _compute(const Vec& points, double* mean, double* stdev, double* mean_neighbor_separation);
    };

    //! Computes a histogram on a set of points.
    //! Requires the input to be sorted.
    class HistogramGenerator : public Pod
    {
    public:
      DECLARE_POD(HistogramGenerator);
      HistogramGenerator(std::string name) :
        Pod(name),
        hist_(new Vec),
        lower_bounds_(new Vec)
      {
        declareParam<double>("BinWidth");
        declareParam<double>("Min");
        declareParam<double>("Max");
        declareParam<bool>("Normalize", false); // Whether or not the final histogram should sum to one. Default false.
        declareInput<Vec::ConstPtr>("Points"); // Must be sorted.
        declareOutput<Vec::ConstPtr>("Histogram");
        declareOutput<Vec::ConstPtr>("LowerBounds"); // Vector of the lower bounds of each bin.
      }

      Vec::Ptr hist_;
      Vec::Ptr lower_bounds_;
      double num_points_;
      
      void compute();
      //! This demonstrates another method of making Pod functionality available outside of a Pipeline.
      void _compute(const Vec& points, Vec::Ptr* hist, Vec::Ptr* lower_bounds);
      //! Common mistake: not making this method const.  It will silently just not get called.
      void debug() const;
    };
    
    //! Merges multiple sets of points.
    //! Requires the inputs to be sorted.
    class Aggregator : public Pod
    {
    public:
      DECLARE_POD(Aggregator);
      Aggregator(std::string name) :
        Pod(name),
        aggregated_(new Vec)
      {
        declareMultiInput<Vec::ConstPtr>("PointSets");
        declareOutput<Vec::ConstPtr>("Aggregated");
      }

      Vec::Ptr aggregated_;
      void compute();
    };

    //! Assembles individual real numbers and subvectors
    //! into a single descriptor vector
    //! for consumption by a classifier or dataset manager.
    class DescriptorAssembler : public Pod
    {
    public:
      DECLARE_POD(DescriptorAssembler);
      DescriptorAssembler(std::string name) :
        Pod(name),
        descriptor_(new Vec)
      {
        declareMultiInput<double>("Elements");
        declareMultiInput<Vec::ConstPtr>("SubVectors");
        declareOutput<Vec::ConstPtr>("Descriptor");
      }

      Vec::Ptr descriptor_;
      void compute();
    };

    class AbstractPod : public Pod
    {
    public:
      // Uncommenting this line causes a compilation error because the class is abstract.
      // You should only DECLARE_POD for concrete classes.
      // DECLARE_POD(AbstractPod);
      AbstractPod(std::string name) :
        Pod(name),
        vals_(new Vec)
      {
        declareInput<Vec::ConstPtr>("Vals");
        declareOutput<Vec::ConstPtr>("ChangedVals");
      }

      Vec::Ptr vals_;
      void display(const Vec& vector) const;
      void debug() const { display(*vals_); }
    };

    // These derived classes will have the declared inputs and outputs of the base class.
    class ConcretePodA : public AbstractPod
    {
    public:
      DECLARE_POD(ConcretePodA);
      ConcretePodA(std::string name) :
        AbstractPod(name)
      {
      }

      void compute();
    };

    class ConcretePodB : public AbstractPod
    {
    public:
      DECLARE_POD(ConcretePodB);
      ConcretePodB(std::string name) :
        AbstractPod(name)
      {
        declareParam<bool>("Something", false);
      }

      void compute();
    };

    void generateDefaultPipeline(Pipeline* pl);
    void registerPods();
    Vec::Ptr generateVec(int num_points);
    double sample();
    std::ostream& operator<<(std::ostream& out, const std::vector<double>& vec);
    bool isRequired(Pod* pod);    
  } 
}

#endif // EXAMPLE_PODS_H
