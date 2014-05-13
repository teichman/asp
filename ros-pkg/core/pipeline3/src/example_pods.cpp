#include <pipeline/example_pods.h>

using namespace std;

namespace pl
{
  namespace example
  {

    void Sorter::compute()
    {
      PL_ASSERT(numIncoming("Points") == 1);
      *sorted_ = *pull<Vec::ConstPtr>("Points");
      sort(sorted_->begin(), sorted_->end());
      push<Vec::ConstPtr>("Sorted", sorted_);
    }

    void Summarizer::compute()
    {
      PL_ASSERT(numIncoming("Points") > 0);
      const Vec& points = *pull<Vec::ConstPtr>("Points");

      double mean, stdev, mean_neighbor_separation;
      _compute(points, &mean, &stdev, &mean_neighbor_separation);
      
      push("Mean", mean);
      push("Stdev", stdev);
      push("MeanNeighborSeparation", mean_neighbor_separation);
    }

    void Summarizer::_compute(const Vec& points, double* mean, double* stdev, double* mean_neighbor_separation)
    {
      *mean = 0;
      *mean_neighbor_separation = 0;
      for(size_t i = 0; i < points.size(); ++i) {
        if(i > 0) { 
          assert(points[i] >= points[i-1]);
          *mean_neighbor_separation += points[i] - points[i-1];
        }
        *mean += points[i];
      }
      *mean_neighbor_separation /= (double)points.size();
      *mean /= (double)points.size();
      
      *stdev = 0;
      for(size_t i = 0; i < points.size(); ++i)
        *stdev += (points[i] - *mean) * (points[i] - *mean);

      *stdev /= (double)points.size();
    }

    void Aggregator::compute()
    {
      aggregated_->clear();
      
      vector<Vec::ConstPtr> point_sets;
      multiPull("PointSets", &point_sets);

      size_t num_points = 0;
      for(size_t i = 0; i < point_sets.size(); ++i)
        num_points += point_sets[i]->size();

      aggregated_->reserve(num_points);
      vector<size_t> index(point_sets.size(), 0);
      while(true) {
        int sel = -1;
        double min = numeric_limits<double>::max();
        for(size_t i = 0; i < point_sets.size(); ++i) {
          if(index[i] == point_sets[i]->size())
            continue;
          double val = point_sets[i]->at(index[i]);
          if(val < min) { 
            min = val;
            sel = i;
          }
        }

        if(sel == -1)
          break;
        
        aggregated_->push_back(min);
        ++index[sel];
      }

      assert(aggregated_->size() == num_points);
      push<Vec::ConstPtr>("Aggregated", aggregated_);
    }

    void DescriptorAssembler::compute()
    {
      descriptor_->clear();
      multiPull("Elements", descriptor_.get());

      vector<Vec::ConstPtr> subvectors;
      multiPull("SubVectors", &subvectors);
      for(size_t i = 0; i < subvectors.size(); ++i)
        descriptor_->insert(descriptor_->end(), subvectors[i]->begin(), subvectors[i]->end());
      
      push<Vec::ConstPtr>("Descriptor", descriptor_);
    }

    void HistogramGenerator::compute()
    {
      const Vec& points = *pull<Vec::ConstPtr>("Points");
      _compute(points, &hist_, &lower_bounds_);
      push<Vec::ConstPtr>("Histogram", hist_);
      push<Vec::ConstPtr>("LowerBounds", lower_bounds_);
    }

    void HistogramGenerator::_compute(const Vec& points, Vec::Ptr* hist, Vec::Ptr* lower_bounds)
    {
      hist_->clear();
      lower_bounds_->clear();
      num_points_ = 0;

      // -- Set up histogram.
      double min = param<double>("Min");
      double max = param<double>("Max");
      double bin_width = param<double>("BinWidth");
      int num_bins = ceil((max - min) / bin_width);

      lower_bounds_->reserve(num_bins);
      for(int i = 0; i < num_bins; ++i)
        lower_bounds_->push_back((double)i * bin_width);

      // -- Accumulate counts.
      hist_->resize(num_bins, 0);
      for(size_t i = 0; i < points.size(); ++i) {
        if(i > 0)
          assert(points[i] >= points[i-1]);

        int bin = floor((points[i] - min) / bin_width);
        if(bin >= 0 && bin < (int)hist_->size()) { 
          ++hist_->at(bin);
          ++num_points_;
        }
      }

      // -- Normalize if desired.
      if(param<bool>("Normalize"))
        for(size_t i = 0; i < hist_->size(); ++i)
          hist_->at(i) /= num_points_;

      *hist = hist_;
      *lower_bounds = lower_bounds_;
    }
 
    void HistogramGenerator::debug() const
    {
      cout << "============================================================" << endl;
      cout << YAML::Dump(YAMLize()) << endl; // Print Pod text specification.

      // -- Do something slow.
      //    This won't get counted in the timing report.
      usleep(1e5);
            
      // -- If normalizing, check that histogram sums to one.
      double sum = 0;
      for(size_t i = 0; i < hist_->size(); ++i)
        sum += hist_->at(i);
      cout << "Sum of histogram elements: " << sum << endl;
      if(param<bool>("Normalize")) {
        assert(fabs(sum - 1.0) < 1e-4);
      }

      // -- Get a histogram with maximum value of 1.
      Vec hist = *hist_;
      double maxval = 0;
      for(size_t i = 0; i < hist.size(); ++i)
        maxval = fmax(maxval, hist[i]);
      for(size_t i = 0; i < hist.size(); ++i)
        hist[i] /= maxval;

      // -- Print histogram to screen.
      ostringstream oss;
      oss << "0 ---------- " << maxval << endl;
      for(size_t i = 0; i < hist.size(); ++i) {
        oss << " |";
        for(double j = 0; j < hist[i]; j += 0.1)
          oss << "*";
        oss << endl;
      }

      cout << oss.str() << endl;

      ofstream f((debugBasePath() + "-histogram.txt").c_str());
      f << oss.str() << endl;
      f.close();
    }

    void AbstractPod::display(const Vec& vector) const
    {
      cout << "AbstractPod::display: " << endl;
      for(size_t i = 0; i < vector.size(); ++i)
        cout << vector[i] << " ";
      cout << endl;
    }

    void ConcretePodA::compute()
    {
      *vals_ = *pull<Vec::ConstPtr>("Vals");
      push<Vec::ConstPtr>("ChangedVals", vals_);
    }

    void ConcretePodB::compute()
    {
      *vals_ = *pull<Vec::ConstPtr>("Vals");
      push<Vec::ConstPtr>("ChangedVals", vals_);
    }


    ostream& operator<<(ostream& out, const std::vector<double>& vec)
    {
      out << "Vec: ";
      for(size_t i = 0; i < vec.size(); ++i) 
        out << " " << vec[i];
      return out;
    }

    double sample()
    {
      double val = 0;
      for(int i = 0; i < 12; ++i) {
        val += ((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
      }
      val /= 2.0;
      return val;
    }

    Vec::Ptr generateVec(int num_points)
    {
      Vec::Ptr vec(new Vec(num_points));
      for(size_t i = 0; i < vec->size(); ++i)
        vec->at(i) = sample();

      return vec;
    }

    void registerPods()
    {
      REGISTER_POD_TEMPLATE(EntryPoint, Vec::ConstPtr);
      REGISTER_POD_TEMPLATE(EntryPoint, boost::shared_ptr<const Vec>);
      REGISTER_POD(Sorter);
      REGISTER_POD(Summarizer);
      REGISTER_POD(Aggregator);
      REGISTER_POD(DescriptorAssembler);
      REGISTER_POD(HistogramGenerator);
      REGISTER_POD(ConcretePodA);
      REGISTER_POD(ConcretePodB);
    }

    void generateDefaultPipeline(Pipeline* pl)
    {
      pl->createPod("EntryPoint<boost::shared_ptr<const Vec>>", "View0");
      pl->createPod("EntryPoint<boost::shared_ptr<const Vec>>", "View1");
      pl->createPod("EntryPoint<boost::shared_ptr<const Vec>>", "View2");

      pl->createPod("Sorter", "Sorter0");
      pl->createPod("Sorter", "Sorter1");
      pl->createPod("Sorter", "Sorter2");
      pl->connect("Sorter0.Points <- View0.Output");
      pl->connect("Sorter1.Points <- View1.Output");
      pl->connect("Sorter2.Points <- View2.Output");

      pl->createPod("Summarizer", "Summarizer0");
      pl->createPod("Summarizer", "Summarizer1");
      pl->createPod("Summarizer", "Summarizer2");
      pl->connect("Summarizer0.Points <- Sorter0.Sorted");
      pl->connect("Summarizer1.Points <- Sorter1.Sorted");
      pl->connect("Summarizer2.Points <- Sorter2.Sorted");
      
      pl->createPod("Aggregator", "Aggregator0");
      pl->connect("Aggregator0.PointSets <- Sorter0.Sorted");
      pl->connect("Aggregator0.PointSets <- Sorter1.Sorted");
      pl->connect("Aggregator0.PointSets <- Sorter2.Sorted");

      pl->createPod("Summarizer", "Summarizer3");
      pl->connect("Summarizer3.Points <- Aggregator0.Aggregated");

      pl->createPod("HistogramGenerator", "HistogramGenerator0");
      pl->createPod("HistogramGenerator", "HistogramGenerator1");
      pl->createPod("HistogramGenerator", "HistogramGenerator2");
      pl->createPod("HistogramGenerator", "HistogramGenerator3");

      pl->connect("HistogramGenerator0.Points <- Sorter0.Sorted");
      pl->pod("HistogramGenerator0")->setParam("BinWidth", 0.5);
      pl->pod("HistogramGenerator0")->setParam("Min", -2.0);
      pl->pod("HistogramGenerator0")->setParam("Max", 2.0);

      pl->connect("HistogramGenerator1.Points <- Sorter1.Sorted");
      pl->pod("HistogramGenerator1")->setParam("BinWidth", 0.5);
      pl->pod("HistogramGenerator1")->setParam("Min", -2.0);
      pl->pod("HistogramGenerator1")->setParam("Max", 2.0);

      pl->connect("HistogramGenerator2.Points <- Sorter2.Sorted");
      pl->pod("HistogramGenerator2")->setParam("BinWidth", 0.5);
      pl->pod("HistogramGenerator2")->setParam("Min", -2.0);
      pl->pod("HistogramGenerator2")->setParam("Max", 2.0);

      pl->connect("HistogramGenerator3.Points <- Aggregator0.Aggregated");
      pl->pod("HistogramGenerator3")->setParam("BinWidth", 0.25);
      pl->pod("HistogramGenerator3")->setParam("Min", 0.0);
      pl->pod("HistogramGenerator3")->setParam("Max", 3.0);

      pl->createPod("DescriptorAssembler", "DescriptorAssembler");

      pl->connect("DescriptorAssembler.SubVectors <- HistogramGenerator0.Histogram");
      pl->connect("DescriptorAssembler.SubVectors <- HistogramGenerator1.Histogram");
      pl->connect("DescriptorAssembler.SubVectors <- HistogramGenerator2.Histogram");
      pl->connect("DescriptorAssembler.SubVectors <- HistogramGenerator3.Histogram");
      pl->connect("DescriptorAssembler.Elements <- Summarizer0.Mean");
      pl->connect("DescriptorAssembler.Elements <- Summarizer0.Stdev");
      pl->connect("DescriptorAssembler.Elements <- Summarizer0.MeanNeighborSeparation");
      pl->connect("DescriptorAssembler.Elements <- Summarizer1.Mean");
      pl->connect("DescriptorAssembler.Elements <- Summarizer1.Stdev");
      pl->connect("DescriptorAssembler.Elements <- Summarizer1.MeanNeighborSeparation");
      pl->connect("DescriptorAssembler.Elements <- Summarizer2.Mean");
      pl->connect("DescriptorAssembler.Elements <- Summarizer2.Stdev");
      pl->connect("DescriptorAssembler.Elements <- Summarizer2.MeanNeighborSeparation");
      pl->connect("DescriptorAssembler.Elements <- Summarizer3.Mean");
      pl->connect("DescriptorAssembler.Elements <- Summarizer3.Stdev");
      pl->connect("DescriptorAssembler.Elements <- Summarizer3.MeanNeighborSeparation");

      pl->createPod("ConcretePodA", "ConcretePodA");
      pl->createPod("ConcretePodB", "ConcretePodB");
      pl->pod("ConcretePodB")->setParam("Something", true);
      
      pl->connect("ConcretePodA.Vals <- DescriptorAssembler.Descriptor");
      pl->connect("ConcretePodB.Vals <- DescriptorAssembler.Descriptor");
    }

    bool isRequired(Pod* pod)
    {
      return (dynamic_cast< EntryPoint<Vec::ConstPtr>* >(pod) || dynamic_cast<DescriptorAssembler*>(pod));
    }

  } // namespace example
} // namespace pl
