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

  } // namespace example
} // namespace pl
