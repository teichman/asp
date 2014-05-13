#include <string>
#include <gtest/gtest.h>
#include <pipeline/pipeline.h>
#include <pipeline/common_pods.h>
#include <pipeline/example_pods.h>

using boost::any_cast;
using boost::any;
using namespace pl;
using namespace pl::example;
using namespace std;

TEST(Pipeline, RemovePodNames)
{
  registerPods();
  Pipeline pl(1);
  generateDefaultPipeline(&pl);
  cout << pl.pod("DescriptorAssembler")->getUniqueString() << endl;
}

TEST(Pipeline, hasParent)
{
  registerPods();
  Pipeline pl(1);
  generateDefaultPipeline(&pl);
  vector<Sorter*> sorters = pl.filterPods<Sorter>();
  for(size_t i = 0; i < sorters.size(); ++i) { 
    EXPECT_TRUE(sorters[i]->hasParent< EntryPoint< boost::shared_ptr<const Vec> > >());
    EXPECT_TRUE(!sorters[i]->hasParent<Aggregator>());
  }
}

bool immune(Pod* pod)
{
  if(pod->isPodType< EntryPoint< boost::shared_ptr<const Vec> > >())
    return true;
  if(pod->isPodType<ConcretePodA>())
    return true;
  if(pod->isPodType<ConcretePodB>())
    return true;

  return false;
}

TEST(Pipeline, disconnect)
{
  registerPods();
  Pipeline pl(1);
  generateDefaultPipeline(&pl);
  EXPECT_TRUE(pl.hasPod("View0"));
  EXPECT_TRUE(pl.pod("Sorter0")->hasParent("View0"));

  pl.disconnect("Sorter0.Points <- View0.Output");
  pl.writeGraphviz("graphvis-disconnect-before_pruning");
  EXPECT_TRUE(pl.hasPod("View0"));
  EXPECT_TRUE(!pl.pod("Sorter0")->hasParent("View0"));

  pl.prune(immune);
  pl.writeGraphviz("graphvis-disconnect-after_pruning");
  EXPECT_TRUE(pl.hasPod("View0"));
  EXPECT_TRUE(!pl.hasPod("Sorter0"));
  EXPECT_TRUE(!pl.hasPod("Histogram0"));
  EXPECT_TRUE(pl.hasPod("Aggregator0"));
  EXPECT_TRUE(pl.hasPod("DescriptorAssembler"));
}

TEST(Pipeline, Serialize)
{
  registerPods();
  Pipeline pl(1);
  generateDefaultPipeline(&pl);

  string filename = "example.yml";
  pl.saveYAML(filename);
  cout << "Serialized Pipeline specification to " << filename << endl;

  cout << "Generating data..." << endl;
  int num_points = 1e5;
  Vec::ConstPtr v0 = generateVec(num_points);
  Vec::ConstPtr v1 = generateVec(num_points);
  Vec::ConstPtr v2 = generateVec(num_points);
  cout << "Done." << endl;
  
  pl.push("View0", v0);
  pl.push("View1", v1);
  pl.push("View2", v2);
  pl.compute();
  Vec::ConstPtr descriptor;
  pl.pull("DescriptorAssembler", "Descriptor", &descriptor);
  cout << pl.reportTiming() << endl;

  Pipeline pl2(10);
  pl2.loadYAML(filename);
  pl2.saveYAML(filename + ".2");
  cout << "Deserialized Pipeline specification from " << filename << endl;
  string graphvis_filename = "graphvis";
  pl2.writeGraphviz(graphvis_filename);
  cout << "Saved graphvis to " << graphvis_filename << endl;

  pl2.push("View0", v0);
  pl2.push("View1", v1);
  pl2.push("View2", v2);
  pl2.compute();
  Vec::ConstPtr descriptor2;
  pl2.pull("DescriptorAssembler", "Descriptor", &descriptor2);
  cout << pl2.reportTiming() << endl;

  cout << "Output of the two runs: " << endl;
  cout << *descriptor << endl;
  cout << *descriptor2 << endl;
      
  EXPECT_TRUE(descriptor->size() == descriptor2->size());
  for(size_t i = 0; i < descriptor->size(); ++i)
    EXPECT_FLOAT_EQ(descriptor->at(i), descriptor2->at(i));

  uint64_t hash = pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor");
  uint64_t hash2 = pl2.pod("DescriptorAssembler")->getUniqueHash("Descriptor");
  EXPECT_TRUE(hash == hash2);
}

TEST(Pipeline, UniqueString)
{
  registerPods();
  
  // -- Set up a simple pipeline.
  Pipeline pl(1);
  pl.createPod("EntryPoint<Vec::ConstPtr>", "View");

  pl.createPod("Sorter", "Sorter");
  pl.connect("Sorter.Points <- View.Output");
    
  pl.createPod("Summarizer", "Summarizer");
  pl.connect("Summarizer.Points <- Sorter.Sorted");
    
  pl.createPod("HistogramGenerator", "HistogramGenerator");
  pl.connect("HistogramGenerator.Points <- Sorter.Sorted");
  pl.pod("HistogramGenerator")->setParam("BinWidth", 0.1);
  pl.pod("HistogramGenerator")->setParam("Min", -2.0);
  pl.pod("HistogramGenerator")->setParam("Max", 2.0);

  pl.createPod("DescriptorAssembler", "DescriptorAssembler");
  pl.connect("DescriptorAssembler.SubVectors <- HistogramGenerator.Histogram");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.Mean");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.Stdev");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.MeanNeighborSeparation");

  // -- Changing the params of an upstream node should change the hash.
  uint64_t orig = pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor");

  pl.pod("HistogramGenerator")->setParam("Normalize", true);
  EXPECT_NE(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));
  pl.pod("HistogramGenerator")->setParam("Normalize", false);
  EXPECT_EQ(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));
  pl.pod("HistogramGenerator")->setParam("BinWidth", 0.05);
  EXPECT_NE(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));
  pl.pod("HistogramGenerator")->setParam("BinWidth", 0.1);
  EXPECT_EQ(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));

  // -- Changing the order of registration should change the hash.
  pl.deletePod("DescriptorAssembler");
  pl.createPod("DescriptorAssembler", "DescriptorAssembler");
  pl.connect("DescriptorAssembler.SubVectors <- HistogramGenerator.Histogram");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.MeanNeighborSeparation");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.Stdev");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.Mean");
  EXPECT_NE(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));

  pl.deletePod("DescriptorAssembler");
  pl.createPod("DescriptorAssembler", "DescriptorAssembler");
  pl.connect("DescriptorAssembler.SubVectors <- HistogramGenerator.Histogram");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.Mean");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.Stdev");
  pl.connect("DescriptorAssembler.Elements <- Summarizer.MeanNeighborSeparation");
  EXPECT_EQ(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));

  // -- Changing the structure should change the hash.
  pl.createPod("HistogramGenerator", "HistogramGenerator2");
  pl.connect("HistogramGenerator2.Points <- Sorter.Sorted");
  pl.pod("HistogramGenerator2")->setParam("BinWidth", 0.1);
  pl.pod("HistogramGenerator2")->setParam("Min", -2.0);
  pl.pod("HistogramGenerator2")->setParam("Max", 2.0);
  pl.connect("DescriptorAssembler.SubVectors <- HistogramGenerator2.Histogram");
  EXPECT_NE(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));
  pl.deletePod("HistogramGenerator2");
  EXPECT_EQ(orig, pl.pod("DescriptorAssembler")->getUniqueHash("Descriptor"));
}

TEST(Pipeline, MultiCompute)
{
  Pipeline pl(10);
  pl.loadYAML("example.yml");

  int num_points = 1e3;
  Vec::ConstPtr v0 = generateVec(num_points);
  Vec::ConstPtr v1 = generateVec(num_points);
  Vec::ConstPtr v2 = generateVec(num_points);
  pl.push("View0", v0);
  pl.push("View1", v1);
  pl.push("View2", v2);
  pl.compute();
  Vec::ConstPtr descriptor = pl.pull<Vec::ConstPtr>("DescriptorAssembler.Descriptor");
  pl.compute();
  Vec::ConstPtr descriptor2 = pl.pull<Vec::ConstPtr>("DescriptorAssembler.Descriptor");
  EXPECT_TRUE(descriptor->size() == descriptor2->size());
  for(size_t i = 0; i < descriptor->size(); ++i)
    EXPECT_FLOAT_EQ(descriptor->at(i), descriptor2->at(i));
}

TEST(Pipeline, OutputFlush)
{
  Pipeline pl(10);
  pl.loadYAML("example.yml");

  int num_points = 1e3;
  Vec::ConstPtr v0 = generateVec(num_points);
  Vec::ConstPtr v1 = generateVec(num_points);
  Vec::ConstPtr v2 = generateVec(num_points);
  pl.push("View0", v0);
  pl.push("View1", v1);
  pl.push("View2", v2);
  pl.compute();
  EXPECT_TRUE(pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor"));

  pl.pod("DescriptorAssembler")->disabled_ = true;
  pl.compute();

  // Running this test causes abort() because the outlet has no data.
  // EXPECT_TRUE(!pl.pull<void*>("DescriptorAssembler", "Descriptor")); 
}

TEST(Pipeline, Debugging)
{
  registerPods();
  Pipeline pl(10);
  generateDefaultPipeline(&pl);

  int num_points = 1e5;
  Vec::ConstPtr v0 = generateVec(num_points);
  Vec::ConstPtr v1 = generateVec(num_points);
  Vec::ConstPtr v2 = generateVec(num_points);
  
  pl.push("View0", v0);
  pl.push("View1", v1);
  pl.push("View2", v2);
  pl.compute();
  cout << pl.reportTiming() << endl;

  // Using more than one thread will result in mixing debugging output on the console.
  // It can also screw up visualization threads in e.g. OpenCV, if you're using them.
  pl.setNumThreads(1);
  pl.setDebug(true);
  pl.compute(); // Inputs remain from previous run.

  // The timing report only counts computation in compute(), so you can do slow things in
  // debug() without affecting anything but the "Start-to-finish wall time".
  cout << pl.reportTiming() << endl;
}

TEST(Pipeline, OptionalInputs)
{
  registerPods();
  
  // pull(input_name, &vec<T> data) should not abort even if zero outputs are registered to input_name.
  
  {
    Pipeline pl(1);
    pl.createPod("EntryPoint<Vec::ConstPtr>", "View");
    
    pl.createPod("Sorter", "Sorter");
    pl.connect("Sorter.Points <- View.Output");
    
    pl.createPod("Summarizer", "Summarizer");
    pl.connect("Summarizer.Points <- Sorter.Sorted");
    
    pl.createPod("HistogramGenerator", "HistogramGenerator");
    pl.connect("HistogramGenerator.Points <- Sorter.Sorted");
    pl.pod("HistogramGenerator")->setParam("BinWidth", 0.1);
    pl.pod("HistogramGenerator")->setParam("Min", -2.0);
    pl.pod("HistogramGenerator")->setParam("Max", 2.0);

    // Connect just to the Elements input.
    pl.createPod("DescriptorAssembler", "DescriptorAssembler");
    pl.connect("DescriptorAssembler.Elements <- Summarizer.Mean");
    pl.connect("DescriptorAssembler.Elements <- Summarizer.Stdev");
    pl.connect("DescriptorAssembler.Elements <- Summarizer.MeanNeighborSeparation");

    pl.push<Vec::ConstPtr>("View", generateVec(100));
    pl.compute();
    EXPECT_TRUE(pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor")->size() == 3);

    // Connect just to the SubVectors input.
    pl.deletePod("DescriptorAssembler");
    pl.createPod("DescriptorAssembler", "DescriptorAssembler");
    pl.connect("DescriptorAssembler.SubVectors <- HistogramGenerator.Histogram");

    pl.push<Vec::ConstPtr>("View", generateVec(100));
    pl.compute();
    EXPECT_TRUE(pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor")->size() == 40);
  }
}

TEST(Pod, ComputeOutsidePipeline)
{
  // Sometimes you want to access the functionality of a Pod
  // from outside of a Pipeline.  If this is the case, one solution
  // is to put the functionality into a static member function.
  Vec::Ptr points = generateVec(100);
  sort(points->begin(), points->end());

  double mean, stdev, mean_neighbor_separation;
  Summarizer::_compute(*points, &mean, &stdev, &mean_neighbor_separation);
  cout << "Got mean: " << mean << endl;
  
  // The above can be non-ideal, so using
  // a non-static member function instead might be preferable.
  HistogramGenerator hg("Aoeu");
  hg.setParam("BinWidth", 0.1);
  hg.setParam("Min", -2.0);
  hg.setParam("Max", 2.0);

  Vec::Ptr hist, lower_bounds;
  hg._compute(*points, &hist, &lower_bounds);
  cout << "Got hist with " << hist->size() << " bins." << endl;
  hg.debug();
}

TEST(Pipeline, Misc)
{
  Pipeline pl(1);
  pl.createPod("EntryPoint<Vec::ConstPtr>", "View0");
  EXPECT_TRUE(pl.pod("View0")->numRegisteredOutputs() == 0);
  EXPECT_TRUE(pl.pod("View0")->hasAllRequiredInputs());

  pl.createPod("Sorter", "Sorter0");
  EXPECT_TRUE(!pl.pod("Sorter0")->hasAllRequiredInputs());
  pl.connect("Sorter0.Points <- View0.Output");
  EXPECT_TRUE(pl.pod("View0")->numRegisteredOutputs() == 1);
  EXPECT_TRUE(pl.pod("Sorter0")->hasAllRequiredInputs());
  pl.createPod("Summarizer", "Summarizer0");
  pl.connect("Summarizer0.Points <- Sorter0.Sorted");

  pl.createPod("Sorter", "Sorter1");
  pl.connect("Sorter1.Points <- View0.Output");
  EXPECT_TRUE(pl.pod("View0")->numRegisteredOutputs() == 2);
  pl.createPod("Summarizer", "Summarizer1");
  pl.connect("Summarizer1.Points <- Sorter1.Sorted");

  pl.createPod("DescriptorAssembler", "DescriptorAssembler");
  pl.connect("DescriptorAssembler.Elements <- Summarizer0.Mean");
  pl.connect("DescriptorAssembler.Elements <- Summarizer0.Stdev");
  pl.connect("DescriptorAssembler.Elements <- Summarizer0.MeanNeighborSeparation");
  pl.connect("DescriptorAssembler.Elements <- Summarizer1.Mean");
  pl.connect("DescriptorAssembler.Elements <- Summarizer1.Stdev");
  pl.connect("DescriptorAssembler.Elements <- Summarizer1.MeanNeighborSeparation");

  EXPECT_TRUE(example::isRequired(pl.pod("DescriptorAssembler")));
  EXPECT_TRUE(example::isRequired(pl.pod("View0")));
  EXPECT_TRUE(!example::isRequired(pl.pod("Sorter0")));
  EXPECT_TRUE(!example::isRequired(pl.pod("Summarizer0")));
    
  pl.push<Vec::ConstPtr>("View0", generateVec(100));
  pl.compute();
  pl.writeGraphviz("original.gv");
  EXPECT_TRUE(pl.hasPod("Summarizer0"));
  EXPECT_TRUE(!pl.hasPod("aoeu"));
  
  for(size_t i = 0; i < pl.pods().size(); ++i) {
    cout << pl.pods()[i]->name() << endl;
    EXPECT_TRUE(pl.pods()[i]->hasAllRequiredInputs());
    assert(pl.pods()[i]->hasAllRequiredInputs());
  }
  
  EXPECT_TRUE(pl.pod("View0")->numRegisteredOutputs() == 2);
  EXPECT_TRUE(pl.pod("Summarizer0")->hasAllRequiredInputs());
  pl.deletePod("Sorter1");
  pl.writeGraphviz("after_deleting.gv");
  EXPECT_TRUE(!pl.hasPod("Sorter1"));
  EXPECT_TRUE(pl.hasPod("Summarizer1"));
  EXPECT_TRUE(pl.pod("View0")->numRegisteredOutputs() == 1);
  EXPECT_TRUE(!pl.pod("Summarizer1")->hasAllRequiredInputs());
  pl.prune(example::isRequired);
  pl.writeGraphviz("after_pruning.gv");
  EXPECT_TRUE(!pl.hasPod("Summarizer1"));
  
  // This should not crash.
  pl.push<Vec::ConstPtr>("View0", generateVec(100));
  pl.compute();
}
     
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
