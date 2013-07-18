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
  EntryPoint<Vec::ConstPtr>* ep0 = new EntryPoint<Vec::ConstPtr>("View0");
  EntryPoint<Vec::ConstPtr>* ep1 = new EntryPoint<Vec::ConstPtr>("View1");
  EntryPoint<Vec::ConstPtr>* ep2 = new EntryPoint<Vec::ConstPtr>("View2");

  Sorter* s0 = new Sorter("Sorter0");
  Sorter* s1 = new Sorter("Sorter1");
  Sorter* s2 = new Sorter("Sorter2");
  s0->registerInput("Points", ep0, "Output");
  s1->registerInput("Points", ep1, "Output");
  s2->registerInput("Points", ep2, "Output");

  Summarizer* summarizer0 = new Summarizer("Summarizer0");
  Summarizer* summarizer1 = new Summarizer("Summarizer1");
  Summarizer* summarizer2 = new Summarizer("Summarizer2");
  summarizer0->registerInput("Points", s0, "Sorted");
  summarizer1->registerInput("Points", s1, "Sorted");
  summarizer2->registerInput("Points", s2, "Sorted");

  Aggregator* aggregator = new Aggregator("Aggregator");
  aggregator->registerInput("PointSets", s0, "Sorted");
  aggregator->registerInput("PointSets", s1, "Sorted");
  aggregator->registerInput("PointSets", s2, "Sorted");

  Summarizer* summarizer3 = new Summarizer("Summarizer3");
  summarizer3->registerInput("Points", aggregator, "Aggregated");

  HistogramGenerator* h0 = new HistogramGenerator("Histogram0");
  HistogramGenerator* h1 = new HistogramGenerator("Histogram1");
  HistogramGenerator* h2 = new HistogramGenerator("Histogram2");
  HistogramGenerator* h3 = new HistogramGenerator("Histogram3");
  h0->registerInput("Points", s0, "Sorted");
  h0->setParam("BinWidth", 0.5);
  h0->setParam("Min", -2.0);
  h0->setParam("Max", 2.0);
  h1->registerInput("Points", s1, "Sorted");
  h1->setParam("BinWidth", 0.5);
  h1->setParam("Min", -2.0);
  h1->setParam("Max", 2.0);
  h2->registerInput("Points", s2, "Sorted");
  h2->setParam("BinWidth", 0.5);
  h2->setParam("Min", -2.0);
  h2->setParam("Max", 2.0);
  h3->registerInput("Points", aggregator, "Aggregated");
  h3->setParam("Normalize", true);
  h3->setParam("BinWidth", 0.25);
  h3->setParam("Min", 0.0);
  h3->setParam("Max", 3.0);
  
  DescriptorAssembler* da = new DescriptorAssembler("DescriptorAssembler");
  da->registerInput("SubVectors", h0, "Histogram");
  da->registerInput("SubVectors", h1, "Histogram");
  da->registerInput("SubVectors", h2, "Histogram");
  da->registerInput("SubVectors", h3, "Histogram");
  da->registerInput("Elements", summarizer0, "Mean");
  da->registerInput("Elements", summarizer0, "Stdev");
  da->registerInput("Elements", summarizer0, "MeanNeighborSeparation");
  da->registerInput("Elements", summarizer1, "Mean");
  da->registerInput("Elements", summarizer1, "Stdev");
  da->registerInput("Elements", summarizer1, "MeanNeighborSeparation");
  da->registerInput("Elements", summarizer2, "Mean");
  da->registerInput("Elements", summarizer2, "Stdev");
  da->registerInput("Elements", summarizer2, "MeanNeighborSeparation");
  da->registerInput("Elements", summarizer3, "Mean");
  da->registerInput("Elements", summarizer3, "Stdev");
  da->registerInput("Elements", summarizer3, "MeanNeighborSeparation");

  pl->addConnectedComponent(ep0);

  pl->addPod(new ConcretePodA("ConcretePodA"));
  pl->addPod(new ConcretePodB("ConcretePodB"));
  pl->pod("ConcretePodB")->setParam("Something", true);
  pl->connect("DescriptorAssembler.Descriptor -> ConcretePodA.Vals");
  pl->connect("DescriptorAssembler.Descriptor -> ConcretePodB.Vals");
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
  
  pl.setInput("View0", v0);
  pl.setInput("View1", v1);
  pl.setInput("View2", v2);
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

  pl2.setInput("View0", v0);
  pl2.setInput("View1", v1);
  pl2.setInput("View2", v2);
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
  // -- Changing the params of an upstream node should change the hash.
  uint64_t orig;
  {
    EntryPoint<Vec::ConstPtr>* ep0 = new EntryPoint<Vec::ConstPtr>("View0");
    Sorter* s0 = new Sorter("Sorter0");
    s0->registerInput("Points", ep0, "Output");
    Summarizer* summarizer0 = new Summarizer("Summarizer0");
    summarizer0->registerInput("Points", s0, "Sorted");
    HistogramGenerator* h0 = new HistogramGenerator("Histogram0");
    h0->registerInput("Points", s0, "Sorted");
    h0->setParam("BinWidth", 0.1);
    h0->setParam("Min", -2.0);
    h0->setParam("Max", 2.0);
    DescriptorAssembler* da = new DescriptorAssembler("DescriptorAssembler");
    da->registerInput("SubVectors", h0, "Histogram");
    da->registerInput("Elements", summarizer0, "Mean");
    da->registerInput("Elements", summarizer0, "Stdev");
    da->registerInput("Elements", summarizer0, "MeanNeighborSeparation");
    
    orig = da->getUniqueHash("Descriptor");
    
    h0->setParam("Normalize", true);
    cout << "Different: " << orig << " " << da->getUniqueHash("Descriptor") << endl;
    EXPECT_TRUE(orig != da->getUniqueHash("Descriptor"));
    h0->setParam("Normalize", false);
    cout << "Same: " << orig << " " << da->getUniqueHash("Descriptor") << endl;
    EXPECT_TRUE(orig == da->getUniqueHash("Descriptor"));
    h0->setParam("BinWidth", 0.05);
    cout << "Different: " << orig << " " << da->getUniqueHash("Descriptor") << endl;
    EXPECT_TRUE(orig != da->getUniqueHash("Descriptor"));
    h0->setParam("BinWidth", 0.1);
    cout << "Same: " << orig << " " << da->getUniqueHash("Descriptor") << endl;
    EXPECT_TRUE(orig == da->getUniqueHash("Descriptor"));

    // Deallocate Pods.
    // The Pipeline object is responsible for cleaning up all Pods on the heap.
    Pipeline pl(1);
    pl.addConnectedComponent(ep0);
  }

  // -- Changing the order of registration should change the hash.
  {
    EntryPoint<Vec::ConstPtr>* ep0 = new EntryPoint<Vec::ConstPtr>("View0");
    Sorter* s0 = new Sorter("Sorter0");
    s0->registerInput("Points", ep0, "Output");
    Summarizer* summarizer0 = new Summarizer("Summarizer0");
    summarizer0->registerInput("Points", s0, "Sorted");
    HistogramGenerator* h0 = new HistogramGenerator("Histogram0");
    h0->registerInput("Points", s0, "Sorted");
    h0->setParam("BinWidth", 0.1);
    h0->setParam("Min", -2.0);
    h0->setParam("Max", 2.0);
    DescriptorAssembler* da = new DescriptorAssembler("DescriptorAssembler");
    da->registerInput("SubVectors", h0, "Histogram");
    da->registerInput("Elements", summarizer0, "Stdev"); // This has been changed.
    da->registerInput("Elements", summarizer0, "Mean");
    da->registerInput("Elements", summarizer0, "MeanNeighborSeparation");
    
    cout << "Different: " << orig << " " << da->getUniqueHash("Descriptor") << endl;
    EXPECT_TRUE(orig != da->getUniqueHash("Descriptor"));

    Pipeline pl(1);
    pl.addConnectedComponent(ep0);
  }

  // -- Changing the structure should change the hash.
  {
    EntryPoint<Vec::ConstPtr>* ep0 = new EntryPoint<Vec::ConstPtr>("View0");
    Sorter* s0 = new Sorter("Sorter0");
    s0->registerInput("Points", ep0, "Output");
    Summarizer* summarizer0 = new Summarizer("Summarizer0");
    summarizer0->registerInput("Points", s0, "Sorted");
    Summarizer* summarizer1 = new Summarizer("Summarizer1");
    summarizer1->registerInput("Points", s0, "Sorted");
    HistogramGenerator* h0 = new HistogramGenerator("Histogram0");
    h0->registerInput("Points", s0, "Sorted");
    h0->setParam("BinWidth", 0.1);
    h0->setParam("Min", -2.0);
    h0->setParam("Max", 2.0);
    DescriptorAssembler* da = new DescriptorAssembler("DescriptorAssembler");
    da->registerInput("SubVectors", h0, "Histogram");
    da->registerInput("Elements", summarizer0, "Mean");
    da->registerInput("Elements", summarizer0, "Stdev");
    da->registerInput("Elements", summarizer0, "MeanNeighborSeparation");
    da->registerInput("Elements", summarizer1, "Mean");
    da->registerInput("Elements", summarizer1, "Stdev");
    da->registerInput("Elements", summarizer1, "MeanNeighborSeparation");

    cout << "Different: " << orig << " " << da->getUniqueHash("Descriptor") << endl;
    EXPECT_TRUE(orig != da->getUniqueHash("Descriptor"));

    Pipeline pl(1);
    pl.addConnectedComponent(ep0);
  }
}

TEST(Pipeline, MultiCompute)
{
  Pipeline pl(10);
  pl.loadYAML("example.pl");

  int num_points = 1e3;
  Vec::ConstPtr v0 = generateVec(num_points);
  Vec::ConstPtr v1 = generateVec(num_points);
  Vec::ConstPtr v2 = generateVec(num_points);
  pl.setInput("View0", v0);
  pl.setInput("View1", v1);
  pl.setInput("View2", v2);
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
  pl.loadYAML("example.pl");

  int num_points = 1e3;
  Vec::ConstPtr v0 = generateVec(num_points);
  Vec::ConstPtr v1 = generateVec(num_points);
  Vec::ConstPtr v2 = generateVec(num_points);
  pl.setInput("View0", v0);
  pl.setInput("View1", v1);
  pl.setInput("View2", v2);
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
  
  pl.setInput("View0", v0);
  pl.setInput("View1", v1);
  pl.setInput("View2", v2);
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
  // pull(input_name, &vec<T> data) should not abort even if zero outputs are registered to input_name.
  
  {
    EntryPoint<Vec::ConstPtr>* ep0 = new EntryPoint<Vec::ConstPtr>("View0");
    Sorter* s0 = new Sorter("Sorter0");
    s0->registerInput("Points", ep0, "Output");
    Summarizer* summarizer0 = new Summarizer("Summarizer0");
    summarizer0->registerInput("Points", s0, "Sorted");
    HistogramGenerator* h0 = new HistogramGenerator("Histogram0");
    h0->registerInput("Points", s0, "Sorted");
    h0->setParam("BinWidth", 0.1);
    h0->setParam("Min", -2.0);
    h0->setParam("Max", 2.0);
    DescriptorAssembler* da = new DescriptorAssembler("DescriptorAssembler");
    da->registerInput("Elements", summarizer0, "Stdev");
    da->registerInput("Elements", summarizer0, "Mean");
    da->registerInput("Elements", summarizer0, "MeanNeighborSeparation");

    Pipeline pl(1);
    pl.addConnectedComponent(ep0);
    pl.setInput<Vec::ConstPtr>("View0", generateVec(100));
    pl.compute();
    EXPECT_TRUE(pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor")->size() == 3);
  }

  {
    EntryPoint<Vec::ConstPtr>* ep0 = new EntryPoint<Vec::ConstPtr>("View0");
    Sorter* s0 = new Sorter("Sorter0");
    s0->registerInput("Points", ep0, "Output");
    Summarizer* summarizer0 = new Summarizer("Summarizer0");
    summarizer0->registerInput("Points", s0, "Sorted");
    HistogramGenerator* h0 = new HistogramGenerator("Histogram0");
    h0->registerInput("Points", s0, "Sorted");
    h0->setParam("BinWidth", 0.1);
    h0->setParam("Min", -2.0);
    h0->setParam("Max", 2.0);
    DescriptorAssembler* da = new DescriptorAssembler("DescriptorAssembler");
    da->registerInput("SubVectors", h0, "Histogram");

    Pipeline pl(1);
    pl.addConnectedComponent(ep0);
    pl.setInput<Vec::ConstPtr>("View0", generateVec(100));
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
  HistogramGenerator hg("aoeu");
  hg.setParam("BinWidth", 0.1);
  hg.setParam("Min", -2.0);
  hg.setParam("Max", 2.0);

  Vec::Ptr hist, lower_bounds;
  hg._compute(*points, &hist, &lower_bounds);
  cout << "Got hist with " << hist->size() << " bins." << endl;
  hg.debug();
}

// TEST(Pod, BadLoad)
// {
//   Pipeline pl(1);
//   pl.load("lib/libpipeline3.so");
// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
