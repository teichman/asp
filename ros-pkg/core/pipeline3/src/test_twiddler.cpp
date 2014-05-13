#include <gtest/gtest.h>
#include <pipeline/twiddler.h>
#include <pipeline/example_pods.h>
#include <pipeline/pipeline.h>
#include <boost/bind.hpp>

using namespace std;
using namespace pl;
using namespace pl::example;
namespace bfs = boost::filesystem;

#define K 1

class TestTwiddler : public Twiddler
{
public:
  TestTwiddler() : Twiddler()
  {
    REGISTER_ACTION(TestTwiddler::toggleType);
    REGISTER_ACTION(TestTwiddler::twiddleW0);
    REGISTER_ACTION(TestTwiddler::twiddleW2);
    REGISTER_ACTION(TestTwiddler::twiddleX);
    registerAction("TwiddleTypeGeneric", boost::bind(&Twiddler::twiddleParam<string>, *this, _1,
                                                     "Type", vector<string>{"constant", "quadratic"}));
  }
  
  YAML::Node evaluate(const YAML::Node& config, std::string evalpath)
  {
    YAML::Node results;
    double x = config["x"].as<double>();
    double objective;
    if(config["Type"].as<string>() == "constant")
      objective = uniform(2, 10);
    else if(config["Type"].as<string>() == "quadratic") {
      objective = config["w0"].as<double>();
      objective += config["w1"].as<double>() * x;
      objective += config["w2"].as<double>() * x * x;
    }
    else
      abort();

    results["Objective"] = objective;
    saveYAML(results, evalpath + "/some_extra_output.txt");
    return results;
  }

  bool done(const YAML::Node& results) const
  {
    return (results["Objective"].as<double>() < 0.01);
  }
    
  double uniform(double low, double high) const
  {
    return (double)rand() / RAND_MAX * (high - low) + low;
  }
  
  void toggleType(YAML::Node config) const
  {
    if(config["Type"].as<string>() == "constant")
      config["Type"] = "quadratic";
    else
      config["Type"] = "constant";
    
    // Have to twiddle Type & w2 simultaneously or you can get stuck.
    twiddleW2(config);
  }

  void twiddleX(YAML::Node config) const
  {
      config["x"] = uniform(-1, 1);
  }

  void twiddleW0(YAML::Node config) const
  {
    config["w0"] = uniform(0, 1);
  }

  void twiddleW2(YAML::Node config) const
  {
    config["w2"] = uniform(0.5, 1);
  }
};

TEST(Twiddler, Easy)
{
  TestTwiddler twiddler;
  twiddler.k_ = K;
  YAML::Node init;
  init["w0"] = 1;
  init["w1"] = 0;
  init["w2"] = 1;
  init["x"] = 1;
  init["Type"] = "constant";
  string root_dir = "twiddler_output_test";
  int retval = system(("rm -rf " + root_dir).c_str()); retval--;
  twiddler.initialize(init, root_dir);
  twiddler.twiddle();
}

TEST(Twiddler, Resume)
{
  TestTwiddler twiddler;
  twiddler.k_ = K;
  YAML::Node init;
  init["w0"] = 1;
  init["w1"] = 0;
  init["w2"] = 1;
  init["x"] = 1;
  init["Type"] = "constant";
  string root_dir = "twiddler_output_test";
  int retval = system(("rm -rf " + root_dir).c_str()); retval--;
  twiddler.initialize(init, root_dir);
  init["w1"] = 0.5;
  saveYAML(init, root_dir + "/hints/test_hint.yml");
  twiddler.twiddle(0.1 / 3600.0);
  cout << "Best so far: " << endl;
  YAML::Node config;
  YAML::Node results;
  twiddler.getBest(&config, &results);
  cout << YAML::Dump(results) << endl;
  cout << YAML::Dump(config) << endl;  
  
  twiddler.load(root_dir);
  init["w1"] = 0.25;
  saveYAML(init, root_dir + "/hints/test_hint.yml");
  twiddler.twiddle();
  EXPECT_TRUE(bfs::exists(root_dir + "/hint_evaluations/test_hint"));
  EXPECT_TRUE(bfs::exists(root_dir + "/hint_evaluations/test_hint-00"));
  cout << "Final results: " << endl;
  twiddler.getBest(&config, &results);
  cout << YAML::Dump(results) << endl;
  cout << YAML::Dump(config) << endl;  
}

TEST(Twiddler, Ordering)
{
  TestTwiddler twiddler;
  twiddler.k_ = K;
  string root_dir = "twiddler_output_test";
  twiddler.load(root_dir);
  vector<YAML::Node> config;
  vector<YAML::Node> results;
  vector<double> objectives;
  twiddler.getOrdering(&config, &results, &objectives);

  for(size_t i = 0; i < min<size_t>(7, config.size()); ++i) {
    cout << "====================" << endl;
    cout << "Objective: " << objectives[i] << endl;
    cout << "--------------------" << endl;
    cout << YAML::Dump(results[i]) << endl;
    cout << "--------------------" << endl;
    cout << YAML::Dump(config[i]) << endl;
    if(i > 0)
      EXPECT_TRUE(objectives[i] >= objectives[i-1]);
  }
}

class ExamplePipelineTwiddler : public PipelineTwiddler
{
public:
  double desired_num_descriptors_;
  
  ExamplePipelineTwiddler(double desired_num_descriptors) :
    PipelineTwiddler(),
    desired_num_descriptors_(desired_num_descriptors)
  {
    double init = 1;
    
    registerAction("deleteRandomPod",
                   boost::bind(&PipelineTwiddler::deleteRandomPod, *this, _1, example::isRequired),
                   init, init);
    registerAction("deleteRandomAndAddSummarizer",
                   boost::bind(&ExamplePipelineTwiddler::deleteRandomAndAddSummarizer, *this, _1, example::isRequired),
                   init, init);
    registerAction("backtrack",
                   boost::bind(&ExamplePipelineTwiddler::backtrack, *this, _1),
                   init, init);
    registerAction("appendSummarizer",
                   boost::bind(&ExamplePipelineTwiddler::appendSummarizer, *this, _1),
                   init, init);
    registerAction("twiddleHistogramParam",
                   boost::bind(&ExamplePipelineTwiddler::twiddleHistogramParam, *this, _1),
                   init, init);
    registerAction("addAggregatorBranch",
                   boost::bind(&ExamplePipelineTwiddler::addAggregatorBranch, *this, _1),
                   init, init);
    registerAction("twiddleHistogramBinWidths",
                   boost::bind(&PipelineTwiddler::twiddlePodParamsLockstep<HistogramGenerator, double>, *this, _1,
                               "BinWidth", vector<double>{0.13, 0.42}));
    registerAction("twiddleRandomHistogramBinWidth",
                   boost::bind(&PipelineTwiddler::twiddleRandomPodParam<HistogramGenerator, double>, *this, _1,
                               "BinWidth", vector<double>{0.13, 0.42}));

  }

  void deleteRandomAndAddSummarizer(YAML::Node config, GenericPodTest isImmune) const
  {
    ROS_ASSERT(config["Pipeline"]);
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);

    // -- Delete a random non-immune pod and prune anything that
    //    is useless now.
    vector<Pod*> pods;
    for(size_t i = 0; i < pl.pods().size(); ++i)
      if(!isImmune(pl.pods()[i]))
        pods.push_back(pl.pods()[i]);
    if(pods.empty())
      return;
    pl.deletePod(pods[rand() % pods.size()]->name());
    pl.prune(isImmune);

    // -- Add a new Summarizer pod.
    //    Make a random connection to a sorter or aggregator.
    Pod* pod = pl.createPod("Summarizer");

    //pl.connect(pod->name() + ".Points <- " + pl.randomOutput<Vec::ConstPtr>());
    // This doesn't work because you can attach to the DescriptorAssembler and make a loop.
    
    vector<Sorter*> sorters = pl.filterPods<Sorter>();
    vector<Aggregator*> aggregators = pl.filterPods<Aggregator>();
    vector<Pod*> sources;
    sources.insert(sources.end(), sorters.begin(), sorters.end());
    sources.insert(sources.end(), aggregators.begin(), aggregators.end());
    if(sources.empty())
      return;
    Pod* src = sources[rand() % sources.size()];
    pl.connect(pod->name() + ".Points <- " + src->name() + separator() + src->randomOutput<Vec::ConstPtr>());

    int num = rand() % 3;
    pl.connect("DescriptorAssembler.Elements <- " + pod->name() + ".Mean");
    if(num > 0)
      pl.connect("DescriptorAssembler.Elements <- " + pod->name() + ".Stdev");
    if(num > 1)
      pl.connect("DescriptorAssembler.Elements <- " + pod->name() + ".MeanNeighborSeparation");
    
    config["Pipeline"] = pl.YAMLize();
  }

  void addAggregatorBranch(YAML::Node config) const
  {
    ROS_ASSERT(config["Pipeline"]);
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);

    vector<Sorter*> sorters = pl.filterPods<Sorter>();
    if(sorters.empty())
      return;

    size_t num = (rand() % sorters.size()) + 1;
    random_shuffle(sorters.begin(), sorters.end());
    
    Pod* agg = pl.createPod("Aggregator");
    for(size_t i = 0; i < num; ++i)
      pl.connect(agg->name() + ".PointSets <- " + sorters[i]->name() + ".Sorted");

    if(rand() % 2) {
      Pod* hg = pl.createPod("HistogramGenerator");
      pl.connect(hg->name() + ".Points <- " + agg->name() + ".Aggregated");
      pl.pod(hg->name())->setParam("BinWidth", 0.5);
      pl.pod(hg->name())->setParam("Min", -2.0);
      pl.pod(hg->name())->setParam("Max", 2.0);

      pl.connect("DescriptorAssembler.SubVectors <- " + hg->name() + ".Histogram");
    }
    else {
      Pod* sum = pl.createPod("Summarizer");
      pl.connect(sum->name() + ".Points <- " + agg->name() + ".Aggregated");

      int num = rand() % 3;
      pl.connect("DescriptorAssembler.Elements <- " + sum->name() + ".Mean");
      if(num > 0)
        pl.connect("DescriptorAssembler.Elements <- " + sum->name() + ".Stdev");
      if(num > 1)
        pl.connect("DescriptorAssembler.Elements <- " + sum->name() + ".MeanNeighborSeparation");
    }

    config["Pipeline"] = pl.YAMLize();
  }

  void twiddleHistogramParam(YAML::Node config) const
  {
    ROS_ASSERT(config["Pipeline"]);
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);

    vector<HistogramGenerator*> hgs = pl.filterPods<HistogramGenerator>();
    if(!hgs.empty()) {
      HistogramGenerator* hg = hgs[rand() % hgs.size()];
      if(rand() % 2)
        hg->setParam("BinWidth", hg->param<double>("BinWidth") * 2);
      else
        hg->setParam("BinWidth", hg->param<double>("BinWidth") / 2);

      if(rand() % 2)
        hg->setParam("Min", hg->param<double>("Min") + hg->param<double>("BinWidth"));
      else
        hg->setParam("Min", hg->param<double>("Min") - hg->param<double>("BinWidth"));
      
      if(rand() % 2)
        hg->setParam("Max", hg->param<double>("Max") + hg->param<double>("BinWidth"));
      else                                                                 
        hg->setParam("Max", hg->param<double>("Max") - hg->param<double>("BinWidth"));

      if(hg->param<double>("Max") <= hg->param<double>("Min"))
        hg->setParam("Max", hg->param<double>("Min") + hg->param<double>("BinWidth") * 5);
    }

    config["Pipeline"] = pl.YAMLize();
  }
  
  void appendSummarizer(YAML::Node config) const
  {
    ROS_ASSERT(config["Pipeline"]);
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);

    vector<Sorter*> sorters = pl.filterPods<Sorter>();
    if(!sorters.empty()) {
      Pod* sum = pl.createPod("Summarizer");
      Sorter* sorter = sorters[rand() % sorters.size()];
      pl.connect(sum->name() + ".Points <- " + sorter->name() + ".Sorted");

      int num = rand() % 3;
      pl.connect("DescriptorAssembler.Elements <- " + sum->name() + ".Mean");
      if(num > 0)
        pl.connect("DescriptorAssembler.Elements <- " + sum->name() + ".Stdev");
      if(num > 1)
        pl.connect("DescriptorAssembler.Elements <- " + sum->name() + ".MeanNeighborSeparation");
    }

    config["Pipeline"] = pl.YAMLize();
  }

  void backtrack(YAML::Node config) const
  {
    if(configs_.empty())
      return;

    // -- Choose a previous configuration.
    vector<YAML::Node> configs;
    vector<YAML::Node> results;
    vector<double> objectives;
    getOrdering(&configs, &results, &objectives);
    config = YAML::Clone(configs[rand() % min(configs.size(), (size_t)10)]);

    // -- Twiddle around.
    int num = rand() % 3 + 1;
    for(int i = 0; i < num; ++i)
      generateVariation(config);
  }
  
  YAML::Node evaluate(const YAML::Node& config, std::string evalpath)
  {
    ROS_ASSERT(config["Pipeline"]);
    YAML::Node results;

    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);
    pl.writeGraphviz(evalpath + "/pipeline.gv");

    Vec::Ptr vec = generateVec(100);
    pl.push<Vec::ConstPtr>("View0", vec);
    pl.push<Vec::ConstPtr>("View1", vec);
    pl.push<Vec::ConstPtr>("View2", vec);
    pl.writeGraphviz("current.gv");
    pl.compute();
    Vec::ConstPtr descriptor = pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor");

    results["Objective"] = fabs((double)descriptor->size() - desired_num_descriptors_);
    return results;
  }

  bool done(const YAML::Node& results) const
  {
    return (results["Objective"].as<double>() < 0.01);
  }

  void improvementHook(const YAML::Node& config, const YAML::Node& results, std::string evalpath) const
  {
    ROS_ASSERT(config["Pipeline"]);
    Twiddler::improvementHook(config, results, evalpath);
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);
    pl.writeGraphviz(root_dir_ + "/best_pipeline.gv");
  }
};

TEST(Twiddler, PipelineTwiddling)
{
  srand(0);
  
  registerPods();
  
  Pipeline pl(1);
  generateDefaultPipeline(&pl);
  pl.writeGraphviz("twiddling_init.gv");

  // Twiddle until we get to an aggregate descriptor of length 13.
  ExamplePipelineTwiddler ept(13);
  ept.k_ = K;
  string root_dir = "example_pipeline_twiddling-013";
  int retval = system(("rm -rf " + root_dir).c_str()); retval--;
  YAML::Node init;
  init["Pipeline"] = pl.YAMLize();
  ROS_ASSERT(init["Pipeline"]);
  ept.initialize(init, root_dir);
  saveYAML(init, "example_pipeline_twiddling-013/hints/dupe_hint.yml");
  pl.pod("HistogramGenerator0")->setParam("BinWidth", (double)1);
  init["Pipeline"] = pl.YAMLize();
  saveYAML(init, "example_pipeline_twiddling-013/hints/test_hint.yml");
  ept.twiddle();
  EXPECT_TRUE(!bfs::exists("example_pipeline_twiddling-013/hint_evaluations/dupe_hint"));
  EXPECT_TRUE(bfs::exists("example_pipeline_twiddling-013/hint_evaluations/test_hint"));

  YAML::Node config, results;
  ept.getBest(&config, &results);
  cout << "============================================================" << endl;
  cout << "= Final conig: " << endl;
  cout << "============================================================" << endl;
  cout << YAML::Dump(config) << endl;
  cout << "============================================================" << endl;
  cout << "= Final results: " << endl;
  cout << "============================================================" << endl;
  cout << YAML::Dump(results) << endl;

  ROS_ASSERT(config["Pipeline"]);
  pl.deYAMLize(config["Pipeline"]);
  Vec::Ptr vec = generateVec(100);
  pl.push<Vec::ConstPtr>("View0", vec);
  pl.push<Vec::ConstPtr>("View1", vec);
  pl.push<Vec::ConstPtr>("View2", vec);
  pl.compute();
  Vec::ConstPtr descriptor = pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor");
  cout << "Final descriptor size: " << descriptor->size() << endl;
  EXPECT_TRUE(descriptor->size() == 13);

  // -- Try a few more times with other values.
  vector<double> targets;
  targets.push_back(42);
  targets.push_back(3);
  targets.push_back(100);
  for(size_t i = 0; i < targets.size(); ++i) { 
    ExamplePipelineTwiddler ept(targets[i]);
    ept.k_ = K;
    ostringstream oss;
    oss << "example_pipeline_twiddling-" << setw(3) << setfill('0') << targets[i];
    string root_dir = oss.str();
    int retval = system(("rm -rf " + root_dir).c_str()); retval--;
    Pipeline pl(1);
    generateDefaultPipeline(&pl);
    YAML::Node init;
    init["Pipeline"] = pl.YAMLize();
    ept.initialize(init, root_dir);
    ept.twiddle();

    YAML::Node config, results;
    ept.getBest(&config, &results);
    ROS_ASSERT(config["Pipeline"]);
    pl.deYAMLize(config["Pipeline"]);
    Vec::Ptr vec = generateVec(100);
    pl.push<Vec::ConstPtr>("View0", vec);
    pl.push<Vec::ConstPtr>("View1", vec);
    pl.push<Vec::ConstPtr>("View2", vec);
    pl.compute();
    Vec::ConstPtr descriptor = pl.pull<Vec::ConstPtr>("DescriptorAssembler", "Descriptor");
    EXPECT_EQ((size_t)targets[i], descriptor->size());
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

