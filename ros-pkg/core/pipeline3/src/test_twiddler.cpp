#include <gtest/gtest.h>
#include <pipeline/twiddler.h>

using namespace std;
using namespace pl;

class TestTwiddler : public Twiddler
{
public:
  TestTwiddler() : Twiddler() {}
  
  Results evaluate(const Params& params, std::string evalpath)
  {
    Results results;
    double x = params.get<double>("x");
    double objective;
    if(params.get<string>("type") == "constant")
      objective = uniform(2, 10);
    else if(params.get<string>("type") == "quadratic") {
      objective = params.get<double>("w0");
      objective += params.get<double>("w1") * x;
      objective += params.get<double>("w2") * x * x;
    }
    else
      abort();

    results["objective"] = objective;
    results.save(evalpath + "/some_extra_output.txt");
    return results;
  }

  bool done(const Results& results) const
  {
    return (results["objective"] < 0.01);
  }
    
  double uniform(double low, double high) const
  {
    return (double)rand() / RAND_MAX * (high - low) + low;
  }
  
  Params generateParamVariation(Params params) const
  {
    double rand = uniform(0, 1);
    if(rand < 0.1) {
      if(params.get<string>("type") == "constant")
        params.set<string>("type", "quadratic");
      else
        params.set<string>("type", "constant");
      
      // Have to twiddle type & w2 simultaneously or you can get stuck.
      params.set("w2", uniform(0.5, 1));
    }

    else if(rand >= 0.1 && rand < 0.4)
      params.set("w0", uniform(0, 1));
    else if(rand >= 0.4 && rand < 0.7)
      params.set("x", uniform(-1, 1));
    else if(rand >= 0.7 && rand < 1.0)
      params.set("w2", uniform(0.5, 1));

    return params;
  }
  
};

TEST(Twiddler, Easy)
{
  TestTwiddler twiddler;
  Params init;
  init.set<double>("w0", 1);
  init.set<double>("w1", 0);
  init.set<double>("w2", 1);
  init.set<double>("x", 1);
  init.set<string>("type", "constant");
  string rootpath = "twiddler_output_test";
  int retval = system(("rm -rf " + rootpath).c_str()); retval--;
  twiddler.initialize(init, rootpath);
  twiddler.twiddle();
}

TEST(Twiddler, Resume)
{
  TestTwiddler twiddler;
  Params init;
  init.set<double>("w0", 1);
  init.set<double>("w1", 0);
  init.set<double>("w2", 1);
  init.set<double>("x", 1);
  init.set<string>("type", "constant");
  string rootpath = "twiddler_output_test";
  int retval = system(("rm -rf " + rootpath).c_str()); retval--;
  twiddler.initialize(init, rootpath);
  twiddler.twiddle(0.1 / 3600.0);
  cout << "Best so far: " << endl;
  Params params;
  Twiddler::Results results;
  twiddler.getBest(&params, &results);
  cout << results << endl;
  cout << params << endl;  
  
  twiddler.load(rootpath);
  twiddler.twiddle();
  cout << "Final results: " << endl;
  twiddler.getBest(&params, &results);
  cout << results << endl;
  cout << params << endl;  
}

TEST(Twiddler, Ordering)
{
  TestTwiddler twiddler;
  string rootpath = "twiddler_output_test";
  twiddler.load(rootpath);
  vector<Params> params;
  vector<Twiddler::Results> results;
  vector<double> objectives;
  twiddler.getOrdering(&params, &results, &objectives);

  for(size_t i = 0; i < min<size_t>(7, params.size()); ++i) {
    cout << "====================" << endl;
    cout << "Objective: " << objectives[i] << endl;
    cout << "--------------------" << endl;
    cout << results[i];
    cout << "--------------------" << endl;
    cout << params[i];
    if(i > 0)
      EXPECT_TRUE(objectives[i] >= objectives[i-1]);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

