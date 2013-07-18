#include <pipeline/twiddler.h>
#include <boost/foreach.hpp>

using namespace std;
using boost::any;
namespace bfs = boost::filesystem;

namespace pl
{

  Twiddler::Twiddler()
  {
  }
  
  Params Twiddler::randomMerge(const Params& params0, const Params& params1)
  {
    Params params;
    map<string, any>::const_iterator it;
    for(it = params0.storage_.begin(); it != params0.storage_.end(); ++it) {
      string name = it->first;
      ROS_ASSERT(params1.exists(name));
      if(rand() % 2 == 0)
        params.storage_[name] = params0.storage_.find(name)->second;
      else
        params.storage_[name] = params1.storage_.find(name)->second;
    }
    return params;
  }

  double Twiddler::objective(const Results& results) const
  {
    if(!results.count("objective"))
      PL_ABORT("\"objective\" field of Results object not set.  You should either fill this field in Twiddler::evaluate or overload Twiddler::objective.");
    return results["objective"];
  }

  void Twiddler::initialize(const Params& init, const std::string& rootpath)
  {
    ROS_ASSERT(!bfs::exists(rootpath));
    ROS_ASSERT(results_.empty());
    bfs::create_directory(rootpath);
    rootpath_ = rootpath;

    // -- Run first evaluation on the initial set of params.
    ostringstream oss;
    oss << rootpath_ << "/" << setw(5) << setfill('0') << results_.size();
    string evalpath = oss.str();
    ROS_ASSERT(!bfs::exists(evalpath));
    bfs::create_directory(evalpath);
    init.save(evalpath + "/params.txt");
    results_[init] = evaluate(init, evalpath);
    improvementHook(init, results_[init], evalpath);
    results_[init].save(evalpath + "/results.txt");
    results_[init].save(rootpath_ + "/best_results.txt");
    init.save(rootpath_ + "/best_params.txt");

    // index_.push_back(pair(objective(results_[init]), init));
    // sort(index_.begin(), index_.end());
  }

  void Twiddler::load(std::string rootpath)
  {
    ROS_ASSERT(bfs::exists(rootpath));
    
    rootpath_ = rootpath;
    results_.clear();
    // todo: clear any other state here
    
    // -- Get all results paths.
    vector<string> paths;
    bfs::directory_iterator it(rootpath_), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      string path = rootpath_ + "/" + p.leaf();
      if(bfs::is_directory(path))
        paths.push_back(path);
    }
    sort(paths.begin(), paths.end());

    // -- Load results.
    for(size_t i = 0; i < paths.size(); ++i) {
      Results results;
      results.load(paths[i] + "/results.txt");
      Params params;
      params.load(paths[i] + "/params.txt");
      results_[params] = results;
      //index_.push_back(pair(objective(results), params));
    }
    //sort(index_.begin(), index_.end());
    ROS_ASSERT(results_.size() == paths.size());
    //ROS_ASSERT(results_.size() == index_.size());
  }

  void Twiddler::getBest(Params* best_params, Results* best_results) const
  {
    ROS_ASSERT(!results_.empty());
    double best_objective = numeric_limits<double>::max();
    map<Params, Results>::const_iterator it, best;
    for(it = results_.begin(); it != results_.end(); ++it) {
      if(objective(it->second) < best_objective) {
       best_objective = objective(it->second);
       best = it;
      }
    }
    *best_params = best->first;
    *best_results = best->second;
    
    // ROS_ASSERT(results_.size() = index_.size());
    // *best_params = index_[0]->second;
    // *best_results = results_[*best_params];
  }
  
  void Twiddler::twiddle(double max_hours)
  {
    ROS_ASSERT(!rootpath_.empty());
    ROS_ASSERT(!results_.empty());
    
    HighResTimer hrt;
    hrt.start();
    
    Params best_params;
    Results best_results;
    getBest(&best_params, &best_results);
    
    int num_duplicates = 0;
    while(true) {
      // -- Check if it's time to stop.
      if(max_hours > 0 && hrt.getHours() > max_hours) {
        cout << "Twiddler is out of time; finishing." << endl;
        break;
      }

      // -- Get another parameter variation.
      Params variation = generateParamVariation(best_params);
      if(results_.count(variation)) {
        ++num_duplicates;
        if(num_duplicates > 100)
          break;
        continue;
      }
      num_duplicates = 0;

      // -- Evaluate and check for improvement.
      ostringstream oss;
      oss << rootpath_ << "/" << setw(5) << setfill('0') << results_.size();
      string evalpath = oss.str();
      ROS_ASSERT(!bfs::exists(evalpath));
      bfs::create_directory(evalpath);
      variation.save(evalpath + "/params.txt");
      Results results = evaluate(variation, evalpath);
      results_[variation] = results;
      if(objective(results) < objective(best_results)) {
        best_params = variation;
        best_results = results;
        best_params.save(rootpath_ + "/best_params.txt");
        best_results.save(rootpath_ + "/best_results.txt");
        improvementHook(best_params, best_results, evalpath);
      }

      // TODO: add to ordering
      
      // -- Save results.
      results.save(evalpath + "/results.txt");

      if(done(results))
        break;
    }
  }

  void Twiddler::improvementHook(const Params& params, const Results& results, std::string evalpath) const
  {
    cout << "Twiddler found improvement!" << endl;
    cout << "New objective: " << objective(results) << endl;
    cout << "Results: " << endl;
    cout << results.status();
    cout << params << endl;
  }

  bool Twiddler::done(const Results& results) const
  {
    return false;
  }

  void Twiddler::getOrdering(std::vector<Params>* params, std::vector<Results>* results, std::vector<double>* objectives) const
  {
    params->clear();
    results->clear();
    objectives->clear();
    params->reserve(results_.size());
    results->reserve(results_.size());
    objectives->reserve(results_.size());

    vector< pair<double, Params> > index;
    index.reserve(results_.size());
    map<Params, Results>::const_iterator it;
    for(it = results_.begin(); it != results_.end(); ++it)
      index.push_back(pair<double, Params>(objective(it->second), it->first));
    sort(index.begin(), index.end());  // ascending

    for(size_t i = 0; i < index.size(); ++i) {
      params->push_back(index[i].second);
      ROS_ASSERT(results_.find(index[i].second) != results_.end());
      results->push_back(results_.find(index[i].second)->second);
      objectives->push_back(index[i].first);
    }
  }
  
}
