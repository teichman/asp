#include <pipeline/twiddler.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

namespace pl
{

  Twiddler::Twiddler() :
    k_(1),
    eval_log_(new ofstream)
  {
  }

  void Twiddler::initializeDirs(std::string root_dir)
  {
    root_dir_ = root_dir;
    eval_dir_ = root_dir_ + "/evaluations";
    action_stats_dir_ = root_dir_ + "/action_stats";
    improvements_dir_ = root_dir_ + "/improvements";
    hints_dir_ = root_dir_ + "/hints";
    hint_evals_dir_ = root_dir_ + "/hint_evaluations";

    if(!bfs::exists(root_dir_)) {
      bfs::create_directory(root_dir_);
      bfs::create_directory(eval_dir_);
      bfs::create_directory(action_stats_dir_);
      bfs::create_directory(improvements_dir_);
      bfs::create_directory(hints_dir_);
      bfs::create_directory(hint_evals_dir_);
    }
    else {
      ROS_ASSERT(bfs::is_directory(root_dir_));
      ROS_ASSERT(bfs::is_directory(eval_dir_));
      ROS_ASSERT(bfs::is_directory(action_stats_dir_));
      ROS_ASSERT(bfs::is_directory(improvements_dir_));
      ROS_ASSERT(bfs::is_directory(hints_dir_));
      ROS_ASSERT(bfs::is_directory(hint_evals_dir_));
    }
  }
  
  double Twiddler::objective(const YAML::Node& results) const
  {
    if(!results["Objective"].IsDefined())
      PL_ABORT("\"Objective\" field does not exist.  You should either fill this field in Twiddler::evaluate or overload Twiddler::objective.  Note this is case sensitive.");
    
    return results["Objective"].as<double>();
  }

  void Twiddler::initialize(const YAML::Node& init, const std::string& root_dir)
  {
    ROS_ASSERT(!bfs::exists(root_dir));
    ROS_ASSERT(results_.empty() && configs_.empty());
    initializeDirs(root_dir);

    // -- Run first evaluation on the initial config.
    ostringstream oss;
    oss << setw(5) << setfill('0') << results_.size();
    string evalstring = oss.str();
    string evalpath = eval_dir_ + "/" + evalstring;
    best_eval_dir_ = evalstring;
    ROS_ASSERT(!bfs::exists(evalpath));
    bfs::create_directory(evalpath);
    saveYAML(init, evalpath + "/config.yml");
    results_[hash(init)] = evaluate(init, evalpath);
    configs_[hash(init)] = YAML::Clone(init);
    insertEntry(init, results_[hash(init)]);
    improvementHook(init, results_[hash(init)], evalpath);
    bfs::create_symlink("../evaluations/" + evalstring, improvements_dir_ + "/" + evalstring);
    saveYAML(results_[hash(init)], evalpath + "/results.yml");
    saveYAML(results_[hash(init)], root_dir_ + "/best_results.yml");
    saveYAML(init, root_dir_ + "/best_config.yml");
  }

  void Twiddler::load(std::string root_dir)
  {
    initializeDirs(root_dir);
    
    results_.clear();
    configs_.clear();
    ordering_.clear();
    
    // -- Get all results paths.
    vector<string> paths;
    bfs::directory_iterator it(eval_dir_), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      string path = eval_dir_ + "/" + p.filename().string();
      if(bfs::is_directory(path))
        paths.push_back(path);
    }
    sort(paths.begin(), paths.end());

    // -- Load results.
    double best_obj = numeric_limits<double>::max();
    for(size_t i = 0; i < paths.size(); ++i) {
      if(!bfs::exists(paths[i] + "/results.yml")) {
        PL_ABORT("Expected to find results at \"" + paths[i] + "/results.yml\".");
      }
      YAML::Node results = YAML::LoadFile(paths[i] + "/results.yml");
      YAML::Node config = YAML::LoadFile(paths[i] + "/config.yml");
      results_[hash(config)] = results;
      configs_[hash(config)] = config;
      insertEntry(config, results);
      if(objective(results) < best_obj) {
        best_obj = objective(results);
        best_eval_dir_ = paths[i].substr(paths[i].find_last_of("/") + 1);
      }
    }
    cout << "Loading: best_eval_dir_ is " << best_eval_dir_ << endl;
    ROS_ASSERT(results_.size() == paths.size());
    ROS_ASSERT(configs_.size() == paths.size());
    ROS_ASSERT(ordering_.size() == paths.size());

    // -- Load any cached action stats that exist.
    for(size_t i = 0; i < action_stats_.size(); ++i) {
      string path = root_dir_ + "/action_stats/" + action_stats_[i].name_ + ".yml";
      if(bfs::exists(path)) {
        action_stats_[i].loadYAML(path);
        cout << "Loaded cached ActionStats located at " << path << "." << endl;
      }
    }
  }

  void Twiddler::getBest(YAML::Node* best_config, YAML::Node* best_results) const
  {
    ROS_ASSERT(!results_.empty());
    double best_objective = numeric_limits<double>::max();
    map<uint64_t, YAML::Node>::const_iterator it, best;
    for(it = results_.begin(); it != results_.end(); ++it) {
      if(objective(it->second) < best_objective) {
       best_objective = objective(it->second);
       best = it;
      }
    }
    ROS_ASSERT(configs_.count(best->first));
    *best_config = YAML::Clone(configs_.find(best->first)->second);
    *best_results = YAML::Clone(best->second);
    
    // ROS_ASSERT(results_.size() = index_.size());
    // *best_config = index_[0]->second;
    // *best_results = results_[*best_config];
  }

  YAML::Node Twiddler::sampleInit() const
  {
    if(k_ == 1 || k_ == 0)
      return YAML::Clone(ordering_[0].config_);
    else
      return YAML::Clone(ordering_[rand() % min(k_, ordering_.size())].config_);
  }

  std::vector<std::string> Twiddler::getHintPaths() const
  {
    vector<string> paths;
    bfs::directory_iterator it(hints_dir_), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      if(p.extension() == ".yml") {
        string path = hints_dir_ + "/" + p.filename().string();
        paths.push_back(path);
      }
    }
    sort(paths.begin(), paths.end());

    return paths;
  }
  
  bool Twiddler::getNextConfig(const std::string& evalstring, YAML::Node* config, int* action_idx)
  {
    // -- Get all non-duplicated hint paths.
    vector<string> paths = getHintPaths();
    for(size_t i = 0; i < paths.size(); ++i) {
      cout << "Loading " << paths[i] << endl;
      YAML::Node config = YAML::LoadFile(paths[i]);
      if(results_.count(hash(config))) {
        cout << "[Twiddler] Removing duplicate hint at " << paths[0] << endl;
        bfs::remove(paths[i]);
      }
    }
    paths = getHintPaths();
    
    // -- If we have a hint, use it.
    if(!paths.empty()) {
      *config = YAML::LoadFile(paths[0]);
      cout << "Using hint config at " << paths[0] << endl;
      bfs::remove(paths[0]);
      string hint_filename = paths[0].substr(0, paths[0].size() - 4);
      string hint_name = hint_filename.substr(hint_filename.find_last_of("/") + 1);
      string hint_name_path = hint_evals_dir_ + "/" + hint_name;
      int idx = 0;
      while(bfs::exists(hint_name_path)) {
        ostringstream oss;
        oss << hint_name_path << "-" << setw(2) << setfill('0') << idx;
        hint_name_path = oss.str();
        ++idx;
      }
      cout << "[Twiddler] hint: creating symlink " << "../evaluations/" + evalstring << endl
           << hint_name_path << endl;
      bfs::create_symlink("../evaluations/" + evalstring, hint_name_path);

      *action_idx = -1;
      return false;
    }
    else {
      // -- Get another parameter variation, throwing away duplicates.
      int num_consecutive_duplicates = 0;
      while(true) {
        *config = sampleInit();
        *action_idx = generateVariation(*config);

        if(results_.count(hash(*config))) {
          ROS_ASSERT(YAML::Dump(configs_[hash(*config)]) == YAML::Dump(*config));
          cout << "[Twiddler] Duplicate found." << endl;
          //cout << YAML::Dump(*config) << endl;
          ++num_consecutive_duplicates;
          if(num_consecutive_duplicates > 999) {
            cout << "[Twiddler] num_consecutive_duplicates is " << num_consecutive_duplicates
                 << ".  Finishing.." << endl;
            return true;
          }
        }
        else
          break;
      }

      // -- Log what action is going to be run.
      ActionStats& astats = action_stats_[*action_idx];
      ++astats.num_attempts_;
      (*eval_log_) << "[Twiddler] Running action " << astats.name_ << endl;
      return false;
    }
  }
  
  void Twiddler::twiddle(double max_hours)
  {
    ROS_ASSERT(!root_dir_.empty());
    ROS_ASSERT(!results_.empty());
    
    HighResTimer hrt;
    hrt.start();
    
    YAML::Node best_config;
    YAML::Node best_results;
    getBest(&best_config, &best_results);
    
    while(true) {
      // -- Check if it's time to stop.
      if(max_hours > 0 && hrt.getHours() > max_hours) {
        cout << "Twiddler is out of time; finishing." << endl;
        break;
      }

      // -- Set up a directory for results to be stored in.
      ostringstream oss;
      oss << setw(5) << setfill('0') << results_.size();
      string evalstring = oss.str();
      string evalpath = eval_dir_ + "/" + evalstring;
      ROS_ASSERT(!bfs::exists(evalpath));
      bfs::create_directory(evalpath);
      bfs::create_symlink("../" + best_eval_dir_, evalpath + "/current_best");
      eval_log_->open((evalpath + "/log.txt").c_str());

      
      // -- Get the next configuration to test.
      //    This might be a user hint or a config changed using
      //    one of the actions.
      int action_idx;  // -1 indicates hint.
      YAML::Node variation;
      bool all_dupes = getNextConfig(evalstring, &variation, &action_idx);
      if(all_dupes) break;
      
      // -- Evaluate and check for improvement.
      //cout << YAML::Dump(variation) << endl;
      saveYAML(variation, evalpath + "/config.yml");

      HighResTimer hrt; hrt.start();
      YAML::Node results = evaluate(variation, evalpath);
      hrt.stop();
      if(action_idx >= 0)
        action_stats_[action_idx].total_minutes_ += hrt.getMinutes();

      configs_[hash(variation)] = YAML::Clone(variation);
      results_[hash(variation)] = YAML::Clone(results);
      if(objective(results) < objective(best_results)) {
        if(action_idx >= 0) {
          ++action_stats_[action_idx].num_improments_;
          action_stats_[action_idx].total_improvement_ += objective(best_results) - objective(results);
        }
        
        best_config = YAML::Clone(variation);
        best_results = YAML::Clone(results);
        saveYAML(best_config, root_dir_ + "/best_config.yml");
        saveYAML(best_results, root_dir_ + "/best_results.yml");
        bfs::create_symlink("../evaluations/" + evalstring, improvements_dir_ + "/" + evalstring);
        improvementHook(best_config, best_results, evalpath);
        best_eval_dir_ = evalstring;
      }

      // -- Add to ordering.
      insertEntry(variation, results);
      
      // -- Save results.
      saveYAML(results, evalpath + "/results.yml");
      if(action_idx >= 0) {
        if(!bfs::exists(root_dir_ + "/action_stats"))
          bfs::create_directory(root_dir_ + "/action_stats");
        string apath = root_dir_ + "/action_stats/" + action_stats_[action_idx].name_ + ".yml";
        action_stats_[action_idx].saveYAML(apath);
      }

      // Close the logfile for this evaluation.
      eval_log_->close();

      if(done(results))
        break;
    }
  }

  void Twiddler::insertEntry(YAML::Node variation, YAML::Node results)
  {
    Entry entry;
    entry.config_ = YAML::Clone(variation);
    entry.results_ = YAML::Clone(results);
    entry.objective_ = objective(results);

    size_t idx;
    for(idx = 0; idx < ordering_.size(); ++idx)
      if(ordering_[idx].objective_ > entry.objective_)
        break;
    ordering_.insert(ordering_.begin() + idx, entry);

    for(size_t i = 1; i < ordering_.size(); ++i)
      ROS_ASSERT(ordering_[i-1].objective_ <= ordering_[i].objective_);
  }
  
  void Twiddler::improvementHook(const YAML::Node& config, const YAML::Node& results, std::string evalpath) const
  {
    cout << "Twiddler found improvement!" << endl;
    cout << "New objective: " << objective(results) << endl;
    cout << "-- Results -- " << endl;
    cout << YAML::Dump(results) << endl;

    (*eval_log_) << "Twiddler found improvement!" << endl;
    (*eval_log_) << "New objective: " << objective(results) << endl;
    (*eval_log_) << "-- Results -- " << endl;
    (*eval_log_) << YAML::Dump(results) << endl;
  }

  bool Twiddler::done(const YAML::Node& results) const
  {
    return false;
  }

  void Twiddler::getOrdering(std::vector<YAML::Node>* config, std::vector<YAML::Node>* results, std::vector<double>* objectives) const
  {
    ScopedTimer st("getOrdering");
    
    config->clear();
    results->clear();
    objectives->clear();
    config->reserve(results_.size());
    results->reserve(results_.size());
    objectives->reserve(results_.size());

    vector< pair<double, YAML::Node> > index;
    index.reserve(results_.size());
    map<uint64_t, YAML::Node>::const_iterator it;
    for(it = results_.begin(); it != results_.end(); ++it)
      index.push_back(pair<double, YAML::Node>(objective(it->second), configs_.find(it->first)->second));
    sort(index.begin(), index.end());  // ascending

    for(size_t i = 0; i < index.size(); ++i) {
      ROS_ASSERT(i == 0 || index[i-1].first <= index[i].first);
      config->push_back(index[i].second);
      ROS_ASSERT(results_.find(hash(index[i].second)) != results_.end());
      results->push_back(results_.find(hash(index[i].second))->second);
      objectives->push_back(index[i].first);
    }
  }

  void Twiddler::registerAction(const std::string& name, Action action,
                                double hallucinated_improvement,
                                double hallucinated_minutes)
  {
    cout << "[Twiddler] Registering action \"" << name << "\"." << endl;
    actions_.push_back(action);

    ActionStats astats;
    astats.name_ = name;
    astats.total_improvement_ = hallucinated_improvement;
    astats.total_minutes_ = hallucinated_minutes;
    action_stats_.push_back(astats);
  }

  size_t Twiddler::generateVariation(YAML::Node config) const
  {
    ROS_ASSERT(!actions_.empty());
    ROS_ASSERT(actions_.size() == action_stats_.size());
    
    VectorXd action_weights(action_stats_.size());
    for(size_t i = 0; i < action_stats_.size(); ++i)
      action_weights(i) = action_stats_[i].impRate();
    //action_weights.setConstant(1);
    
    VectorXi indices(1);
    eigen_extensions::weightedSampleLowVariance(action_weights, &mersenne_, &indices);
    int idx = indices(0);
    actions_[idx](config);

    return idx;
  }

  void Twiddler::multiAction(YAML::Node config, int num_actions) const
  {
    for(int i = 0; i < num_actions; ++i)
      generateVariation(config);
  }
     
  
  /************************************************************
   * PipelineTwiddler
   ************************************************************/

  PipelineTwiddler::PipelineTwiddler() :
    Twiddler()
  {
  }

  PipelineTwiddler::~PipelineTwiddler()
  {
  }

  void PipelineTwiddler::deleteRandomPod(YAML::Node config, GenericPodTest isImmune) const
  {
    ROS_ASSERT(config["Pipeline"]);
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);
    ROS_ASSERT(!pl.pods().empty());
    
    Pod* pod = pl.pods()[rand() % pl.pods().size()];
    if(!isImmune(pod)) {
      pl.deletePod(pod->name());
      pl.prune(isImmune);
    }
    
    config["Pipeline"] = pl.YAMLize();
  }

  
  /************************************************************
   * ActionStats
   ************************************************************/
  
  ActionStats::ActionStats() :
    num_attempts_(0),
    num_improments_(0),
    total_improvement_(0),
    total_minutes_(0)
  {
  }

  YAML::Node ActionStats::YAMLize() const
  {
    YAML::Node node;
    node["Name"] = name_;
    node["NumAttempts"] = num_attempts_;
    node["NumImprovements"] = num_improments_;
    node["TotalImprovement"] = total_improvement_;
    node["TotalMinutes"] = total_minutes_;
    node["ImpRate"] = impRate();

    return node;
  }

  void ActionStats::deYAMLize(const YAML::Node& in)
  {
    name_ = in["Name"].as<string>();
    num_attempts_ = in["NumAttempts"].as<double>();
    num_improments_ = in["NumImprovements"].as<double>();;
    total_improvement_ = in["TotalImprovement"].as<double>();;
    total_minutes_ = in["TotalMinutes"].as<double>();;
  }

}
