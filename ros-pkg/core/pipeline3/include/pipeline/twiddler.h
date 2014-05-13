#ifndef TWIDDLER_H
#define TWIDDLER_H

#include <signal.h>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <ros/assert.h>
#include <ros/console.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <timer/timer.h>
#include <eigen_extensions/random.h>
#include <pipeline/pipeline.h>

namespace pl
{

#define REGISTER_ACTION(name)                          \
  registerAction(#name, boost::bind(&name, *this, _1));

  class ActionStats : public YAMLizable
  {
  public:
    ActionStats();

    std::string name_;
    double num_attempts_;
    double num_improments_;
    double total_improvement_;
    double total_minutes_;

    double impRate() const { return total_improvement_ / total_minutes_; }
    YAML::Node YAMLize() const;
    void deYAMLize(const YAML::Node& in);
  };
  
  //! Twiddles a YAML configuration and saves output.  This can be for simple cross validation
  //! or for complete pipeline learning.  For the latter, you probably want PipelineTwiddler
  //! as it is where commonly-used functions for pipeline learning are kept.
  //! 
  //! The output is saved in another YAML object
  //! so that you can save whatever you want.  For example, you might include "accuracy" and "timing"
  //! output, then optimize accuracy subject to timing constraints.
  //! See method comments for how exactly to do this.
  //! The objective function value is being minimized.
  class Twiddler
  {
  public:

    struct Entry
    {
      YAML::Node config_;
      YAML::Node results_;
      double objective_;
    };
    std::vector<Entry> ordering_;
    
    //! Configuration hash, configuration.
    std::map<uint64_t, YAML::Node> configs_;
    //! Configuration hash, results.
    std::map<uint64_t, YAML::Node> results_;
    size_t k_;
    
    Twiddler();
    virtual ~Twiddler() {}

    //! Creates a directory and seeds the results with the initial config.
    //! Can call twiddle after this.
    //! root_dir is where the output of each evaluation will be saved.
    //! It must not exist and will be created.
    void initialize(const YAML::Node& config, const std::string& root_dir);
    //! Loads the data from a previous run.
    void load(std::string root_dir);
    //! load or initialize must have been called.
    //! generateVariation, evaluate, save.
    void twiddle(double max_hours = 0);
    
    /************************************************************
     * Functions to be implemented by the user
     ************************************************************/

    //! Evaluates a configuration and returns YAML output.
    //! The configuration will be automatically saved in
    //! evalpath + "/config.txt" if you are using an external script.
    //! You can place additional custom output in evalpath if needed.
    virtual YAML::Node evaluate(const YAML::Node& config, std::string evalpath) = 0;
    //! By default, this returns the field called "Objective".
    //! You can overload this to handle constraints as well as different objectives.
    //! Lower is better.
    virtual double objective(const YAML::Node& results) const;
    //! This is called any time an improved configuration is found.
    virtual void improvementHook(const YAML::Node& config,
                                 const YAML::Node& results,
                                 std::string evalpath) const;
    //! Termination criteria.  Never terminates by default.
    virtual bool done(const YAML::Node& results) const;

    /************************************************************
     * Miscellaneous things.
     ************************************************************/

    void getBest(YAML::Node* best_params, YAML::Node* best_results) const;
    //! ordered_params->at(0) is the best set of params so far.  Ascending order.
    void getOrdering(std::vector<YAML::Node>* config,
                     std::vector<YAML::Node>* results,
                     std::vector<double>* objectives) const;
    
    /************************************************************
     * Common functions for generating config variations
     ************************************************************/

    void multiAction(YAML::Node config, int num_actions) const;
    //! Set config[id[0]][id[1]]... to a random element of vals.
    template<typename T> void twiddleParam(YAML::Node config, const std::string& id, const std::vector<T>& vals) const;
        
  protected:
    std::string root_dir_;
    std::string eval_dir_;
    std::string action_stats_dir_;
    std::string improvements_dir_;
    std::string best_eval_dir_;
    std::string hints_dir_;
    std::string hint_evals_dir_;
    boost::shared_ptr<std::ofstream> eval_log_;

    typedef boost::function<void (YAML::Node)> Action;
    std::vector<Action> actions_;
    std::vector<ActionStats> action_stats_;

    void insertEntry(YAML::Node variation, YAML::Node results);
    YAML::Node sampleInit() const;
    void initializeDirs(std::string root_dir);
    //! -1 indicates a hint rather than an action.
    //! Returns true if only duplicates were found.
    bool getNextConfig(const std::string& evalstring, YAML::Node* config, int* action_idx);
    std::vector<std::string> getHintPaths() const;
      
    //! See also REGISTER_ACTION.
    //! hallucinated_* is for setting the initial estimate
    //! of the improvement rate.
    void registerAction(const std::string& name, Action action,
                        double hallucinated_improvement = 1,
                        double hallucinated_minutes = 1);

    //! Given the best current configuration, generate a new configuration.
    //! You can implement whatever search strategy you want here.
    //! Twiddler will check whether your new config is a duplicate.
    //! YAML::Node acts like a shared_ptr, so you should edit config in place.
    //! If you want to make a deep copy, use YAML::Node copy = YAML::Clone(config);
    void exampleAction(YAML::Node config) const {}

    //! Gross.
    mutable std::tr1::mt19937 mersenne_;
    //! Returns index of the action that was taken.
    size_t generateVariation(YAML::Node config) const;
  };

  template<typename T> void Twiddler::twiddleParam(YAML::Node config,
                                                   const std::string& key,
                                                   const std::vector<T>& vals) const
  {
    config[key] = vals[rand() % vals.size()];
  }

  //! A Twiddler that includes functions that are commonly used for
  //! pipeline learning.  They aren't registered by default; you can
  //! pick and choose among which of those you think might be worthwhile
  //! for your particular situation.
  class PipelineTwiddler : public Twiddler
  {
  public:
    PipelineTwiddler();
    virtual ~PipelineTwiddler();
    
    void deleteRandomPod(YAML::Node config, GenericPodTest isImmune) const;


    template<typename PodType, typename ParamType>
    void twiddleRandomPodParam(YAML::Node config,
                               const std::string& param_name,
                               const std::vector<ParamType>& vals) const;

    template<typename PodType, typename ParamType>
    void twiddlePodParamsLockstep(YAML::Node config,
                                  const std::string& param_name,
                                  const std::vector<ParamType>& vals) const;
  };

  template<typename PodType, typename ParamType>
  void PipelineTwiddler::twiddleRandomPodParam(YAML::Node config,
                                               const std::string& param_name,
                                               const std::vector<ParamType>& vals) const
  {
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);
    Pod* pod = pl.randomPod<PodType>();
    if(pod)
      pod->setParam(param_name, vals[rand() % vals.size()]);
    config["Pipeline"] = pl.YAMLize();
  }

  template<typename PodType, typename ParamType>
  void PipelineTwiddler::twiddlePodParamsLockstep(YAML::Node config,
                                                  const std::string& param_name,
                                                  const std::vector<ParamType>& vals) const
  {
    Pipeline pl(1);
    pl.deYAMLize(config["Pipeline"]);
    std::vector<PodType*> pods = pl.filterPods<PodType>();
    ParamType val = vals[rand() % vals.size()];
    for(size_t i = 0; i < pods.size(); ++i)
      pods[i]->setParam(param_name, val);

    config["Pipeline"] = pl.YAMLize();
  }

}

#endif // TWIDDLER_H
