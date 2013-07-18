#ifndef POD_H
#define POD_H

#include <iostream>
#include <map>
#include <locale>
#include <list>
#include <sys/time.h>
#include <set>
#include <queue>
#include <stdint.h>
#include <fstream>
#include <iomanip>
#include <pthread.h>
#include <errno.h>
#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <bag_of_tricks/high_res_timer.h>
#include <pipeline/params.h>
#include <pipeline/outlet.h>

namespace pl
{
  
  //! Abstract base class that represents a node in the computation graph.
  //! All pods inherit from this class, at least indirectly.
  class Pod : public YAMLizable
  {  
  public:
    //! Whether to call debug() after compute().
    bool debug_;
    //! To compute, or not to compute?
    bool disabled_;

    // ----------------------------------------
    // -- Setup
    // ----------------------------------------
    Pod(std::string name);
    //! Connects the output of a Pod to this Pod's input.
    void registerInput(const std::string& input_name, Pod* pod, const std::string& output_name);
    template<typename T> void setParam(std::string name, const T& val);
    void setParams(const Params& params) { params_ = params; };
    
    // ----------------------------------------
    // -- Access
    // ----------------------------------------
    std::string getName() const { return name_; }
    //! Returns a string that uniquely identifies this Pod's output
    //! based on the upstream Pods, how they are connected, and their
    //! Params.
    std::string getUniqueString(const std::string& output) const;
    //! Returns a hash of getUniqueString(output).
    uint64_t getUniqueHash(const std::string& output) const;
    Params getParams() const { return params_; }
    //! Returns computation time in ms of the last run.
    double getComputationTime() const;
    int getNumTimesComputed() const;
    //! Returns a reasonable debug path for Pods to save output to.
    //! ".pipeline-debug/####-PodName"
    //! Creates the directory if necessary.
    //! Typically, you'll use this in an overridden debug() function when
    //! you want to save debugging output to, e.g. debugBasePath() + "-raw.png"
    //! and debugBasePath() + "-overlay.png".
    std::string debugBasePath() const;
    //! Returns the name of the directory in which to save debugging information.
    //! Creates this directory if necessary.
    std::string debugDirectory() const;
    //! Returns pod name with the number of times it has been run.
    std::string runName(int width = 4) const;
    
  protected:
    // ----------------------------------------
    // -- Methods to override with
    // -- your own functionality
    // ----------------------------------------
    //! Performs computation, given data from parent pods.
    //! This is the primary method that must be overloaded by a class that inherits from Pod.
    virtual void compute() = 0;
    //! Display function, called after compute() when debug_ == true.
    virtual void debug() const {}
    //! Called by Pipeline::reset().
    virtual void reset() {}

    // ----------------------------------------
    // -- Declarations to use
    // -- in your constructor
    // ----------------------------------------
    template<typename T> void declareParam(std::string name, T default_value);
    template<typename T> void declareParam(std::string name);
    template<typename T> void declareInput(std::string name);
    template<typename T> void declareMultiInput(std::string name);
    template<typename T> void declareOutput(std::string name);

    // ----------------------------------------
    // -- Input and output during compute()
    // ----------------------------------------
    //! Returns number of items you will get if you pull.
    int numIncoming(std::string input_name) const;
    //! Pulls from exactly one output connected to an input.
    template<typename T> void pull(std::string input_name, T* dest) const;
    //! Pulls from exactly one output connected to an input.
    template<typename T> T pull(std::string input_name) const;
    //! Pulls from any number of outputs connected to an input.
    template<typename T> void multiPull(std::string input_name, std::vector<T>* dest) const;
    //! Pushes data to an output.
    template<typename T> void push(std::string output_name, T val);
    //! Gets the value of a param.
    template<typename T> T param(const std::string& name) const;
    //! Returns vector of "Pod:Output" strings telling what is connected to
    //! input_name.
    //! TODO: There needs to be a good way to refer to the output from another pod
    //! that is being connected to the input to this pod. You can't just call it an
    //! output or input.  Incoming and outgoing would work, but incoming is used above
    //! to mean just those objects that you will receive rather than what pods are
    //! connected.
    std::vector<std::string> upstreamOutputNames(const std::string& input_name) const;
    
    // ----------------------------------------
    // -- Things you don't need to care about
    // ----------------------------------------
  public:
    typedef Pod* (*CreatorFnPtr)(std::string, Params);
    static std::map<std::string, CreatorFnPtr> creator_map_;
    static std::map<std::string, std::string> template_map_;
    static void registerPodType(std::string type_name, CreatorFnPtr fp);
    static void registerPodType(const Pod& pod);
    static Pod* createPod(std::string type_name, std::string name, Params params);
    virtual std::string getClassName() const = 0;
    virtual ~Pod();
    bool hasOutput(const std::string& name) const;

    YAML::Node YAMLize() const;
    void deYAMLize(const YAML::Node& in) { PL_ABORT("Pods cannot be deYAMLized in isolation."); }

  private:
    std::string name_;
    Params params_;
    //! Stores name and type.
    std::map<std::string, boost::any> declared_inputs_;
    //! Stores name and type.
    std::map<std::string, boost::any> declared_params_;
    std::map<std::string, std::vector<const Outlet*> > inputs_;
    std::map<std::string, bool> multi_flags_;
    std::map<std::string, Outlet*> outlets_;
    std::vector<Pod*> parents_;
    std::vector<Pod*> children_;
    int num_finished_parents_;
    bool done_computation_;
    bool started_computation_;
    bool on_queue_;
    pthread_mutex_t mutex_;
    //! The time it took to compute, in milliseconds.
    double time_msec_;
    int num_times_computed_;
    
    bool doneComputation() const {return done_computation_;}
    void flush();
    void pl_compute();
    bool ready();
    bool trylock();
    void lock();
    void unlock();
    std::string getUnnormalizedUniqueString(std::map<std::string, std::string>* nm) const;
    //! Returns a string that uniquely identifies this Pod
    //! based on the upstream Pods, how they are connected, and their
    //! Params.
    std::string getUniqueString() const;
    //! Returns a hash of getUniqueString().
    uint64_t getUniqueHash() const;
    //! Returns all pods that are upstream of this one.
    //! Return order is based on the unique hashes.
    std::vector<Pod*> getUpstreamPods() const;
    const Outlet* getOutlet(std::string name) const;
    
    friend class Pipeline;
    friend void* propagateComputation(void *pipeline);
    friend std::vector<Pod*> getComponent(Pod* pod);
  };


  /************************************************************
   * Outlet template definitions
   ************************************************************/

  template<typename T> T Outlet::pull() const
  {
    if(!has_data_) {
      PL_ABORT("Tried to pull from " << pod_->getName() << separator() << name_ << ", but Outlet does not have data!");
    }
    
    try { 
      return boost::any_cast<T>(data_);
    }
    catch(boost::bad_any_cast& e) {
      PL_ABORT("Type mismatch during pull from " << pod_->getName() << separator() << name_ << ".");
    }
    
    return boost::any_cast<T>(data_);
  }
  
  template<typename T> void Outlet::pull(T* dest) const
  {
    *dest = pull<T>();
  }

  template<typename T> void Outlet::push(T data)
  {
    try { 
      boost::any_cast<T>(type_);
    }
    catch(boost::bad_any_cast& e) {
      PL_ABORT("Type mismatch during push to " << pod_->getName() << separator() << name_ << "."
               << " You may need to explicitly specify the type, i.e. push<T>(...).");
    }
    data_ = data;
    has_data_ = true;
  }

  template<typename T> void Outlet::setType()
  {
    T tmp;
    type_ = tmp;
  }

  
  /************************************************************
   * Pod template definitions
   ************************************************************/
  
  template<typename T> T Pod::param(const std::string& name) const
  {
    // Force the writer of a pod to declare all his params.
    if(declared_params_.count(name) != 1)
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to get undeclared param \"" << name << "\". "
               << "Params should be declared in the Pod constructor using declareParam<type>(name).");
    
    if(!params_.exists(name))
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to get param \"" << name
               << "\", but this param has not been set.");
    
    return params_.get<T>(name);
  }

  template<typename T> void Pod::declareParam(std::string name, T default_value)
  {
    assertValidName(name);
    declared_params_[name] = default_value;
    setParam<T>(name, default_value);
  }
  
  template<typename T> void Pod::declareParam(std::string name)
  {
    assertValidName(name);
    T tmp;
    declared_params_[name] = tmp;
  }

  template<typename T> void Pod::declareOutput(std::string name)
  {
    assertValidName(name);
    PL_ASSERT(outlets_.count(name) == 0);
    outlets_[name] = new Outlet(name, this);
    outlets_[name]->setType<T>();
  }

  template<typename T> void Pod::declareInput(std::string name)
  {
    assertValidName(name);
    PL_ASSERT(declared_inputs_.count(name) == 0);
    T tmp;
    declared_inputs_[name] = tmp;
    multi_flags_[name] = false;
  }

  template<typename T> void Pod::declareMultiInput(std::string name)
  {
    assertValidName(name);
    PL_ASSERT(declared_inputs_.count(name) == 0);
    T tmp;
    declared_inputs_[name] = tmp;
    multi_flags_[name] = true;
  }

  template<typename T> void Pod::push(std::string name, T val)
  {
    if(outlets_.count(name) != 1) {
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to push to undeclared output \"" << name << "\".");
    }
    
    outlets_[name]->push<T>(val);
  }

  template<typename T> void Pod::pull(std::string name, T* dest) const
  {
    *dest = pull<T>(name);
  }

  template<typename T> T Pod::pull(std::string name) const
  {
    if(!declared_inputs_.count(name))
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to pull from undeclared input \"" << name << "\".");
    
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(name);
    if(it == inputs_.end()) {
      PL_ABORT(getClassName() << " \"" << name_
               << "\" tried to pull \"" << name
               << "\" but this input has not been registered.");
    }

    PL_ASSERT(multi_flags_.count(name));
    bool multi = multi_flags_.find(name)->second;
    if(multi) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to (single) pull from multi input \"" << name << "\".  Maybe you want multiPull()?");
    }

    const std::vector<const Outlet*>& outlets = it->second;
    PL_ASSERT(outlets.size() == 1);
    return outlets[0]->pull<T>();
  }
  
  template<typename T> void Pod::multiPull(std::string name, std::vector<T>* dest) const
  {
    PL_ASSERT(dest->empty());
    
    if(!declared_inputs_.count(name))
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to pull from undeclared input \"" << name << "\".");

    PL_ASSERT(multi_flags_.count(name));
    bool multi = multi_flags_.find(name)->second;
    if(!multi) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to multiPull from non-multi input \"" << name << "\".");
    }
    
    // --  If there are no registered inputs, just return.
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(name);
    if(it == inputs_.end())
      return;

    // -- Otherwise, fill dest.
    const std::vector<const Outlet*>& outlets = it->second;
    dest->reserve(outlets.size());
    for(size_t i = 0; i < outlets.size(); ++i)
      dest->push_back(outlets[i]->pull<T>()); // All Outlets must have data.
  }

  template<typename T> void Pod::setParam(std::string name, const T& val)
  {
    if(declared_params_.count(name) == 0)
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to set undeclared param \"" << name << "\".");

    try {
      boost::any_cast<T>(declared_params_[name]);
    }
    catch(boost::bad_any_cast& e) {
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to set param \"" << name << "\" to type that differs from declared type.");
    }
    
    params_.set(name, val);
  }
  
}

#endif // POD_H
