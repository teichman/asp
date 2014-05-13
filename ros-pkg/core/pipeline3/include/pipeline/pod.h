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
#include <timer/timer.h>
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
    template<typename T> void setParam(std::string param_name, const T& val);
    void setParams(const Params& params) { params_ = params; };
    //! Gets the value of a param.
    template<typename T> T param(const std::string& param_name) const;
    
    // ----------------------------------------
    // -- Access
    // ----------------------------------------
    std::string name() const { return name_; }
    //! Returns a string that uniquely identifies this Pod
    //! based on the upstream Pods, how they are connected, and their
    //! Params.  Not meant to be human-readable.
    std::string getUniqueString() const;
    //! Returns a hash of getUniqueString().
    uint64_t getUniqueHash() const;
    //! Returns a string that uniquely identifies this Pod's output
    //! based on the upstream Pods, how they are connected, and their
    //! Params.  Not meant to be human-readable.
    std::string getUniqueString(const std::string& output_name) const;
    //! Returns a hash of getUniqueString(output_name).
    uint64_t getUniqueHash(const std::string& output_name) const;
    //! Returns InstanceName.OutputName:Hash.
    //! Unfortunately this is very slow at the moment.  Use wisely.
    std::string uniqueReadableId(const std::string& output_name) const;
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
    const std::map<std::string, std::vector<const Outlet*> >& inputPipes() const { return inputs_; }
    std::vector<const Outlet*> inputPipes(const std::string& input_name) const;
    //! There can be only one.
    const Outlet* inputPipe(const std::string& input_name) const;
    template<typename T> bool isPodType() const;
    //! Returns true if this Pod has at least one parent of type T.
    template<typename T> bool hasParent() const;
    //! Returns true if this Pod has a parent with the provided name.
    bool hasParent(const std::string& name) const;
    
  protected:
    // ----------------------------------------
    // -- Methods to override with
    // -- your own functionality
    // ----------------------------------------
    //! Performs computation, given data from parent pods.
    //! This is the primary method that must be overloaded by a class that inherits from Pod.
    virtual void compute() = 0;
    //! Display function, called after compute() when debug_ == true.
    //! This should maybe not be const at all.
    //!  * random number generators
    virtual void debug() const {}
    //! Called by Pipeline::reset().
    //! Typically this is used for clearing any data that is shared across
    //! many calls of Pipeline::compute().
    virtual void reset() {}

    // ----------------------------------------
    // -- Declarations to use
    // -- in your constructor
    // ----------------------------------------
    template<typename T> void declareParam(std::string param_name, T default_value);
    template<typename T> void declareParam(std::string param_name);
    template<typename T> void declareInput(std::string input_name);
    template<typename T> void declareMultiInput(std::string input_name);
    template<typename T> void declareOutput(std::string output_name);

    // ----------------------------------------
    // -- Input and output during compute()
    // ----------------------------------------
    //! Returns number of items you will get if you pull.
    size_t numIncoming(std::string input_name) const;
    //! Pulls from exactly one output connected to an input.
    template<typename T> void pull(std::string input_name, T* dest) const;
    //! Pulls from exactly one output connected to an input.
    template<typename T> T pull(std::string input_name) const;
    //! Pulls from any number of outputs connected to an input.
    template<typename T> void multiPull(std::string input_name, std::vector<T>* dest) const;
    //! Pushes data to an output.
    template<typename T> void push(std::string output_name, T val);
    //! Returns vector of "Pod.Output" strings telling what is connected to
    //! input_name.
    //! TODO: There needs to be a good way to refer to the output from another pod
    //! that is being connected to the input to this pod. You can't just call it an
    //! output or input.  Incoming and outgoing would work, but incoming is used above
    //! to mean just those objects that you will receive rather than what pods are
    //! connected.
    std::vector<std::string> upstreamOutputNames(const std::string& input_name) const;
    std::vector<std::string> upstreamOutputNames() const;
    
    // ----------------------------------------
    // -- Things you don't need to care about
    // ----------------------------------------
  public:
    typedef Pod* (*CreatorFnPtr)(std::string);
    static std::map<std::string, CreatorFnPtr> creator_map_;
    static std::map<std::string, std::string> template_map_;
    static void registerPodType(std::string type_name, CreatorFnPtr fp);
    static void registerPodTemplateType(std::string type_name, CreatorFnPtr fp);
    static Pod* createPod(std::string type_name, std::string name);
    virtual std::string getClassName() const = 0;
    virtual ~Pod();
    bool hasOutput(const std::string& output_name) const;
    //! Returns names of all outputs that produce a T.
    template<typename T> std::vector<std::string> outputs() const;
    //! Returns name of a random output that produces a T.
    template<typename T> std::string randomOutput() const;

    YAML::Node YAMLize() const;
    void deYAMLize(const YAML::Node& in) { PL_ABORT("Pods cannot be deYAMLized in isolation."); }
    size_t numRegisteredOutputs() const { return children_.size(); }
    bool hasAllRequiredInputs() const;
    std::vector<Pod*> downstream(const std::string& output_name) const;
    std::vector<Pod*> downstream() const { return children_; }
    
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
    
    //! Connects the output of a Pod to this Pod's input.
    void registerInput(const std::string& input_name, Pod* pod, const std::string& output_name);
    //! Does the opposite of registerInput.
    void unregisterInput(const std::string& input_name, Pod* pod, const std::string& output_name);
    bool doneComputation() const {return done_computation_;}
    void flush();
    void pl_compute();
    bool ready();
    bool trylock();
    void lock();
    void unlock();
    //! Returns *all* pods that are upstream of this one.
    //! This is not just the immediate parents of a Pod.
    //! Return order is based on the unique hashes.
    std::vector<Pod*> getUpstreamPods() const;
    const Outlet* getOutlet(std::string output_name) const;
    
    friend class Pipeline;
    friend void* propagateComputation(void *pipeline);
    friend std::vector<Pod*> getComponent(Pod* pod);
  };
  
  /************************************************************
   * Pod template definitions
   ************************************************************/
  
  template<typename T> T Pod::param(const std::string& param_name) const
  {
    // Force the writer of a pod to declare all his params.
    if(declared_params_.count(param_name) != 1)
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to get undeclared param \"" << param_name << "\". "
               << "Params should be declared in the Pod constructor using declareParam<type>(param_name).");
    
    if(!params_.exists(param_name))
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to get param \"" << param_name
               << "\", but this param has not been set.");
    
    return params_.get<T>(param_name);
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

  template<typename T> void Pod::push(std::string output_name, T val)
  {
    if(outlets_.count(output_name) != 1) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to push to undeclared output \"" << output_name << "\".");
    }
    
    outlets_[output_name]->push<T>(val);
  }

  template<typename T> std::vector<std::string> Pod::outputs() const
  {
    std::vector<std::string> output_names;
    output_names.reserve(outlets_.size());

    T tmp;
    boost::any any = tmp;
    std::map<std::string, Outlet*>::const_iterator it;
    for(it = outlets_.begin(); it != outlets_.end(); ++it)
      if(it->second->checkType(any))
        output_names.push_back(it->first);

    return output_names;
  }

  template<typename T> std::string Pod::randomOutput() const
  {
    std::vector<std::string> output_names = outputs<T>();
    if(output_names.empty()) {
      PL_ABORT("Called randomOutput<T>() on Pod \"" << name_
               << "\", but no outputs of type T were found.");
    }

    return output_names[rand() % output_names.size()];
  }

  template<typename T> void Pod::pull(std::string input_name, T* dest) const
  {
    *dest = pull<T>(input_name);
  }

  template<typename T> T Pod::pull(std::string input_name) const
  {
    if(!declared_inputs_.count(input_name))
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to pull from undeclared input \"" << input_name << "\".");
    
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(input_name);
    if(it == inputs_.end()) {
      PL_ABORT(getClassName() << " \"" << name_
               << "\" tried to pull \"" << input_name
               << "\" but this input has not been registered.");
    }

    PL_ASSERT(multi_flags_.count(input_name));
    bool multi = multi_flags_.find(input_name)->second;
    if(multi) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to (single) pull from multi input \"" << input_name << "\".  Maybe you want multiPull()?");
    }

    const std::vector<const Outlet*>& outlets = it->second;
    PL_ASSERT(outlets.size() == 1);
    return outlets[0]->pull<T>();
  }
  
  template<typename T> void Pod::multiPull(std::string input_name, std::vector<T>* dest) const
  {
    dest->clear();
    
    if(!declared_inputs_.count(input_name))
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to pull from undeclared input \"" << input_name << "\".");

    PL_ASSERT(multi_flags_.count(input_name));
    bool multi = multi_flags_.find(input_name)->second;
    if(!multi) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to multiPull from non-multi input \"" << input_name << "\".");
    }
    
    // --  If there are no registered inputs, just return.
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(input_name);
    if(it == inputs_.end())
      return;

    // -- Otherwise, fill dest.
    const std::vector<const Outlet*>& outlets = it->second;
    dest->reserve(outlets.size());
    for(size_t i = 0; i < outlets.size(); ++i)
      dest->push_back(outlets[i]->pull<T>()); // All Outlets must have data.
  }

  template<typename T> void Pod::setParam(std::string param_name, const T& val)
  {
    if(declared_params_.count(param_name) == 0)
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to set undeclared param \"" << param_name << "\".");

    try {
      boost::any_cast<T>(declared_params_[param_name]);
    }
    catch(boost::bad_any_cast& e) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to set param \"" << param_name << "\" to type that differs from declared type.");
    }
    
    params_.set(param_name, val);
  }

  template<typename T> bool Pod::hasParent() const
  {
    // omg we need c++11
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;  
    for(it = inputPipes().begin(); it != inputPipes().end(); ++it) {
      const std::vector<const Outlet*>& outlets = it->second;
      for(size_t i = 0; i < outlets.size(); ++i)
        if(outlets[i]->pod()->isPodType<T>())
          return true;
    }
    return false;
  }
  
  template<typename T> bool Pod::isPodType() const
  {
    return dynamic_cast<const T*>(this);
  }
  
  /************************************************************
   * Outlet template definitions
   ************************************************************/

  template<typename T> T Outlet::pull() const
  {
    if(!has_data_) {
      PL_ABORT("Tried to pull from " << pod_->name() << separator() << name_ << ", but Outlet does not have data!");
    }
    
    try { 
      return boost::any_cast<T>(data_);
    }
    catch(boost::bad_any_cast& e) {
      PL_ABORT("Type mismatch during pull from " << pod_->name() << separator() << name_ << ".");
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
      PL_ABORT("Type mismatch during push to " << pod_->name() << separator() << name_ << "."
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
  
}

#endif // POD_H
