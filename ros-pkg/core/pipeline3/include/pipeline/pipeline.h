#ifndef PIPELINE_H
#define PIPELINE_H

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <pipeline/pod.h>
#include <pipeline/common_pods.h>

namespace pl
{

/*
 * Todo list
 *
 * - We need a name for Pod.Output.  "Address"?  "Port"?
 * - Pod:Output -> Pod:Input should have a name?  "Connection string"?
 * - It should be impossible to declare void debug() in a pod.  The right function is
 *   void debug() const, and if you use the former it just never gets called and you
 *   don't know why.
 */
 

  typedef bool (*GenericPodTest)(Pod* pod);
  template<typename T> bool isPodType(Pod* pod) { return pod->isPodType<T>(); }
  
  //! Class that represents the entire computation graph and manages its execution.
  //! Pods added to a Pipeline will be deleted by that Pipeline.
  class Pipeline : public YAMLizable {
  public:
    // ----------------------------------------
    // -- Setup
    // ----------------------------------------
    Pipeline(int num_threads);
    //! Creates a Pod and adds it to the Pipeline.
    Pod* createPod(const std::string& type, const std::string& name);
    //! Creates a Pod with an automatically-generated unique name and
    //! adds it to the Pipeline.
    Pod* createPod(const std::string& type);
    //! "Pod.Input <- Pod.Output"
    void connect(std::string connection);
    //! Opposite of connect.
    void disconnect(std::string connection);
    template<typename T> void setParam(std::string pod_name, std::string param_name, T val);
    //! Calls setData on an EntryPoint<T>.
    //! This is the normal way of pushing new data into a Pipeline.
    template<typename T> void push(const std::string& pod_name, T data);
    
    // ----------------------------------------
    // -- Execution
    // ----------------------------------------
    //! Runs all computation until completion.
    void compute();
    //! Calls reset() on all pods.
    //! What this actually does is Pod-specific, but typically it is something like
    //! clearing any data that is shared across many calls of compute().
    void reset();
    //! Convenient way of pulling output from the Pipeline.
    template<typename T> T pull(const std::string& pod_name, const std::string& output_name) const;
    //! Convenient way of pulling output from the Pipeline.
    //! "Pod.Output"
    template<typename T> T pull(const std::string& address) const;
    //! Convenient way of pulling output from the Pipeline.
    template<typename T> void pull(const std::string& pod_name,
                                   const std::string& output_name,
                                   T* dest) const;
    //! Convenient way of pulling output from the Pipeline.
    //! "Pod.Output"
    template<typename T> void pull(const std::string& address, T* dest) const;
    
    // ----------------------------------------
    // -- Control
    // ----------------------------------------
    //! Sets the debug_ flag on all Pods.
    //! If true, also creates .pipeline-debug so that Pods can easily save output there.
    void setDebug(bool debug);
    bool getDebug() const;
    void setNumThreads(int num_threads);
    //! Disables pod and all other pods connected to it.
    int disableComponent(Pod* pod);
    //! Enables pod and all other pods connected to it.
    int enableComponent(Pod* pod);
    void enableAll();
    void disableAll();
    
    // ----------------------------------------
    // -- Pod access
    // ----------------------------------------
    bool hasPod(const std::string& name) const;
    //! Returns true if at least one Pod of type T
    //! exists in the Pipeline.
    template<typename T> bool hasPod() const;
    //! Returns the Pod with the supplied name.
    Pod* pod(const std::string& name) const;
    //! Returns all Pods that pass the test.
    std::vector<Pod*> filterPods(GenericPodTest test) const;
    //! Returns the Pod of type T.  There must be exactly one.
    template<typename T> T* pod() const;
    //! Returns the Pod of type T with the supplied name.
    template<typename T> T* pod(const std::string& name) const;
    //! Return all pods of type T for which the given the test function evaluates to true.
    template<typename T> std::vector<T*> filterPods(bool (*test)(T* pod) = NULL) const;
    const std::vector<Pod*>& pods() const { return pods_; }

    // ----------------------------------------
    // -- Things specifically useful for Pipeline learning
    // ----------------------------------------
    
    //! Deletes the pod and removes all connections that involve it.
    void deletePod(const std::string& name);
    //! Deletes any Pod that either A) has no outputs or B) does not have all its
    //! inputs.  Continues doing this until there is nothing left to delete.
    //! Returns true if it deleted something.
    //! Test should return true for anything that is immune from pruning.
    //! For example, this might include required inputs and outputs.
    //! There is no default because at the minimum, your EntryPoint Pods
    //! and output pods should be immune.  See isPodType() for an example.
    void prune(GenericPodTest isImmune);
    template<typename T> std::string defaultPodName() const;
    std::string defaultPodName(const std::string type) const;
    //! Returns "Pod.Output" for a random output pipe of type T.
    template<typename T> std::string randomOutput() const;
    //! Returns a random Pod of type T, or NULL if there aren't any.
    template<typename T> T* randomPod() const;
    
    // ----------------------------------------
    // -- Reporting
    // ----------------------------------------
    std::string reportTiming() const;
    std::string reportSlowestPath() const;
    std::string getGraphviz() const;
    void writeGraphviz(const std::string& filename) const;
    
    // ----------------------------------------
    // -- Things you don't need to care about
    // ----------------------------------------
    
    virtual ~Pipeline();
    YAML::Node YAMLize() const;
    void deYAMLize(const YAML::Node& in);
      
  private:
    std::vector<Pod*> pods_;
    std::vector<pthread_t> threads_;
    //! Not necessarily equal to threads_.size(); see setDebug().
    int num_threads_;
    bool debug_;
    pthread_mutex_t mutex_;
    pthread_cond_t queue_cv_;
    pthread_cond_t done_cv_;

    std::vector<Pod*> queue_;
    bool done_computation_;
    size_t num_pods_computing_;
    bool destructing_;
    //! The amount of time it took from calling compute() to being done.
    double time_msec_;
    std::map<std::string, Pod*> pod_names_;
    
    //! No copy constructing of Pipelines.
    Pipeline(const Pipeline& other);
    //! No assigment of Pipelines.
    Pipeline& operator=(const Pipeline& p);
    Pod* getReadyPod();
    void registerCompleted(Pod* pod);
    bool trylock();
    void lock();
    void unlock();
    //! Wait on the queue condition variable.
    void wait();
    //! Signal the queue condition variable.
    void signal();
    void spawnThreadPool(int num_threads);
    void killThreadPool();
    bool computing();
    int switchComponent(Pod* pod, bool disabled);
    void run();
    
    friend void* propagateComputation(void *pipeline);
  };


  /****************************************
   * Helper Functions
   ****************************************/

  std::vector<Pod*> getComponent(Pod* pod);
  
  //! Returns pods of type T that pass a user-specified test.
  template<typename T>
  std::vector<T*>
  filterPods(const std::vector<Pod*>& pods, bool (*test)(T* pod) = NULL);

  //! Function that Pipeline worker threads call.
  void* propagateComputation(void *pipeline);

 
  /*****************************************
   * Function Templates
   ****************************************/

  template<typename T>
  inline void Pipeline::setParam(std::string pod_name, std::string param_name, T val)
  {
    pod(pod_name)->setParam(param_name, val);
  }

  template<typename T>
  bool Pipeline::hasPod() const
  {
    return !filterPods<T>().empty();
  }
  
  template<typename T>
  std::vector<T*> Pipeline::filterPods(bool (*test)(T* pod)) const
  {
    return pl::filterPods<T>(pods_, test); // Pipeline:: is required; otherwise g++ thinks that we're calling a member function which doesn't exist.
  }
  
  template<typename T>
  std::vector<T*> filterPods(const std::vector<Pod*>& pods, bool (*test)(T* pod))
  {
    std::vector<T*> passed;
    passed.reserve(pods.size());
    for(size_t i = 0; i < pods.size(); ++i) {
      T* casted = dynamic_cast<T*>(pods[i]);
      if(!casted)
        continue;
      if(test && !test(casted))
        continue;
    
      passed.push_back(casted);
    }
    
    return passed;
  }
  
  //! Returns the one pod of type T.  There must be only one.
  template<typename T>
  T* pod(const std::vector<Pod*>& pods)
  {
    std::vector<T*> passed;
    passed.reserve(pods.size());
    for(size_t i = 0; i < pods.size(); ++i) {
      T* casted = dynamic_cast<T*>(pods[i]);
      if(!casted)
        continue;
      passed.push_back(casted);
    }

    if(!(passed.size() == 1 || passed.size() == 0)) {
      PL_ABORT("Called pod<T>(), but multiple Pods of type T were found."
               << " You probably need to use pod<T>(name).  typeid(T).name():"
               << typeid(T).name());
    }
    if(passed.size() == 1)
      return passed[0];
    else {
      PL_ABORT("Called pod<T>(), but no Pods of type T were found.  typeid(T).name(): "
               << typeid(T).name());
    }
    return NULL;
  }
  
  template<typename T>
  T* Pipeline::pod() const
  {
    return pl::pod<T>(pods_);
  }

  template<typename T> T* pod(const std::string& name,
                              const std::vector<Pod*>& pods)
  {
    std::vector<T*> passed;
    for(size_t i = 0; i < pods.size(); ++i) {
      T* casted = dynamic_cast<T*>(pods[i]);
      if(!casted || casted->name().compare(name) != 0)
        continue;
      
      passed.push_back(casted);
    }

    // Something has gone horribly wrong if two pods have the same name.
    PL_ASSERT(passed.size() < 2); 
    if(passed.size() == 1)
      return passed[0];
    else {
      PL_ABORT("Called pod<T>(\"" << name << "\"), but no Pods of type T with this name were found." << std::endl
               << "Typeid of T is: " << typeid(T).name());
    }
    return NULL;
  }

  template<typename T> T* Pipeline::pod(const std::string& name) const
  {
    return pl::pod<T>(name, pods_);
  }

  template<typename T> T Pipeline::pull(const std::string& pod_name, const std::string& outlet_name) const
  {
    return pod(pod_name)->getOutlet(outlet_name)->pull<T>();
  }

  template<typename T> T Pipeline::pull(const std::string& address) const
  {
    std::vector<std::string> tokens;
    boost::split(tokens, address, boost::is_any_of(separatorString()));
    if(tokens.size() != 2) {
      PL_ABORT("Tried to pull output from pipeline address \"" << address
               << "\", which appears to be malformed.  It should be PodInstanceName.OutputName");
    }
    return pull<T>(tokens[0], tokens[1]);
  }

  template<typename T> void Pipeline::pull(const std::string& pod_name,
                                           const std::string& outlet_name,
                                           T* dest) const
  {
    *dest = pod(pod_name)->getOutlet(outlet_name)->pull<T>();
  }

  template<typename T> void Pipeline::pull(const std::string& address, T* dest) const
  {
    std::vector<std::string> tokens;
    boost::split(tokens, address, boost::is_any_of(separatorString()));
    PL_ASSERT(tokens.size() == 2);
    return pull<T>(tokens[0], tokens[1], dest);
  }

  template<typename T> void Pipeline::push(const std::string& pod_name, T data)
  {
    pod< EntryPoint<T> >(pod_name)->setData(data);
  }

  template<typename T> std::string Pipeline::defaultPodName() const
  {
    std::vector<T*> pods = filterPods<T>();
    std::ostringstream oss;
    T tmp("foo");
    oss << tmp.getClassName() << std::setw(2) << std::setfill('0') << pods.size();
    return oss.str();
  }

  template<typename T> std::string Pipeline::randomOutput() const
  {
    std::vector<std::string> addresses;
    for(size_t i = 0; i < pods_.size(); ++i) {
      std::vector<std::string> outputs = pods_[i]->outputs<T>();
      for(size_t j = 0; j < outputs.size(); ++j)
        addresses.push_back(pods_[i]->name() + separator() + outputs[j]);
    }

    return addresses[rand() % addresses.size()];
  }

  template<typename T> T* Pipeline::randomPod() const
  {
    std::vector<T*> pods = filterPods<T>();
    if(pods.empty())
      return NULL;
    else
      return pods[rand() % pods.size()];
  }
  
} // namespace pl

#endif // PIPELINE_H
