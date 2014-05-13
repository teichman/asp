#include <pipeline/pipeline.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;

namespace pl {

  Pipeline::Pipeline(int num_threads) :
    num_threads_(num_threads),
    debug_(false),
    done_computation_(false),
    num_pods_computing_(0),
    destructing_(false)
  {
    pthread_mutex_init(&mutex_, NULL);
    pthread_cond_init(&queue_cv_, NULL);
    pthread_cond_init(&done_cv_, NULL);
    
    spawnThreadPool(num_threads_);
  }

  Pipeline::~Pipeline() {
    killThreadPool();
    pthread_mutex_destroy(&mutex_);
    pthread_cond_destroy(&queue_cv_);
    pthread_cond_destroy(&done_cv_);

    for(size_t i = 0; i < pods_.size(); ++i)
      delete pods_[i];
  }

  Pod* Pipeline::createPod(const std::string& type, const std::string& name)
  {
    Pod* pod = Pod::createPod(type, name);
    pods_.push_back(pod);
    if(pod_names_.count(name) != 0) {
      PL_ABORT("Tried to create Pod with name \"" << name << "\", but a Pod with that name already exists.");
    }
    pod_names_[pod->name()] = pod;
    PL_ASSERT(pods_.size() == pod_names_.size());
    return pod;
  }

  std::string Pipeline::defaultPodName(const std::string type) const
  {
    string name;
    for(size_t i = 0; i < 100; ++i) { 
      ostringstream oss;
      oss << type << setw(2) << setfill('0') << i;
      name = oss.str();
      if(!hasPod(name))
        break;
    }
    return name;
  }
  
  Pod* Pipeline::createPod(const std::string& type)
  {
    string name = defaultPodName(type);
    return createPod(type, name);
  }
  
  void Pipeline::setDebug(bool debug)
  {
    debug_ = debug;
    for(size_t i = 0; i < pods_.size(); ++i) {
      //cout << "Setting debug to " << debug << " for pod \"" << pods_[i]->name() << "\"" << endl;
      pods_[i]->debug_ = debug;
    }

    if(debug && !bfs::exists(".pipeline-debug"))
      bfs::create_directory(".pipeline-debug");
  }

  bool Pipeline::getDebug() const
  {
    return debug_;
  }
  
  void Pipeline::setNumThreads(int num_threads)
  {
    killThreadPool();
    spawnThreadPool(num_threads);
  }

  void parseConnection(const std::string& connection,
                       std::string* sink_pod,
                       std::string* sink_input,
                       std::string* source_pod,
                       std::string* source_output)
  {
    const string& c = connection;
    *sink_pod = c.substr(0, c.find_first_of(separator()));
    *sink_input = c.substr(c.find_first_of(separator()) + 1, c.find_first_of(" ") - c.find_first_of(separator()) - 1);
    string r = c.substr(c.find_first_of("<- ")).substr(4);
    *source_pod = r.substr(0, r.find_first_of(separator()));
    *source_output = r.substr(r.find_first_of(separator()) + 1);
  }

  void Pipeline::connect(std::string connection)
  {
    string sink_pod, sink_input, source_pod, source_output;
    parseConnection(connection, &sink_pod, &sink_input, &source_pod, &source_output);
    pod(sink_pod)->registerInput(sink_input, pod(source_pod), source_output);
  }

  void Pipeline::disconnect(std::string connection)
  {
    string sink_pod, sink_input, source_pod, source_output;
    parseConnection(connection, &sink_pod, &sink_input, &source_pod, &source_output);
    pod(sink_pod)->unregisterInput(sink_input, pod(source_pod), source_output);
  }

  bool Pipeline::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void Pipeline::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void Pipeline::unlock() {
    pthread_mutex_unlock(&mutex_);
  }
    
  void Pipeline::reset()
  {
    for(size_t i = 0; i < pods_.size(); ++i)
      pods_[i]->reset();
  }

  string Pipeline::reportSlowestPath() const
  {
    // -- Check for duplicated node.
    set<Pod*> all;
    for(size_t i = 0; i < pods_.size(); ++i)
      all.insert(pods_[i]);
    PL_ASSERT(pods_.size() == all.size());
          
    map<Pod*, double> times;
    map<Pod*, Pod*> backptrs;

    // -- Initialize with entry points.
    queue<Pod*> to_check;
    set<Pod*> marked;
    for(size_t i = 0; i < pods_.size(); ++i) { 
      if(pods_[i]->parents_.empty()) { 
        to_check.push(pods_[i]);
        marked.insert(pods_[i]);
        backptrs[pods_[i]] = NULL;
        times[pods_[i]] = pods_[i]->getComputationTime();
        PL_ASSERT(times.count(pods_[i]) == 1);
      }
    }

    // -- Propagate max path times through the graph.
    while(!to_check.empty()) {
      // Check parents.
      Pod* active = to_check.front();
      to_check.pop();
      
      double max = -std::numeric_limits<double>::max();
      for(size_t i = 0; i < active->parents_.size(); ++i) {
        PL_ASSERT(marked.count(active->parents_[i]));
        PL_ASSERT(times.count(active->parents_[i]));
        double val = times[active->parents_[i]] + active->getComputationTime();
        if(val > max) { 
          max = val;
          times[active] = val;
          backptrs[active] = active->parents_[i];
        }
      }
      
      // Add children.
      for(size_t i = 0; i < active->children_.size(); ++i) {
        Pod* child = active->children_[i];
        if(marked.count(child)) {
          continue;
        }
        bool all_parents_done = true;
        for(size_t j = 0; j < child->parents_.size(); ++j) {
          if(!times.count(child->parents_[j])) { 
            all_parents_done = false;
            break;
          }
        }

        if(all_parents_done) { 
          to_check.push(child);
          PL_ASSERT(marked.count(child) == 0);
          marked.insert(child);
        }


      }
    }

    if(marked.size() != pods_.size()) { 
      for(size_t i = 0; i < pods_.size(); ++i)
        if(marked.count(pods_[i]) == 0)
          cout << "Node not in marked: " << pods_[i]->name() << endl;

      cout << "Error in pipeline2::reportSlowestPath.  marked.size() == "
           << marked.size() << ", pods_.size() == " << pods_.size() << endl;
      abort();
    }
    
    // -- Find the endpoint with longest time.
    double max = 0;
    Pod* active = NULL;
    map<Pod*, double>::iterator it;
    for(it = times.begin(); it != times.end(); ++it) {
      if(it->second > max) {
        max = it->second;
        active = it->first;
      }
    }

    // -- Trace the path back to the entry point.
    vector<Pod*> path;
    while(active) {
      path.push_back(active);
      active = backptrs[active];
    }

    ostringstream oss;
    oss << "Slowest path (" << max << " ms): " << endl;
    for(int i = (int)path.size() - 1; i >= 0; --i) {
      oss << "  " << fixed << setprecision(2) << setw(8) << times[path[i]]
          << "\t" << fixed << setprecision(2) << setw(8) << path[i]->getComputationTime()
          << "\t" << path[i]->name() << endl;
    }
    return oss.str();
  }
  
  string Pipeline::reportTiming() const
  {
    assert(done_computation_);
    
    ostringstream oss;
    oss << "============================================================" << endl;
    oss << "Pipeline timing report" << endl;
    oss << "Time (ms) \tPod name" << endl;
    oss << "------------------------------------------------------------" << endl;
    vector<string> names;
    vector<double> times;
    vector< pair<double, size_t> > index;
    double total_time = 0;
    for(size_t i = 0; i < pods_.size(); ++i) {
      names.push_back(pods_[i]->name());
      times.push_back(pods_[i]->getComputationTime());
      index.push_back(pair<double, size_t>(times.back(), i));
      total_time += times.back();
    }
    sort(index.begin(), index.end(), greater< pair<double, size_t> >());
    
    for(size_t i = 0; i < index.size(); ++i) {
      size_t idx = index[i].second;
      oss << times[idx] << "\t" << "\t" << names[idx] << endl;
    }
    oss << "Number of threads:\t\t" << threads_.size() << endl;
    oss << "Sum of pod compute times:\t" << total_time << " ms." << endl;
    if(debug_)
      oss << "Start-to-finish wall time (including debug() calls):\t" << time_msec_ << " ms." << endl;
    else
      oss << "Start-to-finish wall time:\t" << time_msec_ << " ms." << endl;
    oss << endl;
    oss << reportSlowestPath() << endl;
    oss << "============================================================" << endl;
    
    return oss.str();
  }

  string sanitize(const string& name) {
    string sani = name.substr(0, name.find_first_of("_"));
    return sani;
  }
  
  string Pipeline::getGraphviz() const {
    ostringstream oss;
    oss << "digraph {" << endl;
    oss << endl;

    for(size_t i = 0; i < pods_.size(); ++i) {
      oss << (uint64_t)pods_[i] << " [label=\"" << sanitize(pods_[i]->name()) << "\"]" << endl;
    }
    oss << endl;

    for(size_t i = 0; i < pods_.size(); ++i) {
      vector<Pod*>& children = pods_[i]->children_;
      for(size_t j = 0; j < children.size(); ++j) {
        oss << (uint64_t)pods_[i] << "->" << (uint64_t)children[j] << endl;
      }
    }
    oss << endl;
    
    oss << "}" << endl;
    return oss.str();
  }
  
  void Pipeline::writeGraphviz(const string& filename) const {
    ofstream file;
    file.open(filename.c_str());
    assert(file);
    file << getGraphviz() << endl;
    file.close();
  }

  int Pipeline::switchComponent(Pod* pod, bool disabled)
  {
    // -- Ensure one of this Pipeline's pods is the one being disabled.
    assert(pod);
    bool valid = false;
    for(size_t i = 0; i < pods_.size(); ++i) {
      if(pod == pods_[i])
        valid = true;
    }
    assert(valid);

    vector<Pod*> component = getComponent(pod);
    for(size_t i = 0; i < component.size(); ++i)
      component[i]->disabled_ = disabled;

    return component.size();
  }

  void Pipeline::enableAll()
  {
    for(size_t i = 0; i < pods_.size(); ++i)
      pods_[i]->disabled_ = false;
  }
  
  void Pipeline::disableAll()
  {
    for(size_t i = 0; i < pods_.size(); ++i)
      pods_[i]->disabled_ = true;
  }
  
  int Pipeline::disableComponent(Pod* pod)
  {
    return switchComponent(pod, true);
  }

  int Pipeline::enableComponent(Pod* pod)
  {
    return switchComponent(pod, false);
  }

  void Pipeline::spawnThreadPool(int num_threads) {
    assert(threads_.empty());
    threads_.resize(num_threads);
    for(size_t i=0; i<threads_.size(); ++i) {
      timeval start, end;
      gettimeofday(&start, NULL);
      pthread_create(&(threads_[i]), NULL, propagateComputation, (void*)this);
      gettimeofday(&end, NULL);
    }
  }

  void Pipeline::killThreadPool()
  {
    lock();
    destructing_ = true;
    pthread_cond_broadcast(&queue_cv_);
    unlock();
    
    for(size_t i = 0; i < threads_.size(); ++i)
      pthread_join(threads_[i], NULL);

    lock();
    threads_.clear();
    destructing_ = false;
    unlock();
  }

  void Pipeline::compute() {
    lock();
    HighResTimer hrt;
    hrt.start();
    assert(queue_.empty());
    assert(!threads_.empty());

    // -- Set up.
    num_pods_computing_ = 0;
    done_computation_ = false;
    for(size_t i = 0; i < pods_.size(); ++i) { 
      pods_[i]->flush();
      assert(!pods_[i]->done_computation_);
    }

    // -- Find all ready pods and put them into the queue.
    for(size_t i = 0; i < pods_.size(); ++i) {
      pods_[i]->lock();
      if(pods_[i]->ready()) {
        pods_[i]->on_queue_ = true;
        queue_.push_back(pods_[i]);
        pthread_cond_signal(&queue_cv_);
      }
      pods_[i]->unlock();
    }
    assert(!queue_.empty());

    // -- Wait for the worker pods to complete.
    pthread_cond_wait(&done_cv_, &mutex_);
    
    done_computation_ = true;
    hrt.stop();
    time_msec_ = hrt.getMilliseconds();
    unlock();
  }
  
  void Pipeline::registerCompleted(Pod* pod) { 
    for(size_t i = 0; i < pod->children_.size(); ++i) {
      pod->children_[i]->lock();
      if(pod->children_[i]->ready()) {
        // Debugging.  TODO: remove.
        for(size_t j = 0; j < queue_.size(); ++j)
          assert(queue_[j] != pod->children_[i]);
        
        pod->children_[i]->on_queue_ = true;
        queue_.push_back(pod->children_[i]);
        pthread_cond_signal(&queue_cv_);
      }
      pod->children_[i]->unlock();
    }
  }

  void Pipeline::run()
  {
    lock();
    while(true) {
      if(destructing_) { 
        unlock();
        break;
      }
            
      if(queue_.empty()) {
        if(num_pods_computing_ == 0)
          pthread_cond_signal(&done_cv_);
        
        pthread_cond_wait(&queue_cv_, &mutex_);
      }

      // pthread signal might awaken more than one thread.
      if(!queue_.empty()) {
        Pod* pod = queue_.back();
        queue_.pop_back();

        // Debugging. TODO: remove.
        assert(pod->on_queue_);
        for(size_t i = 0; i < queue_.size(); ++i)
          assert(queue_[i] != pod);

        ++num_pods_computing_;
        unlock();

        pod->lock();
        pod->pl_compute();
        pod->unlock();

        lock();
        registerCompleted(pod);
        --num_pods_computing_;
      }
    }
  }
  
  void* propagateComputation(void *pipeline)
  {
    Pipeline& pl = *((Pipeline*) pipeline);
    pl.run();
    return NULL;
  }

  vector<Pod*> getComponent(Pod* pod)
  {
    queue<Pod*> to_check;
    to_check.push(pod);

    set<Pod*> component;
    while(!to_check.empty()) {
      Pod* active = to_check.front();
      to_check.pop();
      component.insert(active); //Won't insert duplicates.
      for(size_t i = 0; i < active->children_.size(); ++i) {
        if(component.count(active->children_[i]) == 0)
          to_check.push(active->children_[i]);
      }
      for(size_t i = 0; i < active->parents_.size(); ++i) { 
        if(component.count(active->parents_[i]) == 0)
          to_check.push(active->parents_[i]);
      }
    }

    vector<Pod*> vec;
    vec.reserve(component.size());
    set<Pod*>::const_iterator it;
    for(it = component.begin(); it != component.end(); ++it) {
      vec.push_back(*it);
    }
    return vec;
  }

  vector<string> explode(const string& str)
  {
    vector<string> result;
    istringstream iss(str);
    while(!iss.eof()) {
      string buf;
      iss >> buf;
      if(buf != "")
        result.push_back(buf);
    }
    return result;
  }
    
  void Pipeline::deYAMLize(const YAML::Node& in)
  {
    // -- Clean up existing pipeline.
    for(size_t i = 0; i < pods_.size(); ++i) {
      PL_ASSERT(pods_[i]);
      delete pods_[i];
    }
    pods_.clear();
    pod_names_.clear();

    // -- Create pods.
    for(size_t i = 0; i < in["Pods"].size(); ++i) {
      const YAML::Node& podyaml = in["Pods"][i];
      string name = podyaml["Name"].as<string>();
      string type = podyaml["Type"].as<string>();
      createPod(type, name);
      pod(name)->setParams(podyaml["Params"].as<Params>());
    }

    // -- Connect everything.
    for(size_t i = 0; i < in["Pods"].size(); ++i) {
      const YAML::Node& podyaml = in["Pods"][i];
      string dst_name = podyaml["Name"].as<string>();

      const YAML::Node& inputs = podyaml["Inputs"];
      for(YAML::const_iterator it = inputs.begin(); it != inputs.end(); ++it) {
        string dst_input = it->first.as<string>();
        string sources = it->second.as<string>();
        vector<string> split = explode(sources);
        for(size_t i = 0; i < split.size(); ++i) {
          string src = split[i];
          string src_name = src.substr(0, src.find(separator()));
          string src_output = src.substr(src.find(separator()) + 1);
          connect(dst_name + separator() + dst_input + " <- " + src_name + separator() + src_output);
        }
      }
    }
  }

  YAML::Node Pipeline::YAMLize() const
  {
    YAML::Node doc;
    for(size_t i = 0; i < pods_.size(); ++i)
      doc["Pods"].push_back(pods_[i]->YAMLize());
    return doc;
  }
  
  Pod* Pipeline::pod(const std::string& name) const
  {
    PL_ASSERT(pod_names_.size() == pods_.size());
    map<string, Pod*>::const_iterator it;
    it = pod_names_.find(name);
    if(it == pod_names_.end()) {
      PL_ABORT("Called pod(\"" << name << "\"), but no Pod with that name exists.");
    }
    return it->second;
  }

  bool Pipeline::hasPod(const std::string& name) const
  {
    PL_ASSERT(pod_names_.size() == pods_.size());
    return (pod_names_.find(name) != pod_names_.end());
  }

  std::vector<Pod*> Pipeline::filterPods(GenericPodTest test) const
  {
    vector<Pod*> pods;
    pods.reserve(pods_.size());
    for(size_t i = 0; i < pods_.size(); ++i)
      if(test(pods_[i]))
        pods.push_back(pods_[i]);
    return pods;
  }

  bool isPod(Pod* pod0, Pod* pod1)
  {
    return (pod0 == pod1);
  }
  
  void Pipeline::deletePod(const std::string& name)
  {
    assert(queue_.empty());

    Pod* ptr = pod(name);

    // -- Delete records of this Pod in the Pipeline object.
    vector<Pod*>::iterator it = find(pods_.begin(), pods_.end(), ptr);
    pods_.erase(it);
    pod_names_.erase(pod_names_.find(name));

    // -- For every other Pod in the Pipeline,
    //    delete any evidence of that Pod.
    //    TODO: This could be asymptotically faster.
    for(size_t i = 0; i < pods_.size(); ++i) {
      Pod& pod = *pods_[i];

      // parents_ and children_
      it = remove_if(pod.parents_.begin(), pod.parents_.end(), boost::bind(isPod, _1, ptr));
      pod.parents_.erase(it, pod.parents_.end());
      it = remove_if(pod.children_.begin(), pod.children_.end(), boost::bind(isPod, _1, ptr));
      pod.children_.erase(it, pod.children_.end());
      
      for(size_t j = 0; j < pod.parents_.size(); ++j)
        PL_ASSERT(pod.parents_[j] != ptr);
      for(size_t j = 0; j < pod.children_.size(); ++j)
        PL_ASSERT(pod.children_[j] != ptr);

      // inputs_
      std::map<std::string, std::vector<const Outlet*> >::iterator iit;
      for(iit = pod.inputs_.begin(); iit != pod.inputs_.end(); ++iit) {
        vector<const Outlet*>& outlets = iit->second;
        for(int j = 0; j < (int)outlets.size(); ++j) {
          if(outlets[j]->pod() == ptr) {
            outlets.erase(outlets.begin() + j);
            --j;
          }
        }
      }
      for(iit = pod.inputs_.begin(); iit != pod.inputs_.end(); ++iit) {
        vector<const Outlet*>& outlets = iit->second;
        for(int j = 0; j < (int)outlets.size(); ++j)
          PL_ASSERT(outlets[j]->pod() != ptr);
      }
    }

    delete ptr;
  }
  
  void Pipeline::prune(GenericPodTest isImmune)
  {
    if(pods_.empty()) {
      PL_ABORT("No Pods left during a call to Pipeline::prune()."
               << "  Did you specify prune's Pod immunity function correctly?"
               << "  At the minimum, your input and output Pods should be immune.");
    }

    Pod* to_delete = NULL;
    
    // -- Find a Pod that has no outputs.
    //    or a Pod that does not have all required inputs.
    for(size_t i = 0; !to_delete && i < pods_.size(); ++i) {
      if(isImmune(pods_[i])) {
        //cout << "Ignoring " << pods_[i]->name() << " because it is immune." << endl;
        continue;
      }
      if(pods_[i]->numRegisteredOutputs() == 0 || !pods_[i]->hasAllRequiredInputs()) {
        to_delete = pods_[i];
        // cout << "Marking " << to_delete->name() << " for deletion." << endl;
        // cout << "  numRegisteredOutputs: " << to_delete->numRegisteredOutputs() << endl;
        // cout << "  hasAllRequiredInputs: " << to_delete->hasAllRequiredInputs() << endl;
        break;
      }
    }

    // -- If there is nothing to delete, then we're done.
    //    Otherwise, delete until there is nothing left to delete.
    if(!to_delete)
      return;
    else {
      //cout << "Deleting " << to_delete->name() << endl;
      deletePod(to_delete->name());
      prune(isImmune);
    }
  }
  
} // namespace pl
