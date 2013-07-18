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
    
    assertCompleteness();
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

  void Pipeline::setDebug(bool debug)
  {
    debug_ = debug;
    for(size_t i = 0; i < pods_.size(); ++i) {
      cout << "Setting debug to " << debug << " for pod \"" << pods_[i]->getName() << "\"" << endl;
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
  

  void Pipeline::assertNoDuplicates() const
  {
    // Check for duplicate pod pointers.
    set<Pod*> all;
    for(size_t i = 0; i < pods_.size(); ++i)
      all.insert(pods_[i]);
    if(all.size() != pods_.size())
      PL_ABORT("Duplicate Pod pointers found in Pipeline.");

    // Check for duplicated names.
    set<string> names;
    for(size_t i = 0; i < pods_.size(); ++i) {
      if(names.count(pods_[i]->getName()) != 0)
        PL_ABORT("Duplicate Pod name \"" << pods_[i]->getName() << "\".");
      names.insert(pods_[i]->getName());
    }
  }
  
  void Pipeline::addConnectedComponent(Pod* pod)
  {
    addPods(getComponent(pod));
    assertCompleteness();
  }

  void Pipeline::addPod(Pod* pod)
  {
    pods_.push_back(pod);
    pod_names_[pod->getName()] = pod;
    //assertCompleteness();
    assertNoDuplicates();
    PL_ASSERT(pods_.size() == pod_names_.size());
  }

  void Pipeline::addPods(const std::vector<Pod*>& pods)
  {
    for(size_t i = 0; i < pods.size(); ++i)
      addPod(pods[i]);
  }
  
  void Pipeline::connect(std::string source_pod, std::string source_output,
                         std::string sink_pod, std::string sink_input)
  {
    pod(sink_pod)->registerInput(sink_input, pod(source_pod), source_output);
    assertCompleteness();
  }
  
  void Pipeline::connect(std::string connection)
  {
    // TODO: Make this use explode()
    
    const string& c = connection;
    string source_pod = c.substr(0, c.find_first_of(separator()));
    //cout << "source_pod: " << source_pod << endl;
    string source_output = c.substr(c.find_first_of(separator()) + 1, c.find_first_of(" ") - c.find_first_of(separator()) - 1);
    //cout << "source_output: " << source_output << endl;
    string r = c.substr(c.find_first_of("-> ")).substr(4);
    //cout << "r: " << r << endl;
    string sink_pod = r.substr(0, r.find_first_of(separator()));
    //cout << "sink_pod: " << sink_pod << endl;
    string sink_input = r.substr(r.find_first_of(separator()) + 1);
    //cout << "sink_input: " << sink_input << endl;

    connect(source_pod, source_output, sink_pod, sink_input);
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
  
  void Pipeline::assertCompleteness() {
    queue<Pod*> to_check;
    for(size_t i = 0; i < pods_.size(); ++i) {
      if(pods_[i]->parents_.empty())
        to_check.push(pods_[i]);
    }

    set<Pod*> found;
    while(!to_check.empty()) {
      Pod* active = to_check.front();
      to_check.pop();
      found.insert(active);  // Won't insert duplicates.
      for(size_t i = 0; i < active->children_.size(); ++i) {
        to_check.push(active->children_[i]);
      }
    }

    assert(found.size() == pods_.size());  
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
          cout << "Node not in marked: " << pods_[i]->getName() << endl;

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
          << "\t" << path[i]->getName() << endl;
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
      names.push_back(pods_[i]->getName());
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
      oss << (uint64_t)pods_[i] << " [label=\"" << sanitize(pods_[i]->getName()) << "\"]" << endl;
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

  void Pipeline::deYAMLize(const YAML::Node& in)
  {
    // -- Clean up existing pipeline.
    for(size_t i = 0; i < pods_.size(); ++i)
      delete pods_[i];
    pods_.clear();
    pod_names_.clear();

    // -- Set up everything but the connections.
    //    This is unnecessarily complicated.
    vector<string> input_lines;
    vector<string> multi_input_lines;
    map<string, Pod*> pod_names; // Storage until we can get them hooked up.
    vector<Pod*> pods;
    for(size_t i = 0; i < in["Pods"].size(); ++i) {
      const YAML::Node& podyaml = in["Pods"][i];
      string name = podyaml["Name"].as<string>();
      string type = podyaml["Type"].as<string>();

      const YAML::Node& inputs = podyaml["Inputs"];
      for(YAML::const_iterator it = inputs.begin(); it != inputs.end(); ++it) {
        input_lines.push_back(name + " " + it->first.as<string>() + " <- " + it->second.as<string>());
      }

      Params params = podyaml["Params"].as<Params>();
      Pod* pod = Pod::createPod(type, name, params);

      PL_ASSERT(pod_names.count(pod->getName()) == 0);
      pod_names[pod->getName()] = pod;
      pods.push_back(pod);
    }

    // -- Connect everything.
    for(size_t i = 0; i < input_lines.size(); ++i) {
      istringstream iss(input_lines[i]);
      string consumer_pod_name;
      string input_name;
      string buf;
      iss >> consumer_pod_name;
      iss >> input_name;
      PL_ASSERT(pod_names.count(consumer_pod_name) == 1);
      iss >> buf; // <-

      vector<string> producer_pod_names;
      vector<string> output_names;
      while(!iss.eof()) {
        iss >> buf;
        producer_pod_names.push_back(buf.substr(0, buf.find(separator())));
        output_names.push_back(buf.substr(buf.find(separator()) + 1));
        PL_ASSERT(pod_names.count(producer_pod_names.back()) == 1);
      }
      PL_ASSERT(!producer_pod_names.empty());
      PL_ASSERT(producer_pod_names.size() == output_names.size());
      for(size_t j = 0; j < producer_pod_names.size(); ++j)
        pod_names[consumer_pod_name]->registerInput(input_name, pod_names[producer_pod_names[j]], output_names[j]);
    }

    addPods(pods);
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
  
} // namespace pl
