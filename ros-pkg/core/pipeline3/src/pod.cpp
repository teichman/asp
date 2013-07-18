#include <pipeline/pod.h>

using namespace std;
namespace bfs = boost::filesystem;
using boost::any;

namespace pl
{

  map<string, Pod::CreatorFnPtr> Pod::creator_map_ = map<string, Pod::CreatorFnPtr>();
  map<string, string> Pod::template_map_ = map<string, string>();
  
  Outlet::Outlet(std::string name, Pod* pod) :
    has_data_(false),
    name_(name),
    pod_(pod)
  {
    PL_ASSERT(type_.empty());
    assertValidName(name);
  }

  std::string Outlet::address() const
  {
    return pod_->getName() + separator() + name_;
  }
  
  Pod::Pod(std::string name) :
    debug_(false),
    disabled_(false),
    name_(name),
    num_finished_parents_(0),
    done_computation_(false),
    started_computation_(false),
    on_queue_(false),
    mutex_(pthread_mutex_t()),
    time_msec_(-1),
    num_times_computed_(0)
  {
    assertValidName(name);
  }

  Pod::~Pod()
  {
    map<string, Outlet*>::iterator it;
    for(it = outlets_.begin(); it != outlets_.end(); ++it)
      delete it->second;
  }

  std::vector<std::string> Pod::upstreamOutputNames(const std::string& input_name) const
  {
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(input_name);
    if(it == inputs_.end()) {
      PL_ABORT(getClassName() << " \"" << name_
               << "\" tried to check if \"" << input_name
               << "\" has data, but this input has not been registered.");
    }

    const vector<const Outlet*>& inputs = it->second;
    vector<string> names;
    for(size_t i = 0; i < inputs.size(); ++i)
      names.push_back(inputs[i]->pod()->getName());
    return names;
  }
  
  void Pod::registerPodType(std::string type_name, CreatorFnPtr fp)
  {
    creator_map_[type_name] = fp;
  }
  
  Pod* Pod::createPod(std::string type_name, std::string name, Params params)
  {
    map<string, CreatorFnPtr>::const_iterator it;
    it = creator_map_.find(type_name);
    if(it == creator_map_.end()) {
      PL_ABORT("Tried to create pod type \"" << type_name << "\", but it has not been registered.  See the REGISTER_POD macro.");
    }
    return it->second(name, params);
  }

  int Pod::numIncoming(std::string input_name) const
  {
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(input_name);
    if(it == inputs_.end()) {
      PL_ABORT(getClassName() << " \"" << name_
               << "\" tried to check if \"" << input_name
               << "\" has data, but this input has not been registered.");
    }

    const std::vector<const Outlet*>& outlets = it->second;
    int num = 0;
    for(size_t i = 0; i < outlets.size(); ++i)
      if(outlets[i]->hasData())
        ++num;

    return num;
  }
  
  bool Pod::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void Pod::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void Pod::unlock() {
    pthread_mutex_unlock(&mutex_);
  }
  
  double Pod::getComputationTime() const {
    assert(done_computation_);
    return time_msec_;
  }

  int Pod::getNumTimesComputed() const
  {
    return num_times_computed_;
  }
  
  void Pod::flush()
  {
    done_computation_ = false;
    started_computation_ = false;
    time_msec_ = -1;
    num_finished_parents_ = 0;
    assert(!on_queue_);

    // -- Flush all outlets.
    // Otherwise, if a Pod pushes output on run t,
    // but doesn't push anything on run t+1, the output from
    // run t will be used by child nodes.
    map<string, Outlet*>::iterator it;
    for(it = outlets_.begin(); it != outlets_.end(); ++it)
      it->second->flush();
  }
  
  void Pod::pl_compute() {
    assert(on_queue_);
    on_queue_ = false;

    assert(!started_computation_);
    if(done_computation_)
      cerr << name_ << " is done computation, but its pl_compute() function is being called!" << endl;
    assert(!done_computation_);
  
    started_computation_ = true;
  
    timeval start, end;
    gettimeofday(&start, NULL);
    compute();
    gettimeofday(&end, NULL);
    done_computation_ = true;
    time_msec_ = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;

    if(debug_)
      debug();

    // -- Inform child pods of completion.
    for(size_t i = 0; i < children_.size(); ++i) {
      children_[i]->lock();
      ++children_[i]->num_finished_parents_;
      children_[i]->unlock();
    }

    ++num_times_computed_;
  }
  
  inline string Pod::runName(int width) const
  {
    ostringstream oss;
    oss << setw(width) << setfill('0') << num_times_computed_ << "-" << name_;
    return oss.str();
  }

  inline string Pod::debugDirectory() const
  {
    string name = ".pipeline-debug";
    if(!bfs::exists(name))
      bfs::create_directory(name);
    return name;
  }

  string Pod::debugBasePath() const
  {
    return debugDirectory() + "/" + runName(4);
  }
  
  bool Pod::ready() {
    if(disabled_ || on_queue_ || done_computation_ || started_computation_)
      return false;
    
    if((size_t)num_finished_parents_ == parents_.size())
      return true;
    else
      return false;
  }

  void Pod::registerInput(const std::string& input_name, Pod* pod, const std::string& output_name)
  {
    if(declared_inputs_.count(input_name) == 0) {
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to register undeclared input \"" << input_name << "\".");
    }

    if(!pod->hasOutput(output_name)) {
      PL_ABORT("Tried to connect " << getName() << separator() << input_name << " <- " << pod->getName() << separator() << output_name
               << ", but Pod \"" << pod->getName() << "\" does not have an output named \"" << output_name << "\".");
    }
    
    const Outlet* outlet = pod->getOutlet(output_name);
    if(!outlet->checkType(declared_inputs_[input_name])) { 
      PL_ABORT(getClassName() << " \"" << getName() << "\" tried to register \"" << input_name << " <- "
               << pod->getName() << separator() << output_name << "\", but types do not match.");
    }

    // -- Check multi status.
    PL_ASSERT(multi_flags_.count(input_name));
    bool multi = multi_flags_.find(input_name)->second;
    if(!multi && !inputs_[input_name].empty()) {
      PL_ABORT(getClassName() << " \"" << name_ << "\" tried to register single input \"" << input_name << "\" more than once.");
    }
    
    inputs_[input_name].push_back(outlet);
    parents_.push_back(pod);
    pod->children_.push_back(this);
  }
  
  const Outlet* Pod::getOutlet(std::string name) const
  {
    if(outlets_.count(name) != 1) {
      PL_ABORT("Attempted to get output (\"" << name << "\") on Pod \""
               << getName() << "\", but no output with that name exists.");
    }

    return outlets_.find(name)->second; 
  }
  
  std::string Pod::getUniqueString(const std::string& output_name) const
  {
    ostringstream oss;
    oss << getUniqueString();
    oss << "Output " << output_name << endl;
    return oss.str();
  }

  uint64_t hashDjb2(const char *str)
  {
    uint64_t hash = 5381;
    int c;

    // See http://www.cse.yorku.ca/~oz/hash.html.
    while((c = *str++))
      hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
  
    return hash;
  }
  
  uint64_t Pod::getUniqueHash(const std::string& output_name) const
  {
    return hashDjb2(getUniqueString(output_name).c_str());
  }
  
  std::string Pod::getUniqueString() const
  {
    map<string, string> nm;
    string working = getUnnormalizedUniqueString(&nm);

    // -- Replace pod names with normalized pod names.
    // map<string, string>::const_iterator it;
    // for(it = nm.begin(); it != nm.end(); ++it) { 
    //   boost::regex reg("^" + it->first + "$");
    //   working = boost::regex_replace(working, reg, it->second);
    //   boost::regex reg2("<\\- " + it->first + " ");
    //   working = boost::regex_replace(working, reg2, "<- " + it->second + " ");
    // }

    return working;
  }
  
  std::string Pod::getUnnormalizedUniqueString(std::map<std::string, std::string>* nm) const
  {
    vector<Pod*> upstream = getUpstreamPods();
    ostringstream oss;
    for(size_t i = 0; i < upstream.size(); ++i) {
      ostringstream podname_oss;
      podname_oss << "Pod" << i;
      (*nm)[upstream[i]->getName()] = podname_oss.str();

      oss << YAML::Dump(upstream[i]->YAMLize());
    }

    ostringstream podname_oss;
    podname_oss << "Pod" << nm->size();
    (*nm)[name_] = podname_oss.str();
    oss << YAML::Dump(YAMLize());
    
    return oss.str();
  }

  vector<Pod*> Pod::getUpstreamPods() const
  {
    // -- Get all upstream nodes.
    set<Pod*> upstream;
    for(size_t i = 0; i < parents_.size(); ++i) {
      upstream.insert(parents_[i]);
      vector<Pod*> tmp = parents_[i]->getUpstreamPods();
      for(size_t j = 0; j < tmp.size(); ++j)
        upstream.insert(tmp[j]);
    }
    
    // -- Put upstream nodes in order of unique hash.
    vector< pair<uint64_t, Pod*> > index;
    set<Pod*>::iterator it;
    for(it = upstream.begin(); it != upstream.end(); ++it) {
      Pod* pod = *it;
      index.push_back(pair<uint64_t, Pod*>(pod->getUniqueHash(), pod));
    }
    sort(index.begin(), index.end());

    vector<Pod*> sorted(index.size());
    for(size_t i = 0; i < sorted.size(); ++i)
      sorted[i] = index[i].second;

    return sorted;
  }
  
  uint64_t Pod::getUniqueHash() const
  {
    return hashDjb2(getUniqueString().c_str());
  }

  YAML::Node Pod::YAMLize() const
  {
    YAML::Node pod;
    pod["Name"] = getName();
    pod["Type"] = getClassName();

    // -- Inputs.
    map<string, std::vector<const Outlet*> >::const_iterator it;
    YAML::Node inputs;
    for(it = inputs_.begin(); it != inputs_.end(); ++it) {
      // -- Get the string describing all connections.
      const vector<const Outlet*>& outlets = it->second;
      ostringstream oss;
      for(size_t i = 0; i < outlets.size(); ++i) {
        oss << outlets[i]->pod()->getName() << separator() << outlets[i]->getName();
        if(i < outlets.size() - 1)
          oss << " ";
      }

      // -- Add the YAML Node.
      inputs[it->first] = oss.str();
    }
    pod["Inputs"] = inputs;  // If empty, we'll still have a placeholder in the YAML string.

    // -- Params.
    pod["Params"] = params_.YAMLize();

    return pod;
  }
  
  bool Pod::hasOutput(const std::string& name) const
  {
    return outlets_.count(name);
  }
  
} // namespace pl
