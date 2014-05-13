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
    return pod_->name() + separator() + name_;
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
      names.push_back(inputs[i]->pod()->name() + separator() + inputs[i]->name());
    return names;
  }

  std::vector<std::string> Pod::upstreamOutputNames() const
  {
    vector<string> names;
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    for(it = inputs_.begin(); it != inputs_.end(); ++it) {
      vector<string> n = upstreamOutputNames(it->first);
      names.insert(names.end(), n.begin(), n.end());
    }
    return names;
  }

  void Pod::registerPodTemplateType(std::string type_name, CreatorFnPtr fp)
  {
    // We don't want to assertValidName here because Pod class template
    // names are allowed to be anything.
    creator_map_[type_name] = fp;
  }
  
  void Pod::registerPodType(std::string type_name, CreatorFnPtr fp)
  {
    // Here we want to enforce CamelCase.
    assertValidName(type_name);
    creator_map_[type_name] = fp;
  }
  
  Pod* Pod::createPod(std::string type_name, std::string name)
  {
    map<string, CreatorFnPtr>::const_iterator it;
    it = creator_map_.find(type_name);
    if(it == creator_map_.end()) {
      PL_ABORT("Tried to create pod type \"" << type_name << "\", but it has not been registered.  See the REGISTER_POD macro.");
    }
    return it->second(name);
  }

  size_t Pod::numIncoming(std::string input_name) const
  {
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;
    it = inputs_.find(input_name);
    if(it == inputs_.end()) {
      PL_ABORT(getClassName() << " \"" << name_
               << "\" tried to check if \"" << input_name
               << "\" has data, but this input has not been registered.");
    }

    const std::vector<const Outlet*>& outlets = it->second;
    size_t num = 0;
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
    PL_ASSERT(done_computation_);
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
    PL_ASSERT(!on_queue_);

    // -- Flush all outlets.
    // Otherwise, if a Pod pushes output on run t,
    // but doesn't push anything on run t+1, the output from
    // run t will be used by child nodes.
    map<string, Outlet*>::iterator it;
    for(it = outlets_.begin(); it != outlets_.end(); ++it)
      it->second->flush();
  }
  
  void Pod::pl_compute() {
    PL_ASSERT(on_queue_);
    on_queue_ = false;

    PL_ASSERT(!started_computation_);
    if(done_computation_)
      cerr << name_ << " is done computation, but its pl_compute() function is being called!" << endl;
    PL_ASSERT(!done_computation_);
  
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
  
  std::vector<const Outlet*> Pod::inputPipes(const std::string& input_name) const
  {
    PL_ASSERT(inputs_.count(input_name));
    return inputs_.find(input_name)->second;
  }

  const Outlet* Pod::inputPipe(const std::string& input_name) const
  {
    PL_ASSERT(inputPipes(input_name).size() == 1);
    return inputPipes(input_name)[0];
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
      PL_ABORT(getClassName() << " \"" << name() << "\" tried to register undeclared input \"" << input_name << "\".");
    }

    if(!pod->hasOutput(output_name)) {
      PL_ABORT("Tried to connect " << name() << separator() << input_name << " <- " << pod->name() << separator() << output_name
               << ", but Pod \"" << pod->name() << "\" does not have an output named \"" << output_name << "\".");
    }
    
    const Outlet* outlet = pod->getOutlet(output_name);
    if(!outlet->checkType(declared_inputs_[input_name])) { 
      PL_ABORT(getClassName() << " \"" << name() << "\" tried to register \"" << input_name << " <- "
               << pod->name() << separator() << output_name << "\", but types do not match.");
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

  void Pod::unregisterInput(const std::string& input_name, Pod* pod, const std::string& output_name)
  {
    if(declared_inputs_.count(input_name) == 0) {
      PL_ABORT(getClassName() << " \"" << name() << "\" tried to unregister undeclared input \"" << input_name << "\".");
    }

    if(!pod->hasOutput(output_name)) {
      PL_ABORT("Tried to disconnect " << name() << separator() << input_name << " <- " << pod->name() << separator() << output_name
               << ", but Pod \"" << pod->name() << "\" does not have an output named \"" << output_name << "\".");
    }
    
    const Outlet* outlet = pod->getOutlet(output_name);
    if(!outlet->checkType(declared_inputs_[input_name])) { 
      PL_ABORT(getClassName() << " \"" << name() << "\" tried to unregister \"" << input_name << " <- "
               << pod->name() << separator() << output_name << "\", but types do not match.");
    }

    if(!inputs_.count(input_name)) {
      PL_ABORT(getClassName() << " \"" << name() << "\" tried to unregister input \"" << input_name << "\", but no registered inputs were found.");
    }

    // -- Delete the outlet from inputs_.
    vector<const Outlet*>& outlets = inputs_[input_name];
    vector<const Outlet*>::iterator it = find(outlets.begin(), outlets.end(), outlet);
    if(it == outlets.end()) {
      PL_ABORT(getClassName() << " \"" << name() << "\" tried to unregister \"" << input_name << " <- "
               << pod->name() << separator() << output_name << "\", but that connection was not found.");
    }
    outlets.erase(it);
    
    // -- If there are no more connections between this Pod and the parent Pod,
    //    remove the corresponding entries from parents_ and pod->children_.
    //    hasParent(name) only uses inputs_, which has been updated above at this point.
    if(!hasParent(pod->name())) {
      vector<Pod*>::iterator it0 = find(parents_.begin(), parents_.end(), pod);
      PL_ASSERT(it0 != parents_.end());
      parents_.erase(it0);

      vector<Pod*>::iterator it1 = find(pod->children_.begin(), pod->children_.end(), this);
      PL_ASSERT(it1 != pod->children_.end());
      pod->children_.erase(it1);
    }
  }

  const Outlet* Pod::getOutlet(std::string output_name) const
  {
    if(outlets_.count(output_name) != 1) {
      PL_ABORT("Attempted to get output (\"" << output_name << "\") on Pod \""
               << name() << "\", but no output with that name exists.");
    }

    return outlets_.find(output_name)->second; 
  }
  
  std::string Pod::getUniqueString(const std::string& output_name) const
  {
    ostringstream oss;
    oss << getUniqueString();
    oss << "Output " << output_name << endl;
    return oss.str();
  }
  
  uint64_t Pod::getUniqueHash(const std::string& output_name) const
  {
    return hashDjb2(getUniqueString(output_name).c_str());
  }

  std::string Pod::uniqueReadableId(const std::string& output_name) const
  {
    ostringstream oss;
    oss << name_ << separator() << output_name << ":" << getUniqueHash(output_name);
    return oss.str();
  }
  
  // Doesn't remove Pod names in inputs.
  // std::string removePodNames(const std::string& str)
  // {
  //   istringstream iss(str);
  //   string line;
  //   ostringstream oss;
  //   while(!iss.eof()) {
  //     getline(iss, line);
  //     if(line.find("Name:") == string::npos)
  //       oss << line << endl;
  //   }
  //   return oss.str();
  // }

  class Blah
  {
  public:
    double val;
  };
  
  std::string Pod::getUniqueString() const
  {
    vector<Pod*> upstream = getUpstreamPods();
    ostringstream oss;
    for(size_t i = 0; i < upstream.size(); ++i)
      oss << YAML::Dump(upstream[i]->YAMLize()) << endl;

    oss << YAML::Dump(YAMLize()) << endl;
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
    pod["Name"] = name();
    pod["Type"] = getClassName();

    // -- Inputs.
    map<string, std::vector<const Outlet*> >::const_iterator it;
    YAML::Node inputs;
    for(it = inputs_.begin(); it != inputs_.end(); ++it) {
      // -- Get the string describing all connections.
      const vector<const Outlet*>& outlets = it->second;
      ostringstream oss;
      for(size_t i = 0; i < outlets.size(); ++i) {
        oss << outlets[i]->pod()->name() << separator() << outlets[i]->name();
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
  
  bool Pod::hasOutput(const std::string& output_name) const
  {
    return outlets_.count(output_name);
  }
  
  bool Pod::hasAllRequiredInputs() const
  {
    map<string, boost::any>::const_iterator it;
    for(it = declared_inputs_.begin(); it != declared_inputs_.end(); ++it) {
      PL_ASSERT(multi_flags_.find(it->first) != multi_flags_.end());
      if(multi_flags_.find(it->first)->second)
        continue;
      if(inputs_.count(it->first) == 0 || inputs_.find(it->first)->second.empty())
        return false;  
    }
    
    return true;
  }

  std::vector<Pod*> Pod::downstream(const std::string& output_name) const
  {
    vector<Pod*> pods;
    string address = name() + separator() + output_name;
    for(size_t i = 0; i < children_.size(); ++i) {
      vector<string> addresses = children_[i]->upstreamOutputNames();
      for(size_t j = 0; j < addresses.size(); ++j)
        if(addresses[j] == address)
          pods.push_back(children_[i]);
    }
    return pods;
  }

  bool Pod::hasParent(const std::string& name) const
  {
    // omg we need c++11
    std::map<std::string, std::vector<const Outlet*> >::const_iterator it;  
    for(it = inputPipes().begin(); it != inputPipes().end(); ++it) {
      const std::vector<const Outlet*>& outlets = it->second;
      for(size_t i = 0; i < outlets.size(); ++i)
        if(outlets[i]->pod()->name() == name)
          return true;
    }
    return false;
  }
  
} // namespace pl
