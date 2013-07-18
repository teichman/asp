#ifndef PIPELINE_COMMON_H
#define PIPELINE_COMMON_H

#include <iostream>

// ROS console and assert seem good, but they pollute the valgrind output.
// Maybe this will be resolved with a newer version of log4cxx:
// http://comments.gmane.org/gmane.comp.apache.logging.log4cxx.user/2991

//#include <ros/console.h>
//#define PL_ABORT(x) ROS_FATAL_STREAM(x); abort();

// PodName.OutletName
inline char separator() { return '.'; }
inline std::string separatorString() { return std::string(1, separator()); }

// ... for now we'll just use our own thing.
#define PL_ABORT(x)                                                \
  do {                                                                \
    std::cerr << "\033[1;31m[PIPELINE] " << x                        \
              << "\033[0m" << std::endl;                        \
    abort();                                                        \
  } while(0)

#define PL_ASSERT(cond)                                                \
  do {                                                                \
    if(!(cond)) {                                                \
      PL_ABORT("ASSERTION FAILED" << std::endl                        \
               << "\tfile = " << __FILE__ << std::endl                \
               << "\tline = " << __LINE__ << std::endl                \
               << "\tcond = " << #cond);                        \
    }                                                                \
  } while(0)


namespace pl
{

  inline bool isInvalidChar(char c)
  {
    // TODO: Are the underscore and colon ok?
    //return (!isalnum(c) && c != '_' && c != '<' && c != '>' && c != ':');
    return (!isalnum(c) && c != '_' && c != ':');  
  }

  inline bool isValidName(const std::string &name)
  {
    if(name.size() == 0)
      return false;
    
    return (find_if(name.begin(), name.end(), isInvalidChar) == name.end());
  }

  //! Asserts that this name won't screw something up in serialization.
  inline void assertValidName(const std::string& name)
  {
    if(!isValidName(name))
      PL_ABORT("Name \"" << name << "\" is invalid.  See isValidName().");
  }
  
} // namespace pl


/************************************************************
 * Introspection and serialization macros
 ************************************************************/

// If using non-templated Pod types, the following two
// macros are all you need to use.
#define DECLARE_POD(POD_TYPE)                                        \
  static Pod* create(std::string name, pl::Params params)        \
  {                                                                \
    POD_TYPE* pod = new POD_TYPE(name);                                \
    pod->setParams(params);                                        \
    return pod;                                                        \
  }                                                                \
  std::string getClassName() const { return #POD_TYPE; }

#define REGISTER_POD(POD_TYPE)                                        \
  pl::Pod::registerPodType(#POD_TYPE, &POD_TYPE::create);

// There is limited support for templated Pod types.
// They must have only one template type, named T.
// The type name must be alphanumeric with no spaces.
// You'll probably want to use a typedef.

// TODO: It'd be great to allow colons here, e.g. cv::Mat3b.
#define DECLARE_POD_TEMPLATE(POD_TYPE)                                  \
  static Pod* create(std::string name, pl::Params params)         \
  {                                                                     \
    POD_TYPE<T>* pod = new POD_TYPE<T>(name);                           \
    pod->setParams(params);                                             \
    return pod;                                                         \
  }                                                                     \
  std::string getClassName() const                                      \
  {                                                                     \
    std::map<std::string, std::string>::const_iterator it;              \
    it = pl::Pod::template_map_.find(typeid(T).name());           \
    if(it == pl::Pod::template_map_.end()) {                      \
      PL_ABORT("Attempted to get class name of Pod \"" << getName()     \
               << "\", but this template Pod is of unregistered template type" \
               << " (typeid \"" << typeid(T).name() << "\")."           \
               << " You probably need to call"                          \
               << " REGISTER_POD_TEMPLATE(ClassName, TemplateTypeName)."); \
    }                                                                   \
    return std::string(#POD_TYPE) + "<" + it->second + ">";             \
  }

#define REGISTER_POD_TEMPLATE(POD_TYPE, T)                                \
  do {                                                                        \
    if(!isValidName(#T)) {                                                 \
      PL_ABORT("Template name \"" << #T << "\" is invalid."                \
               << " Use only alphanumeric characters."                        \
               << " You probably want a typedef.");                        \
    }                                                                        \
    std::map<std::string, std::string>::const_iterator it;                \
    pl::Pod::template_map_[typeid(T).name()] = #T;                \
    pl::Pod::registerPodType(std::string(#POD_TYPE) + "<" + #T + ">", &POD_TYPE<T>::create); \
  } while(0)

#endif // PIPELINE_COMMON_H
