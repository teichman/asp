#ifndef COMMON_PODS_H
#define COMMON_PODS_H

#include <pipeline/pod.h>

namespace pl
{

  template<typename T>
  class EntryPoint : public Pod
  {
  public:
    DECLARE_POD_TEMPLATE(EntryPoint);
    EntryPoint(std::string name) :
      Pod(name),
      has_data_(false)
    {
      declareOutput<T>("Output");
    }
    void setData(T data) { data_ = data; has_data_ = true; }
    void compute()
    {
      // if(!has_data_)
      //         PL_ABORT(getClassName() << " \"" << getName() << "\" was not given data before start of Pipeline computation.  See Pipeline::setInput.");

      // It's probably better to allow an EntryPoint to not get input data and let everything else proceed.
      // If a downstream Pod depends on having this data, it will fail as soon as it pulls, so there's
      // no need to cut things off right here.
      if(has_data_)
        push("Output", data_);
    }

  protected:
    T data_;
    bool has_data_;
  };

}

#endif // COMMON_PODS_H
