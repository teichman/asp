#ifndef OUTLET_H
#define OUTLET_H

#include <pipeline/common.h>

namespace pl
{
  
  class Pod;

  //! Generic class for passing data out of a pl::Pod.
  //! The T will be copied a bit so it should generally be something small,
  //! like a pointer or shared_ptr.
  //!
  //! Convention: Outlets that pass shared_ptrs will reallocate every time
  //! rather than erase the data from under the pointer.  Note that reallocation
  //! is generally expensive and that you should probably be using fixed storage
  //! whenever possible, and in this case your Outlets should pass pointers.
  //!
  //! Finally, you should probably prefer const pointers or const shared_ptrs.
  //! Different threads executing downstream Pods are likely to simultaneously read the data
  //! that these pointers point to, so nothing should be modifying them.
  class Outlet
  {
  public:    
    //! T is set with the default constructor.
    Outlet(std::string name, Pod* pod);
    template<typename T> void pull(T* dest) const;
    template<typename T> T pull() const;
    //! Only the owner should push.
    template<typename T> void push(T data);
    template<typename T> void setType();
    void flush() { data_ = (void*)NULL; has_data_ = false; }
    Pod* pod() const { return pod_; }
    std::string getName() const { return name_; }
    std::string address() const;
    bool hasData() const { return has_data_; }
    bool checkType(boost::any test) const { return (type_.type() == test.type()); }
    
  private:
    bool has_data_;
    std::string name_;
    Pod* pod_;
    boost::any data_;
    boost::any type_;
    
    //! No copy constructing.
    Outlet(const Outlet& otl);
    //! No assignment.
    Outlet& operator=(const Outlet& otl);
  };

}

#endif // OUTLET_H
