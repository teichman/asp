#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H

#include <cstring>
#include <math.h>
#include <algorithm>
#include <deque>
#include <limits>

template<typename T0, typename T1>
class Synchronizer
{
public:
  T0 current0_;
  T1 current1_;
  double ts0_;
  double ts1_;
  //! Set to true any time new data is added.
  bool updated_;
  
  Synchronizer(double max_dt);
  void addT0(T0 data0, double timestamp0);
  void addT1(T1 data1, double timestamp1);
  double mostRecent0() const;
  double mostRecent1() const;
  
protected:
  double max_dt_;
  std::deque<double> timestamps0_;
  std::deque<double> timestamps1_;
  std::deque<T0> data0_;
  std::deque<T1> data1_;

  //! These two indices match; set currentX to reflect this
  //! and erase anything older.
  void update(size_t idx0, size_t idx1);
};


/************************************************************
 * Method template implementations
 ************************************************************/
  
template<typename T0, typename T1>
Synchronizer<T0, T1>::Synchronizer(double max_dt) :
  ts0_(std::numeric_limits<double>::quiet_NaN()),
  ts1_(std::numeric_limits<double>::quiet_NaN()),
  updated_(false),
  max_dt_(max_dt)
{
}

template<typename T0, typename T1>
void Synchronizer<T0, T1>::addT0(T0 data0, double timestamp0)
{
  timestamps0_.push_back(timestamp0);
  data0_.push_back(data0);

  for(int i = timestamps1_.size() - 1; i >= 0; --i) {
    if(fabs(timestamps1_[i] - timestamp0) < max_dt_) {
      update(timestamps0_.size() - 1, i);
      break;
    }
  }
}

template<typename T0, typename T1>
void Synchronizer<T0, T1>::addT1(T1 data1, double timestamp1)
{
  timestamps1_.push_back(timestamp1);
  data1_.push_back(data1);

  for(int i = timestamps0_.size() - 1; i >= 0; --i) {
    if(fabs(timestamps0_[i] - timestamp1) < max_dt_) {
      update(i, timestamps1_.size() - 1);
      break;
    }
  }
}

template<typename T0, typename T1>
void Synchronizer<T0, T1>::update(size_t idx0, size_t idx1)
{
  ts0_ = timestamps0_[idx0];
  ts1_ = timestamps1_[idx1];
  current0_ = data0_[idx0];
  current1_ = data1_[idx1];

  data0_.erase(data0_.begin(), data0_.begin() + idx0 + 1);
  data1_.erase(data1_.begin(), data1_.begin() + idx1 + 1);
  timestamps0_.erase(timestamps0_.begin(), timestamps0_.begin() + idx0 + 1);
  timestamps1_.erase(timestamps1_.begin(), timestamps1_.begin() + idx1 + 1);

  updated_ = true;
}

template<typename T0, typename T1>
double Synchronizer<T0, T1>::mostRecent0() const
{
  if(timestamps0_.empty())
    return ts0_;
  else {
    if(isnan(ts0_))
      return timestamps0_.back();
    else
      return std::max(ts0_, timestamps0_.back());
  }
}

template<typename T0, typename T1>
double Synchronizer<T0, T1>::mostRecent1() const
{
  if(timestamps1_.empty())
    return ts1_;
  else {
    if(isnan(ts1_))
      return timestamps1_.back();
    else
      return std::max(ts1_, timestamps1_.back());
  }
}

#endif // SYNCHRONIZER_H
