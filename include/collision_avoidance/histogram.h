#ifndef COLLISION_AVOIDANCE_HISTOGRAM_H
#define COLLISION_AVOIDANCE_HISTOGRAM_H

#include <vector>

namespace collision_avoidance
{
template <class T, class Allocator = std::allocator<T>>
class PolarHistogram
{
private:
  std::vector<T> histogram;

  size_t num_buckets_;
  double bucket_size_;
  double half_bucket_size_;

private:
  inline double radBounded(double value) const
  {
    return std::remainder(value, 2.0 * M_PI);
  }

  inline double posRad(double value) const
  {
    value = radBounded(value);
    if (value < 0)
    {
      return value + (2.0 * M_PI);
    }
    return value;
  }

public:
  PolarHistogram(size_t num_buckets, const T& value = T(), const Allocator& alloc = Allocator())
    : histogram(num_buckets, default_value, alloc)
    , num_buckets_(num_buckets)
    , bucket_size_(2.0 * M_PI / num_buckets_)
    , half_bucket_size_(bucket_size_ / 2.0)
  {
  }

  inline double bucketSize() const
  {
    return bucket_size_;
  }

  inline size_t numBuckets() const
  {
    return num_buckets_;
  }

  T& operator[](double direction)
  {
    direction += half_bucket_size_;
    return histogram[posRad(direction) / bucket_size_];
  }

  const T& operator[](double direction)
  {
    direction += half_bucket_size_;
    return histogram[posRad(direction) / bucket_size_];
  }

  typedef std::vector<T>::iterator iterator;
  typedef std::vector<T>::const_iterator const_iterator;
  iterator begin()
  {
    histogram.begin();
  }
  const_iterator begin() const
  {
    histogram.begin();
  }
  iterator end()
  {
    return histogram.end();
  }
  const_iterator end() const
  {
    return histogram.end();
  }
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_HISTOGRAM_H