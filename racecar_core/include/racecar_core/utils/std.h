//
// Created by yenkn on 19-5-17.
//

#ifndef PROJECT_UTILS_STD_H
#define PROJECT_UTILS_STD_H

#include <vector>
#include <map>
#include <sys/time.h>

namespace utils {

template <typename TKey, typename TValue>
std::vector<TKey> mapKeys(const std::map<TKey, TValue> &map) {
  std::vector<TKey> keys(map.size());
  for(size_t i = 0; i < keys.size(); i++) {
    keys[i] = (map.begin()+i)->first;
  }
  return keys;
}

/**
 * Returns the current time in microseconds.
 */
long getMicrotime() {
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}

}

#endif //PROJECT_UTILS_STD_H
