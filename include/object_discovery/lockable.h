#pragma once

#include <errno.h>
#include <pthread.h>

class Lockable {
 public:
  pthread_mutex_t mutex_;

  Lockable();
  void lock();
  void unlock();
  bool trylock();
};
