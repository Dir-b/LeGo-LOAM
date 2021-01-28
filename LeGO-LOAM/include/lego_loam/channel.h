#ifndef CHANNEL_H
#define CHANNEL_H

#include <thread>
#include <mutex>
#include <condition_variable>

// 以线程安全的方式将信息从一个线程移动到另一个线程的简单机制，线程互斥
// 发布者将唤醒接收者
// 接收者只针对一个写入者和一个读取者工作

// Simplistic mechanism to move information from one thread to another in a thread-safe way.
// Sender will wake up the receiver.
// This will work only for 1 writer and 1 reader. Not buffered.
template<class T> class Channel {
 private:
  T _item;
  bool _empty;
  bool _blocking_send;
  std::mutex _m;
  std::condition_variable _cv;
 public:

  Channel(bool blocking_send ): _empty(true), _blocking_send(blocking_send) {}

  // Move an item into the channel.
  // Block if not empty
  void send(T&& item) {
    std::unique_lock<std::mutex> lock(_m);
    if(_blocking_send){
      // condition_variable对象的某个wait函数被调用的时候，使用mutex阻塞当前线程
      // 直到另外一个线程在相同的condition_variable对象上调用notification来唤醒当前线程
      _cv.wait(lock, [&](){ return _empty; });
    }
    _item = std::move(item);
    _empty = false;
    _cv.notify_all();  // 唤醒所有线程
  }

  // Copy an item into the channel.
  // Block if not empty
  void send(const T &item) {
    std::unique_lock<std::mutex> lock(_m);
    if(_blocking_send){
      _cv.wait(lock, [&](){ return _empty; });
    }
    _item = item;
    _empty = false;
    _cv.notify_all();
  }

  // Move an item out of the channel.
  // Block if empty
  void receive(T &item) {
    std::unique_lock<std::mutex> lock(_m);
    _cv.wait(lock, [&](){ return !_empty; });
    item = std::move(_item);
    _empty = true;
    _cv.notify_all();
  }

};

#endif // CHANNEL_H
